#!/usr/bin/env python3

import cv2 as cv
import message_filters
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


class BothMaskLines(Node):
    def __init__(self):
        super().__init__("lane_to_pointcloud")

        self.declare_parameter("max_distance", 5.0)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=60))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Camera Info Subscriber
        self.create_subscription(
            CameraInfo, "/routecam/camera_info", self.camera_info_callback, qos_profile_sensor_data
        )

        # Synchronized Subscribers for the two masks
        self.white_sub = message_filters.Subscriber(self, Image, "/white_mask", qos_profile=qos)
        self.yellow_sub = message_filters.Subscriber(self, Image, "/yellow_mask", qos_profile=qos)

        # Use ApproximateTimeSynchronizer to group the masks together by timestamp
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.white_sub, self.yellow_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.masks_callback)

        # Separate Publishers
        self.white_pub = self.create_publisher(PointCloud2, "/white_lane_points", qos)
        self.yellow_pub = self.create_publisher(PointCloud2, "/yellow_lane_points", qos)

        self.camera_frame = "route_cam_link"

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def quat_to_rot_matrix(self, qx, qy, qz, qw):
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, xw = qx * qy, qx * qz, qx * qw
        yz, yw, zw = qy * qz, qy * qw, qz * qw

        return np.array([
            [1 - 2 * (yy + zz), 2 * (xy - zw),     2 * (xz + yw)],
            [2 * (xy + zw),     1 - 2 * (xx + zz), 2 * (yz - xw)],
            [2 * (xz - yw),     2 * (yz + xw),     1 - 2 * (xx + yy)]
        ])

    def process_mask_to_points(self, mask_cv, R, O, tz):
        """Helper function to apply vectorized transforms to a single mask."""
        _, mask_thresh = cv.threshold(mask_cv, 127, 255, cv.THRESH_BINARY)
        
        white_pixels = np.where(mask_thresh == 255)
        if len(white_pixels[0]) == 0:
            return None

        rows = white_pixels[0]
        cols = white_pixels[1]

        ray_cam_x = (self.cx - cols) / self.fx
        ray_cam_y = (self.cy - rows) / self.fy
        ray_cam_z = np.ones_like(cols, dtype=np.float32)

        rays_cam = np.vstack((ray_cam_x, ray_cam_y, ray_cam_z))

        rays_map = R @ rays_cam
        rays_map_z = rays_map[2, :]

        valid_mask = np.abs(rays_map_z) > 1e-6
        rays_map = rays_map[:, valid_mask]
        rays_map_z = rays_map_z[valid_mask]

        t = -tz / rays_map_z

        front_mask = t > 0
        rays_map = rays_map[:, front_mask]
        t = t[front_mask]

        if len(t) == 0:
            return None

        points_map = O + rays_map * t
        points_map = points_map.T

        # --- Distance Cropping ---
        max_dist = self.get_parameter("max_distance").value
        distances = np.linalg.norm(points_map - O.T, axis=1)
        dist_mask = distances <= max_dist
        points_map = points_map[dist_mask]

        if len(points_map) == 0:
            return None

        return points_map

    def masks_callback(self, white_msg: Image, yellow_msg: Image):
        if self.fx is None:
            return

        # Lookup TF exactly ONCE for both images
        try:
            trans = self.tf_buffer.lookup_transform(
                "map",
                self.camera_frame,
                white_msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return

        # Prepare Translation and Rotation matrices ONCE
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        O = np.array([[tx], [ty], [tz]])

        R = self.quat_to_rot_matrix(
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )

        # Extract points
        white_cv = self.bridge.imgmsg_to_cv2(white_msg, "mono8")
        yellow_cv = self.bridge.imgmsg_to_cv2(yellow_msg, "mono8")

        white_points = self.process_mask_to_points(white_cv, R, O, tz)
        yellow_points = self.process_mask_to_points(yellow_cv, R, O, tz)

        # Prepare shared PointCloud2 header and fields
        header = Header()
        header.frame_id = "map"
        header.stamp = white_msg.header.stamp

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Publish White
        if white_points is not None:
            white_cloud = pc2.create_cloud(header, fields, white_points.tolist())
            self.white_pub.publish(white_cloud)

        # Publish Yellow
        if yellow_points is not None:
            yellow_cloud = pc2.create_cloud(header, fields, yellow_points.tolist())
            self.yellow_pub.publish(yellow_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = BothMaskLines()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()