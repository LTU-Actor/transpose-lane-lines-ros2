#!/usr/bin/env python3

import cv2 as cv
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


class SingleMaskLines(Node):
    def __init__(self):
        super().__init__("lane_to_pointcloud")

        self.declare_parameter("max_distance", 5.0)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=60))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Image, "/mask", self.mask_callback, qos)
        self.create_subscription(
            CameraInfo, "/routecam/camera_info", self.camera_info_callback, qos_profile_sensor_data
        )

        self.cloud_pub = self.create_publisher(PointCloud2, "/lane_points", qos)

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

    def mask_callback(self, msg: Image):
        if self.fx is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                "map",
                self.camera_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return

        mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
        _, mask = cv.threshold(mask, 127, 255, cv.THRESH_BINARY)
        
        white_pixels = np.where(mask == 255)
        if len(white_pixels[0]) == 0:
            return

        rows = white_pixels[0]
        cols = white_pixels[1]

        ray_cam_x = (self.cx - cols) / self.fx
        ray_cam_y = (self.cy - rows) / self.fy
        ray_cam_z = np.ones_like(cols, dtype=np.float32)

        rays_cam = np.vstack((ray_cam_x, ray_cam_y, ray_cam_z))

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
            return

        points_map = O + rays_map * t
        points_map = points_map.T

        # --- Distance Cropping ---
        max_dist = self.get_parameter("max_distance").value
        distances = np.linalg.norm(points_map - O.T, axis=1)
        dist_mask = distances <= max_dist
        points_map = points_map[dist_mask]

        if len(points_map) == 0:
            return
        # -------------------------

        header = Header()
        header.frame_id = "map"
        header.stamp = msg.header.stamp

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud = pc2.create_cloud(header, fields, points_map.tolist())
        self.cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = SingleMaskLines()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()