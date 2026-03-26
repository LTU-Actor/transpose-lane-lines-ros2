import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from rclpy.qos import qos_profile_sensor_data

class LaneDetector(Node): # Ros2 Node. Node is base ROS2 class

    # Initialize the node with the name lane_detector
    def __init__(self):
        super().__init__('lane_detector')

        self.get_logger().info("LaneDetector node started")

        # Setup TF
        self.tf_buffer = Buffer() # Initialize the buffer. Buffer() stores all TF transforms
        self.tf_listener = TransformListener(self.tf_buffer, self) # Subscribe to TF topics and populate the buffer

        # Create a publisher that will publish nav_msgs/Path messages, topic=/lane_path, queue size=10
        self.lane_path_pub = self.create_publisher(Path, '/lane_path', 10)

        # Initialize camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.bridge = CvBridge()

        # Subscript to image_raw
        self.create_subscription(
            Image, 
            '/routecam/image_raw', # subscribe to /routecam/image_raw topic
            self.image_callback, 
            qos_profile_sensor_data
        )

        # Subscribe to camera_info
        self.create_subscription(
            CameraInfo,
            '/routecam/camera_info', # subscribe to /routecam/camera_info topic
            self.camera_info_callback,  # call camera_info_callback when a new message arrives
            qos_profile_sensor_data
        )

        # ignore lines with slope < 0.3 &|| > 5.0
        self.config = {
            "line_min_slope" : 0.3,
            "line_max_slope" : 5.0
        }

    # ---------------------------------------------------------------------------
    # Image Callback
    # ---------------------------------------------------------------------------
    def image_callback(self, msg:Image):

        # self.get_logger().info("Image received")

        if self.fx is None:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        self.get_logger().info(f"Image frame_id: {msg.header.frame_id}")

        self.last_image_stamp = msg.header.stamp
        self.last_image_frame = msg.header.frame_id

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info(f"Frame shape: {frame.shape}, dtype: {frame.dtype}")
        
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        height, width = gray.shape
        mask = np.zeros_like(gray)

        polygon = np.array([[
            (0, height),
            (width, height),
            (width, int(height * 0.55)),
            (0, int(height * 0.55))
        ]], dtype=np.int32)

        cv.fillPoly(mask, polygon, 255)
        roi = cv.bitwise_and(gray, mask)

        # edge detection
        edges = cv.Canny(roi, 50, 150)
        edge_count = np.count_nonzero(edges)
        self.get_logger().info(f"Edge pixels detected: {edge_count}")

        # hough transform
        lines = cv.HoughLinesP(
            edges, 
            rho = 1,
            theta = np.pi / 180,
            threshold = 50, 
            minLineLength = 50, 
            maxLineGap = 10
        )

        if lines is None:
            self.get_logger().warn("Hough detected NO lines")
        else:
            self.get_logger().info(f"Hough detected {len(lines)} raw lines")

        line_mat = np.zeros_like(gray)

        self.process_lines(lines, line_mat, frame)

    # ---------------------------------------------------------------------------
    # Camera Info Callback
    # ---------------------------------------------------------------------------
    def camera_info_callback(self, msg:CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_frame = msg.header.frame_id

        self.get_logger().info(
            f"Camera intrinsics loaded: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}, frame={self.camera_frame}"
        )
        # camera instrinsic matrix k
        # [fx, 0, cx, 
        #  0, fy, cy,
        #  0, 0, 1]

    # ---------------------------------------------------------------------------
    # Main Line Processing
    # ---------------------------------------------------------------------------
    def process_lines(self, lines, line_mat, debug_image=None):

        # Create one path per frame
        lane_path = Path()
        lane_path.header.frame_id = "map"
        lane_path.header.stamp = self.get_clock().now().to_msg()

        if lines is None:
            self.get_logger().info("No lines detected")

        elif lines is not None and self.fx is not None:

            for l in lines:
                l = l[0]

                x1, y1 = l[0], l[1] # starting pixel
                x2, y2 = l[2], l[3] # ending pixel

                # calculate the slope 
                diffx = x1 - x2 # run
                diffy = y1 - y2 # rise

                # cannot divide by zero, so skip
                if diffx == 0:
                    continue
                
                # m = rise / run
                slope = diffy / diffx

                # debug slope filter
                ##self.get_logger().info(f"Detected line slope: {slope:.2f}")

                # ignore lines that break the threshold
                if abs(slope) < self.config["line_min_slope"] or \
                   abs(slope) > self.config["line_max_slope"]:
                    continue

                # Extend the line
                diffx *= 5
                diffy *= 5
                x1 -= diffx
                y1 -= diffy
                x2 += diffx
                y2 += diffy

                # Draw for visualization
                cv.line(line_mat, (int(x1), int(y1)),
                        (int(x2), int(y2)), 255, 5)
                
                if debug_image is not None:
                    cv.line(debug_image, (int(x1), int(y1)),
                            (int(x2), int(y2)), 255, 5)
                    
                # convert endpoints to map frame
                pose1 = self.pixel_to_map(x1, y1)
                pose2 = self.pixel_to_map(x2, y2)

                if pose1 is not None:
                    lane_path.poses.append(pose1)
                if pose2 is not None:
                    lane_path.poses.append(pose2)

        # Publish path once
        self.lane_path_pub.publish(lane_path)

        return line_mat
    
    # Pixel -> Map Frame Porjection (fixed-z)
    def pixel_to_map(self, u, v):

        if self.fx is None:
            return None
        
        # meters in front of camera (adjust as needed)
        Z = 1.5

        # Project pixel into camera 3D
        X_cam = (u - self.cx) * Z / self.fx
        Y_cam = (v - self.cy) * Z / self.fy
        Z_cam = Z

        # debug 3D projection
        # self.get_logger().info(f"Pixel ({u}, {v}) projected to camera frame: X={X_cam:.2f}, Y={Y_cam:.2f}, Z={Z_cam:.2f}")

        # package the points to a ros2 message
        point_cam = PointStamped()
        point_cam.header.frame_id = self.camera_frame
        point_cam.header.stamp = self.last_image_stamp
        point_cam.point.x = float(X_cam)
        point_cam.point.y = float(Y_cam)
        point_cam.point.z = float(Z_cam)

        # find the point on the global map
        try:
            point_map = self.tf_buffer.transform(
                point_cam, 
                "map",
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

        # package the pose in a posestamped message
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point_map.point.x
        pose.pose.position.y = point_map.point.y
        pose.pose.position.z = 0.0  # Map is flat
        pose.pose.orientation.w = 1.0   # No rotation

        # return the 3D map point to the process_lines() function to publish
        return pose
    
def main(args = None):
    rclpy.init(args=args)

    node = LaneDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()