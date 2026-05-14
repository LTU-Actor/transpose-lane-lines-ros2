from launch import LaunchDescription
from launch_ros.actions import Node


def launch_lines():
    return Node(
        package="transpose_lane_lines",
        executable="single_mask_lines",
        name="single_mask_lines",
        output="screen",
        remappings=[
            ("/routecam/image_raw", "/routecam/image_raw"),
            ("/routecam/camera_info", "/routecam/camera_info"),
        ],
        parameters=[],
    )


def generate_launch_description():

    ld = LaunchDescription(
        [
            launch_lines(),
        ]
    )

    return LaunchDescription(
        [
            ld,
        ]
    )
