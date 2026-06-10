from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("inference"),
        "config",
        "inference_depth.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="inference",
                executable="depth_node",
                name="depth_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="inference",
                executable="inference_node",
                name="inference_node",
                parameters=[config],
                output="screen",
            ),
        ]
    )
