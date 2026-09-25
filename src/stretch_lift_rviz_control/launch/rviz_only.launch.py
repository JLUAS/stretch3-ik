"""RViz frontend; safety/execution components remain on the Stretch computer."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        raise RuntimeError("rviz_only.launch.py requires ROS_DOMAIN_ID=42")
    config = os.path.join(
        get_package_share_directory("stretch_lift_rviz_control"),
        "rviz", "lift_control.rviz")
    return LaunchDescription([
        Node(package="rviz2", executable="rviz2", name="lift_control_rviz",
             arguments=["-d", config], output="screen"),
    ])
