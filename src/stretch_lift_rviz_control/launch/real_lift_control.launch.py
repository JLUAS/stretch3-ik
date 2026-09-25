"""Lift-only guarded executor and RViz marker backend for a real Stretch."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        raise RuntimeError("real_lift_control.launch.py requires ROS_DOMAIN_ID=42")
    guard_share = get_package_share_directory("stretch_guarded_executor")
    control_share = get_package_share_directory("stretch_lift_rviz_control")
    guard_config = os.path.join(guard_share, "config", "executor_real_test.yaml")
    rviz_config = os.path.join(control_share, "rviz", "lift_control.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("execution_enabled", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="stretch_guarded_executor",
            executable="guarded_executor",
            namespace="guarded_executor",
            name="executor",
            output="screen",
            parameters=[guard_config, {
                "dry_run": ParameterValue(
                    LaunchConfiguration("dry_run"), value_type=bool),
                "execution_enabled": ParameterValue(
                    LaunchConfiguration("execution_enabled"), value_type=bool),
            }],
        ),
        Node(
            package="stretch_lift_rviz_control",
            executable="lift_marker",
            namespace="lift_control",
            name="lift_marker",
            output="screen",
            parameters=[{
                "joint_state_topic": "/stretch/joint_states",
                "state_timeout": 0.5,
                "future_stamp_tolerance": 0.15,
                "use_sim_time": False,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="lift_control_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
