"""Starts only the executor. Never starts/reconfigures a driver or sends a goal."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('real_test.launch.py requires ROS_DOMAIN_ID=42')
    config = os.path.join(get_package_share_directory('stretch_guarded_executor'),
                          'config', 'executor_real_test.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('execution_enabled', default_value='false'),
        Node(package='stretch_guarded_executor', executable='guarded_executor',
             namespace='guarded_executor', name='executor', output='screen',
             parameters=[config, {
                 'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                 'execution_enabled': ParameterValue(LaunchConfiguration('execution_enabled'), value_type=bool),
             }]),
    ])
