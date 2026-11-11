"""Semantic grasp node in the perception venv (torch + transformers for OWLv2)."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('python_executable', default_value=os.environ.get(
            'STRETCH_PERCEPTION_PYTHON', os.path.expanduser('~/venvs/stretch_perception/bin/python3'))),
        Node(package='stretch_semantic_grasp', executable='semantic_grasp_node',
             name='semantic_grasp', output='screen',
             prefix=[LaunchConfiguration('python_executable'), ' -B']),
    ])
