"""Camera + shared RGB-D pipeline + optional desktop image viewer. No motion nodes."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('stretch_rgbd_perception')
    config = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('serial', default_value=''),
        DeclareLaunchArgument('pose_source', default_value='tf'),
        DeclareLaunchArgument('reference_frame', default_value='odom'),
        DeclareLaunchArgument('ocr_enabled', default_value='true'),
        DeclareLaunchArgument('python_executable', default_value=os.environ.get('STRETCH_PERCEPTION_PYTHON', os.path.expanduser('~/venvs/stretch_perception/bin/python3'))),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'perception.yaml')),
        SetEnvironmentVariable('OMP_NUM_THREADS', '4'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
            condition=IfCondition(config('start_camera')),
            launch_arguments={'camera_namespace': '', 'camera_name': 'camera',
                'serial_no': config('serial'), 'device_type': 'd435', 'enable_color': 'true', 'enable_depth': 'true',
                'enable_infra1': 'false', 'enable_infra2': 'false',
                'enable_gyro': 'false', 'enable_accel': 'false',
                'rgb_camera.color_profile': '848x480x30', 'depth_module.depth_profile': '848x480x30',
                'align_depth.enable': 'true', 'enable_sync': 'true',
                'pointcloud.enable': 'false', 'initial_reset': 'false'}.items()),
        Node(package='stretch_rgbd_perception', executable='perception_node', name='rgbd_perception',
             prefix=[config('python_executable'), ' -B'], output='screen',
             parameters=[config('config'), {'pose_source': config('pose_source'),
                 'reference_frame': config('reference_frame'),
                 'ocr_enabled': ParameterValue(config('ocr_enabled'), value_type=bool)}]),
        Node(package='rqt_image_view', executable='rqt_image_view', name='rgbd_image_view',
             arguments=['/perception/preview'], condition=IfCondition(config('viewer')), output='screen'),
    ])
