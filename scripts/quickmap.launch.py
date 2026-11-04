"""Driver (odometry TF) + lidar + slam_toolbox, for a map from where the robot stands."""
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, name, **arguments):
    path = Path(get_package_share_directory(package)) / 'launch' / name
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())


def slam_params():
    """Stretch's SLAM settings, but every scan is inserted while standing still
    (by default slam_toolbox waits for 0.5 m / 0.5 rad of motion)."""
    source = Path(get_package_share_directory('stretch_nav2')) / 'config' / \
        'mapper_params_online_async.yaml'
    data = yaml.safe_load(source.read_text())
    params = data['slam_toolbox']['ros__parameters']
    params['minimum_travel_distance'] = 0.0
    params['minimum_travel_heading'] = 0.0
    params['map_update_interval'] = 1.0
    out = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump(data, out)
    return out.name


def generate_launch_description():
    return LaunchDescription([
        include('stretch_core', 'stretch_driver.launch.py',
                mode='navigation', broadcast_odom_tf='True'),
        include('stretch_core', 'rplidar.launch.py'),
        include('stretch_nav2', 'online_async_launch.py', slam_params_file=slam_params()),
    ])
