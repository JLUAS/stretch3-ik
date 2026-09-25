"""Isolated RViz IK preview. This launch contains no hardware control nodes."""
import os
from pathlib import Path
from importlib.util import find_spec
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    share = Path(get_package_share_directory('stretch_custom_kinematics'))
    offline = LaunchConfiguration('offline').perform(context).lower() == 'true'
    if offline and os.environ.get('ROS_DOMAIN_ID') != '20':
        raise RuntimeError('offline:=true is allowed only on ROS_DOMAIN_ID=20')
    if os.environ.get('ROS_DOMAIN_ID') not in ('20', '42'):
        raise RuntimeError('Select domain 20 or 42 with use_sim.sh or use_real.sh')
    meshes = LaunchConfiguration('mesh_package_path').perform(context)
    if not meshes:
        # Optional local vendor assets; never changes the runtime kinematic URDF.
        spec = find_spec('stretch_urdf')
        if spec and spec.submodule_search_locations:
            candidate = Path(next(iter(spec.submodule_search_locations))) / 'SE3'
            if (candidate / 'meshes').exists():
                meshes = str(candidate)
    nodes = []
    remaps = []
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    topic = LaunchConfiguration('joint_state_topic').perform(context)
    if offline:
        if use_sim_time:
            raise RuntimeError('Fixed offline fixture uses wall time; use_sim_time must be false')
        urdf = (share / 'config' / 'captured_robot.urdf').read_text()
        nodes += [Node(package='stretch_custom_kinematics', executable='offline_seed.py', output='screen'),
                  Node(package='robot_state_publisher', executable='robot_state_publisher',
                       namespace='ik_input', parameters=[{'robot_description': urdf}],
                       remappings=[('/tf', '/ik_input/tf'), ('/tf_static', '/ik_input/tf_static')])]
        topic = '/ik_input/joint_states'
        remaps = [(f'/{name}', f'/ik_input/{name}') for name in
                  ('robot_description', 'is_homed', 'is_runstopped', 'tf', 'tf_static')]
    nodes.append(Node(package='stretch_custom_kinematics', executable='ik_preview',
                      namespace='ik_preview', output='screen',
                      parameters=[{'joint_state_topic': topic, 'use_sim_time': use_sim_time,
                                   'mesh_package_path': meshes,
                                   'hardware_limits_file': str(share / 'config' / 'hardware_limits.csv')}],
                      remappings=remaps))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz', namespace='ik_preview',
                          arguments=['-d', str(share / 'rviz' / 'ik_preview.rviz')],
                          parameters=[{'use_sim_time': use_sim_time}],
                          remappings=[('/tf', '/ik_preview/tf'), ('/tf_static', '/ik_preview/tf_static')]))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('offline', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joint_state_topic', default_value='/stretch/joint_states'),
        DeclareLaunchArgument('mesh_package_path', default_value=''),
        OpaqueFunction(function=setup)])
