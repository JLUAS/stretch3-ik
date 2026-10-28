"""Unified bringup: sim:=true (domain 20, local kinematic controller) or sim:=false (real driver, domain 42)."""
from pathlib import Path
from importlib.util import find_spec

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def setup(context):
    share = Path(get_package_share_directory('stretch_custom_manipulation'))
    kinematics_share = Path(get_package_share_directory('stretch_custom_kinematics'))

    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    dry_run = LaunchConfiguration('dry_run').perform(context).lower() == 'true'
    
    execution_enabled_str = LaunchConfiguration('execution_enabled').perform(context)
    if not execution_enabled_str:
        execution_enabled = True
    else:
        execution_enabled = execution_enabled_str.lower() == 'true'

    use_sim_time = False

    meshes = ''
    spec = find_spec('stretch_urdf')
    if spec and spec.submodule_search_locations:
        candidate = Path(next(iter(spec.submodule_search_locations))) / 'SE3'
        if (candidate / 'meshes').exists():
            meshes = str(candidate)

    common_config = str(share / 'config' / 'common.yaml')
    sim_config = str(share / 'config' / 'sim.yaml')
    real_config = str(share / 'config' / 'real.yaml')

    actions = []

    if sim:
        actions.append(SetEnvironmentVariable('ROS_DOMAIN_ID', '20'))
        actions.append(SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'))
        
        urdf = (kinematics_share / 'config' / 'captured_robot.urdf').read_text()
        if meshes:
            urdf = urdf.replace('package://stretch_description', f'file://{meshes}')
        
        actions.append(Node(package='stretch_custom_kinematics', executable='offline_seed.py', output='screen'))
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            parameters=[{'robot_description': urdf}],
                            remappings=[('/joint_states', '/ik_input/joint_states')]))
        
        actions.append(Node(package='stretch_custom_kinematics', executable='ik_preview',
                            namespace='ik_preview', output='screen',
                            parameters=[{'joint_state_topic': '/ik_input/joint_states', 'use_sim_time': use_sim_time,
                                         'mesh_package_path': meshes,
                                         'hardware_limits_file': str(kinematics_share / 'config' / 'hardware_limits.csv')}],
                            remappings=[
                                ('/is_homed', '/ik_input/is_homed'),
                                ('/is_runstopped', '/ik_input/is_runstopped'),
                                ('ik_preview/joint_states', '/ik_preview/joint_states'),
                                ('ik_preview/status', '/ik_preview/status'),
                                ('tf', '/tf'),
                                ('tf_static', '/tf_static')
                            ]))

        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='ghost_base_link',
                            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ik_preview/base_link']))

        actions.append(Node(package='stretch_guarded_executor', executable='guarded_executor',
                            namespace='guarded_executor', name='executor', output='screen',
                            parameters=[common_config, sim_config, {
                                'dry_run': ParameterValue(dry_run, value_type=bool),
                                'execution_enabled': ParameterValue(execution_enabled, value_type=bool),
                            }],
                            remappings=[
                                ('/is_homed', '/ik_input/is_homed'),
                                ('/is_runstopped', '/ik_input/is_runstopped'),
                                ('ik_preview/joint_states', '/ik_preview/joint_states'),
                                ('ik_preview/status', '/ik_preview/status')
                            ]))

    else:
        actions.append(SetEnvironmentVariable('ROS_DOMAIN_ID', '42'))
        actions.append(SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'))

        actions.append(Node(package='stretch_custom_kinematics', executable='ik_preview',
                            namespace='ik_preview', output='screen',
                            parameters=[{'joint_state_topic': '/stretch/joint_states', 'use_sim_time': use_sim_time,
                                         'mesh_package_path': meshes,
                                         'hardware_limits_file': str(kinematics_share / 'config' / 'hardware_limits.csv')}],
                            remappings=[
                                ('tf', '/tf'),
                                ('tf_static', '/tf_static')
                            ]))

        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='ghost_base_link',
                            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ik_preview/base_link']))

        actions.append(Node(package='stretch_guarded_executor', executable='guarded_executor',
                            namespace='guarded_executor', name='executor', output='screen',
                            parameters=[common_config, real_config, {
                                'dry_run': ParameterValue(dry_run, value_type=bool),
                                'execution_enabled': ParameterValue(execution_enabled, value_type=bool),
                            }],
                            remappings=[
                                ('ik_preview/joint_states', '/ik_preview/joint_states'),
                                ('ik_preview/status', '/ik_preview/status')
                            ]))

    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz',
                            arguments=['-d', str(share / 'rviz' / 'bringup.rviz')],
                            parameters=[{'use_sim_time': use_sim_time}],
                            output='screen'))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('dry_run', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('execution_enabled', default_value='', choices=['', 'true', 'false']),
        OpaqueFunction(function=setup)
    ])
