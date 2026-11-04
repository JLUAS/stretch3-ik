"""Grasp-mission integration stack against the mock driver (isolated tests).

Real nodes: ik_preview, guarded_executor (real profile, transport_test_mode,
primitive desk scene), gripper_bridge, grasp_mission. Mocks: driver,
perception. Start only through ws/scripts/sim_isolated.sh.

mission_script:=<path> runs another grasp_mission.py (the baseline copy).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent


def setup(context):
    if os.environ.get('ROS_DOMAIN_ID') != '20' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('grasp_sim must run in domain 20 with ROS_LOCALHOST_ONLY=1')
    if [n for n in os.listdir('/sys/class/net') if n != 'lo']:
        raise RuntimeError('grasp_sim must run in a loopback-only network namespace')
    manip = Path(get_package_share_directory('stretch_custom_manipulation'))
    kin = Path(get_package_share_directory('stretch_custom_kinematics'))
    urdf = (kin / 'config' / 'captured_robot.urdf').read_text()
    script = LaunchConfiguration('mission_script').perform(context)
    tolerance = float(LaunchConfiguration('grasp_approach_tolerance').perform(context))
    mission_params = LaunchConfiguration('mission_params').perform(context)
    actions = [
        ExecuteProcess(cmd=['python3', str(HERE / 'mock_stretch_driver.py')], output='screen'),
        ExecuteProcess(cmd=['python3', str(HERE / 'mock_perception.py')], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}],
             remappings=[('/joint_states', '/stretch/joint_states')]),
        Node(package='stretch_custom_kinematics', executable='ik_preview', namespace='ik_preview',
             prefix=LaunchConfiguration('ik_preview_prefix').perform(context) or None,
             parameters=[{'joint_state_topic': '/stretch/joint_states',
                          'grasp_approach_tolerance': tolerance,
                          'hardware_limits_file': str(kin / 'config' / 'hardware_limits.csv')}],
             remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ik_preview/base_link']),
        Node(package='stretch_guarded_executor', executable='guarded_executor',
             namespace='guarded_executor', name='executor',
             parameters=[str(manip / 'config' / 'common.yaml'), str(manip / 'config' / 'real.yaml'),
                         {'transport_test_mode': True, 'dry_run': False, 'execution_enabled': True,
                          'collision_environment': 'primitive', 'collision_planning.enabled': True}],
             remappings=[('ik_preview/joint_states', '/ik_preview/joint_states'),
                         ('ik_preview/status', '/ik_preview/status')]),
        Node(package='stretch_custom_manipulation', executable='gripper_bridge.py',
             name='gripper_bridge'),
    ]
    cmd = ['python3', script, '--ros-args', '-r', '__node:=grasp_mission']
    if mission_params:
        cmd += ['--params-file', mission_params]
    actions.append(ExecuteProcess(cmd=cmd, output='screen',
                                  additional_env={'PYTHONPATH': str(Path(script).parent) + ':' +
                                                  os.environ.get('PYTHONPATH', '')}))
    return actions


def generate_launch_description():
    default = str(HERE.parents[1] / 'scripts' / 'grasp_mission.py')
    return LaunchDescription([
        DeclareLaunchArgument('mission_script', default_value=default),
        DeclareLaunchArgument('mission_params', default_value=''),
        DeclareLaunchArgument('grasp_approach_tolerance', default_value='0.12'),
        # Baseline runs: a wrapper that execs the saved baseline ik_preview binary.
        DeclareLaunchArgument('ik_preview_prefix', default_value=''),
        OpaqueFunction(function=setup)])
