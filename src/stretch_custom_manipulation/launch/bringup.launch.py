import os
import yaml
from pathlib import Path
from importlib.util import find_spec

from ament_index_python.packages import (PackageNotFoundError, get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

NAVIGATION_FOOTPRINT = (
    '[[0.10, 0.24], [0.10, -0.24], '
    '[-0.30, -0.24], [-0.30, 0.24]]')


def plugin_installed(plugin):
    """True when the package of a 'pkg::Class' or 'pkg/Class' plugin is installed."""
    package = plugin.replace('::', '/').split('/')[0]
    try:
        get_package_prefix(package)
        return True
    except PackageNotFoundError:
        return False


def configure_nav2(nav2_data, pointcloud_topic=None):
    """Apply the measured stowed envelope and split local/global sensing."""
    for costmap in ('local_costmap', 'global_costmap'):
        params = nav2_data[costmap][costmap]['ros__parameters']
        # A layer whose plugin package is missing stops the costmap from
        # configuring; Nav2 then reports ERROR on /diagnostics and the
        # executor (fail-closed on any ERROR) refuses to move the arm.
        for layer in list(params.get('plugins', [])):
            plugin = params.get(layer, {}).get('plugin', '')
            if plugin and not plugin_installed(plugin):
                params['plugins'].remove(layer)
                print(f'[bringup] {costmap}: dropped layer {layer!r}, plugin {plugin} not installed')
        params['footprint'] = NAVIGATION_FOOTPRINT
        params['footprint_padding'] = 0.01
        params['transform_tolerance'] = 0.3
        params['inflation_layer']['inflation_radius'] = 0.4
        params['inflation_layer']['cost_scaling_factor'] = 3.0
    local_params = nav2_data['local_costmap']['local_costmap']['ros__parameters']
    local_params['update_frequency'] = 5.0
    local_params['publish_frequency'] = 2.0
    nav2_data['controller_server']['ros__parameters']['FollowPath'][
        'min_vel_x'] = 0.0
    if 'voxel_layer' not in local_params['plugins']:
        local_params['plugins'].insert(-1, 'voxel_layer')
    if pointcloud_topic:
        voxel = local_params['voxel_layer']
        sources = voxel.get('observation_sources', '').split()
        if 'depth' not in sources:
            sources.append('depth')
        voxel['observation_sources'] = ' '.join(sources)
        voxel['origin_z'] = 0.0
        voxel['z_resolution'] = 0.1
        voxel['z_voxels'] = 16
        voxel['max_obstacle_height'] = 1.6
        voxel['depth'] = {
            'topic': pointcloud_topic,
            'data_type': 'PointCloud2',
            'clearing': True,
            'marking': True,
            'min_obstacle_height': 0.03,
            'max_obstacle_height': 1.6,
            'obstacle_max_range': 2.5,
            'obstacle_min_range': 0.15,
            'raytrace_max_range': 3.0,
            'raytrace_min_range': 0.15,
            'observation_persistence': 0.0,
            'expected_update_rate': 0.0,
        }
    global_params = nav2_data['global_costmap']['global_costmap']['ros__parameters']
    global_params['update_frequency'] = 2.0
    global_params['publish_frequency'] = 1.0
    global_params['plugins'] = [
        plugin for plugin in global_params['plugins']
        if plugin != 'voxel_layer'
    ]
    global_params.pop('voxel_layer', None)
    return nav2_data

def with_gazebo_perception(urdf):
    return urdf

def camera_scan_urdf():
    return '''<robot name="gazebo_camera_scan">
  <link name="base_link"/>
  <link name="gazebo_camera_mount"/>
  <link name="gazebo_camera_pan_frame"/>
  <link name="gazebo_camera_sensor_frame"/>
  <link name="gazebo_camera_depth_optical_frame"/>
  <joint name="gazebo_camera_mount_joint" type="fixed">
    <parent link="base_link"/><child link="gazebo_camera_mount"/>
    <origin xyz="0.35 0.25 1.25" rpy="0 0 0"/>
  </joint>
  <joint name="gazebo_camera_pan_joint" type="revolute">
    <parent link="gazebo_camera_mount"/><child link="gazebo_camera_pan_frame"/>
    <axis xyz="0 0 1"/><limit lower="-2.8" upper="-1.5" effort="10" velocity="1"/>
  </joint>
  <joint name="gazebo_camera_tilt_joint" type="revolute">
    <parent link="gazebo_camera_pan_frame"/><child link="gazebo_camera_sensor_frame"/>
    <axis xyz="0 1 0"/><limit lower="0.35" upper="0.9" effort="10" velocity="1"/>
  </joint>
  <joint name="gazebo_camera_optical_joint" type="fixed">
    <parent link="gazebo_camera_sensor_frame"/><child link="gazebo_camera_depth_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
  </joint>
</robot>'''

def setup(context):
    share = Path(get_package_share_directory('stretch_custom_manipulation'))
    kinematics_share = Path(get_package_share_directory('stretch_custom_kinematics'))

    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    scenario = LaunchConfiguration('scenario').perform(context)
    small_house = sim and scenario == 'small_house'
    # MuJoCo stretch_mujoco_driver: same ROS interface as the real driver, so it
    # takes the robot path below with sim time, domain 20 and no hardware nodes.
    mujoco = sim and scenario == 'mujoco'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    dry_run = LaunchConfiguration('dry_run').perform(context).lower() == 'true'
    octomap_enabled = LaunchConfiguration('octomap_enabled').perform(context).lower() == 'true'
    collision_environment = LaunchConfiguration('collision_environment').perform(context).lower()
    pointcloud_topic = LaunchConfiguration('pointcloud_topic').perform(context)
    camera_scan = LaunchConfiguration('camera_scan').perform(context).lower() == 'true'
    gazebo_gui = LaunchConfiguration('gazebo_gui').perform(context).lower() == 'true'
    gripper_enabled = LaunchConfiguration('gripper_enabled').perform(context).lower() == 'true'
    start_robot_driver = (
        LaunchConfiguration('start_robot_driver').perform(context).lower() == 'true')
    start_robot_camera = (
        LaunchConfiguration('start_robot_camera').perform(context).lower() == 'true')
    start_robot_lidar = (
        LaunchConfiguration('start_robot_lidar').perform(context).lower() == 'true')
    start_nav2 = LaunchConfiguration('start_nav2').perform(context).lower() == 'true'
    use_slam = LaunchConfiguration('use_slam').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    gazebo_target_object = (
        LaunchConfiguration('gazebo_target_object').perform(context).lower() == 'true')
    if not collision_environment:
        collision_environment = 'primitive' if sim else 'octomap'
    perception_enabled = octomap_enabled or collision_environment == 'octomap'
    grasp_mission = ((not sim or mujoco) and
                     LaunchConfiguration('grasp_mission').perform(context).lower() == 'true')
    if mujoco:
        start_robot_driver = start_robot_camera = start_robot_lidar = False
    if small_house or mujoco:
        collision_environment = 'octomap'
        perception_enabled = True
    
    execution_enabled_str = LaunchConfiguration('execution_enabled').perform(context)
    if not execution_enabled_str:
        execution_enabled = True
    else:
        execution_enabled = execution_enabled_str.lower() == 'true'

    # The MuJoCo driver stamps every message with wall time (its /clock is not
    # used for stamps), so that scenario runs on wall time like the robot.
    use_sim_time = bool(sim and not mujoco and (perception_enabled or small_house))

    meshes = ''
    spec = find_spec('stretch_urdf')
    if spec and spec.submodule_search_locations:
        candidate = Path(next(iter(spec.submodule_search_locations))) / 'SE3'
        if (candidate / 'meshes').exists():
            meshes = str(candidate)

    common_config = str(share / 'config' / 'common.yaml')
    sim_config = str(share / 'config' / 'sim.yaml')
    real_config = str(share / 'config' / 'real.yaml')
    mujoco_config = str(share / 'config' / 'mujoco.yaml')
    perception_config = str(share / 'config' / 'perception.yaml')

    actions = []

    if sim and not mujoco:
        actions.append(SetEnvironmentVariable('ROS_DOMAIN_ID', '20'))
        actions.append(SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'))
        
        if small_house:
            # Nav2 is simulation-only in this bringup.  Import it lazily so the
            # real robot does not need the small-house navigation stack.
            from nav2_common.launch import RewrittenYaml

            simulation_share = Path(
                get_package_share_directory('stretch_small_house_sim'))
            nav2_params = RewrittenYaml(
                source_file=str(simulation_share / 'config' / 'nav2_params.yaml'),
                param_rewrites={
                    'xy_goal_tolerance': '0.05',
                    'yaw_goal_tolerance': '0.08',
                    'movement_time_allowance': '60.0',
                    'min_speed_theta': '0.15',
                    'inflation_radius': '0.4',
                },
                convert_types=True)
            nav2_params_path = nav2_params.perform(context)
            with open(nav2_params_path, encoding='utf-8') as stream:
                nav2_data = yaml.safe_load(stream)
            nav2_data['controller_server']['ros__parameters']['FollowPath'][
                'xy_goal_tolerance'] = 0.05
            # Collision AABB in the commanded navigation stow is
            # x=[-0.280, 0.063], y=[-0.221, 0.221] in base_link.  Keep a small
            # geometry margin here and a separate 1 cm costmap padding.
            # RGB-D remains continuous only in the local navigation costmap.
            # The one-shot frozen OctoMap is still a separate manipulation
            # product captured after the base stops.
            # Dense depth in both costmaps previously duplicated work and
            # starved planning.  Global planning keeps the planar LiDAR (and
            # optional SLAM process); low obstacles belong to the local voxel
            # layer where avoidance is executed.
            configure_nav2(nav2_data)
            with open(nav2_params_path, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(nav2_data, stream)
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    simulation_share / 'launch' / 'stretch_small_house.launch.py')),
                launch_arguments={
                    'gui': LaunchConfiguration('gazebo_gui'),
                    'camera_rate': LaunchConfiguration('camera_rate'),
                }.items()))
            if gazebo_target_object:
                actions.append(Node(
                    package='gazebo_ros', executable='spawn_entity.py',
                    arguments=[
                        '-entity', LaunchConfiguration('target_model_name'),
                        '-file', str(share / 'worlds' / 'approach_target.sdf'),
                        '-x', LaunchConfiguration('target_x'),
                        '-y', LaunchConfiguration('target_y'),
                        '-z', LaunchConfiguration('target_z'),
                        '-timeout', '120'],
                    output='screen'))
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    simulation_share / 'launch' / 'nav2_host.launch.py')),
                launch_arguments={
                    'use_slam': LaunchConfiguration('use_slam'),
                    'params_file': nav2_params_path,
                }.items()))
        else:
            urdf = (kinematics_share / 'config' / 'captured_robot.urdf').read_text()
            if meshes:
                urdf = urdf.replace('package://stretch_description', f'file://{meshes}')
            if perception_enabled:
                urdf = with_gazebo_perception(urdf)

            actions.append(Node(package='stretch_custom_kinematics', executable='offline_seed.py',
                                output='screen',
                                parameters=[{'use_sim_time': use_sim_time}]))
            actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                                parameters=[{'robot_description': urdf, 'use_sim_time': use_sim_time}],
                                remappings=[('/joint_states', '/ik_input/joint_states')]))

        if perception_enabled and not small_house:
            actions.append(ExecuteProcess(
                cmd=['gzserver',
                     str(share / 'worlds' / 'stretch_perception_benchmark.world'),
                     '-s', 'libgazebo_ros_init.so',
                     '-s', 'libgazebo_ros_factory.so'],
                output='screen'))
            if gazebo_gui:
                actions.append(ExecuteProcess(
                    cmd=['gzclient', '--gui-client-plugin=libgazebo_ros_eol_gui.so'],
                    output='screen'))
            actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                                name='gazebo_camera_state_publisher',
                                parameters=[{'robot_description': camera_scan_urdf(),
                                             'use_sim_time': True}],
                                remappings=[
                                    ('/joint_states', '/camera_scan/joint_states'),
                                    ('/robot_description', '/camera_scan/robot_description'),
                                ]))
            actions.append(Node(package='stretch_custom_manipulation',
                                executable='camera_scan.py',
                                name='camera_scan',
                                output='screen',
                                parameters=[{'enabled': camera_scan,
                                             'use_sim_time': True}]))

        joint_state_topic = ('/stretch/joint_states' if small_house
                             else '/ik_input/joint_states')
        actions.append(Node(package='stretch_custom_kinematics', executable='ik_preview',
                            namespace='ik_preview', output='screen',
                            parameters=[{'joint_state_topic': joint_state_topic, 'use_sim_time': use_sim_time,
                                         'mesh_package_path': meshes,
                                         'hardware_limits_file': str(kinematics_share / 'config' / 'hardware_limits.csv')}],
                            remappings=[
                                ('/is_homed', '/is_homed' if small_house else '/ik_input/is_homed'),
                                ('/is_runstopped', '/is_runstopped' if small_house else '/ik_input/is_runstopped'),
                                ('ik_preview/joint_states', '/ik_preview/joint_states'),
                                ('ik_preview/status', '/ik_preview/status'),
                                ('tf', '/tf'),
                                ('tf_static', '/tf_static')
                            ]))

        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='ghost_base_link',
                            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ik_preview/base_link']))
        if not small_house:
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                name='fixture_odom',
                                arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']))

        actions.append(Node(package='stretch_guarded_executor', executable='guarded_executor',
                            namespace='guarded_executor', name='executor', output='screen',
                            parameters=[common_config, sim_config, {
                                'dry_run': ParameterValue(dry_run, value_type=bool),
                                'execution_enabled': ParameterValue(execution_enabled, value_type=bool),
                                'collision_environment': collision_environment,
                                'collision_planning.enabled': True,
                                'use_sim_time': use_sim_time,
                                'joint_state_topic': joint_state_topic,
                                'trajectory_action': ('/stretch_arm_controller/follow_joint_trajectory'
                                                      if small_house else
                                                      '/stretch_controller/follow_joint_trajectory'),
                            }],
                            remappings=[
                                ('/is_homed', '/is_homed' if small_house else '/ik_input/is_homed'),
                                ('/is_runstopped', '/is_runstopped' if small_house else '/ik_input/is_runstopped'),
                                ('ik_preview/joint_states', '/ik_preview/joint_states'),
                                ('ik_preview/status', '/ik_preview/status')
                            ]))

        if small_house and LaunchConfiguration('autonomous_mission').perform(context).lower() == 'true':
            actions.append(Node(
                package='stretch_custom_manipulation',
                executable='autonomous_mission.py',
                name='autonomous_manipulation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'simulation_mode': True,
                    'trajectory_action': '/stretch_arm_controller/follow_joint_trajectory',
                    'execute_manipulation': ParameterValue(
                        execution_enabled, value_type=bool),
                    'manipulation_timeout': 300.0,
                    'navigation_xy_goal_tolerance': 0.05,
                    'navigation_yaw_goal_tolerance': 0.08,
                    'stow_service_required': False,
                    'gazebo_target_model': (
                        LaunchConfiguration('target_model_name')
                        if gazebo_target_object else ''),
                    'auto_run_on_target_move': ParameterValue(
                        LaunchConfiguration('auto_follow_target'), value_type=bool),
                    'navigate_only': ParameterValue(
                        LaunchConfiguration('navigate_only'), value_type=bool),
                }]))

    else:
        actions.append(SetEnvironmentVariable('ROS_DOMAIN_ID', '20' if mujoco else '42'))
        actions.append(SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1' if mujoco else '0'))

        stretch_core_share = None
        if start_robot_driver or start_robot_camera or (start_nav2 and start_robot_lidar):
            stretch_core_share = Path(get_package_share_directory('stretch_core'))

        if start_robot_driver:
            if not os.environ.get('HELLO_FLEET_ID'):
                raise RuntimeError(
                    'start_robot_driver:=true requires running on the Stretch. '
                    'Run this bringup onboard, or use start_robot_driver:=false '
                    'when connecting to an already-running remote driver.')
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    stretch_core_share / 'launch' / 'stretch_driver.launch.py')),
                launch_arguments={
                    'mode': 'navigation' if start_nav2 else 'position',
                    'broadcast_odom_tf': 'True' if start_nav2 else 'False',
                    'fail_out_of_range_goal': 'True',
                }.items()))

        if start_robot_camera:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    stretch_core_share / 'launch' / 'stretch_realsense.launch.py')),
                launch_arguments={
                    'resolution': LaunchConfiguration('robot_camera_resolution'),
                    'publish_frustum_viz': 'false',
                    'publish_upright_img': 'false',
                    'depth_module.profile': '848x480x30',
                    'depth_module.depth_profile': '848x480x30',
                    'rgb_camera.profile': '848x480x30',
                    'rgb_camera.color_profile': '848x480x30',
                }.items()))

        if start_nav2:
            from nav2_common.launch import RewrittenYaml

            stretch_nav2_share = Path(get_package_share_directory('stretch_nav2'))
            if not use_slam and (not map_file or not Path(map_file).is_file()):
                raise RuntimeError(
                    'use_slam:=false requires map:=/absolute/path/to/map.yaml')
            nav2_params = RewrittenYaml(
                source_file=str(stretch_nav2_share / 'config' / 'nav2_params.yaml'),
                param_rewrites={}, convert_types=True)
            nav2_params_path = nav2_params.perform(context)
            with open(nav2_params_path, encoding='utf-8') as stream:
                nav2_data = yaml.safe_load(stream)
            configure_nav2(nav2_data, pointcloud_topic)
            if mujoco:
                # The map is generated from the scene, so it is exact. Scans in the
                # map frame crashed planner_server (uncaught tf2 extrapolation when
                # a scan predates the first map->odom); the local costmap still
                # sees live obstacles in odom.
                g = nav2_data['global_costmap']['global_costmap']['ros__parameters']
                g['plugins'] = [l for l in g['plugins'] if l not in ('obstacle_layer', 'voxel_layer')]
                # Sim odometry is the ground-truth base pose in the world frame and the
                # map is generated in that frame, so map->odom is exactly identity.
                # AMCL published it at ~1 Hz under this load (Nav2 TF extrapolation
                # errors); a static identity replaces it. Localization is not tested.
                nav2_data['amcl']['ros__parameters']['tf_broadcast'] = False
                # object_localizer needs RealSense-style registered depth (16UC1 mm).
                actions.append(Node(package='stretch_custom_manipulation',
                                    executable='sim_aligned_depth.py', name='sim_aligned_depth',
                                    output='screen'))
                # DWB oscillated at the 1 m doorways (sim runs at ~0.56x real time,
                # velocities lag): Regulated Pure Pursuit, sim-sized inflation
                # (Hello Robot suggests 0.2 m for indoor sim scenes), slack progress check.
                c = nav2_data['controller_server']['ros__parameters']
                c['controller_frequency'] = 10.0
                c['progress_checker'].update(required_movement_radius=0.15, movement_time_allowance=30.0)
                c['general_goal_checker'].update(xy_goal_tolerance=0.08, yaw_goal_tolerance=0.12)
                c['FollowPath'] = {
                    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                    'desired_linear_vel': 0.2, 'lookahead_dist': 0.4, 'min_lookahead_dist': 0.25,
                    'max_lookahead_dist': 0.6, 'use_velocity_scaled_lookahead_dist': False,
                    'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.6,
                    'rotate_to_heading_angular_vel': 0.6, 'max_angular_accel': 1.5,
                    'allow_reversing': False, 'use_collision_detection': True,
                    'max_allowed_time_to_collision_up_to_carrot': 1.0,
                    'use_regulated_linear_velocity_scaling': True,
                    'use_cost_regulated_linear_velocity_scaling': False,
                    'regulated_linear_scaling_min_radius': 0.6, 'regulated_linear_scaling_min_speed': 0.05,
                    'transform_tolerance': 0.3}
                # Global: wide, gentle cost slope so paths run mid-doorway; local:
                # tight, so the controller's footprint check does not see the door.
                for cm, radius, scaling in (('local_costmap', 0.3, 6.0), ('global_costmap', 0.55, 4.0)):
                    nav2_data[cm][cm]['ros__parameters']['inflation_layer'].update(
                        inflation_radius=radius, cost_scaling_factor=scaling)
                actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                    name='map_odom_ground_truth',
                                    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']))
            if not use_slam:
                # Start localized so the map frame (RViz fixed frame) exists at
                # once; 2D Pose Estimate corrects it when the robot is elsewhere.
                amcl = nav2_data['amcl']['ros__parameters']
                amcl['set_initial_pose'] = True
                amcl['initial_pose'] = {
                    'x': float(LaunchConfiguration('initial_x').perform(context)),
                    'y': float(LaunchConfiguration('initial_y').perform(context)),
                    'z': 0.0,
                    'yaw': float(LaunchConfiguration('initial_yaw').perform(context)),
                }
            with open(nav2_params_path, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(nav2_data, stream)

            if start_robot_lidar:
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(
                        stretch_core_share / 'launch' / 'rplidar.launch.py'))))
            # stretch_nav2 remaps /cmd_vel to the driver only for the
            # controller; behaviors (DriveOnHeading, BackUp, Spin) would
            # publish to a topic nobody reads. Remap every Nav2 node.
            actions.append(GroupAction([
                SetRemap(src='/cmd_vel', dst='/stretch/cmd_vel'),
                # slam_toolbox reads /scan; the MuJoCo driver has only the filtered scan.
                *([SetRemap(src='/scan', dst='/scan_filtered')] if mujoco else []),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(
                        stretch_nav2_share / 'launch' / 'bringup_launch.py')),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'autostart': 'true',
                        'slam': 'True' if use_slam else 'False',
                        'map': map_file,
                        'params_file': nav2_params_path,
                    }.items()),
            ]))

        actions.append(Node(package='stretch_custom_kinematics', executable='ik_preview',
                            namespace='ik_preview', output='screen',
                            parameters=[{'joint_state_topic': '/stretch/joint_states', 'use_sim_time': use_sim_time,
                                         # Achieved approach within 0.05 rad of the plan; 0.12
                                         # let top-down solutions 0.1 rad off pass as success.
                                         'grasp_approach_tolerance': 0.05,
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
                                'collision_environment': collision_environment,
                                'collision_planning.enabled': True,
                                'use_sim_time': use_sim_time,
                                'sim_bypass_hardware_checks': mujoco,
                            }],
                            remappings=[
                                ('ik_preview/joint_states', '/ik_preview/joint_states'),
                                ('ik_preview/status', '/ik_preview/status')
                            ]))

        if grasp_mission:
            actions.append(Node(package='stretch_custom_manipulation',
                                executable='gripper_bridge.py', name='gripper_bridge',
                                parameters=[mujoco_config if mujoco else real_config,  # aperture calibration
                                            {'use_sim_time': use_sim_time}],
                                output='screen'))
            actions.append(Node(package='stretch_custom_manipulation',
                                executable='object_localizer.py', name='object_localizer',
                                prefix=LaunchConfiguration('perception_python'),
                                parameters=[{'use_sim_time': use_sim_time}],
                                output='screen'))
            # Continuous detection/tracking/OCR; reuse this bringup's camera.
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(Path(get_package_share_directory(
                    'stretch_rgbd_perception')) / 'launch' / 'perception.launch.py')),
                launch_arguments={
                    'start_camera': 'false',
                    'viewer': LaunchConfiguration('perception_viewer'),
                    'pose_source': LaunchConfiguration('perception_pose_source'),
                    'reference_frame': 'odom',
                    'python_executable': LaunchConfiguration('perception_python'),
                    'ocr_enabled': LaunchConfiguration('perception_ocr'),
                    'use_sim_time': 'true' if use_sim_time else 'false',
                    **({'override_config': str(share / 'config' / 'perception_mujoco.yaml')}
                       if mujoco else {}),
                }.items()))
            actions.append(Node(package='stretch_custom_manipulation',
                                executable='grasp_mission.py', name='grasp_mission',
                                parameters=[mujoco_config if mujoco else real_config,  # max_aperture
                                            {'dry_run': ParameterValue(dry_run, value_type=bool),
                                             'use_sim_time': use_sim_time}],
                                # The MuJoCo driver publishes only the filtered scan.
                                remappings=[('/scan', '/scan_filtered')] if mujoco else [],
                                output='screen'))
        elif start_nav2 and LaunchConfiguration(
                'autonomous_mission').perform(context).lower() == 'true':
            actions.append(Node(
                package='stretch_custom_manipulation',
                executable='autonomous_mission.py',
                name='autonomous_manipulation',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'simulation_mode': False,
                    # The mission target is odom-relative so it appears as soon
                    # as the real driver publishes odometry; Nav2 transforms it
                    # through map when SLAM/localization is active.
                    'global_frame': 'odom',
                    'trajectory_action': '/stretch_controller/follow_joint_trajectory',
                    'navigation_xy_goal_tolerance': 0.20,
                    'navigation_yaw_goal_tolerance': 0.15,
                    'navigation_lift_height': ParameterValue(
                        LaunchConfiguration('navigation_lift_height'),
                        value_type=float),
                    'navigation_wrist_pitch': ParameterValue(
                        LaunchConfiguration('navigation_wrist_pitch'),
                        value_type=float),
                    'execute_manipulation': ParameterValue(
                        execution_enabled, value_type=bool),
                    'stow_service_required': True,
                    'auto_run_on_target_move': False,
                    'navigate_only': ParameterValue(
                        LaunchConfiguration('navigate_only'), value_type=bool),
                }]))

    if gripper_enabled:
        config_file = sim_config if sim else real_config
        if sim:
            actions.append(Node(
                package='stretch_custom_manipulation',
                executable='sim_gripper.py',
                name='sim_gripper',
                output='screen',
                parameters=[config_file, {'use_sim_time': use_sim_time}]))

        actions.append(Node(
            package='stretch_custom_manipulation',
            executable='pick_state_machine.py',
            name='pick_state_machine',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]))

        actions.append(Node(
            package='stretch_custom_manipulation',
            executable='place_state_machine.py',
            name='place_state_machine',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]))

    if perception_enabled:
        actions.append(Node(package='stretch_collision_planning',
                            executable='octomap_builder',
                            name='octomap_builder',
                            output='screen',
                            parameters=[perception_config, {
                                'input_cloud_topic': pointcloud_topic,
                                'use_sim_time': use_sim_time,
                                # Simulation fixtures have no /odom topic.
                                'map_frame': 'base_link' if sim and not mujoco else 'odom',
                            }]))

    if rviz:
        rviz_arguments = ['-d', str(share / 'rviz' / 'bringup.rviz')]
        if (not sim or mujoco) and start_nav2:
            rviz_arguments += ['-f', 'map']
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz',
                            arguments=rviz_arguments,
                            parameters=[{'use_sim_time': use_sim_time}],
                            output='screen'))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('scenario', default_value='fixture',
                              choices=['fixture', 'small_house', 'mujoco']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('dry_run', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('execution_enabled', default_value='', choices=['', 'true', 'false']),
        DeclareLaunchArgument('octomap_enabled', default_value='false', choices=['true', 'false']),
        # Empty: primitive desk in simulation, live OctoMap on the robot.
        DeclareLaunchArgument('collision_environment', default_value='',
                              choices=['', 'primitive', 'octomap']),
        DeclareLaunchArgument('grasp_mission', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('perception_python',
                              default_value=os.environ.get('STRETCH_PERCEPTION_PYTHON',
                                  os.path.expanduser('~/venvs/stretch_perception/bin/python3'))),
        DeclareLaunchArgument('perception_pose_source', default_value='tf', choices=['tf', 'board', 'none']),
        DeclareLaunchArgument('perception_ocr', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('perception_viewer', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('pointcloud_topic', default_value='/camera/depth/color/points'),
        DeclareLaunchArgument('camera_scan', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gazebo_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera_rate', default_value='5.0'),
        DeclareLaunchArgument('use_slam', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('autonomous_mission', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gripper_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_robot_driver', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_robot_camera', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_robot_lidar', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_nav2', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('robot_camera_resolution', default_value='low',
                              choices=['low', 'high']),
        DeclareLaunchArgument('map', default_value=''),
        # Robot pose on the map at startup (AMCL initial pose, no SLAM).
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('gazebo_target_object', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('target_model_name', default_value='approach_target'),
        DeclareLaunchArgument('target_x', default_value='1.0'),
        DeclareLaunchArgument('target_y', default_value='-3.5'),
        DeclareLaunchArgument('target_z', default_value='0.13'),
        DeclareLaunchArgument('auto_follow_target', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('navigate_only', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('navigation_lift_height', default_value='0.45'),
        DeclareLaunchArgument('navigation_wrist_pitch', default_value='-0.20'),
        OpaqueFunction(function=setup)
    ])
