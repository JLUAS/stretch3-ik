import ast
import importlib.util
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET


path = Path(__file__).parents[1] / 'scripts' / 'autonomous_mission.py'
spec = importlib.util.spec_from_file_location('autonomous_mission', path)
mission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mission)

launch_path = Path(__file__).parents[1] / 'launch' / 'bringup.launch.py'
launch_spec = importlib.util.spec_from_file_location('bringup_launch', launch_path)
bringup = importlib.util.module_from_spec(launch_spec)
launch_spec.loader.exec_module(bringup)


def test_approach_keeps_target_on_robot_right():
    x, y, yaw = mission.approach_pose(1.0, 0.0, 0.0, 0.0, 0.62)
    assert math.isclose(x, 0.38)
    assert math.isclose(y, 0.0)
    right_x, right_y = math.sin(yaw), -math.cos(yaw)
    assert math.isclose(right_x, 1.0)
    assert math.isclose(right_y, 0.0, abs_tol=1.0e-12)


def test_real_base_pose_uses_target_ray_not_marker_orientation():
    target = mission.PoseStamped()
    target.pose.position.x = 1.0
    target.pose.orientation.z = 1.0  # Must not reverse the approach direction.
    odom = SimpleNamespace(pose=SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))))
    fake = SimpleNamespace(
        odom=odom, global_frame='odom',
        get_parameter=lambda name: SimpleNamespace(value=0.55))

    goal = mission.AutonomousMission._base_pose(fake, target)

    assert math.isclose(goal.pose.position.x, 0.45)
    assert math.isclose(goal.pose.position.y, 0.0)
    assert math.isclose(
        mission.yaw_from_quaternion(goal.pose.orientation), math.pi / 2.0)


def test_near_goal_handoff_requires_position_and_orientation():
    assert mission.near_goal_handoff(0.163, 0.20, 0.22, 0.35)
    assert not mission.near_goal_handoff(0.23, 0.20, 0.22, 0.35)
    assert not mission.near_goal_handoff(0.16, 0.40, 0.22, 0.35)


def test_head_angles_are_calibratable():
    pan, tilt = mission.head_angles(0.0, -1.0, 0.5, 0.0, 0.0, 1.0,
                                    pan_offset=0.1, tilt_offset=-0.2)
    assert math.isclose(pan, -math.pi / 2.0 + 0.1)
    assert math.isclose(tilt, math.atan2(-0.5, 1.0) - 0.2)


def test_closest_costmap_obstacle_includes_rgbd_cells():
    origin = SimpleNamespace(x=-1.0, y=-1.0)
    base = SimpleNamespace(x=0.0, y=0.0)
    message = SimpleNamespace(
        info=SimpleNamespace(
            origin=SimpleNamespace(position=origin),
            resolution=1.0, width=2),
        data=[0, 100, 0, 0])
    assert math.isclose(
        mission.closest_costmap_obstacle(message, base),
        math.hypot(0.5, -0.5))


def test_gazebo_object_pose_tracks_world_to_odom_transform():
    def pose(x, y, yaw):
        value = mission.Pose()
        value.position.x, value.position.y = x, y
        value.orientation.z = math.sin(yaw / 2.0)
        value.orientation.w = math.cos(yaw / 2.0)
        return value

    # Robot faces +Y in Gazebo but odom starts with the robot facing +X.
    # An object one metre in front must therefore be (1, 0) in odom.
    transformed = mission.gazebo_pose_in_odom(
        pose(0.0, -2.5, math.pi / 2.0),
        pose(0.0, -3.5, math.pi / 2.0),
        pose(0.0, 0.0, 0.0))
    assert math.isclose(transformed.position.x, 1.0, abs_tol=1.0e-12)
    assert math.isclose(transformed.position.y, 0.0, abs_tol=1.0e-12)
    assert math.isclose(
        mission.yaw_from_quaternion(transformed.orientation), 0.0,
        abs_tol=1.0e-12)


def test_navigate_only_stops_before_camera_and_arm():
    class FakeMission:
        def __init__(self):
            self.events = []
            self.running = True
            self.lock = threading.Lock()

        def emit(self, value):
            self.events.append(value)

        def _stow_for_navigation(self):
            self.events.append('stow')

        def _navigate(self, _target):
            self.events.append('navigate')

        def _wait_stopped(self):
            self.events.append('stopped')

        def _aim_head(self, _target):
            raise AssertionError('camera must not move in navigate-only mode')

        def get_parameter(self, name):
            assert name == 'navigate_only'
            return SimpleNamespace(value=True)

    fake = FakeMission()
    fake.gazebo_target_model = 'approach_target'
    mission.AutonomousMission._run(fake, object())
    assert fake.events == [
        'STOWING: retracting arm into the navigation envelope', 'stow',
        'NAVIGATING: computing a right-side approach with Nav2', 'navigate',
        'stopped',
        'COMPLETE: base reached approach_target without arm motion']
    assert not fake.running


def test_rclpy_logging_never_uses_unsupported_printf_arguments():
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        function = call.func
        if (isinstance(function, ast.Attribute) and
                function.attr in {'debug', 'info', 'warning', 'warn', 'error', 'fatal'}):
            assert len(call.args) <= 1, (
                f'rclpy logger {function.attr} at line {call.lineno} received '
                f'{len(call.args)} positional arguments')


def test_real_mode_never_publishes_simulated_safety_state():
    fake = SimpleNamespace(simulation_mode=False, odom_tf_configured=False)
    mission.AutonomousMission._publish_sim_safety_state(fake)
    mission.AutonomousMission._configure_odom_tf_authority(fake)
    assert fake.odom_tf_configured


def test_real_navigation_uses_reachable_approach_tolerances():
    class Client:
        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.request = request
            return SimpleNamespace(add_done_callback=lambda _callback: None)

    client = Client()
    values = {
        'navigation_xy_goal_tolerance': 0.20,
        'navigation_yaw_goal_tolerance': 0.15,
    }
    fake = SimpleNamespace(
        navigation_tolerances_configured=False,
        navigation_parameters=client,
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        _navigation_tolerances_configured=lambda _future: None)

    mission.AutonomousMission._configure_navigation_tolerances(fake)

    configured = {
        parameter.name: parameter.value.double_value
        for parameter in client.request.parameters
        if parameter.name.startswith('general_goal_checker.')
    }
    assert configured == {
        'general_goal_checker.xy_goal_tolerance': 0.20,
        'general_goal_checker.yaw_goal_tolerance': 0.15,
    }


def test_real_trajectory_switches_mode_and_starts_at_measured_state():
    class Future:
        def __init__(self, value):
            self.value = value

        def done(self):
            return True

        def result(self):
            return self.value

    class Service:
        def __init__(self):
            self.calls = 0

        def wait_for_service(self, timeout_sec):
            return timeout_sec == 10.0

        def call_async(self, _request):
            self.calls += 1
            fake.joints_received = time.monotonic()
            return Future(SimpleNamespace(success=True, message=''))

    class Action:
        def wait_for_server(self, timeout_sec):
            return timeout_sec == 10.0

        def send_goal_async(self, goal):
            self.goal = goal
            result = SimpleNamespace(status=mission.GoalStatus.STATUS_SUCCEEDED)
            handle = SimpleNamespace(
                accepted=True,
                get_result_async=lambda: Future(result))
            return Future(handle)

    mode = Service()
    action = Action()
    fake = SimpleNamespace(
        simulation_mode=False, trajectory_mode=mode, trajectory=action,
        joints_received=time.monotonic(),
        _joint_positions=lambda: {
            'joint_head_pan': 0.06, 'joint_head_tilt': 0.04},
        _wait_future=mission.AutonomousMission._wait_future,
        _wait_for=lambda predicate, _timeout, error: (
            None if predicate() else (_ for _ in ()).throw(RuntimeError(error))),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None))

    mission.AutonomousMission._send_trajectory(
        fake, ['joint_head_pan', 'joint_head_tilt'], [-1.2, -0.4], 2.0,
        'camera alignment')

    assert mode.calls == 1
    assert len(action.goal.trajectory.points) == 2
    assert list(action.goal.trajectory.points[0].positions) == [0.06, 0.04]
    assert list(action.goal.trajectory.points[1].positions) == [-1.2, -0.4]
    assert action.goal.trajectory.points[1].time_from_start.sec == 2


def _lift_fake(height, current, simulation_mode=False, fail=False):
    sent = []

    def send(names, positions, duration, description, control_mode):
        if fail:
            raise RuntimeError(f'{control_mode} refused')
        sent.append((names, positions, control_mode))

    warnings = []
    fake = SimpleNamespace(
        simulation_mode=simulation_mode, joints=object(),
        joints_received=time.monotonic(),
        get_parameter=lambda name: SimpleNamespace(value={
            'navigation_lift_height': height, 'stow_tolerance': 0.02}[name]),
        _wait_for=lambda predicate, _timeout, error: None,
        _joint_positions=lambda: {'joint_lift': current},
        _send_trajectory=send,
        _wait_joints_reached=lambda *args: None,
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None, warning=warnings.append))
    return fake, sent, warnings


def test_lift_is_raised_after_factory_stow():
    fake, sent, _ = _lift_fake(0.30, 0.20)
    mission.AutonomousMission._raise_lift_after_stow(fake)
    assert sent == [(['joint_lift'], [0.30], 'navigation')]


def test_lift_raise_is_optional_and_never_runs_in_simulation():
    for fake, sent, _ in (_lift_fake(0.0, 0.20),
                          _lift_fake(0.30, 0.20, simulation_mode=True),
                          _lift_fake(0.30, 0.30)):
        mission.AutonomousMission._raise_lift_after_stow(fake)
        assert sent == []


def test_lift_raise_failure_does_not_abort_mission():
    fake, sent, warnings = _lift_fake(0.30, 0.20, fail=True)
    mission.AutonomousMission._raise_lift_after_stow(fake)
    assert sent == []
    assert len(warnings) == 1 and 'could not raise the lift' in warnings[0]


def test_navigation_wrist_pitch_is_configurable():
    sent = []
    fake = SimpleNamespace(
        simulation_mode=False,
        get_parameter=lambda name: SimpleNamespace(value={
            'navigation_wrist_pitch': -0.20,
            'stow_tolerance': 0.02,
        }[name]),
        _joint_positions=lambda: {'joint_wrist_pitch': -0.63},
        _send_trajectory=lambda names, positions, duration, description,
                                control_mode: sent.append(
                                    (names, positions, control_mode)),
        _wait_joints_reached=lambda *args: None,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None))

    mission.AutonomousMission._orient_wrist_after_stow(fake)

    assert sent == [(['joint_wrist_pitch'], [-0.20], 'navigation')]



class _Future:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value


def test_head_navigation_mode_sends_single_position_goal():
    class Service:
        def __init__(self):
            self.calls = 0

        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, _request):
            self.calls += 1
            return _Future(SimpleNamespace(success=True, message=''))

    class Action:
        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal):
            self.goal = goal
            result = SimpleNamespace(status=mission.GoalStatus.STATUS_SUCCEEDED)
            return _Future(SimpleNamespace(
                accepted=True, get_result_async=lambda: _Future(result)))

    navigation, trajectory_mode, action = Service(), Service(), Action()
    fake = SimpleNamespace(
        simulation_mode=False, navigation_mode=navigation,
        trajectory_mode=trajectory_mode, trajectory=action,
        _wait_future=mission.AutonomousMission._wait_future,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None))
    fake._switch_driver_mode = (
        lambda client, name: mission.AutonomousMission._switch_driver_mode(
            fake, client, name))

    mission.AutonomousMission._send_trajectory(
        fake, ['joint_head_pan', 'joint_head_tilt'], [-1.5, -0.3], 3.0,
        'camera alignment', control_mode='navigation')

    assert navigation.calls == 1
    assert trajectory_mode.calls == 0
    assert len(action.goal.trajectory.points) == 1
    assert list(action.goal.trajectory.points[0].positions) == [-1.5, -0.3]


def test_move_head_falls_back_when_head_does_not_move():
    attempts = []

    def send(names, positions, duration, description, control_mode):
        attempts.append(control_mode)

    def reached(names, positions, tolerance, timeout):
        if attempts[-1] != 'position':
            raise RuntimeError('head did not reach the commanded pose')

    fake = SimpleNamespace(
        simulation_mode=False,
        get_parameter=lambda name: SimpleNamespace(value={
            'head_motion_duration': 3.0, 'head_reach_tolerance': 0.1,
            'head_control_modes': ['navigation', 'position', 'trajectory'],
        }[name]),
        _joint_positions=lambda: {'joint_head_pan': 0.0, 'joint_head_tilt': 0.0},
        _send_trajectory=send, _wait_joints_reached=reached,
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None, warning=lambda _message: None))

    mission.AutonomousMission._move_head(fake, -1.5, -0.3)
    assert attempts == ['navigation', 'position']


def test_move_head_reports_every_failed_mode():
    def send(names, positions, duration, description, control_mode):
        raise RuntimeError(f'{control_mode} refused')

    fake = SimpleNamespace(
        simulation_mode=False,
        get_parameter=lambda name: SimpleNamespace(value={
            'head_motion_duration': 3.0, 'head_reach_tolerance': 0.1,
            'head_control_modes': ['navigation', 'trajectory'],
        }[name]),
        _joint_positions=lambda: {},
        _send_trajectory=send,
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None, warning=lambda _message: None))

    try:
        mission.AutonomousMission._move_head(fake, -1.5, -0.3)
    except RuntimeError as exc:
        assert 'navigation: navigation refused' in str(exc)
        assert 'trajectory: trajectory refused' in str(exc)
    else:
        raise AssertionError('expected a RuntimeError')


def test_real_marker_does_not_require_tcp_transform():
    rotation = mission.Pose().orientation
    rotation.w = 1.0
    base = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        rotation=rotation))

    class Tf:
        def lookup_transform(self, _parent, child, _time):
            if child == 'link_grasp_center':
                raise mission.TransformException()
            return base

    class MarkerServer:
        def insert(self, marker, feedback_callback):
            self.marker = marker
            self.callback = feedback_callback

        def applyChanges(self):
            pass

    fake = SimpleNamespace(
        gazebo_target_model='', target=None, tf=Tf(), global_frame='odom',
        base_frame='base_link', marker_server=MarkerServer(),
        menu=SimpleNamespace(apply=lambda *_args: None),
        _marker_feedback=lambda _feedback: None, emitted=[],
        emit=lambda value: fake.emitted.append(value),
        get_parameter=lambda _name: SimpleNamespace(value=False))
    mission.AutonomousMission._create_marker_when_ready(fake)
    assert fake.target is not None
    assert fake.marker_server.marker.name == 'grasp_target'
    assert math.isclose(fake.target.pose.position.z, 0.75)
    assert fake.emitted == [
        'READY: move grasp_target, then right-click it to run the mission']


def test_nav2_uses_stowed_footprint_and_local_depth_only():
    def costmap():
        return {'ros__parameters': {
            'plugins': ['obstacle_layer', 'voxel_layer', 'inflation_layer'],
            'inflation_layer': {},
            'voxel_layer': {'observation_sources': 'depth_mark depth_clear'},
        }}
    data = {
        'controller_server': {'ros__parameters': {
            'FollowPath': {'min_vel_x': -0.05}}},
        'local_costmap': {'local_costmap': costmap()},
        'global_costmap': {'global_costmap': costmap()},
    }
    bringup.configure_nav2(data, '/camera/depth/color/points')
    local = data['local_costmap']['local_costmap']['ros__parameters']
    global_ = data['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint'] == bringup.NAVIGATION_FOOTPRINT
    assert global_['footprint'] == bringup.NAVIGATION_FOOTPRINT
    assert local['footprint_padding'] == global_['footprint_padding'] == 0.01
    assert local['inflation_layer']['inflation_radius'] == 0.4
    assert data['controller_server']['ros__parameters']['FollowPath'][
        'min_vel_x'] == 0.0
    assert 'voxel_layer' in local['plugins']
    assert 'depth' in local['voxel_layer']['observation_sources'].split()
    assert local['voxel_layer']['depth']['data_type'] == 'PointCloud2'
    assert local['voxel_layer']['depth']['topic'] == '/camera/depth/color/points'
    assert 'voxel_layer' not in global_['plugins']
    assert 'voxel_layer' not in global_


def test_nav2_drops_costmap_layers_whose_plugin_is_not_installed(monkeypatch):
    params = {'plugins': ['obstacle_layer', 'tracked', 'inflation_layer'],
              'obstacle_layer': {'plugin': 'nav2_costmap_2d::ObstacleLayer'},
              'tracked': {'plugin': 'stretch_palettes_costmap/TrackedObstacleLayer'},
              'inflation_layer': {}}
    data = {'controller_server': {'ros__parameters': {'FollowPath': {}}},
            'local_costmap': {'local_costmap': {'ros__parameters': params}},
            'global_costmap': {'global_costmap': {'ros__parameters': {
                'plugins': ['inflation_layer'], 'inflation_layer': {}}}}}
    monkeypatch.setattr(bringup, 'plugin_installed',
                        lambda p: not p.startswith('stretch_palettes_costmap'))
    bringup.configure_nav2(data)
    assert 'tracked' not in params['plugins']
    assert params['plugins'][0] == 'obstacle_layer' and 'inflation_layer' in params['plugins']


def test_gazebo_target_is_movable_gravity_free_and_camera_masked():
    sdf = ET.parse(
        path.parents[1] / 'worlds' / 'approach_target.sdf').getroot()
    model = sdf.find('model')
    link = model.find('link')
    assert model.findtext('static') == 'false'
    assert link.findtext('gravity') == 'false'
    assert link.findtext('kinematic') == 'true'
    assert link.find('visual').findtext('visibility_flags') == '2147483648'


def test_rviz_marker_move_queues_automatic_mission():
    fake = SimpleNamespace(
        target=mission.PoseStamped(), lock=threading.Lock(),
        object_last_xy=(0.0, 0.0), object_last_started=(0.0, 0.0),
        object_last_moved=0.0, object_pending=False,
        get_parameter=lambda name: SimpleNamespace(value=0.10))
    feedback = SimpleNamespace(
        event_type=1, POSE_UPDATE=1,
        header=SimpleNamespace(frame_id='odom'), pose=mission.Pose())
    feedback.pose.position.x = 0.25
    mission.AutonomousMission._marker_feedback(fake, feedback)
    assert fake.object_pending
    assert fake.object_last_xy == (0.25, 0.0)
    assert fake.object_last_moved > 0.0


def test_rviz_shows_collision_planned_trajectory():
    config = (path.parents[1] / 'rviz' / 'bringup.rviz').read_text(
        encoding='utf-8')
    assert 'Value: /guarded_executor/planned_path' in config
    assert 'executable_tcp_trajectory: true' in config


def test_real_bringup_exposes_official_runtime_controls():
    arguments = {
        entity.name: entity.default_value
        for entity in bringup.generate_launch_description().entities
        if hasattr(entity, 'name') and hasattr(entity, 'default_value')
    }
    assert arguments['start_robot_driver'][0].text == 'true'
    assert arguments['start_robot_camera'][0].text == 'true'
    assert arguments['start_robot_lidar'][0].text == 'true'
    assert arguments['start_nav2'][0].text == 'true'
    assert arguments['robot_camera_resolution'][0].text == 'low'


def test_rviz_exposes_nav2_initial_pose_and_goal_tools():
    config = (path.parents[1] / 'rviz' / 'bringup.rviz').read_text(
        encoding='utf-8')
    assert 'Class: nav2_rviz_plugins/Navigation 2' in config
    assert 'Class: rviz_default_plugins/SetInitialPose' in config
    assert 'Class: nav2_rviz_plugins/GoalTool' in config
