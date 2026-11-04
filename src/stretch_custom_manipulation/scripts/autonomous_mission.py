#!/usr/bin/env python3
"""Interactive target -> Nav2 approach -> head aim -> OctoMap -> IK execution."""

import copy
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
try:
    from gazebo_msgs.msg import ModelStates
except ImportError:
    ModelStates = None
from geometry_msgs.msg import PolygonStamped, Pose, PoseStamped, Twist
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


def approach_pose(target_x, target_y, base_x, base_y, distance):
    """Place the base on the arrival ray with its right side facing target."""
    dx, dy = target_x - base_x, target_y - base_y
    norm = math.hypot(dx, dy)
    if norm < 1.0e-6:
        dx, dy, norm = 1.0, 0.0, 1.0
    ux, uy = dx / norm, dy / norm
    return target_x - distance * ux, target_y - distance * uy, math.atan2(uy, ux) + math.pi / 2.0


def head_angles(target_x, target_y, target_z, camera_x, camera_y, camera_z,
                pan_offset=0.0, tilt_offset=0.0):
    dx, dy, dz = target_x - camera_x, target_y - camera_y, target_z - camera_z
    return (math.atan2(dy, dx) + pan_offset,
            math.atan2(dz, math.hypot(dx, dy)) + tilt_offset)


def closest_costmap_obstacle(message, base):
    origin = message.info.origin.position
    resolution = message.info.resolution
    width = message.info.width
    distances = (
        math.hypot(origin.x + (index % width + 0.5) * resolution - base.x,
                   origin.y + (index // width + 0.5) * resolution - base.y)
        for index, value in enumerate(message.data) if value >= 99)
    return min(distances, default=math.inf)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def near_goal_handoff(distance, yaw_error, max_distance, max_yaw_error):
    return (math.isfinite(distance) and math.isfinite(yaw_error) and
            distance <= max_distance and yaw_error <= max_yaw_error)


def gazebo_pose_in_odom(object_world, robot_world, robot_odom):
    """Convert a Gazebo world pose using the observed world->odom relation."""
    world_yaw = yaw_from_quaternion(robot_world.orientation)
    odom_yaw = yaw_from_quaternion(robot_odom.orientation)
    dx = object_world.position.x - robot_world.position.x
    dy = object_world.position.y - robot_world.position.y
    base_x = math.cos(world_yaw) * dx + math.sin(world_yaw) * dy
    base_y = -math.sin(world_yaw) * dx + math.cos(world_yaw) * dy
    result = Pose()
    result.position.x = (robot_odom.position.x +
                         math.cos(odom_yaw) * base_x -
                         math.sin(odom_yaw) * base_y)
    result.position.y = (robot_odom.position.y +
                         math.sin(odom_yaw) * base_x +
                         math.cos(odom_yaw) * base_y)
    result.position.z = (robot_odom.position.z + object_world.position.z -
                         robot_world.position.z)
    yaw = (odom_yaw + yaw_from_quaternion(object_world.orientation) -
           world_yaw)
    result.orientation.z = math.sin(yaw / 2.0)
    result.orientation.w = math.cos(yaw / 2.0)
    return result


class AutonomousMission(Node):
    def __init__(self):
        super().__init__('autonomous_manipulation')
        self.declare_parameter('global_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('approach_distance', 0.55)
        self.declare_parameter('head_pan_offset', 0.0)
        self.declare_parameter('head_tilt_offset', 0.0)
        self.declare_parameter('camera_alignment_tolerance', 0.45)
        self.declare_parameter('navigation_timeout', 300.0)
        self.declare_parameter('navigation_stall_timeout', 30.0)
        self.declare_parameter('navigation_progress_distance', 0.02)
        self.declare_parameter('navigation_xy_goal_tolerance', 0.20)
        self.declare_parameter('navigation_yaw_goal_tolerance', 0.15)
        self.declare_parameter('navigation_handoff_distance', 0.22)
        self.declare_parameter('navigation_handoff_yaw_tolerance', 0.35)
        self.declare_parameter('manipulation_timeout', 90.0)
        self.declare_parameter('execute_manipulation', True)
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter(
            'trajectory_action', '/stretch_controller/follow_joint_trajectory')
        # Driver modes tried (in order) to move the head on the real robot.
        # 'navigation' and 'position' send a plain position goal (the driver
        # applies its head calibration offsets); 'trajectory' sends a
        # waypoint trajectory and is kept only as a last resort.
        self.declare_parameter(
            'head_control_modes', ['navigation', 'position', 'trajectory'])
        self.declare_parameter('head_motion_duration', 3.0)
        self.declare_parameter('head_reach_tolerance', 0.55)
        self.declare_parameter('stow_service_required', True)
        self.declare_parameter('stow_arm_extension', 0.01)
        self.declare_parameter('stow_lift', 0.22)
        # Real robot: /stow_the_robot leaves the lift near 0.2 m, where the
        # stowed gripper sits at LiDAR height and shows up as an obstacle.
        # After the stow service, raise the lift to this height (metres).
        # Set to 0.0 or a negative value to keep the factory stow height.
        self.declare_parameter('navigation_lift_height', 0.45)
        self.declare_parameter('navigation_wrist_pitch', -0.20)
        self.declare_parameter('stow_wrist_yaw', 3.30)
        self.declare_parameter('stow_wrist_pitch', 0.0)
        self.declare_parameter('stow_wrist_roll', 0.0)
        self.declare_parameter('stow_tolerance', 0.02)
        self.declare_parameter('gazebo_target_model', '')
        self.declare_parameter('gazebo_robot_model', 'stretch')
        self.declare_parameter('auto_run_on_target_move', False)
        self.declare_parameter('navigate_only', False)
        self.declare_parameter('target_settle_time', 1.0)
        self.declare_parameter('target_move_threshold', 0.10)

        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.gazebo_target_model = self.get_parameter('gazebo_target_model').value
        self.gazebo_robot_model = self.get_parameter('gazebo_robot_model').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.target = None
        self.odom = None
        self.odom_received = 0.0
        self.joints = None
        self.joints_received = 0.0
        self.ik_ok = False
        self.ik_received = 0.0
        self.scene_status = ''
        self.scene_received = 0.0
        self.executor_status = ''
        self.executor_received = 0.0
        self.closest_lidar = math.inf
        self.closest_costmap = math.inf
        self.costmap_received = 0.0
        self.navigation_footprint = 'unavailable'
        self.last_nav_debug = 0.0
        self.object_last_xy = None
        self.object_last_moved = 0.0
        self.object_last_started = None
        self.object_pending = False
        self.object_announced = False
        self.object_wait_reason = ''
        self.object_wait_logged = 0.0
        self.running = False
        self.lock = threading.Lock()

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.trajectory = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter('trajectory_action').value)
        self.stow = self.create_client(Trigger, '/stow_the_robot')
        self.navigation_mode = self.create_client(
            Trigger, '/switch_to_navigation_mode')
        self.position_mode = self.create_client(
            Trigger, '/switch_to_position_mode')
        self.trajectory_mode = self.create_client(
            Trigger, '/switch_to_trajectory_mode')
        self.capture_map = self.create_client(
            Trigger, '/octomap_builder/capture_once')
        self.execute = self.create_client(Trigger, '/guarded_executor/move_to_preview')
        self.create_service(Trigger, '~/run', self._run_service)
        self.target_pub = self.create_publisher(PoseStamped, '/ik_preview/goal_pose', 10)
        self.zero_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            String, '/autonomous_manipulation/status', latched)
        self.homed_pub = None
        self.runstop_pub = None
        if self.simulation_mode:
            self.homed_pub = self.create_publisher(Bool, '/is_homed', 10)
            self.runstop_pub = self.create_publisher(Bool, '/is_runstopped', 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        if self.gazebo_target_model:
            if ModelStates is None:
                raise RuntimeError('gazebo_msgs is required for a Gazebo target')
            self.create_subscription(
                ModelStates, '/gazebo/model_states', self._on_model_states, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self._on_costmap, 10)
        self.create_subscription(
            PolygonStamped, '/local_costmap/published_footprint',
            self._on_footprint, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(DiagnosticArray, '/ik_preview/status', self._on_ik, 10)
        self.create_subscription(String, '/perception/scene_status', self._on_scene, 10)
        self.create_subscription(String, '/guarded_executor/status', self._on_executor, 10)
        if self.simulation_mode:
            self.create_timer(
                0.1, self._publish_sim_safety_state,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.base_parameters = self.create_client(
            SetParameters, '/base_controller/set_parameters')
        self.navigation_parameters = self.create_client(
            SetParameters, '/controller_server/set_parameters')
        self.odom_tf_configured = not self.simulation_mode
        self.navigation_tolerances_configured = False
        self.create_timer(0.5, self._configure_odom_tf_authority)
        self.create_timer(0.5, self._configure_navigation_tolerances)

        self.marker_server = InteractiveMarkerServer(self, '/manipulation_target')
        self.menu = MenuHandler()
        self.menu.insert('Run navigation + grasp mission', callback=self._start_from_menu)
        self.create_timer(0.5, self._create_marker_when_ready)
        self.create_timer(0.25, self._auto_start_target)

    def emit(self, value):
        self.status_pub.publish(String(data=value))
        self.get_logger().info(value)

    def _publish_sim_safety_state(self):
        if not self.simulation_mode:
            return
        self.homed_pub.publish(Bool(data=True))
        self.runstop_pub.publish(Bool(data=False))

    def _configure_odom_tf_authority(self):
        if not self.simulation_mode:
            self.odom_tf_configured = True
            return
        if self.odom_tf_configured or not self.base_parameters.service_is_ready():
            return
        self.odom_tf_configured = True
        request = SetParameters.Request()
        request.parameters = [
            Parameter('enable_odom_tf', Parameter.Type.BOOL, False).to_parameter_msg()
        ]
        future = self.base_parameters.call_async(request)
        future.add_done_callback(self._odom_tf_configured)

    def _odom_tf_configured(self, future):
        try:
            results = future.result().results
            if not results or not all(result.successful for result in results):
                raise RuntimeError('base controller rejected enable_odom_tf=false')
            self.get_logger().info(
                'TF authority fixed: stretch_sim_interface_bridges exclusively owns odom -> base_link')
        except Exception as exc:
            self.odom_tf_configured = False
            self.get_logger().error('Could not configure unique odometry TF: ' + str(exc))

    def _configure_navigation_tolerances(self):
        if (self.navigation_tolerances_configured or
                not self.navigation_parameters.service_is_ready()):
            return
        self.navigation_tolerances_configured = True
        xy_tolerance = float(
            self.get_parameter('navigation_xy_goal_tolerance').value)
        yaw_tolerance = float(
            self.get_parameter('navigation_yaw_goal_tolerance').value)
        request = SetParameters.Request()
        request.parameters = [
            Parameter('general_goal_checker.xy_goal_tolerance',
                      Parameter.Type.DOUBLE, xy_tolerance).to_parameter_msg(),
            Parameter('general_goal_checker.yaw_goal_tolerance',
                      Parameter.Type.DOUBLE, yaw_tolerance).to_parameter_msg(),
            Parameter('progress_checker.movement_time_allowance',
                      Parameter.Type.DOUBLE, 60.0).to_parameter_msg(),
            Parameter('FollowPath.min_speed_theta',
                      Parameter.Type.DOUBLE, 0.15).to_parameter_msg(),
        ]
        future = self.navigation_parameters.call_async(request)
        future.add_done_callback(self._navigation_tolerances_configured)

    def _navigation_tolerances_configured(self, future):
        try:
            results = future.result().results
            if not results or not all(result.successful for result in results):
                raise RuntimeError('Nav2 rejected manipulation goal tolerances')
            xy_tolerance = float(
                self.get_parameter('navigation_xy_goal_tolerance').value)
            yaw_tolerance = float(
                self.get_parameter('navigation_yaw_goal_tolerance').value)
            self.get_logger().info(
                f'Nav2 manipulation approach tolerances configured: '
                f'xy={xy_tolerance:.2f} m yaw={yaw_tolerance:.2f} rad')
        except Exception as exc:
            self.navigation_tolerances_configured = False
            self.get_logger().error('Could not configure Nav2 approach: ' + str(exc))

    def _on_odom(self, message):
        self.odom = message
        self.odom_received = time.monotonic()

    def _on_scan(self, message):
        valid = [value for value in message.ranges
                 if math.isfinite(value) and
                 message.range_min <= value <= message.range_max]
        self.closest_lidar = min(valid, default=math.inf)

    def _on_costmap(self, message):
        self.costmap_received = time.monotonic()
        if self.odom is None:
            return
        base = self.odom.pose.pose.position
        self.closest_costmap = closest_costmap_obstacle(message, base)

    def _on_footprint(self, message):
        self.navigation_footprint = '[' + ','.join(
            f'[{point.x:.3f},{point.y:.3f}]'
            for point in message.polygon.points) + ']'

    def _on_model_states(self, message):
        if self.odom is None:
            return
        try:
            object_index = message.name.index(self.gazebo_target_model)
            robot_index = message.name.index(self.gazebo_robot_model)
        except ValueError:
            return
        target = PoseStamped()
        target.header.frame_id = self.global_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose = gazebo_pose_in_odom(
            message.pose[object_index], message.pose[robot_index],
            self.odom.pose.pose)
        xy = (target.pose.position.x, target.pose.position.y)
        now = time.monotonic()
        with self.lock:
            if (self.object_last_xy is None or
                    math.dist(xy, self.object_last_xy) > 0.01):
                self.object_last_moved = now
                self.object_last_xy = xy
            self.target = target
            if (self.object_last_started is None or
                    math.dist(xy, self.object_last_started) >= float(
                        self.get_parameter('target_move_threshold').value)):
                self.object_pending = True
        if not self.object_announced:
            self.object_announced = True
            self.emit(
                f'OBJECT_TARGET_READY: move {self.gazebo_target_model} in Gazebo')

    def _auto_start_target(self):
        if not self.get_parameter('auto_run_on_target_move').value:
            return
        now = time.monotonic()
        with self.lock:
            ready = (self.object_pending and not self.running and
                     self.target is not None and
                     now - self.object_last_moved >= float(
                         self.get_parameter('target_settle_time').value))
        blockers = []
        if not ready:
            blockers.append('target_not_settled')
        if self.joints is None or now - self.joints_received >= 0.5:
            blockers.append('joint_states')
        if self.odom is None or now - self.odom_received >= 0.5:
            blockers.append('odom')
        if not self.odom_tf_configured:
            blockers.append('odom_tf')
        if not self.navigation_tolerances_configured:
            blockers.append('nav2_parameters')
        if not self.nav.server_is_ready():
            blockers.append('navigate_to_pose')
        if blockers:
            reason = ','.join(blockers)
            if (self.object_pending and
                    (reason != self.object_wait_reason or
                     now - self.object_wait_logged >= 5.0)):
                self.get_logger().info(
                    f'[TARGET] waiting_for=[{reason}]')
                self.object_wait_reason = reason
                self.object_wait_logged = now
            return
        self.get_logger().info('[TARGET] settled; starting mission')
        self._start()

    def _on_joints(self, message):
        self.joints = message
        self.joints_received = time.monotonic()

    def _on_ik(self, message):
        self.ik_ok = bool(message.status and message.status[0].message == 'success')
        self.ik_received = time.monotonic()

    def _on_scene(self, message):
        self.scene_status = message.data
        self.scene_received = time.monotonic()

    def _on_executor(self, message):
        self.executor_status = message.data
        self.executor_received = time.monotonic()

    def _create_marker_when_ready(self):
        if self.gazebo_target_model or self.target is not None:
            return
        try:
            base = self.tf.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return
        try:
            grasp = self.tf.lookup_transform(
                self.global_frame, 'link_grasp_center', rclpy.time.Time())
        except TransformException:
            grasp = None
        marker = InteractiveMarker()
        marker.header.frame_id = self.global_frame
        marker.name = 'grasp_target'
        marker.description = 'Global grasp target (right-click to run)'
        marker.scale = 0.30
        yaw = yaw_from_quaternion(base.transform.rotation)
        marker.pose.position.x = (
            base.transform.translation.x + math.cos(yaw) + 0.65 * math.sin(yaw))
        marker.pose.position.y = (
            base.transform.translation.y + math.sin(yaw) - 0.65 * math.cos(yaw))
        marker.pose.position.z = max(
            0.35, grasp.transform.translation.z if grasp else 0.75)
        marker.pose.orientation = (
            grasp.transform.rotation if grasp else base.transform.rotation)
        visual = InteractiveMarkerControl()
        visual.always_visible = True
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06
        sphere.color.r, sphere.color.g, sphere.color.a = 1.0, 0.55, 0.9
        sphere.pose.orientation.w = 1.0
        visual.markers.append(sphere)
        marker.controls.append(visual)
        for axis in range(3):
            for rotate in (False, True):
                control = InteractiveMarkerControl()
                control.name = ('rotate_' if rotate else 'move_') + str(axis)
                control.orientation.w = 1.0
                if axis == 0:
                    control.orientation.x = 1.0
                elif axis == 1:
                    control.orientation.z = 1.0
                else:
                    control.orientation.y = 1.0
                control.orientation_mode = InteractiveMarkerControl.FIXED
                control.interaction_mode = (InteractiveMarkerControl.ROTATE_AXIS if rotate
                                            else InteractiveMarkerControl.MOVE_AXIS)
                marker.controls.append(control)
        self.target = PoseStamped()
        self.target.header.frame_id = self.global_frame
        self.target.pose = marker.pose
        self.object_last_xy = (marker.pose.position.x, marker.pose.position.y)
        self.object_last_started = self.object_last_xy
        self.marker_server.insert(marker, feedback_callback=self._marker_feedback)

        self.menu.apply(self.marker_server, marker.name)
        self.marker_server.applyChanges()
        if self.get_parameter('auto_run_on_target_move').value:
            self.emit('READY: move grasp_target and release it to run automatically')
        else:
            self.emit('READY: move grasp_target, then right-click it to run the mission')

    def _marker_feedback(self, feedback):
        if feedback.event_type == feedback.POSE_UPDATE:
            xy = (feedback.pose.position.x, feedback.pose.position.y)
            now = time.monotonic()
            with self.lock:
                self.target.header.frame_id = feedback.header.frame_id
                self.target.pose = feedback.pose
                if math.dist(xy, self.object_last_xy) > 0.01:
                    self.object_last_xy = xy
                    self.object_last_moved = now
                if math.dist(xy, self.object_last_started) >= float(
                        self.get_parameter('target_move_threshold').value):
                    self.object_pending = True

    def _start_from_menu(self, _feedback):
        self._start()

    def _run_service(self, _request, response):
        response.success, response.message = self._start()
        return response

    def _start(self):
        with self.lock:
            if self.running:
                self.emit('REJECTED: a mission is already running')
                return False, 'a mission is already running'
            if self.target is None:
                return False, 'target is not ready'
            self.running = True
            target = copy.deepcopy(self.target)
            self.object_last_started = (
                target.pose.position.x, target.pose.position.y)
            self.object_pending = False
        threading.Thread(target=self._run, args=(target,), daemon=True).start()
        return True, 'mission started'

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('ROS request timed out')
        return future.result()

    def _wait_for(self, predicate, timeout, error):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        raise RuntimeError(error)

    def _base_pose(self, target):
        distance = float(self.get_parameter('approach_distance').value)
        if self.odom is None:
            raise RuntimeError('odometry unavailable for approach pose')
        base = self.odom.pose.pose.position
        x, y, yaw = approach_pose(
            target.pose.position.x, target.pose.position.y,
            base.x, base.y, distance)
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _navigate(self, target):
        if not self.simulation_mode:
            if not self.navigation_mode.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('/switch_to_navigation_mode is unavailable')
            response = self._wait_future(
                self.navigation_mode.call_async(Trigger.Request()), 10.0)
            if response is None or not response.success:
                raise RuntimeError(
                    'driver rejected navigation mode: ' +
                    (response.message if response else 'no response'))
        self._wait_for(
            lambda: self.odom_tf_configured and self.navigation_tolerances_configured,
            15.0, 'simulation TF/Nav2 configuration did not become ready')
        if not self.nav.wait_for_server(timeout_sec=60.0):
            raise RuntimeError('Nav2 NavigateToPose is unavailable')
        goal = NavigateToPose.Goal()
        goal.pose = self._base_pose(target)
        progress = {
            'best_distance': math.inf,
            'distance': math.inf,
            'last_progress': time.monotonic(),
        }

        def feedback(message):
            distance = message.feedback.distance_remaining
            progress['distance'] = distance
            threshold = float(
                self.get_parameter('navigation_progress_distance').value)
            if (math.isfinite(distance) and
                    distance < progress['best_distance'] - threshold):
                progress['best_distance'] = distance
                progress['last_progress'] = time.monotonic()
            now = time.monotonic()
            if now - self.last_nav_debug >= 5.0:
                age = (now - self.costmap_received
                       if self.costmap_received else math.inf)
                self.get_logger().info(
                    f'[NAV_DEBUG] footprint={self.navigation_footprint} '
                    f'closest_obstacle='
                    f'{min(self.closest_lidar, self.closest_costmap):.3f} '
                    f'closest_lidar={self.closest_lidar:.3f} '
                    f'closest_costmap={self.closest_costmap:.3f} '
                    f'local_costmap_age={age:.3f} '
                    'active_observation_sources=[lidar:/scan,'
                    'rgbd:/camera/depth/color/points] '
                    f'distance_remaining={distance:.3f}')
                self.last_nav_debug = now

        handle = self._wait_future(
            self.nav.send_goal_async(goal, feedback_callback=feedback), 10.0)
        if handle is None or not handle.accepted:
            raise RuntimeError('Nav2 rejected the approach pose')
        result_future = handle.get_result_async()
        deadline = (time.monotonic() +
                    float(self.get_parameter('navigation_timeout').value))
        stall_timeout = float(
            self.get_parameter('navigation_stall_timeout').value)
        while not result_future.done():
            now = time.monotonic()
            stalled = now - progress['last_progress'] >= stall_timeout
            if stalled:
                yaw_error = math.inf
                if self.odom is not None:
                    yaw_error = abs(math.atan2(
                        math.sin(yaw_from_quaternion(goal.pose.orientation) -
                                 yaw_from_quaternion(
                                     self.odom.pose.pose.orientation)),
                        math.cos(yaw_from_quaternion(goal.pose.orientation) -
                                 yaw_from_quaternion(
                                     self.odom.pose.pose.orientation))))
                if near_goal_handoff(
                        progress['distance'], yaw_error,
                        float(self.get_parameter(
                            'navigation_handoff_distance').value),
                        float(self.get_parameter(
                            'navigation_handoff_yaw_tolerance').value)):
                    self._wait_future(handle.cancel_goal_async(), 5.0)
                    self.get_logger().warning(
                        f'[NAV] accepting near-goal handoff after safe stop: '
                        f'distance={progress["distance"]:.3f} m '
                        f'yaw_error={yaw_error:.3f} rad')
                    return
            if now >= deadline or stalled:
                self._wait_future(handle.cancel_goal_async(), 5.0)
                reason = ('timed out' if now >= deadline else
                          f'stalled for {stall_timeout:.0f}s')
                raise RuntimeError(f'Nav2 {reason}; approach cancelled safely')
            time.sleep(0.05)
        result = result_future.result()
        if result is None:
            self._wait_future(handle.cancel_goal_async(), 5.0)
            raise RuntimeError('Nav2 returned no approach result')
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Nav2 failed with action status {result.status}')
        self.get_logger().info('[NAV] Goal succeeded')

    def _wait_stopped(self):
        self.zero_pub.publish(Twist())
        stable = 0
        def stopped():
            nonlocal stable
            if self.odom is None or time.monotonic() - self.odom_received > 0.5:
                stable = 0
                return False
            twist = self.odom.twist.twist
            if max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z)) > 0.02:
                stable = 0
                return False
            stable += 1
            return stable >= 4
        self._wait_for(stopped, 5.0, 'base did not become stationary')
        self.get_logger().info('[NAV] Base stationary')

    def _joint_positions(self):
        if self.joints is None or len(self.joints.name) != len(self.joints.position):
            return {}
        return dict(zip(self.joints.name, self.joints.position))

    def _switch_driver_mode(self, client, name):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'/switch_to_{name}_mode is unavailable')
        response = self._wait_future(client.call_async(Trigger.Request()), 10.0)
        if response is None or not response.success:
            raise RuntimeError(
                f'driver rejected {name} mode: ' +
                (response.message if response else 'no response'))
        self.get_logger().info(f'[MODE] {name} mode active')

    def _send_trajectory(self, names, positions, duration, description,
                         control_mode='trajectory'):
        """Send a FollowJointTrajectory goal to the Stretch driver.

        control_mode='trajectory' switches the driver to trajectory mode and
        sends a start-point + goal waypoint trajectory.  'navigation' and
        'position' switch to that driver mode and send a single position
        goal (time_from_start is ignored by the driver in those modes).
        """
        start_positions = None
        if not self.simulation_mode:
            if control_mode == 'trajectory':
                if not self.trajectory_mode.wait_for_service(timeout_sec=10.0):
                    raise RuntimeError('/switch_to_trajectory_mode is unavailable')
                switched_at = time.monotonic()
                response = self._wait_future(
                    self.trajectory_mode.call_async(Trigger.Request()), 10.0)
                if response is None or not response.success:
                    raise RuntimeError(
                        'driver rejected trajectory mode: ' +
                        (response.message if response else 'no response'))
                self._wait_for(
                    lambda: self.joints_received > switched_at and
                    time.monotonic() - self.joints_received < 0.5,
                    5.0, 'fresh joint state unavailable after switching to trajectory mode')
                current = self._joint_positions()
                missing = [name for name in names if name not in current]
                if missing:
                    raise RuntimeError(
                        f'joint state is missing commanded joints: {missing}')
                start_positions = [current[name] for name in names]
                self.get_logger().info(
                    f'[MODE] trajectory mode active for {description}')
            elif control_mode == 'navigation':
                self._switch_driver_mode(self.navigation_mode, 'navigation')
            elif control_mode == 'position':
                self._switch_driver_mode(self.position_mode, 'position')
            else:
                raise RuntimeError(f'unknown control mode {control_mode!r}')

        if not self.trajectory.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('joint trajectory controller is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        if start_positions is not None:
            start = JointTrajectoryPoint()
            start.positions = start_positions
            goal.trajectory.points.append(start)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1.0e9)
        goal.trajectory.points.append(point)
        handle = self._wait_future(
            self.trajectory.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            raise RuntimeError(description + ' trajectory was rejected')
        result_future = handle.get_result_async()
        try:
            result = self._wait_future(result_future, duration + 12.0)
        except RuntimeError:
            self._wait_future(handle.cancel_goal_async(), 5.0)
            raise RuntimeError(description + ' trajectory timed out')
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            detail = ''
            if result is not None and getattr(result, 'result', None) is not None:
                message = getattr(result.result, 'error_string', '')
                if message:
                    detail = ': ' + message
            raise RuntimeError(description + ' trajectory failed' + detail)

    def _arm_extension(self):
        positions = self._joint_positions()
        names = ['joint_arm_l0', 'joint_arm_l1',
                 'joint_arm_l2', 'joint_arm_l3']
        if not all(name in positions for name in names):
            return math.inf
        return sum(positions[name] for name in names)

    def _raise_lift_after_stow(self):
        """Lift the stowed gripper above the LiDAR scan plane (real robot)."""
        if self.simulation_mode:
            return
        target = float(self.get_parameter('navigation_lift_height').value)
        if target <= 0.0:
            return
        names = ['joint_lift']
        tolerance = float(self.get_parameter('stow_tolerance').value)
        self._wait_for(
            lambda: self.joints is not None and
            time.monotonic() - self.joints_received < 0.5,
            5.0, 'fresh joint state unavailable after stowing')
        current = self._joint_positions().get('joint_lift')
        if current is None:
            self.get_logger().warning(
                '[STOW] joint_lift missing from joint states; lift not raised')
            return
        if abs(current - target) <= tolerance:
            return
        # The lift moves at roughly 0.05 m/s when stowing; the duration is
        # only used to bound how long we wait for the driver to finish.
        duration = max(2.0, 1.2 * abs(current - target) / 0.05)
        failures = []
        for mode in ('navigation', 'position'):
            try:
                self.get_logger().info(
                    f'[STOW] raising lift {current:.3f} -> {target:.3f} '
                    f'via {mode} mode')
                self._send_trajectory(
                    names, [target], duration, 'lift raise',
                    control_mode=mode)
                self._wait_joints_reached(names, [target], 0.03, 10.0)
                self.get_logger().info(
                    f'[STOW] lift raised to {target:.3f} m for LiDAR clearance')
                return
            except RuntimeError as exc:
                failures.append(f'{mode}: {exc}')
        self.get_logger().warning(
            '[STOW] could not raise the lift after stowing; continuing with '
            'the factory stow height (' + '; '.join(failures) + ')')

    def _orient_wrist_after_stow(self):
        """Keep the retracted gripper above the planar LiDAR scan."""
        if self.simulation_mode:
            return
        target = float(self.get_parameter('navigation_wrist_pitch').value)
        tolerance = float(self.get_parameter('stow_tolerance').value)
        current = self._joint_positions().get('joint_wrist_pitch')
        if current is None:
            raise RuntimeError('joint_wrist_pitch missing after stowing')
        if abs(current - target) <= tolerance:
            return
        duration = max(2.0, 1.2 * abs(current - target) / 0.5)
        failures = []
        for mode in ('navigation', 'position'):
            try:
                self._send_trajectory(
                    ['joint_wrist_pitch'], [target], duration,
                    'navigation wrist pitch', control_mode=mode)
                self._wait_joints_reached(
                    ['joint_wrist_pitch'], [target], tolerance, 8.0)
                self.get_logger().info(
                    f'[STOW] wrist pitch raised to {target:.3f} rad for '
                    'LiDAR clearance')
                return
            except RuntimeError as exc:
                failures.append(f'{mode}: {exc}')
        raise RuntimeError(
            'could not orient wrist for navigation (' + '; '.join(failures) + ')')

    def _stow_for_navigation(self):
        self._wait_for(
            lambda: self.joints is not None and
            time.monotonic() - self.joints_received < 0.5,
            5.0, 'fresh joint state unavailable before stowing')
        extension = float(self.get_parameter('stow_arm_extension').value)
        tolerance = float(self.get_parameter('stow_tolerance').value)

        if self.get_parameter('stow_service_required').value:
            if not self.stow.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('/stow_the_robot service is unavailable')
            response = self._wait_future(
                self.stow.call_async(Trigger.Request()), 60.0)
            if response is None or not response.success:
                raise RuntimeError('robot stow failed: ' +
                                   (response.message if response else 'no response'))
            self._raise_lift_after_stow()
            self._orient_wrist_after_stow()
        else:
            positions = self._joint_positions()
            arm_names = ['joint_arm_l0', 'joint_arm_l1',
                         'joint_arm_l2', 'joint_arm_l3']
            segment = extension / 4.0
            max_arm_delta = max(
                abs(positions[name] - segment) for name in arm_names)
            if max_arm_delta > tolerance / 4.0:
                self._send_trajectory(
                    arm_names, [segment] * 4,
                    max(2.0, 1.2 * max_arm_delta / 0.0125),
                    'arm retraction')

            positions = self._joint_positions()
            fold_names = ['joint_lift', 'joint_wrist_yaw',
                          'joint_wrist_pitch', 'joint_wrist_roll']
            fold_target = [
                float(self.get_parameter('stow_lift').value),
                float(self.get_parameter('stow_wrist_yaw').value),
                float(self.get_parameter('stow_wrist_pitch').value),
                float(self.get_parameter('stow_wrist_roll').value),
            ]
            rates = [0.05, 0.375, 0.5, 0.5]
            max_fold_error = max(
                abs(positions[name] - target)
                for name, target in zip(fold_names, fold_target))
            duration = max(
                abs(positions[name] - target) / rate
                for name, target, rate in zip(fold_names, fold_target, rates))
            if max_fold_error > tolerance:
                self._send_trajectory(
                    fold_names, fold_target, max(2.0, 1.2 * duration),
                    'navigation stow')

        self._wait_for(
            lambda: self._arm_extension() <= extension + tolerance,
            5.0, 'arm did not reach the navigation stow envelope')
        self.get_logger().info('[STOW] Arm is inside the navigation envelope')

    def _wait_upper_body_stopped(self):
        stable = 0
        watched = {
            'joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2',
            'joint_arm_l3', 'joint_wrist_yaw', 'joint_wrist_pitch',
            'joint_wrist_roll', 'joint_head_pan', 'joint_head_tilt'}

        def stopped():
            nonlocal stable
            if (self.joints is None or
                    time.monotonic() - self.joints_received > 0.5 or
                    len(self.joints.velocity) != len(self.joints.name)):
                stable = 0
                return False
            velocities = dict(zip(self.joints.name, self.joints.velocity))
            if any(abs(velocities.get(name, math.inf)) > 0.01
                   for name in watched):
                stable = 0
                return False
            stable += 1
            return stable >= 4

        self._wait_for(stopped, 5.0, 'upper body did not become stationary')

    def _wait_joints_reached(self, names, positions, tolerance, timeout):
        """Wait until /joint_states reports the joints at the commanded pose."""
        deadline = time.monotonic() + timeout
        while True:
            current = self._joint_positions()
            fresh = (self.joints is not None and
                     time.monotonic() - self.joints_received < 0.5)
            errors = {name: (abs(current[name] - value)
                             if name in current else math.inf)
                      for name, value in zip(names, positions)}
            if fresh and all(error <= tolerance for error in errors.values()):
                return
            if time.monotonic() >= deadline:
                detail = ', '.join(
                    f'{name}: commanded={value:.3f} '
                    f'measured={current.get(name, math.nan):.3f}'
                    for name, value in zip(names, positions))
                raise RuntimeError(
                    'joints did not reach the commanded pose (' + detail + ')')
            time.sleep(0.05)

    def _move_head(self, pan, tilt):
        names = ['joint_head_pan', 'joint_head_tilt']
        positions = [pan, tilt]
        duration = float(self.get_parameter('head_motion_duration').value)
        if self.simulation_mode:
            self._send_trajectory(names, positions, duration, 'camera alignment')
            return
        tolerance = float(self.get_parameter('head_reach_tolerance').value)
        failures = []
        for mode in self.get_parameter('head_control_modes').value:
            try:
                current = self._joint_positions()
                self.get_logger().info(
                    f'[HEAD] {mode} mode: pan {current.get(names[0], math.nan):.3f}'
                    f' -> {pan:.3f}, tilt {current.get(names[1], math.nan):.3f}'
                    f' -> {tilt:.3f}')
                self._send_trajectory(
                    names, positions, duration, 'camera alignment',
                    control_mode=mode)
                self._wait_joints_reached(names, positions, tolerance, 8.0)
                self.get_logger().info(f'[HEAD] camera aligned via {mode} mode')
                return
            except RuntimeError as exc:
                failures.append(f'{mode}: {exc}')
                self.get_logger().warning(f'[HEAD] {mode} mode failed: {exc}')
        raise RuntimeError(
            'camera alignment failed in every driver mode (' +
            '; '.join(failures) + ')')

    def _aim_head(self, target):
        in_base = do_transform_pose_stamped(
            target, self.tf.lookup_transform(
                self.base_frame, target.header.frame_id, rclpy.time.Time()))
        camera = self.tf.lookup_transform(
            self.base_frame, self.camera_frame, rclpy.time.Time())
        pan, tilt = head_angles(
            in_base.pose.position.x, in_base.pose.position.y, in_base.pose.position.z,
            camera.transform.translation.x, camera.transform.translation.y,
            camera.transform.translation.z,
            float(self.get_parameter('head_pan_offset').value),
            float(self.get_parameter('head_tilt_offset').value))
        # Keep a small margin inside the Stretch head limits: the driver is
        # launched with fail_out_of_range_goal=True and rejects any goal that
        # lands outside its calibrated range.
        pan = max(-3.7, min(1.4, pan))
        tilt = max(-1.4, min(0.7, tilt))
        self._move_head(pan, tilt)
        time.sleep(0.3)
        optical = do_transform_pose_stamped(
            target, self.tf.lookup_transform(
                self.camera_frame, target.header.frame_id, rclpy.time.Time()))
        p = optical.pose.position
        norm = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        angle = math.acos(max(-1.0, min(1.0, p.z / norm))) if norm else math.pi
        if p.z <= 0.0 or angle > float(self.get_parameter('camera_alignment_tolerance').value):
            raise RuntimeError(f'camera did not face target (error={angle:.3f} rad)')

    def _refresh_octomap(self):
        self._wait_upper_body_stopped()
        if not self.capture_map.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('OctoMap one-shot capture service is unavailable')
        reset_at = time.monotonic()
        response = self._wait_future(
            self.capture_map.call_async(Trigger.Request()), 5.0)
        if response is None or not response.success:
            raise RuntimeError('OctoMap one-shot capture failed')
        self._wait_for(
            lambda: (self.scene_received > reset_at and
                     self.scene_status.startswith('SCENE_AVAILABLE') and
                     'frozen=1' in self.scene_status),
            10.0, 'no frozen OctoMap arrived from the stationary head camera')

    def _reach(self, target):
        transformed = do_transform_pose_stamped(
            target, self.tf.lookup_transform(
                self.base_frame, target.header.frame_id, rclpy.time.Time()))
        self.get_logger().info(
            f'[LIFT_TRACE] stage=request '
            f'target_z={target.pose.position.z:.6f} '
            f'target_frame={target.header.frame_id} '
            f'transformed_z={transformed.pose.position.z:.6f} '
            f'transformed_frame={self.base_frame}')
        transformed.header.frame_id = 'ik_preview/' + self.base_frame
        transformed.header.stamp = rclpy.time.Time().to_msg()
        self.target_pub.publish(transformed)
        sent_at = time.monotonic()
        self._wait_for(lambda: self.ik_received > sent_at and self.ik_ok,
                       10.0, 'IK could not reach the selected target')
        if not self.get_parameter('execute_manipulation').value:
            self.emit('COMPLETE: navigation, camera, OctoMap and IK validated; execution disabled')
            return
        if not self.execute.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('guarded manipulation service is unavailable')
        started = time.monotonic()
        response = self._wait_future(self.execute.call_async(Trigger.Request()), 15.0)
        if response is None or not response.success:
            raise RuntimeError('manipulation planner rejected target: ' +
                               (response.message if response else 'no response'))
        timeout = float(self.get_parameter('manipulation_timeout').value)
        self._wait_for(
            lambda: (self.executor_received > started and
                     ('state=COMPLETE' in self.executor_status or
                      'state=ABORTED' in self.executor_status)),
            timeout, 'manipulation execution timed out')
        if 'state=ABORTED' in self.executor_status:
            raise RuntimeError('guarded execution aborted: ' + self.executor_status)

    def _run(self, target):
        try:
            self.emit('STOWING: retracting arm into the navigation envelope')
            self._stow_for_navigation()
            self.emit('NAVIGATING: computing a right-side approach with Nav2')
            self._navigate(target)
            self._wait_stopped()
            if self.get_parameter('navigate_only').value:
                self.emit(
                    f'COMPLETE: base reached {self.gazebo_target_model or "target"} '
                    'without arm motion')
                return
            self.emit('AIMING_CAMERA: base stopped; pointing head at target')
            self._aim_head(target)
            self.emit('MAPPING: clearing motion-corrupted voxels and rebuilding local OctoMap')
            self._refresh_octomap()
            self.emit('MANIPULATING: solving IK and collision-aware path')
            self._reach(target)
            if self.get_parameter('execute_manipulation').value:
                if 'dry_run=1' in self.executor_status:
                    self.emit(
                        'COMPLETE: navigation + camera + OctoMap + IK dry-run '
                        'validated; no arm motion was commanded')
                else:
                    self.emit(
                        'COMPLETE: navigation + camera + OctoMap + IK '
                        'execution succeeded')
        except Exception as exc:
            self.zero_pub.publish(Twist())
            self.emit('FAILED: ' + str(exc))
            self.get_logger().error(str(exc))
        finally:
            with self.lock:
                self.running = False


def main():
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = AutonomousMission()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.zero_pub.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
