#!/usr/bin/env python3
"""Shelf grasp mission for the real Stretch 3.

Flow: Nav2 drives (head looks ahead so the OctoMap fills) -> on arrival the
head turns right and stays fixed on the shelf -> the operator clicks the
object (segmenter + object_localizer publish an ObjectObservation) ->
grasps are planned and shown -> one GraspObject goal (or `~/pick`) runs the
whole mission: base correction along X if the arm cannot reach, pre-grasp,
adaptive opening, fresh re-observation, straight approach in short steps,
close, verify, lift, retreat, verify again.

One mission at a time. Readiness is checked before a mission starts; every
mission ends exactly once with SUCCEEDED (pickup verified), ABORTED (error
code + stage) or CANCELED. A failure after motion may have started stops the
executor and waits for the stop to be confirmed; an unconfirmed stop latches
a fault that blocks new missions until `~/clear_fault`.

Every arm motion goes through ik_preview (grasp goals) and the guarded
executor (RRT, guards, dynamic obstacle stop). Events are JSON lines on
`~/events` and in the log: mission_id, stage, event, monotonic and ROS time.
"""
import collections
import json
import math
import os
import re
import threading
import time
import uuid

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.action import FollowJointTrajectory, GripperCommand
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, JointState, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from stretch_custom_manipulation.action import GraspObject
from stretch_custom_manipulation.msg import ObjectObservation

import grasp_planning as gp

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
# link_grasp_center y in base_link, wrist straight, arm retracted / extended
# (captured URDF, hardware_limits.csv arm_extension 0..0.517 m).
ARM_Y_RETRACTED, ARM_Y_EXTENDED = -0.423, -0.940
SETTLE_STEP = np.array([3e-4] * 5 + [3e-3] * 3)
ARM_JOINTS = ('joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
              'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll')

# Executor planning/preflight status -> mission error code.
EXECUTOR_CODES = (
    ('IK_FAILED', 'IK_FAILED'),
    ('START_IN_COLLISION', 'COLLISION'), ('GOAL_IN_COLLISION', 'COLLISION'),
    ('PLANNING_FAILED', 'COLLISION'), ('TRAJECTORY_COLLISION', 'COLLISION'),
    ('LIVE_PRE_EXECUTION_REJECTED', 'COLLISION'), ('SCENE_UNAVAILABLE', 'COLLISION'),
    ('PATH_INVALID', 'COLLISION'),
    ('EXECUTION_REJECTED', 'CONTROLLER_REJECTED'),
    ('TRAJECTORY_GENERATION_FAILED', 'CONTROLLER_REJECTED'),
)


class MissionError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def executor_code(text, default='CONTROLLER_REJECTED'):
    for key, code in EXECUTOR_CODES:
        if key in text:
            return code
    return default


def parse_executor(status):
    """Fields of a /guarded_executor/status line the mission relies on."""
    def field(name, default=''):
        m = re.search(r'(?:^| )' + name + r'=(\S+)', status)
        return m.group(1) if m else default
    rid = re.search(r'REQUEST_ID=(\d+)', status)
    return {'state': field('state'), 'pending': field('pending_goal') == '1',
            'sequence': field('sequence_active') == '1',
            'stop_unconfirmed': field('stop_unconfirmed') == '1',
            'request_id': int(rid.group(1)) if rid else 0,
            'direct': 'PLANNER=DIRECT' in status,
            'reason': status.split(' reason=', 1)[-1][:240] if ' reason=' in status else ''}


def angle(a, b):
    return math.acos(max(-1.0, min(1.0, float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))))))


def pose_from_text(text):
    v = [float(x) for x in text.split()]
    m = np.eye(4)
    x, y, z, w = v[3:]
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = v[:3]
    return m


def pose_msg(m, frame):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y, p.pose.position.z = m[:3, 3].tolist()
    (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,
     p.pose.orientation.w) = gp.quaternion(m)
    return p


def cloud_array(msg):
    pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                         skip_nans=True)
    return np.asarray(pts, float).reshape(-1, 3)


class GraspMission(Node):
    def __init__(self):
        super().__init__('grasp_mission')
        p = self.declare_parameter
        self.search_head = (p('search_pan', -1.571).value, p('search_tilt', -0.45).value)
        self.drive_head = (p('navigation_pan', 0.0).value, p('navigation_tilt', -0.6).value)
        self.offset = p('pregrasp_offset', 0.10).value
        # Opening beyond object width + 2 x 1.5 cm; read at each plan, so
        # `ros2 param set /grasp_mission extra_opening` tunes it live.
        self.extra_opening = p('extra_opening', 0.04).value
        # auto: side first, top-down only if no side grasp fits; side or top: only that
        # kind. Read at each plan: `ros2 param set /grasp_mission grasp_mode top`.
        p('grasp_mode', 'auto')
        # Open fully (13 cm), narrower only where a neighbor is closer than the
        # fingers; false: object width + 3 cm + extra_opening. Read at each plan.
        p('fully_open', True)
        # A valid click starts the mission once a collision-free grasp is planned.
        self.auto_start = p('auto_start', True).value
        # Carry after the grasp: up, retract the arm, wrist pose, lift height.
        # Fingers closed while the arm travels to the pre-grasp: a smaller
        # gripper for the long move; they open only before the approach.
        self.close_for_transit = p('close_for_transit', True).value
        self.carry_up = p('carry_lift_up', 0.05).value
        self.carry_extension = p('carry_extension', 0.0).value
        # [yaw, pitch, roll] in rad.
        p('carry_wrist', [0.0, 0.0, 0.0])
        self.carry_lift = p('carry_lift', 0.4).value
        self.dx_tolerance = p('base_dx_tolerance', 0.01).value
        self.max_dx = p('max_base_correction', 0.5).value
        self.squeeze = p('close_aperture', -0.02).value
        self.min_held = p('min_held_aperture', 0.01).value
        # Closing must move the fingers at least this far, or it did not close.
        self.close_margin = p('min_close_travel', 0.005).value
        self.move_timeout = p('move_timeout', 60.0).value
        self.ik_timeout = p('ik_timeout', 5.0).value
        # Cans and medicine boxes; anything longer is a shelf, a wall or a
        # mask that leaked, never something to grasp.
        self.max_object_size = p('max_object_size', 0.35).value
        self.max_observation_age = p('max_observation_age', 120.0).value
        self.min_confidence = p('min_confidence', 0.5).value
        # FK of the IK answer must match the planned grasp: the fixed base
        # cannot move the TCP along x at a fixed wrist orientation.
        self.max_approach_error = p('max_approach_error', 0.05).value
        self.max_approach_change = p('max_approach_change', 0.03).value
        self.max_finger_error = p('max_finger_error', 0.10).value
        self.approach_step = p('approach_step', 0.05).value
        # Off by default: on the robot the gripper at the pre-grasp covers the
        # object in the head camera view and the new mask was not the object.
        self.reobserve = p('reobserve_at_pregrasp', False).value
        self.reobserve_timeout = p('reobserve_timeout', 12.0).value
        self.max_target_shift = p('max_target_shift', 0.03).value
        self.replan_threshold = p('replan_threshold', 0.005).value
        self.settle_timeout = p('settle_timeout', 4.0).value
        # Dry run: the executor validates without moving; head, gripper and
        # base commands are logged instead of sent. Never reports success.
        self.dry_run = p('dry_run', False).value
        # Stowed base outline in base_link (x back, x front, half width) and
        # the free distance the lidar must see ahead of it while correcting.
        self.footprint = (-0.28, 0.063, 0.22)
        self.clearance = p('base_correction_clearance', 0.10).value
        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.lock = threading.Lock()
        # Object points in base_link. Not odom: the driver resets odometry to
        # zero on every switch to trajectory mode (stretch_body first_step).
        self.obs = None
        self.object_base = None
        self.scene_odom = np.zeros((0, 3))
        self.scene_frame = 'odom'
        self.candidates = []
        self.joints, self.joint_time, self.efforts = {}, 0.0, {}
        self.chain = None
        self.ik, self.ik_time = {}, 0.0
        self.exec_status, self.exec_time = '', 0.0
        self.mode, self.mode_time = '', 0.0
        self.homed = self.runstopped = None
        self.robot_time = 0.0
        self.odom, self.odom_epoch = None, 0
        self.scan = None
        self.camera_info = None
        self.nav_active = False
        self.active = None          # job holding the arm: mission id, 'look', ...
        self.mission = None
        self.results = collections.OrderedDict()
        self.fault = ''
        self.stop_requested = False
        self.handles = []           # driver/gripper goals sent by this node
        self.reobs_wait = None
        self.stamp_ns = 0

        sub = self.create_subscription
        sub(ObjectObservation, '/perception/object_observation', self.on_observation, 1,
            callback_group=group)
        # The OctoMap builder publishes its filtered cloud best-effort.
        sub(PointCloud2, '/perception/filtered_points', self.on_scene,
            qos_profile_sensor_data, callback_group=group)
        sub(JointState, p('joint_state_topic', '/stretch/joint_states').value,
            self.on_joints, 10, callback_group=group)
        sub(DiagnosticArray, '/ik_preview/status', self.on_ik, 10, callback_group=group)
        sub(String, '/robot_description', lambda m: setattr(self, 'chain', gp.urdf_chain(m.data)),
            latched, callback_group=group)
        sub(String, '/guarded_executor/status', self.on_exec, 10, callback_group=group)
        sub(String, '/mode', self.on_mode, qos_profile_sensor_data, callback_group=group)
        sub(Bool, '/is_homed', lambda m: self.on_flag('homed', m), qos_profile_sensor_data,
            callback_group=group)
        sub(Bool, '/is_runstopped', lambda m: self.on_flag('runstopped', m),
            qos_profile_sensor_data, callback_group=group)
        sub(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_nav, 10,
            callback_group=group)
        sub(LaserScan, '/scan', lambda m: setattr(self, 'scan', m),
            qos_profile_sensor_data, callback_group=group)
        sub(Odometry, '/odom', self.on_odom, qos_profile_sensor_data, callback_group=group)
        sub(CameraInfo, p('camera_info_topic', '/camera/color/camera_info').value,
            lambda m: setattr(self, 'camera_info', m), qos_profile_sensor_data,
            callback_group=group)

        self.status_pub = self.create_publisher(String, '~/status', latched)
        self.result_pub = self.create_publisher(String, '~/result', latched)
        self.events_pub = self.create_publisher(String, '~/events', 100)
        self.goal_pub = self.create_publisher(PoseStamped, '/ik_preview/grasp_goal', 10)
        self.prompt_pub = self.create_publisher(Point, '/perception/segment_prompt', 1)
        self.box_pub = self.create_publisher(Marker, '/octomap_builder/target_box', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '~/grasps', latched)
        self.debug_pub = self.create_publisher(MarkerArray, '~/debug', latched)

        self.trajectory = ActionClient(self, FollowJointTrajectory,
                                       '/stretch_controller/follow_joint_trajectory',
                                       callback_group=group)
        self.gripper = ActionClient(self, GripperCommand,
                                    '/stretch_gripper_controller/gripper_action',
                                    callback_group=group)
        self.move_srv = self.create_client(Trigger, '/guarded_executor/move_to_preview',
                                           callback_group=group)
        self.stop_srv = self.create_client(Trigger, '/guarded_executor/stop',
                                           callback_group=group)
        self.traj_mode = self.create_client(Trigger, '/switch_to_trajectory_mode',
                                            callback_group=group)
        self.pos_mode = self.create_client(Trigger, '/switch_to_position_mode',
                                           callback_group=group)

        ActionServer(self, GraspObject, '~/grasp', self.execute_goal,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=self.on_cancel, callback_group=group)
        srv = self.create_service
        srv(Trigger, '~/look', lambda q, r: self.start_job(r, 'look', self.look_at_shelf),
            callback_group=group)
        srv(Trigger, '~/pick', self.on_pick, callback_group=group)
        srv(Trigger, '~/release', lambda q, r: self.start_job(r, 'release', self.release),
            callback_group=group)
        srv(Trigger, '~/stop', self.on_stop, callback_group=group)
        srv(Trigger, '~/clear_fault', self.on_clear_fault, callback_group=group)
        self.say('IDLE: click the object in the camera image' +
                 ('' if self.auto_start else ', then call ~/pick'))

    # ---------------------------------------------------------------- inputs
    def say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def event(self, name, **kw):
        m = self.mission
        rec = {'mission_id': m['id'] if m else '', 'stage': m['stage'] if m else '',
               'event': name, 't_mono': round(time.monotonic(), 4),
               't_ros': self.get_clock().now().nanoseconds * 1e-9}
        if m:
            rec['elapsed'] = round(time.monotonic() - m['t0'], 3)
        rec.update(kw)
        line = json.dumps(rec, default=lambda v: np.round(np.asarray(v, float), 4).tolist())
        self.get_logger().info(line)
        self.events_pub.publish(String(data=line))

    def on_joints(self, msg):
        with self.lock:
            self.joints.update(zip(msg.name, msg.position))
            if len(msg.effort) == len(msg.name):
                self.efforts.update(zip(msg.name, msg.effort))
            self.joint_time = time.monotonic()

    def on_ik(self, msg):
        if msg.status:
            s = msg.status[0]
            with self.lock:
                self.ik = {kv.key: kv.value for kv in s.values}
                self.ik['_message'] = s.message
                self.ik_time = time.monotonic()

    def on_exec(self, msg):
        with self.lock:
            self.exec_status = msg.data
            self.exec_time = time.monotonic()

    def on_mode(self, msg):
        self.mode, self.mode_time = msg.data, time.monotonic()

    def on_flag(self, name, msg):
        setattr(self, name, msg.data)
        self.robot_time = time.monotonic()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        # The driver resets odometry on every switch to trajectory mode.
        if self.odom and math.hypot(pose[0] - self.odom[0], pose[1] - self.odom[1]) > 0.05:
            self.odom_epoch += 1
        self.odom = pose

    def on_scene(self, msg):
        self.scene_odom = cloud_array(msg)
        self.scene_frame = msg.header.frame_id

    def base_from(self, frame, stamp=None):
        try:
            t = self.tf.lookup_transform('base_link', frame, stamp or Time(),
                                         Duration(seconds=0.5))
        except TransformException as e:
            raise MissionError('NOT_READY', f'TF base_link <- {frame}: {e}')
        return gp.matrix_from_transform(t.transform)

    def on_observation(self, msg):
        obs = {'id': msg.object_id, 'source': msg.source, 'valid': msg.valid,
               'reason': msg.reason, 'confidence': float(msg.confidence),
               'stamp': Time.from_msg(msg.header.stamp), 'recv': time.monotonic()}
        if msg.valid:
            try:
                # The base pose at capture, not now: the cloud is in the frame
                # it had when the image was taken.
                m = self.base_from(msg.header.frame_id, obs['stamp'])
                obs['points'] = gp.apply(m, cloud_array(msg.cloud))
                odom = np.linalg.inv(self.base_from('odom', obs['stamp']))
                obs['base_odom'] = (odom[0, 3], odom[1, 3], math.atan2(odom[1, 0], odom[0, 0]))
                obs['epoch'] = self.odom_epoch
            except MissionError as e:
                obs.update(valid=False, reason=str(e))
            if obs['valid'] and len(obs['points']) < 10:
                obs.update(valid=False, reason='fewer than 10 object points')
        box = self.reobs_wait
        if box is not None:
            box.append(obs)
            return
        self.obs = obs
        if self.active is None:
            if not obs['valid']:
                self.say(f'OBSERVATION INVALID: {obs["reason"]}')
                return
            self.object_base = obs['points']
            try:
                self.plan()
            except MissionError as e:
                self.say(f'PLAN FAILED: {e.code}: {e}')
                return
            if not self.auto_start:
                self.say(f'PLANNED: {len(self.candidates)} grasps for {obs["id"]}; call ~/pick')
                return
            mission_id = 'click-' + uuid.uuid4().hex[:8]
            problem = self.admit(mission_id, obs['id'])
            if problem:
                self.say(f'NOT STARTED: {problem[0]}: {problem[1]}')
                self.publish_result(self.result(mission_id, problem[0], 'READINESS', problem[1]))
                return
            self.say(f'PLANNED: {len(self.candidates)} grasps for {obs["id"]}; starting {mission_id}')
            threading.Thread(target=self.run_mission, args=(mission_id, None), daemon=True).start()

    def on_nav(self, msg):
        # Head ahead while Nav2 drives (fills the map), right on arrival.
        active = any(s.status in ACTIVE for s in msg.status_list)
        if active == self.nav_active:
            return
        self.nav_active = active
        succeeded = msg.status_list and msg.status_list[-1].status == GoalStatus.STATUS_SUCCEEDED
        if active or succeeded:
            head = self.drive_head if active else self.search_head
            with self.lock:
                if self.active is not None:
                    return
                self.active = 'nav-head'
            threading.Thread(target=self.run_job, args=('nav-head', lambda: self.head(*head)),
                             daemon=True).start()

    # --------------------------------------------------------- entry points
    def claim(self, job):
        with self.lock:
            if self.active is not None:
                return 'BUSY', f'{self.active} is running; call ~/stop to cancel it'
            self.active = job
            self.stop_requested = False
        return None, ''

    def start_job(self, response, name, job):
        code, why = self.claim(name)
        if code:
            response.message = f'{code}: {why}'
            self.say(response.message)
            return response
        threading.Thread(target=self.run_job, args=(name, job), daemon=True).start()
        response.success, response.message = True, 'started'
        return response

    def run_job(self, name, job):
        try:
            job()
        except MissionError as e:
            self.say(f'FAILED: {name}: {e.code}: {e}')
        finally:
            with self.lock:
                self.active = None

    def on_pick(self, _, response):
        mission_id = 'pick-' + uuid.uuid4().hex[:8]
        problem = self.admit(mission_id, '')
        if problem:
            response.message = f'{problem[0]}: {problem[1]}'
            self.say(f'REJECTED: {mission_id}: {response.message}')
            self.publish_result(self.result(mission_id, problem[0], 'READINESS', problem[1]))
            return response
        threading.Thread(target=self.run_mission, args=(mission_id, None), daemon=True).start()
        response.success, response.message = True, f'started mission_id={mission_id}'
        return response

    def on_cancel(self, handle):
        m = self.mission
        if m and m.get('handle') is handle:
            self.stop_requested = True
        return CancelResponse.ACCEPT

    def execute_goal(self, handle):
        mission_id = handle.request.mission_id or 'grasp-' + uuid.uuid4().hex[:8]
        if mission_id in self.results:
            # Same ID again: report the finished mission, never move again.
            r = dict(self.results[mission_id])
            self.event('duplicate_finished_goal', duplicate_of=mission_id)
            return self.finish_handle(handle, r)
        problem = self.admit(mission_id, handle.request.target_id)
        if problem:
            r = self.result(mission_id, problem[0], 'READINESS', problem[1])
            self.publish_result(r)
            return self.finish_handle(handle, r)
        return self.finish_handle(handle, self.run_mission(mission_id, handle))

    def finish_handle(self, handle, r):
        res = GraspObject.Result(mission_id=r['mission_id'], success=r['success'],
                                 error_code=r['error_code'], failed_stage=r['failed_stage'],
                                 message=r['message'], held_aperture=r['held_aperture'],
                                 duration=r['duration'])
        if r['success']:
            handle.succeed()
        elif r['error_code'] == 'CANCELED' and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return res

    def admit(self, mission_id, target_id):
        """Claim the arm for a mission, or (code, reason) without claiming."""
        with self.lock:
            if self.mission and self.mission['id'] == mission_id:
                return 'DUPLICATE_ACTIVE', f'{mission_id} is already running'
        code, why = self.claim(mission_id)
        if code:
            return code, why
        problem = self.readiness(target_id)
        if problem:
            with self.lock:
                self.active = None
        return problem

    def readiness(self, target_id):
        if self.fault:
            return 'NOT_READY', f'fault latched: {self.fault}; inspect, then call ~/clear_fault'
        obs = self.obs
        if obs is None:
            return 'TARGET_UNKNOWN', 'no object observation; click the object'
        if target_id and target_id != obs['id']:
            return 'TARGET_UNKNOWN', f'latest observation is {obs["id"]}, not {target_id}'
        if not obs['valid']:
            return 'PERCEPTION_INVALID', obs['reason']
        if obs['confidence'] < self.min_confidence:
            return 'PERCEPTION_INVALID', f'confidence {obs["confidence"]:.2f} < {self.min_confidence}'
        age = time.monotonic() - obs['recv']
        if age > self.max_observation_age:
            return 'STALE_OBSERVATION', f'observation is {age:.0f} s old; click again'
        if self.odom and obs['epoch'] == self.odom_epoch:
            (x0, y0, t0), (x, y, t) = obs['base_odom'], self.odom
            if math.hypot(x - x0, y - y0) > 0.02 or abs(math.remainder(t - t0, math.tau)) > 0.02:
                return 'STALE_OBSERVATION', 'the base moved since the observation; click again'
        now = time.monotonic()
        missing = []
        if now - self.joint_time > 0.5:
            missing.append('joint states')
        if now - self.robot_time > 1.0 or self.homed is not True or self.runstopped is not False:
            missing.append(f'robot homed/runstop (homed={self.homed}, runstopped={self.runstopped})')
        if now - self.ik_time > 1.0:
            missing.append('ik_preview status')
        e = parse_executor(self.exec_status)
        if now - self.exec_time > 1.0:
            missing.append('guarded executor status')
        elif e['state'] == 'EXECUTING' or e['pending'] or e['sequence'] or e['stop_unconfirmed']:
            missing.append(f'guarded executor idle (state={e["state"]})')
        if not self.move_srv.service_is_ready() or not self.stop_srv.service_is_ready():
            missing.append('executor services')
        if not self.dry_run:
            if not self.trajectory.server_is_ready():
                missing.append('driver trajectory action')
            if not self.gripper.server_is_ready():
                missing.append('gripper bridge')
            if not self.traj_mode.service_is_ready():
                missing.append('mode services')
            if now - self.mode_time > 1.0:
                missing.append('driver mode')
            if self.reobserve and self.camera_info is None:
                missing.append('camera info')
        if not self.tf.can_transform('base_link', 'odom', Time()):
            missing.append('TF base_link <- odom')
        return ('NOT_READY', 'missing: ' + ', '.join(missing)) if missing else None

    def on_stop(self, _, response):
        self.stop_requested = True
        if self.active is None and self.stop_srv.service_is_ready():
            self.stop_srv.call_async(Trigger.Request())
        response.success = True
        response.message = 'stop requested'
        self.say('STOP requested')
        return response

    def on_clear_fault(self, _, response):
        e = parse_executor(self.exec_status)
        if self.active is not None or e['pending'] or e['stop_unconfirmed'] or e['state'] == 'EXECUTING':
            response.message = 'not cleared: a job or a motion is still active'
        else:
            self.event('fault_cleared', fault=self.fault)
            self.fault = ''
            response.success, response.message = True, 'fault cleared'
        return response

    # -------------------------------------------------------------- mission
    def result(self, mission_id, code, stage, message, held=0.0, duration=0.0):
        return {'mission_id': mission_id, 'success': code == 'NONE', 'error_code': code,
                'failed_stage': '' if code == 'NONE' else stage, 'message': message,
                'held_aperture': float(held), 'duration': float(duration)}

    def publish_result(self, r):
        self.result_pub.publish(String(data=json.dumps(r)))

    def stage(self, name, detail=''):
        m = self.mission
        now = time.monotonic()
        self.event('stage_end', duration=round(now - m['stage_t0'], 3))
        m['stage'], m['stage_t0'] = name, now
        self.event('stage_start', detail=detail)
        self.say(f'{name}: {detail}' if detail else name)
        if m.get('handle') is not None:
            m['handle'].publish_feedback(GraspObject.Feedback(
                mission_id=m['id'], stage=name, detail=detail, elapsed=now - m['t0']))

    def run_mission(self, mission_id, handle):
        now = time.monotonic()
        self.mission = {'id': mission_id, 'handle': handle, 't0': now, 'stage': 'ACCEPTED',
                        'stage_t0': now, 'motion': False, 'held': 0.0}
        self.event('accepted', observation=self.obs['id'], dry_run=self.dry_run)
        code, message = 'NONE', ''
        try:
            message = self.pick()
            if self.dry_run:
                code = 'DRY_RUN'
        except MissionError as e:
            code, message = e.code, str(e)
            if self.stop_requested and code not in ('STOP_UNCONFIRMED',):
                code = 'CANCELED'
        except Exception as e:  # a bug must still end the mission once
            code, message = 'INTERNAL_ERROR', repr(e)
            self.get_logger().error(message)
        m = self.mission
        if code not in ('NONE', 'DRY_RUN') and m['motion']:
            stopped = self.safe_stop(code)
            if not stopped:
                message += '; STOP NOT CONFIRMED (fault latched)'
                code = 'STOP_UNCONFIRMED'
        r = self.result(mission_id, code, m['stage'], message, m['held'], time.monotonic() - m['t0'])
        self.event('result', **r)
        self.results[mission_id] = r
        while len(self.results) > 50:
            self.results.popitem(last=False)
        self.publish_result(r)
        if code == 'NONE':
            self.say(f'DONE: {message}')
        elif code == 'DRY_RUN':
            self.say(f'DRY RUN DONE: {message} (no pickup verified)')
        elif code == 'CANCELED':
            self.say(f'CANCELED: {mission_id} at {m["stage"]}: {message}')
        else:
            self.say(f'FAILED: {mission_id} at {m["stage"]}: {code}: {message}')
        with self.lock:
            self.mission = None
            self.active = None
        return r

    def safe_stop(self, why):
        """Stop everything this mission may have started and confirm it."""
        self.event('stopping', reason=why)
        for h in list(self.handles):
            h.cancel_goal_async()
        if self.stop_srv.service_is_ready():
            self.stop_srv.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        called = time.monotonic()
        while time.monotonic() < end:
            with self.lock:
                e, stamp = parse_executor(self.exec_status), self.exec_time
            handles_done = all(h.status not in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                                GoalStatus.STATUS_CANCELING) for h in self.handles)
            if stamp > called and not e['pending'] and not e['stop_unconfirmed'] and \
                    e['state'] != 'EXECUTING' and handles_done:
                self.handles.clear()
                self.event('stop_confirmed')
                return True
            time.sleep(0.05)
        self.fault = f'stop after {why} not confirmed'
        self.event('stop_unconfirmed')
        return False

    def check(self):
        m = self.mission
        if self.stop_requested or (m and m.get('handle') is not None and
                                   m['handle'].is_cancel_requested):
            self.stop_requested = True
            raise MissionError('CANCELED', 'stopped by operator')

    def wait(self, future, timeout, what, code='TIMEOUT'):
        end = time.monotonic() + timeout
        while not future.done():
            self.check()
            if time.monotonic() > end:
                raise MissionError(code, f'{what} timed out')
            time.sleep(0.02)
        return future.result()

    def pick(self):
        self.object_base = self.obs['points']
        # The head stays where it is: it sees the object, which was just clicked,
        # and that view is also what the dynamic obstacle stop watches.
        self.stage('SELECT_GRASP')
        c = self.choose()
        self.publish_debug(c)
        self.event('grasp_selected', mode=c['mode'], key=c['key'], aperture=c['aperture'],
                   grasp=c['grasp'], frame='base_link', residual=c['ik_residual'])
        if self.close_for_transit and not self.dry_run:
            self.stage('CLOSE_FOR_TRANSIT')
            self.grip(0.0)
        self.prepare_arm()
        lo, hi = self.object_base.min(0), self.object_base.max(0)
        g = c['grasp'][:3, 3]
        self.stage('MOVE_PREGRASP',
                   f'{c["mode"]} grasp, opening {c["aperture"] * 100:.1f} cm, grasp center '
                   f'(base_link) x={g[0]:.3f} y={g[1]:.3f} z={g[2]:.3f}; object x {lo[0]:.3f}..'
                   f'{hi[0]:.3f} y {lo[1]:.3f}..{hi[1]:.3f} z {lo[2]:.3f}..{hi[2]:.3f} '
                   f'({len(self.object_base)} points)')
        self.move(c['pregrasp'], 'pre-grasp')
        self.stage('OPEN_GRIPPER')
        opened = self.grip(c['aperture'])
        if self.reobserve and not self.dry_run:
            self.stage('REOBSERVE')
            c = self.refresh(c)
            self.publish_debug(c)
        # Planned contact: the object leaves the collision map for the approach.
        lo, hi = self.object_base.min(0), self.object_base.max(0)
        self.set_target_box('base_link', (lo + hi) / 2, hi - lo + 0.02)
        time.sleep(0.6)  # next map publication without the object
        self.prepare_arm()
        self.stage('APPROACH')
        self.approach(c)
        self.stage('CLOSE_GRIPPER')
        held = self.grip(self.squeeze)
        self.mission['held'] = held
        with self.lock:
            effort = self.efforts.get('joint_gripper_finger_left')
        self.event('closed', aperture=held, effort=effort)
        if not self.dry_run and held > opened - self.close_margin:
            raise MissionError('GRIPPER_NOT_CLOSED',
                               f'fingers stayed at {held * 100:.1f} cm after closing from '
                               f'{opened * 100:.1f} cm')
        # Thin objects flex the fingers almost as far as an empty close, so a
        # small aperture is not proof of a miss: keep holding, never reopen here.
        verified = held >= self.min_held
        # The object now travels with the gripper.
        size = hi - lo
        side = max(size[0], size[1]) + 0.03
        self.set_target_box('link_grasp_center', (0.0, 0.0, 0.0), (side, side, size[2] + 0.03))
        self.carry(c)
        self.stage('VERIFY_HOLD')
        # Squeezing again reports where the fingers stop: still on the object?
        again = self.grip(self.squeeze)
        self.mission['held'] = again
        with self.lock:
            effort = self.efforts.get('joint_gripper_finger_left')
        self.event('hold_check', aperture=again, effort=effort)
        if verified and again < self.min_held:
            raise MissionError('OBJECT_SLIPPED', f'fingers closed to {again * 100:.1f} cm after the carry')
        if not verified:
            raise MissionError('PICKUP_UNVERIFIED',
                               f'carried, gripper still closed, but fingers at {again * 100:.1f} cm '
                               'cannot tell a thin object from an empty grip; check visually')
        self.stage('VERIFIED')
        return f'object held at {again * 100:.1f} cm; call ~/release to open'

    def carry(self, c):
        """Up, retract the arm, wrist pose, lift height: each a guarded IK move."""
        self.prepare_arm()
        self.stage('LIFT', f'+{self.carry_up * 100:.0f} cm')
        up = c['grasp'].copy()
        up[2, 3] += self.carry_up
        self.move(up, 'lift')
        self.stage('RETRACT_ARM')
        self.move_joints('retract arm', ext=self.carry_extension)
        wrist = list(self.get_parameter('carry_wrist').value)
        if len(wrist) == 3:
            self.stage('WRIST', str(wrist))
            self.move_joints('wrist', joint_wrist_yaw=wrist[0], joint_wrist_pitch=wrist[1],
                             joint_wrist_roll=wrist[2])
        self.stage('LOWER', f'lift {self.carry_lift:.2f} m')
        self.move_joints('lower', joint_lift=self.carry_lift)

    def move_joints(self, label, ext=None, **targets):
        """Guarded move to a joint configuration: its TCP pose (URDF FK) goes
        through ik_preview and the executor like every other motion."""
        if self.chain is None:
            raise MissionError('NOT_READY', 'no robot_description for forward kinematics')
        with self.lock:
            q = dict(self.joints)
        q.update(targets)
        if ext is not None:
            for i in range(4):
                q[f'joint_arm_l{i}'] = ext / 4
        self.move(gp.fk(self.chain, q), label)

    # --------------------------------------------------------- arm helpers
    def prepare_arm(self):
        """Trajectory mode, verified, and a still arm before the next plan.

        The gripper bridge leaves the driver in position mode; without this the
        executor switches modes itself and plans asynchronously after its own
        Trigger already answered."""
        if self.dry_run:
            return
        self.check()
        if self.mode != 'trajectory':
            if not self.traj_mode.wait_for_service(timeout_sec=2.0):
                raise MissionError('MODE_SWITCH', 'trajectory mode service unavailable')
            r = self.wait(self.traj_mode.call_async(Trigger.Request()), 5, 'trajectory mode',
                          'MODE_SWITCH')
            if not r.success:
                raise MissionError('MODE_SWITCH', f'driver refused trajectory mode: {r.message}')
            self.event('mode_switch', mode='trajectory')
        end = time.monotonic() + 3.0
        while self.mode != 'trajectory' or time.monotonic() - self.mode_time > 0.5:
            self.check()
            if time.monotonic() > end:
                raise MissionError('MODE_SWITCH', f'driver reports mode {self.mode!r}')
            time.sleep(0.05)
        self.wait_settled()

    def wait_settled(self):
        """Arm joints still for 0.5 s: the arm sags a few mm after a mode switch,
        and the executor rejects a start that drifted 1 mm from the IK seed."""
        end = time.monotonic() + self.settle_timeout
        last, still_since = None, None
        while time.monotonic() < end:
            self.check()
            with self.lock:
                q = np.array([self.joints.get(n, math.nan) for n in ARM_JOINTS])
            now = time.monotonic()
            # 0.3 mm prismatic, 3 mrad wrists: above the servos' tick jitter
            # (~1.5 mrad), below the executor's start drift limit (1 mm, 5 mrad).
            if last is not None and np.all(np.abs(q - last) < SETTLE_STEP):
                still_since = still_since or now
                if now - still_since >= 0.5:
                    return
            else:
                still_since, last = None, q
            time.sleep(0.05)
        raise MissionError('ARM_NOT_SETTLED', f'arm still moving after {self.settle_timeout:.0f} s')

    def send_trajectory(self, names, target, duration):
        with self.lock:
            start = [self.joints.get(n) for n in names]
        if None in start:
            raise MissionError('NOT_READY', f'no joint state for {names}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for t, q in ((0.0, start), (duration, list(target))):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in q]
            point.time_from_start = Duration(seconds=t).to_msg()
            goal.trajectory.points.append(point)
        if not self.trajectory.wait_for_server(timeout_sec=2.0):
            raise MissionError('NOT_READY', 'driver trajectory action unavailable')
        handle = self.wait(self.trajectory.send_goal_async(goal), 5, 'head goal')
        if not handle.accepted:
            raise MissionError('CONTROLLER_REJECTED', 'head goal rejected')
        self.handles.append(handle)
        result = self.wait(handle.get_result_async(), duration + 5, 'head motion')
        self.handles.remove(handle)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise MissionError('MOTION_ABORTED', f'head goal ended with status {result.status}')

    def head(self, pan, tilt):
        if self.dry_run:
            self.say(f'DRY RUN: head would move to pan {pan:.2f}, tilt {tilt:.2f}')
            return
        self.send_trajectory(['joint_head_pan', 'joint_head_tilt'], (pan, tilt), 1.5)

    def look_at_shelf(self):
        self.head(*self.search_head)
        self.say('LOOKING: head fixed on the right side; click the object')

    def grip(self, aperture):
        if self.dry_run:
            self.say(f'DRY RUN: gripper would go to {aperture * 100:.1f} cm')
            return max(aperture, self.min_held)
        if self.mission:
            self.mission['motion'] = True
        if not self.gripper.wait_for_server(timeout_sec=2.0):
            raise MissionError('GRIPPER_FAILED', 'gripper bridge unavailable')
        goal = GripperCommand.Goal()
        goal.command.position = float(aperture)
        handle = self.wait(self.gripper.send_goal_async(goal), 5, 'gripper goal')
        if not handle.accepted:
            raise MissionError('GRIPPER_FAILED', 'gripper goal rejected')
        self.handles.append(handle)
        result = self.wait(handle.get_result_async(), 15, 'gripper motion')
        self.handles.remove(handle)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise MissionError('GRIPPER_FAILED', f'gripper action ended with status {result.status}')
        self.event('gripper', command=aperture, measured=result.result.position)
        return result.result.position

    def next_stamp(self):
        ns = max(self.get_clock().now().nanoseconds, self.stamp_ns + 1)
        self.stamp_ns = ns
        return Time(nanoseconds=ns).to_msg(), ns

    def solve(self, pose_base):
        """IK grasp goal; the answer is matched by the goal's header stamp."""
        msg = pose_msg(pose_base, 'base_link')
        msg.header.stamp, ns = self.next_stamp()
        self.goal_pub.publish(msg)
        end = time.monotonic() + self.ik_timeout
        resend = time.monotonic() + 1.0
        while time.monotonic() < end:
            self.check()
            if time.monotonic() > resend:
                self.goal_pub.publish(msg)
                resend = time.monotonic() + 1.0
            with self.lock:
                ik = dict(self.ik)
            if ik.get('goal_stamp_ns') == str(ns) and ik.get('_message') != 'stale state':
                ok = ik.get('success') == 'true'
                answer = {'ok': ok, 'base_dx': float(ik.get('base_dx', 'nan')),
                          'reason': ik.get('_message', '')}
                if ok:
                    answer['achieved'] = pose_from_text(ik['solved_pose'])
                return answer
            time.sleep(0.02)
        raise MissionError('IK_TIMEOUT', 'IK preview did not answer')

    def geometry(self, c, ik_grasp, ik_pre):
        """Reject a grasp whose FK-achieved frame breaks the plan; returns residuals."""
        g, p = ik_grasp['achieved'], ik_pre['achieved']
        planned = c['grasp'][:3, :3]
        r = {'position_m': float(np.linalg.norm(g[:3, 3] - c['grasp'][:3, 3])),
             'approach_rad': angle(g[:3, 0], planned[:, 0]),
             'finger_rad': angle(g[:3, 1], planned[:, 1]),
             'approach_change_rad': angle(g[:3, 0], p[:3, 0])}
        if r['approach_rad'] > self.max_approach_error:
            return r, f'achieved approach off by {r["approach_rad"]:.3f} rad'
        if r['finger_rad'] > self.max_finger_error:
            return r, f'achieved finger axis off by {r["finger_rad"]:.3f} rad'
        if r['approach_change_rad'] > self.max_approach_change:
            return r, f'approach turns {r["approach_change_rad"]:.3f} rad between pre-grasp and grasp'
        scene = self.scene_base()
        rot = np.eye(4)
        rot[:3, :3] = g[:3, :3]
        if gp.fits(self.object_base, scene, rot, self.offset) is None:
            return r, 'fingers do not fit the object at the achieved orientation'
        return r, ''

    def move(self, pose_base, label, straight=False):
        """One guarded motion. A preflight rejection (nothing moved) is retried
        once after a pause: diagnostics or odometry can blip for a moment."""
        try:
            return self.move_once(pose_base, label, straight)
        except MissionError as e:
            if e.code != 'CONTROLLER_REJECTED' or 'executor rejected' not in str(e):
                raise
            self.event('retry', label=label, reason=str(e)[:200])
            time.sleep(1.0)
            self.prepare_arm()
            return self.move_once(pose_base, label, straight)

    def move_once(self, pose_base, label, straight=False):
        """IK grasp goal + guarded executor; returns when the executor reports
        COMPLETE for this request (it checks the measured endpoint)."""
        self.check()
        answer = self.solve(pose_base)
        if not answer['ok']:
            raise MissionError('IK_FAILED', f'{label} unreachable: {answer["reason"]} '
                                            f'(base_dx={answer["base_dx"]:.3f})')
        approach_err = angle(answer['achieved'][:3, 0], pose_base[:3, 0])
        self.event('ik', label=label, target=pose_base, frame='base_link',
                   achieved=answer['achieved'], approach_rad=approach_err,
                   residual_m=float(np.linalg.norm(answer['achieved'][:3, 3] - pose_base[:3, 3])))
        if straight and approach_err > self.max_approach_error:
            raise MissionError('GRASP_GEOMETRY', f'{label}: achieved approach off by '
                                                 f'{approach_err:.3f} rad')
        if not self.move_srv.wait_for_service(timeout_sec=2.0):
            raise MissionError('NOT_READY', 'guarded executor unavailable')
        with self.lock:
            before = parse_executor(self.exec_status)['request_id']
        self.mission['motion'] = True
        called = time.monotonic()
        response = self.wait(self.move_srv.call_async(Trigger.Request()), 10, 'executor')
        self.event('executor_response', label=label, success=response.success,
                   message=response.message[:300])
        if not response.success:
            raise MissionError(executor_code(response.message),
                               f'{label}: executor rejected: {response.message[:300]}')
        end = called + self.move_timeout
        while time.monotonic() < end:
            self.check()
            with self.lock:
                e, stamp = parse_executor(self.exec_status), self.exec_time
            if stamp > called and e['request_id'] > before and not e['pending'] and not e['sequence']:
                if e['state'] == 'COMPLETE':
                    if straight and not e['direct']:
                        raise MissionError('GRASP_GEOMETRY', f'{label}: planner did not use the '
                                                             'straight path')
                    self.event('motion_complete', label=label)
                    return
                if e['state'] in ('ABORTED', 'DISARMED'):
                    raise MissionError(executor_code(e['reason'], 'MOTION_ABORTED'),
                                       f'{label} aborted: {e["reason"]}')
            time.sleep(0.05)
        raise MissionError('TIMEOUT', f'{label} timed out after {self.move_timeout:.0f} s')

    def approach(self, c):
        """Straight line pre-grasp -> grasp in steps of at most approach_step."""
        start, goal = c['pregrasp'][:3, 3], c['grasp'][:3, 3]
        n = max(1, math.ceil(np.linalg.norm(goal - start) / self.approach_step - 1e-9))
        for k in range(1, n + 1):
            pose = c['grasp'].copy()
            pose[:3, 3] = start + (goal - start) * k / n
            self.move(pose, f'approach {k}/{n}', straight=True)

    def refresh(self, c):
        """Fresh observation from pre-grasp; re-plan the same grasp from it."""
        info, old = self.camera_info, self.object_base.mean(0)
        try:
            cam = self.base_from(info.header.frame_id)  # camera -> base_link
        except MissionError as e:
            raise MissionError('LOST_TARGET', f'cannot project the target: {e}')
        p = np.linalg.inv(cam) @ np.append(old, 1.0)
        k = info.k
        u, v = k[0] * p[0] / p[2] + k[2], k[4] * p[1] / p[2] + k[5]
        if p[2] <= 0 or not (0 <= u < info.width and 0 <= v < info.height):
            raise MissionError('LOST_TARGET', 'target is outside the camera image')
        box = []
        self.reobs_wait = box
        # The segmenter uses its latest image, taken up to one frame before the
        # prompt; anything newer than the end of the last motion is fresh.
        requested = self.get_clock().now() - Duration(seconds=0.25)
        try:
            self.prompt_pub.publish(Point(x=float(u), y=float(v)))
            self.event('reobserve_requested', pixel=[u, v])
            end = time.monotonic() + self.reobserve_timeout
            while not any(o['stamp'] > requested for o in box):
                self.check()
                if time.monotonic() > end:
                    stale = [o['id'] for o in box]
                    raise MissionError('LOST_TARGET', 'no fresh observation from pre-grasp' +
                                       (f' (only older ones: {stale})' if stale else ''))
                time.sleep(0.05)
        finally:
            self.reobs_wait = None
        obs = next(o for o in box if o['stamp'] > requested)
        if not obs['valid'] or obs['confidence'] < self.min_confidence:
            raise MissionError('LOST_TARGET', f'fresh observation unusable: {obs["reason"] or "low confidence"}')
        shift = float(np.linalg.norm(obs['points'].mean(0) - old))
        self.event('reobserved', observation=obs['id'], shift_m=shift)
        if shift > self.max_target_shift:
            raise MissionError('TARGET_MOVED', f'target moved {shift * 100:.1f} cm since planning')
        self.object_base = obs['points']
        fresh = [x for x in gp.plan_grasps(self.object_base, self.scene_base(), self.offset,
                                    extra_opening=self.opening_extra())
                 if x['key'] == c['key']]
        if not fresh:
            raise MissionError('NO_FEASIBLE_GRASP', 'the fresh observation no longer admits the grasp')
        n = fresh[0]
        if np.linalg.norm(n['grasp'][:3, 3] - c['grasp'][:3, 3]) <= self.replan_threshold:
            return c
        ik_g, ik_p = self.solve(n['grasp']), self.solve(n['pregrasp'])
        why = '' if ik_g['ok'] and ik_p['ok'] else 'unreachable'
        if not why:
            n['achieved'] = ik_g['achieved']
            n['ik_residual'], why = self.geometry(n, ik_g, ik_p)
        if why:
            # The target stayed within max_target_shift: the planned grasp still
            # holds it, the refinement was only a few millimeters.
            self.event('refinement_skipped', reason=why, shift_m=shift)
            return c
        if n['aperture'] > c['aperture'] + 0.002:
            self.grip(n['aperture'])
        self.prepare_arm()
        self.move(n['pregrasp'], 'refined pre-grasp')
        return n

    def base_position(self):
        return np.linalg.inv(self.base_from('odom'))[:3, 3]

    def drive_base(self, dx):
        """Move dx along base X with the driver's relative `translate_mobile_base`
        (position mode). Velocity control in navigation mode leaves the wheel
        setpoints behind, and the base rolled back on the next switch to
        trajectory mode. The lidar (which does not see the arm) guards the path."""
        self.say(f'BASE: correcting {dx:+.3f} m along X')
        if self.dry_run:
            raise MissionError('DRY_RUN', 'stopping before the base correction')
        sign = 1.0 if dx > 0 else -1.0
        if self.blocked(sign):
            raise MissionError('BASE_CORRECTION_FAILED', 'lidar sees an obstacle in the base path')
        if not self.pos_mode.wait_for_service(timeout_sec=2.0):
            raise MissionError('MODE_SWITCH', 'position mode service unavailable')
        self.wait(self.pos_mode.call_async(Trigger.Request()), 5, 'position mode', 'MODE_SWITCH')
        time.sleep(0.5)
        x0, y0, _ = self.odom
        start_tf = self.base_from('odom')
        start = self.base_position()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['translate_mobile_base']
        point = JointTrajectoryPoint()
        point.positions = [float(dx)]
        goal.trajectory.points.append(point)
        if not self.trajectory.wait_for_server(timeout_sec=2.0):
            raise MissionError('NOT_READY', 'driver trajectory action unavailable')
        self.mission['motion'] = True
        handle = self.wait(self.trajectory.send_goal_async(goal), 5, 'base goal')
        if not handle.accepted:
            raise MissionError('CONTROLLER_REJECTED', 'driver rejected the base correction')
        self.handles.append(handle)
        done = handle.get_result_async()
        deadline = time.monotonic() + abs(dx) / 0.05 + 15
        while not done.done():
            self.check()
            if self.blocked(sign):
                raise MissionError('BASE_CORRECTION_FAILED', 'lidar sees an obstacle in the base path')
            if time.monotonic() > deadline:
                raise MissionError('TIMEOUT', 'base correction timed out')
            time.sleep(0.05)
        self.handles.remove(handle)
        time.sleep(0.5)
        x, y, _ = self.odom
        by_odom = math.hypot(x - x0, y - y0)
        by_tf = float(np.linalg.norm(self.base_position() - start))
        self.event('base_corrected', commanded=dx, by_odom=by_odom, by_tf=by_tf)
        if by_tf < 0.5 * abs(dx):
            raise MissionError('BASE_CORRECTION_FAILED',
                               f'base did not move enough (odometry {by_odom:.3f} m, '
                               f'TF {by_tf:.3f} m of {abs(dx):.3f} m)')
        # Carry the object into the moved base frame now: the switch to
        # trajectory mode below resets odometry.
        self.object_base = gp.apply(self.base_from('odom') @ np.linalg.inv(start_tf),
                                    self.object_base)
        self.prepare_arm()

    def blocked(self, sign):
        scan = self.scan
        if scan is None:
            return True
        try:
            m = gp.matrix_from_transform(self.tf.lookup_transform(
                'base_link', scan.header.frame_id, Time()).transform)
        except TransformException:
            return True
        r = np.asarray(scan.ranges)
        a = scan.angle_min + scan.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > scan.range_min)
        pts = gp.apply(m, np.column_stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)])[ok])
        back, front, half = self.footprint
        edge = front if sign > 0 else back
        ahead = (sign * (pts[:, 0] - edge) > 0) & (sign * (pts[:, 0] - edge) < self.clearance)
        return int((ahead & (np.abs(pts[:, 1]) < half)).sum()) >= 3

    def set_target_box(self, frame, center, size):
        m = Marker(type=Marker.CUBE, action=Marker.ADD)
        m.header.frame_id = frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in center]
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = [float(v) for v in size]
        self.box_pub.publish(m)

    def clear_target_box(self):
        self.box_pub.publish(Marker(action=Marker.DELETEALL))

    # -------------------------------------------------------------- planning
    def scene_base(self):
        scene = self.scene_odom
        return gp.apply(self.base_from(self.scene_frame), scene) if len(scene) else np.zeros((0, 3))

    def plan(self):
        if self.object_base is None:
            raise MissionError('TARGET_UNKNOWN', 'no object selected')
        obj = self.object_base
        size = obj.max(0) - obj.min(0)
        if size.max() > self.max_object_size:
            self.candidates = []
            self.publish_candidates()
            raise MissionError('PERCEPTION_INVALID',
                               f'selected region is {size.max():.2f} m long, not a single product; '
                               'click on the object itself')
        why, scene = [], self.scene_base()
        self.candidates = gp.plan_grasps(obj, scene, self.offset,
                                         extra_opening=self.opening_extra(), why=why)
        self.plan_why = why
        if not self.candidates:
            self.save_failed_plan(obj, scene, why)
        mode = self.get_parameter('grasp_mode').value
        if mode not in ('auto', 'side', 'top'):
            raise MissionError('NO_FEASIBLE_GRASP', f'grasp_mode must be auto, side or top, not {mode!r}')
        if mode != 'auto':
            self.candidates = [c for c in self.candidates if c['mode'] == mode]
        self.publish_candidates()
        if not self.candidates:
            raise MissionError('NO_FEASIBLE_GRASP', f'no collision-free {"" if mode == "auto" else mode + " "}'
                                                    'grasp fits the object: ' + ' | '.join(self.plan_why))

    def save_failed_plan(self, obj, scene, why):
        """Object and scene (base_link) of a failed plan, for offline replay."""
        try:
            folder = os.path.expanduser('~/grasp_debug')
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, time.strftime('%Y%m%d_%H%M%S') + '_plan_failed.npz')
            np.savez_compressed(path, object=obj, scene=scene, why=np.array(why))
            self.get_logger().info(f'Failed plan saved: {path}')
        except OSError as e:
            self.get_logger().warn(f'Could not save the failed plan: {e}')

    def publish_debug(self, c):
        """RViz: object frame, planned grasp and pre-grasp frames, FK-achieved
        TCP frame (red/green/blue = x approach / y fingers / z), approach line."""
        obj = np.eye(4)
        obj[:3, 3] = self.object_base.mean(0)
        frames = [('object', obj), ('grasp', c['grasp']), ('pregrasp', c['pregrasp'])]
        if 'achieved' in c:
            frames.append(('achieved_tcp', c['achieved']))
        array = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for ns, m in frames:
            for axis in range(3):
                a = Marker(type=Marker.ARROW, action=Marker.ADD, ns=ns, id=axis)
                a.header.frame_id = 'base_link'
                a.points = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in
                            (m[:3, 3], m[:3, 3] + 0.05 * m[:3, axis])]
                a.scale.x, a.scale.y = 0.004, 0.008
                a.color.a = 1.0
                a.color.r, a.color.g, a.color.b = [float(axis == k) for k in range(3)]
                array.markers.append(a)
        line = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='approach', id=0)
        line.header.frame_id = 'base_link'
        line.points = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in
                       (c['pregrasp'][:3, 3], c['grasp'][:3, 3])]
        line.scale.x = 0.003
        line.color.a = line.color.r = line.color.g = 1.0
        array.markers.append(line)
        self.debug_pub.publish(array)

    def opening_extra(self):
        if self.get_parameter('fully_open').value:
            return gp.MAX_APERTURE  # capped to MAX_APERTURE and neighbor clearance
        return self.get_parameter('extra_opening').value

    def publish_candidates(self):
        array = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, c in enumerate(self.candidates[:5]):
            m = Marker(type=Marker.ARROW, action=Marker.ADD, ns='grasp', id=i)
            m.header.frame_id = 'base_link'
            m.pose = pose_msg(c['pregrasp'], 'base_link').pose
            m.scale.x, m.scale.y, m.scale.z = self.offset, 0.01, 0.01
            m.color.a = 1.0 if i == 0 else 0.3
            m.color.b = 1.0
            array.markers.append(m)
        self.marker_pub.publish(array)

    def choose(self):
        """First candidate whose IK answers match the plan; at most three base
        corrections when only a moved base can reach."""
        center = self.object_base.mean(0)
        if center[1] > -0.25:
            raise MissionError('NO_FEASIBLE_GRASP',
                               f'object at x={center[0]:.2f} y={center[1]:.2f} m is not on the arm '
                               'side; park the robot with the shelf on its right')
        key = None
        for _ in range(3):
            self.plan()
            # Keep the grasp the last correction was computed for, so rounds
            # do not chase different candidates back and forth.
            cands = sorted(self.candidates, key=lambda c: c['key'] != key)
            correction, rejected, failed = None, [], []
            for c in cands:
                ik_g = self.solve(c['grasp'])
                ik_p = self.solve(c['pregrasp']) if ik_g['ok'] else None
                if not ik_g['ok'] or not ik_p['ok']:
                    which, a = ('grasp', ik_g) if not ik_g['ok'] else ('pre-grasp', ik_p)
                    failed.append(f'{c["mode"]}: {which} {a["reason"]}')
                    self.event('grasp_unreachable', key=c['key'], pose=which, reason=a['reason'],
                               base_dx=a['base_dx'], grasp=c['grasp'], pregrasp=c['pregrasp'])
                if ik_p and ik_p['ok']:
                    c['achieved'] = ik_g['achieved']
                    c['ik_residual'], why = self.geometry(c, ik_g, ik_p)
                    if not why:
                        return c
                    rejected.append(why)
                    self.event('grasp_rejected', key=c['key'], reason=why, residual=c['ik_residual'])
                    continue
                dx = ik_g['base_dx']
                if correction is None and math.isfinite(dx) and abs(dx) > self.dx_tolerance:
                    correction, key = dx, c['key']
            if correction is None:
                if rejected:
                    raise MissionError('GRASP_GEOMETRY', '; '.join(rejected))
                raise MissionError('IK_FAILED', self.unreachable_hint(cands) +
                                   ' (' + '; '.join(failed) + ')')
            if abs(correction) > self.max_dx:
                raise MissionError('BASE_CORRECTION_FAILED',
                                   f'base correction {correction:+.2f} m too large; re-navigate closer')
            self.stage('BASE_CORRECTION', f'{correction:+.3f} m')
            self.drive_base(correction)
            self.stage('SELECT_GRASP', 'after base correction')
        raise MissionError('BASE_CORRECTION_FAILED', 'base corrections did not converge')

    def unreachable_hint(self, cands):
        """Side grasps move the TCP along the arm (base_link -y): from y=-0.42 m
        (arm retracted) to y=-0.94 m (extended), so distance cannot be fixed
        by the base correction, which only moves along x."""
        side = [c for c in cands if c['mode'] == 'side']
        if not side:
            return 'no reachable grasp'
        near = max(c['pregrasp'][1, 3] for c in side)
        far = min(c['grasp'][1, 3] for c in side)
        if near > ARM_Y_RETRACTED - 0.005:
            return (f'object too close: the pre-grasp needs the arm shorter than it goes; park '
                    f'the robot about {(near - ARM_Y_RETRACTED) * 100 + 3:.0f} cm farther from it')
        if far < ARM_Y_EXTENDED + 0.005:
            return (f'object too far: park the robot about '
                    f'{(ARM_Y_EXTENDED - far) * 100 + 3:.0f} cm closer to it')
        return 'no reachable grasp, even with a base correction'

    def release(self):
        self.grip(0.10)
        self.clear_target_box()
        self.say('RELEASED')


def main():
    rclpy.init()
    node = GraspMission()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
