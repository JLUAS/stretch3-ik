#!/usr/bin/env python3
"""Kinematic stand-in for stretch_driver. Isolated tests only.

Refuses to start outside ROS domain 20 or when any non-loopback network
interface exists (run it through ws/scripts/sim_isolated.sh).

Mirrors the driver behaviors the grasp mission depends on, as read from the
mission/executor/bridge code and docs/GRASPING_PLAN.md:
- modes navigation/position/trajectory; odometry resets to zero on every
  switch to trajectory mode; the arm settles a few millimeters after a switch
  (`settle_mm` over `settle_s`, an assumption: the real profile is unmeasured);
- trajectory mode checks that the first point matches the current state
  (fail_if_motor_initial_point_is_not_trajectory_first_point);
- `gripper_aperture` and `translate_mobile_base` single-point goals in
  position mode; head goals in any mode; one goal at a time.

World model (kinematic, no contact physics): one upright cylinder. A
fingertip entering it during an arm motion pushes it 2 cm (grasp then
misses); closing captures it when it sits between the fingers.
World and fault settings arrive as JSON on /mock/world (latched).
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

ARM = ['joint_lift', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0',
       'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
HEAD = ['joint_head_pan', 'joint_head_tilt']
FINGER_LENGTH = 0.171


def isolated():
    links = [n for n in os.listdir('/sys/class/net') if n != 'lo']
    return os.environ.get('ROS_DOMAIN_ID') == '20' and not links


def seconds(d):
    return d.sec + d.nanosec * 1e-9


class MockDriver(Node):
    def __init__(self):
        super().__init__('stretch_driver')
        if not isolated():
            raise RuntimeError('mock driver runs only in an isolated domain-20 namespace')
        self.declare_parameter('fail_if_motor_initial_point_is_not_trajectory_first_point', True)
        g = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.q = {n: 0.0 for n in ARM + HEAD + ['joint_gripper_finger_left',
                                                 'joint_gripper_finger_right',
                                                 'joint_left_wheel', 'joint_right_wheel']}
        self.world = {}
        self.events = []
        self.reset({'arm': [0.6, 0.0, 3.0, -0.2, 0.0], 'odom': [0, 0, 0], 'mode': 'navigation'})
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mock/world', self.on_world, latched, callback_group=g)
        pub = self.create_publisher
        self.js_pub = pub(JointState, '/stretch/joint_states', 10)
        self.homed_pub = pub(Bool, '/is_homed', 10)
        self.runstop_pub = pub(Bool, '/is_runstopped', 10)
        self.mode_pub = pub(String, '/mode', 10)
        self.stream_pub = pub(Bool, '/is_streaming_position', 10)
        self.diag_pub = pub(DiagnosticArray, '/diagnostics', 10)
        self.odom_pub = pub(Odometry, '/odom', 10)
        self.scan_pub = pub(LaserScan, '/scan', 10)
        self.truth_pub = pub(String, '/mock/truth', 10)
        self.event_pub = pub(String, '/mock/events', 50)
        self.tfb = TransformBroadcaster(self)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        srv = self.create_service
        for mode in ('trajectory', 'position', 'navigation'):
            srv(Trigger, f'/switch_to_{mode}_mode',
                lambda q, r, m=mode: self.switch(m, r), callback_group=g)
        srv(Trigger, '/stop_the_robot', self.on_stop, callback_group=g)
        srv(SetBool, '/runstop', self.on_runstop, callback_group=g)
        ActionServer(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory',
                     self.execute, goal_callback=self.on_goal,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=g)
        self.create_timer(1 / 30, self.publish_state, callback_group=g)
        self.create_timer(0.1, self.publish_diag, callback_group=g)

    # ------------------------------------------------------------- world
    def reset(self, w):
        with self.lock:
            lift, ext, yaw, pitch, roll = w.get('arm', [0.6, 0.0, 3.0, -0.2, 0.0])
            self.q['joint_lift'] = lift
            for n in ARM[1:5]:
                self.q[n] = ext / 4
            self.q['joint_wrist_yaw'], self.q['joint_wrist_pitch'], self.q['joint_wrist_roll'] = yaw, pitch, roll
            self.q['joint_head_pan'], self.q['joint_head_tilt'] = 0.0, -0.6
            self.set_aperture(w.get('aperture', 0.0))
            x, y, th = w.get('odom', [0, 0, 0])
            self.base = np.array([x, y, th], float)       # base pose in world
            self.odom_origin = np.zeros(3)                  # odom frame in world
            self.mode = w.get('mode', 'navigation')
            self.busy = False
            self.stop = threading.Event()
            self.faults = w.get('faults', {})
            self.runstopped = bool(self.faults.get('runstopped', False))
            self.switches = 0
            self.diag_error_until = 0.0
            self.settle_until = 0.0
            self.settle_rate = 0.0
            self.faults = w.get('faults', {})
            obj = w.get('object')
            # Object given in base_link at reset; kept in world from then on.
            self.obj = None if obj is None else {
                'c': self.to_world(np.array(obj['center'], float)),
                'r': float(obj['radius']), 'h': float(obj['height']),
                'held': False, 'pushed': False}
            self.neighbors = [dict(n, c=self.to_world(np.array(n['center'], float)).tolist())
                              for n in w.get('neighbors', [])]
            self.world = w

    def on_world(self, msg):
        self.reset(json.loads(msg.data))
        self.event('world_reset')

    def to_world(self, p_base):
        x, y, th = self.base
        c, s = math.cos(th), math.sin(th)
        return np.array([x + c * p_base[0] - s * p_base[1], y + s * p_base[0] + c * p_base[1], p_base[2]])

    def odom_pose(self):
        ox, oy, oth = self.odom_origin
        dx, dy = self.base[0] - ox, self.base[1] - oy
        c, s = math.cos(-oth), math.sin(-oth)
        return c * dx - s * dy, s * dx + c * dy, self.base[2] - oth

    def world_to_odom(self, p):
        ox, oy, oth = self.odom_origin
        c, s = math.cos(-oth), math.sin(-oth)
        return [c * (p[0] - ox) - s * (p[1] - oy), s * (p[0] - ox) + c * (p[1] - oy), p[2]]

    def event(self, kind, **kw):
        kw.update(kind=kind, t=time.monotonic())
        self.event_pub.publish(String(data=json.dumps(kw)))

    # ------------------------------------------------------------- state
    def aperture(self):
        return 2 * self.q['joint_gripper_finger_left'] * FINGER_LENGTH

    def set_aperture(self, a):
        self.q['joint_gripper_finger_left'] = self.q['joint_gripper_finger_right'] = a / (2 * FINGER_LENGTH)

    def publish_state(self):
        now = self.get_clock().now()
        with self.lock:
            if time.monotonic() < self.settle_until:
                self.q['joint_lift'] -= self.settle_rate / 30
            js = JointState(name=list(self.q), position=[float(v) for v in self.q.values()])
            x, y, th = self.odom_pose()
            mode, runstopped = self.mode, self.runstopped
            moving_base = getattr(self, 'base_moving', False)
        js.header.stamp = now.to_msg()
        self.js_pub.publish(js)
        self.homed_pub.publish(Bool(data=True))
        self.runstop_pub.publish(Bool(data=runstopped))
        self.mode_pub.publish(String(data=mode))
        self.stream_pub.publish(Bool(data=False))
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        odom.twist.twist.linear.x = 0.05 if moving_base else 0.0
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = odom.pose.pose.orientation
        self.tfb.sendTransform(t)
        with self.lock:
            obj = self.obj
            truth = None if obj is None else {
                'stamp_ns': now.nanoseconds, 'frame': 'odom', 'base': [x, y, th],
                'center': self.world_to_odom(obj['c']), 'radius': obj['r'], 'height': obj['h'],
                'held': obj['held'], 'pushed': obj['pushed'],
                'neighbors': [dict(n, center=self.world_to_odom(n['c'])) for n in self.neighbors]}
        if truth:
            self.truth_pub.publish(String(data=json.dumps(truth)))

    def publish_diag(self):
        a = DiagnosticArray()
        a.header.stamp = self.get_clock().now().to_msg()
        error = time.monotonic() < self.diag_error_until
        a.status.append(DiagnosticStatus(
            level=DiagnosticStatus.ERROR if error else DiagnosticStatus.OK,
            name='stretch_driver: Lift/mock', message='injected transient' if error else 'OK'))
        self.diag_pub.publish(a)
        scan = LaserScan(angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi / 180,
                         range_min=0.15, range_max=12.0, ranges=[math.inf] * 360)
        scan.header.frame_id = 'base_link'
        scan.header.stamp = a.header.stamp
        self.scan_pub.publish(scan)

    # ---------------------------------------------------------- services
    def switch(self, mode, response):
        delay = self.faults.get('mode_switch_delay', 0.0)
        time.sleep(delay)
        with self.lock:
            changed = self.mode != mode
            self.mode = mode
            if mode == 'trajectory':
                self.odom_origin = self.base.copy()  # stretch_body first_step
            if changed and mode == 'trajectory':
                self.switches += 1
                blip = self.faults.get('diag_error_after_switch')
                if blip and blip['n'] == self.switches:
                    self.diag_error_until = time.monotonic() + blip['duration']
            if changed and self.faults.get('settle_mm', 0) > 0:
                dur = self.faults.get('settle_s', 1.5)
                self.settle_rate = self.faults['settle_mm'] / 1000 / dur
                self.settle_until = time.monotonic() + dur
        self.event('mode', mode=mode)
        response.success = True
        response.message = f'switched to {mode} mode'
        return response

    def on_stop(self, _, response):
        self.stop.set()
        self.event('stop')
        response.success = True
        return response

    def on_runstop(self, request, response):
        with self.lock:
            self.runstopped = request.data
        if request.data:
            self.stop.set()
        self.event('runstop', value=request.data)
        response.success = True
        return response

    # ------------------------------------------------------------ action
    def on_goal(self, goal):
        t = goal.trajectory
        names, pts = list(t.joint_names), t.points
        with self.lock:
            reason = None
            if self.runstopped:
                reason = 'runstopped'
            elif self.busy:
                reason = 'busy'
            elif not names or not pts:
                reason = 'empty'
            elif set(names) <= set(ARM):
                if self.mode == 'navigation':
                    reason = 'arm goal in navigation mode'
                elif self.mode == 'position' and len(pts) != 1:
                    reason = 'multi-point goal needs trajectory mode'
                elif self.mode == 'trajectory':
                    if seconds(pts[0].time_from_start) != 0.0:
                        reason = 'first point not at t=0'
                    else:
                        err = max(abs(p - self.q[n]) for n, p in zip(names, pts[0].positions))
                        if err > 0.004:
                            reason = f'first point differs from current state by {err:.4f}'
            elif set(names) <= set(HEAD):
                pass
            elif names in (['gripper_aperture'], ['translate_mobile_base']):
                if self.mode != 'position' or len(pts) != 1:
                    reason = f'{names[0]} needs one point in position mode'
            else:
                reason = f'unsupported joints {names}'
            if reason is None:
                self.busy = True
                self.stop.clear()
        if reason:
            self.event('goal_rejected', joints=names, reason=reason)
            return GoalResponse.REJECT
        self.event('goal_accepted', joints=names, points=len(pts))
        return GoalResponse.ACCEPT

    def execute(self, handle):
        t = handle.request.trajectory
        names = list(t.joint_names)
        result = FollowJointTrajectory.Result()
        try:
            if self.faults.pop('hang_next_arm_goal', False) and names[0] == 'joint_lift':
                self.event('hang')
                while not (handle.is_cancel_requested or self.stop.is_set()):
                    time.sleep(0.02)
                handle.abort() if not handle.is_cancel_requested else handle.canceled()
                return result
            if names == ['gripper_aperture']:
                return self.gripper(handle, t.points[0].positions[0], result)
            if names == ['translate_mobile_base']:
                return self.translate(handle, t.points[0].positions[0], result)
            pts = t.points
            if len(pts) == 1:
                start = [self.q[n] for n in names]
                times, poss = [0.0, max(1.0, seconds(pts[0].time_from_start))], [start, list(pts[0].positions)]
            else:
                times = [seconds(p.time_from_start) for p in pts]
                poss = [list(p.positions) for p in pts]
            t0 = time.monotonic()
            while True:
                if handle.is_cancel_requested or self.stop.is_set():
                    self.event('motion_stopped', joints=names)
                    handle.canceled() if handle.is_cancel_requested else handle.abort()
                    result.error_code = result.PATH_TOLERANCE_VIOLATED
                    return result
                el = time.monotonic() - t0
                done = el >= times[-1]
                i = min(max(1, int(np.searchsorted(times, el))), len(times) - 1)
                u = 1.0 if done else min(1.0, max(0.0, (el - times[i - 1]) / (times[i] - times[i - 1])))
                with self.lock:
                    for k, n in enumerate(names):
                        self.q[n] = poss[i - 1][k] + u * (poss[i][k] - poss[i - 1][k])
                fb = FollowJointTrajectory.Feedback()
                fb.joint_names = names
                fb.actual.positions = [float(self.q[n]) for n in names]
                fb.desired.positions = list(fb.actual.positions)
                handle.publish_feedback(fb)
                if names[0] == 'joint_lift':
                    self.check_fingers()
                if done:
                    break
                time.sleep(1 / 60)
            handle.succeed()
            result.error_code = result.SUCCESSFUL
            return result
        finally:
            with self.lock:
                self.busy = False

    def lookup(self, frame):
        try:
            t = self.tf.lookup_transform('odom', frame, Time())
        except Exception:
            return None
        tr, r = t.transform.translation, t.transform.rotation
        x, y, z, w = r.x, r.y, r.z, r.w
        m = np.eye(4)
        m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        m[:3, 3] = [tr.x, tr.y, tr.z]
        return m

    def obj_odom(self):
        return np.array(self.world_to_odom(self.obj['c']))

    def check_fingers(self):
        """A fingertip inside the free object pushes it (kinematic contact)."""
        obj = self.obj
        if obj is None or obj['held'] or obj['pushed']:
            return
        c = self.obj_odom()
        for link in ('link_gripper_fingertip_left', 'link_gripper_fingertip_right'):
            m = self.lookup(link)
            if m is None:
                continue
            p = m[:3, 3]
            d = np.linalg.norm(p[:2] - c[:2])
            if d < obj['r'] + 0.008 and abs(p[2] - c[2]) < obj['h'] / 2:
                push = (c[:2] - p[:2]) / max(d, 1e-6) * 0.02
                with self.lock:
                    obj['c'][:2] += push
                    obj['pushed'] = True
                self.event('object_pushed', link=link, depth=float(obj['r'] + 0.008 - d))
                return

    def gripper(self, handle, target, result):
        start = self.aperture()
        stop_at = None
        obj = self.obj
        info = {}
        if obj is not None and target < start:
            m = self.lookup('link_grasp_center')
            if m is not None and not obj['held']:
                p = (np.linalg.inv(m) @ np.append(self.obj_odom(), 1.0))[:3]
                info = {'object_in_tcp': p.round(4).tolist(), 'aperture': start}
                # Captured when it sits between the finger pads: the tips (URDF:
                # ~1.7 cm short of link_grasp_center) must reach past its widest
                # line, and it must be in front of the gripper body.
                if m[2, 0] < -0.7:  # top-down: the can's axis runs along the approach
                    inside = (abs(p[1]) + obj['r'] <= start / 2 + 0.002 and
                              abs(p[2]) <= obj['r'] and
                              p[0] - obj['h'] / 2 <= -0.017 - 0.01)  # tips 1 cm below its top
                else:
                    inside = (abs(p[1]) + obj['r'] <= start / 2 + 0.002 and
                              -0.12 <= p[0] <= -0.017 + 0.3 * obj['r'] and
                              abs(p[2]) <= obj['h'] / 2 - 0.01)
                if inside and not self.faults.get('gripper_stuck'):
                    stop_at = 2 * obj['r']
                    with self.lock:
                        obj['held'] = True
            elif obj['held']:
                if self.faults.pop('slip', False):
                    with self.lock:
                        obj['held'] = False
                else:
                    stop_at = 2 * obj['r']
        if self.faults.get('gripper_stuck'):
            target = start
        goal = target if stop_at is None else max(target, stop_at)
        steps = 20
        for k in range(1, steps + 1):
            if handle.is_cancel_requested or self.stop.is_set():
                handle.canceled() if handle.is_cancel_requested else handle.abort()
                return result
            with self.lock:
                self.set_aperture(start + (goal - start) * k / steps)
            time.sleep(0.02)
        self.event('gripper', target=target, final=self.aperture(), held=bool(obj and obj['held']), **info)
        if stop_at is not None and target < stop_at:
            handle.abort()  # the real driver reports a contact error
        else:
            handle.succeed()
        return result

    def translate(self, handle, dx, result):
        with self.lock:
            self.base_moving = True
        steps = max(1, int(abs(dx) / 0.05 * 30))
        x0 = self.base.copy()
        for k in range(1, steps + 1):
            if handle.is_cancel_requested or self.stop.is_set():
                break
            with self.lock:
                f = k / steps
                self.base[0] = x0[0] + math.cos(x0[2]) * dx * f
                self.base[1] = x0[1] + math.sin(x0[2]) * dx * f
            time.sleep(1 / 30)
        with self.lock:
            self.base_moving = False
        self.event('base', dx=dx)
        handle.succeed()
        return result


def main():
    rclpy.init()
    node = MockDriver()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
