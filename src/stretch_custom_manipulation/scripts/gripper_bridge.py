#!/usr/bin/env python3
"""GripperCommand server for the real Stretch gripper.

The pick logic speaks control_msgs/GripperCommand in meters of aperture; the
Stretch driver only offers FollowJointTrajectory. Goals go in position mode
as a single `gripper_aperture` point: trajectory mode rejects a gripper
trajectory whose first point does not match the servo exactly. The guarded
executor switches back to trajectory mode before the next arm motion.
Success is judged on the measured aperture: closing on an object ends in a
driver "contact" error, which is a grasp, while an open that falls short is
a failure. A negative aperture squeezes a grasped object.
"""
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

FINGER_LENGTH = 0.171  # hello_helpers GripperConversion
JOINT = 'joint_gripper_finger_left'
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
          GoalStatus.STATUS_CANCELING)


def finger_to_aperture(finger):
    return 2.0 * finger * FINGER_LENGTH


class GripperBridge(Node):
    def __init__(self):
        super().__init__('gripper_bridge')
        self.min_aperture = self.declare_parameter('min_aperture', -0.10).value
        self.max_aperture = self.declare_parameter('max_aperture', 0.13).value
        self.speed = self.declare_parameter('finger_speed', 0.6).value  # rad/s
        self.tolerance = self.declare_parameter('goal_tolerance', 0.01).value
        action = self.declare_parameter(
            'trajectory_action', '/stretch_controller/follow_joint_trajectory').value
        group = ReentrantCallbackGroup()
        self.finger = None
        self.finger_time = 0.0
        self.arm_busy = False
        self.mode = ''
        self.create_subscription(String, '/mode', lambda m: setattr(self, 'mode', m.data),
                                 10, callback_group=group)
        self.position_mode = self.create_client(Trigger, '/switch_to_position_mode',
                                                callback_group=group)
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/stretch/joint_states', self.on_joints,
                                 10, callback_group=group)
        self.create_subscription(GoalStatusArray, action + '/_action/status',
                                 self.on_status, 10, callback_group=group)
        self.client = ActionClient(self, FollowJointTrajectory, action,
                                   callback_group=group)
        self.server = ActionServer(
            self, GripperCommand, '/stretch_gripper_controller/gripper_action',
            self.execute, goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

    def on_joints(self, msg):
        if JOINT in msg.name:
            with self.lock:
                self.finger = msg.position[msg.name.index(JOINT)]
                self.finger_time = time.monotonic()

    def on_status(self, msg):
        # The driver serves one trajectory at a time: a gripper goal would
        # preempt an executing arm trajectory, so refuse while one is active.
        self.arm_busy = any(s.status in ACTIVE for s in msg.status_list)

    def on_goal(self, goal):
        p = goal.command.position
        if not (self.min_aperture <= p <= self.max_aperture):
            self.get_logger().error(f'Rejected aperture {p:.3f} m')
            return GoalResponse.REJECT
        if self.arm_busy:
            self.get_logger().error('Rejected gripper goal: trajectory active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def measured(self):
        with self.lock:
            if self.finger is None or time.monotonic() - self.finger_time > 0.5:
                return None
            return self.finger

    def wait(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def execute(self, handle):
        result = GripperCommand.Result()
        start = self.measured()
        if start is None or not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper state or driver action unavailable')
            handle.abort()
            return result
        if self.mode != 'position':
            if not self.position_mode.wait_for_service(timeout_sec=2.0) or \
                    self.wait(self.position_mode.call_async(Trigger.Request()), 5) is None:
                self.get_logger().error('Could not switch to position mode')
                handle.abort()
                return result
            end = time.monotonic() + 3.0
            while self.mode != 'position' and time.monotonic() < end:
                time.sleep(0.05)
        target = handle.request.command.position
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_aperture']
        point = JointTrajectoryPoint()
        point.positions = [float(target)]
        goal.trajectory.points.append(point)
        driver = self.wait(self.client.send_goal_async(goal), 5)
        if driver is None or not driver.accepted:
            self.get_logger().error('Driver rejected the gripper goal')
            handle.abort()
            return result
        done = driver.get_result_async()
        end = time.monotonic() + 15.0
        while not done.done() and time.monotonic() < end:
            if handle.is_cancel_requested:
                driver.cancel_goal_async()
                handle.canceled()
                return result
            time.sleep(0.05)
        time.sleep(0.3)  # let the fingers settle on the object
        end_finger = self.measured()
        if end_finger is None:
            handle.abort()
            return result
        result.position = finger_to_aperture(end_finger)
        result.reached_goal = abs(result.position - target) <= self.tolerance
        result.stalled = not result.reached_goal and target < result.position
        opening = target > finger_to_aperture(start) + self.tolerance
        if opening and not result.reached_goal:
            self.get_logger().error(
                f'Gripper did not open: {result.position:.3f} m of {target:.3f} m')
            handle.abort()
            return result
        handle.succeed()
        return result


def main():
    rclpy.init()
    node = GripperBridge()
    executor = MultiThreadedExecutor(num_threads=4)
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
