#!/usr/bin/env python3
"""Domain-20-only mock hardware input fixture; provides an action server for simulation."""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
import time
import math
import threading

from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.srv import GetParameters

class MockHardware(Node):
    def __init__(self):
        super().__init__('offline_seed', namespace='/ik_input')
        if os.environ.get('ROS_DOMAIN_ID') != '20':
            raise RuntimeError('Offline seed is restricted to ROS_DOMAIN_ID=20')
        self.joints_pub = self.create_publisher(JointState, '/ik_input/joint_states', 10)
        self.homed_pub = self.create_publisher(Bool, '/ik_input/is_homed', 10)
        self.runstop_pub = self.create_publisher(Bool, '/ik_input/is_runstopped', 10)
        for pub in (self.joints_pub, self.homed_pub, self.runstop_pub):
            if not pub.topic_name.startswith('/ik_input/'):
                raise RuntimeError('Unsafe fixture publisher remapping')
        
        self.joint_names = ['joint_lift', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1',
                            'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_pitch',
                            'joint_wrist_roll', 'joint_head_pan', 'joint_head_tilt',
                            'joint_right_wheel', 'joint_left_wheel',
                            'joint_gripper_finger_left', 'joint_gripper_finger_right']
        
        self.current_positions = [.4, .025, .025, .025, .025, 0., -.6, 0., 0., 0., 0., 0., 0., 0.]
        
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.stopped = threading.Event()
        self.runstopped = False
        
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory',
            self.execute_callback,
            goal_callback=self.accept_goal,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=self.cb_group
        )
        
        # Mock services required by guarded_executor when dry_run=false
        self.srv_stop = self.create_service(Trigger, '/stop_the_robot', self.dummy_trigger_cb, callback_group=self.cb_group)
        self.srv_runstop = self.create_service(SetBool, '/runstop', self.dummy_setbool_cb, callback_group=self.cb_group)
        self.srv_get_params = self.create_service(GetParameters, '/stretch_driver/get_parameters', self.dummy_get_params_cb, callback_group=self.cb_group)
        
        self.timer = self.create_timer(.033, self.publish_state, callback_group=self.cb_group)
        
    def dummy_trigger_cb(self, request, response):
        self.stopped.set()
        response.success = True
        response.message = "Mock stop"
        return response
        
    def dummy_setbool_cb(self, request, response):
        self.runstopped = request.data
        if request.data:
            self.stopped.set()
        response.success = True
        response.message = "Mock runstop"
        return response
        
    def dummy_get_params_cb(self, request, response):
        return response

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        self.joints_pub.publish(msg)
        self.homed_pub.publish(Bool(data=True))
        self.runstop_pub.publish(Bool(data=self.runstopped))

    def accept_goal(self, goal):
        trajectory = goal.trajectory
        names = trajectory.joint_names
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                 for p in trajectory.points]
        valid = (bool(names) and len(set(names)) == len(names)
                 and all(n in self.joint_names for n in names)
                 and len(times) >= 2 and times[0] == 0
                 and all(b > a for a, b in zip(times, times[1:])))
        valid = valid and all(len(p.positions) == len(names)
                              and all(math.isfinite(v) for v in p.positions)
                              for p in trajectory.points)
        with self.lock:
            if not valid or self.busy or self.runstopped:
                return GoalResponse.REJECT
            self.busy = True
            self.stopped.clear()
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        trajectory = goal_handle.request.trajectory
        
        # accept_goal has validated all names, dimensions and timestamps.
        joint_indices = [self.joint_names.index(name) for name in trajectory.joint_names]
                
        start_time = time.monotonic()
        final_point = trajectory.points[-1]
        target_time = final_point.time_from_start.sec + final_point.time_from_start.nanosec * 1e-9
        
        while True:
            if goal_handle.is_cancel_requested or self.stopped.is_set():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                with self.lock:
                    self.busy = False
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = 'Stopped by operator'
                return result
            elapsed = time.monotonic() - start_time
            if elapsed >= target_time:
                break
                
            # Follow the sampled trajectory, including its quintic timing.
            points = trajectory.points
            index = 1
            while index < len(points) - 1 and (
                    points[index].time_from_start.sec +
                    points[index].time_from_start.nanosec * 1e-9) < elapsed:
                index += 1
            left, right = points[index - 1], points[index]
            t0 = left.time_from_start.sec + left.time_from_start.nanosec * 1e-9
            t1 = right.time_from_start.sec + right.time_from_start.nanosec * 1e-9
            progress = min(1., max(0., (elapsed - t0) / (t1 - t0)))
            
            # Interpolate
            for i, pos in enumerate(right.positions):
                idx = joint_indices[i]
                if idx != -1:
                    start_pos = left.positions[i]
                    self.current_positions[idx] = start_pos + (pos - start_pos) * progress
            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = trajectory.joint_names
            feedback.actual.positions = [self.current_positions[i] for i in joint_indices]
            feedback.desired.positions = list(feedback.actual.positions)
            feedback.actual.time_from_start = rclpy.duration.Duration(seconds=elapsed).to_msg()
            feedback.desired.time_from_start = feedback.actual.time_from_start
            goal_handle.publish_feedback(feedback)
                    
            time.sleep(0.01)
            if not rclpy.ok():
                goal_handle.abort()
                self.busy = False
                return FollowJointTrajectory.Result()
                
        # Set final positions exactly
        for i, pos in enumerate(final_point.positions):
            idx = joint_indices[i]
            if idx != -1:
                self.current_positions[idx] = pos
                
        self.publish_state()
        goal_handle.succeed()
        with self.lock:
            self.busy = False
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MockHardware()
    try:
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
