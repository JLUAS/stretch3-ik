#!/usr/bin/env python3
"""Domain-20-only mock hardware gripper fixture; provides an action server for simulation."""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import GripperCommand
import time
import math
import threading

from std_srvs.srv import Trigger
from stretch_custom_manipulation.msg import GripperState

class MockGripper(Node):
    def __init__(self):
        super().__init__('sim_gripper')
        if os.environ.get('ROS_DOMAIN_ID') != '20':
            raise RuntimeError('sim_gripper is restricted to ROS_DOMAIN_ID=20 (simulation)')
            
        self.state_pub = self.create_publisher(GripperState, '/gripper/state', 10)
        self.joints_pub = self.create_publisher(JointState, '/gripper/joint_states', 10)
        
        self.joint_names = ['joint_gripper_finger_left', 'joint_gripper_finger_right']
        
        # Current logical aperture (meters, 0.0 to 0.13)
        self.current_aperture = 0.13
        self.target_aperture = 0.13
        self.object_detected = False
        self.is_grasping = False
        
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        
        self.action_server = ActionServer(
            self,
            GripperCommand,
            '/stretch_gripper_controller/gripper_action',
            self.execute_callback,
            goal_callback=self.accept_goal,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=self.cb_group
        )
        
        self.srv_open = self.create_service(Trigger, '/gripper/open_full', self.open_full_cb, callback_group=self.cb_group)
        self.srv_close = self.create_service(Trigger, '/gripper/close_full', self.close_full_cb, callback_group=self.cb_group)
        
        self.timer = self.create_timer(0.033, self.publish_state, callback_group=self.cb_group)

    def aperture_to_raw(self, aperture):
        # Map [0, 0.13] to [-100, 200]
        return -100.0 + (aperture / 0.13) * 300.0
        
    def open_full_cb(self, request, response):
        self.target_aperture = 0.13
        self.current_aperture = 0.13
        self.object_detected = False
        self.is_grasping = False
        response.success = True
        response.message = "Opened"
        return response
        
    def close_full_cb(self, request, response):
        self.target_aperture = 0.0
        self.current_aperture = 0.0
        self.object_detected = True
        self.is_grasping = True
        response.success = True
        response.message = "Closed"
        return response
        
    def publish_state(self):
        now = self.get_clock().now().to_msg()
        
        # Publish GripperState
        state_msg = GripperState()
        state_msg.header.stamp = now
        state_msg.aperture = self.current_aperture
        state_msg.effort = -10.0 if self.is_grasping else 0.0
        state_msg.object_detected = self.object_detected
        state_msg.is_grasping = self.is_grasping
        state_msg.raw_position = self.aperture_to_raw(self.current_aperture)
        self.state_pub.publish(state_msg)
        
        # Publish JointState
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = self.joint_names
        # Approximate finger joints as taking half the aperture each? 
        # Or standard Stretch is angle in radians. Let's just use aperture / 2.0 or 
        # linear mapping to radians. Let's provide aperture/2 for now.
        half_aperture = self.current_aperture / 2.0
        joint_msg.position = [half_aperture, half_aperture]
        self.joints_pub.publish(joint_msg)

    def accept_goal(self, goal_request):
        with self.lock:
            if self.busy:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing gripper goal...')
        target_position = goal_handle.request.command.position
        
        # Assuming target_position is in radians or meters from real driver, 
        # but let's clamp it to [0, 0.13] as our aperture
        target_position = max(0.0, min(0.13, target_position))
        
        start_time = time.monotonic()
        duration = 1.0 # 1 second simulated motion
        start_aperture = self.current_aperture
        
        # Simulate object detection if closing past a threshold
        will_detect_object = False
        final_aperture = target_position
        if start_aperture > target_position and target_position < 0.02:
            will_detect_object = True
            # Stop earlier to simulate grasping an object
            final_aperture = 0.03
        
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with self.lock:
                    self.busy = False
                result = GripperCommand.Result()
                result.position = self.current_aperture
                result.effort = 0.0
                result.stalled = False
                result.reached_goal = False
                return result
                
            elapsed = time.monotonic() - start_time
            if elapsed >= duration:
                break
                
            progress = elapsed / duration
            self.current_aperture = start_aperture + (final_aperture - start_aperture) * progress
            
            feedback = GripperCommand.Feedback()
            feedback.position = self.current_aperture
            feedback.effort = 0.0
            feedback.stalled = False
            feedback.reached_goal = False
            goal_handle.publish_feedback(feedback)
            
            time.sleep(0.033)
            if not rclpy.ok():
                goal_handle.abort()
                self.busy = False
                return GripperCommand.Result()
                
        self.current_aperture = final_aperture
        self.object_detected = will_detect_object
        self.is_grasping = will_detect_object
        
        goal_handle.succeed()
        with self.lock:
            self.busy = False
            
        result = GripperCommand.Result()
        result.position = self.current_aperture
        result.effort = -10.0 if self.is_grasping else 0.0
        result.stalled = self.object_detected
        result.reached_goal = not self.object_detected
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MockGripper()
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
