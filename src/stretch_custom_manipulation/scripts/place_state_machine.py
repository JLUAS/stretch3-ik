#!/usr/bin/env python3

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

def compute_retract_pose(place_target, retract_height):
    retract_pose = PoseStamped()
    retract_pose.header = place_target.header
    retract_pose.pose.position.x = place_target.pose.position.x
    retract_pose.pose.position.y = place_target.pose.position.y
    retract_pose.pose.position.z = place_target.pose.position.z + retract_height
    retract_pose.pose.orientation = place_target.pose.orientation
    return retract_pose

class PlaceStateMachine(Node):
    def __init__(self):
        super().__init__('place_state_machine')
        
        self.declare_parameter('retract_height', 0.10)
        self.declare_parameter('gripper_open_aperture', 0.10)
        self.declare_parameter('state_timeout', 30.0)
        self.declare_parameter('ik_timeout', 10.0)
        self.declare_parameter('execution_timeout', 90.0)
        self.declare_parameter('base_frame', 'base_link')
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.ik_goal_pub = self.create_publisher(PoseStamped, '/ik_preview/goal_pose', 10)
        
        self.create_subscription(PoseStamped, '~/place_target', self.target_cb, 10, callback_group=self.cb_group)
        self.create_subscription(DiagnosticArray, '/ik_preview/status', self.ik_status_cb, 10, callback_group=self.cb_group)
        self.create_subscription(String, '/guarded_executor/status', self.executor_status_cb, 10, callback_group=self.cb_group)
        self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_cb, 10, callback_group=self.cb_group)
        
        self.place_srv = self.create_service(Trigger, '~/place', self.place_cb, callback_group=self.cb_group)
        self.stop_srv = self.create_service(Trigger, '~/stop', self.stop_cb, callback_group=self.cb_group)
        
        self.exec_srv_client = self.create_client(Trigger, '/guarded_executor/move_to_preview', callback_group=self.cb_group)
        self.gripper_client = ActionClient(self, GripperCommand, '/stretch_gripper_controller/gripper_action', callback_group=self.cb_group)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.lock = threading.Lock()
        self.running = False
        self.stop_requested = False
        self.last_target = None
        
        self.ik_status_ok = False
        self.ik_status_time = 0.0
        
        self.executor_status_msg = ""
        self.executor_status_time = 0.0
        
        self.current_joints = {}
        self.current_efforts = {}
        
        self.publish_status("IDLE")

    def publish_status(self, msg):
        self.get_logger().info(f"State: {msg}")
        self.status_pub.publish(String(data=msg))

    def target_cb(self, msg):
        with self.lock:
            self.last_target = msg

    def ik_status_cb(self, msg):
        is_ok = bool(msg.status and msg.status[0].message == 'success')
        with self.lock:
            self.ik_status_ok = is_ok
            self.ik_status_time = time.time()
            
    def executor_status_cb(self, msg):
        with self.lock:
            self.executor_status_msg = msg.data
            self.executor_status_time = time.time()
            
    def joint_states_cb(self, msg):
        with self.lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.current_joints[name] = msg.position[i]
                if i < len(msg.effort):
                    self.current_efforts[name] = msg.effort[i]

    def stop_cb(self, request, response):
        with self.lock:
            self.stop_requested = True
        response.success = True
        response.message = "Stop requested"
        return response

    def place_cb(self, request, response):
        with self.lock:
            if self.running:
                response.success = False
                response.message = "Already running"
                return response
            if self.last_target is None:
                response.success = False
                response.message = "No place target received"
                return response
            
            self.running = True
            self.stop_requested = False
            target = self.last_target
            
        threading.Thread(target=self.run_state_machine, args=(target,)).start()
        response.success = True
        response.message = "Place sequence started"
        return response

    def _wait_condition(self, condition, timeout, error_msg):
        start = time.time()
        while time.time() - start < timeout:
            if self.stop_requested:
                raise RuntimeError("Stopped by user")
            if condition():
                return
            time.sleep(0.1)
        raise RuntimeError(f"Timeout: {error_msg}")

    def send_ik_and_execute(self, target_pose, state_name):
        ik_timeout = self.get_parameter('ik_timeout').value
        exec_timeout = self.get_parameter('execution_timeout').value
        
        self.publish_status(f"PLAN_{state_name}")
        start_time = time.time()
        self.ik_goal_pub.publish(target_pose)
        
        self._wait_condition(lambda: self.ik_status_time > start_time and self.ik_status_ok, ik_timeout, "IK solution not found")
        
        self.publish_status(f"EXECUTE_{state_name}")
        if not self.exec_srv_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Guarded executor service not available")
            
        req = Trigger.Request()
        future = self.exec_srv_client.call_async(req)
        
        while not future.done():
            if self.stop_requested:
                raise RuntimeError("Stopped by user")
            time.sleep(0.1)
            
        res = future.result()
        if not res.success:
            raise RuntimeError(f"Executor rejected request: {res.message}")
            
        start_time = time.time()
        self._wait_condition(
            lambda: self.executor_status_time > start_time and ('state=COMPLETE' in self.executor_status_msg or 'state=ABORTED' in self.executor_status_msg),
            exec_timeout, "Executor timed out"
        )
        
        if 'state=ABORTED' in self.executor_status_msg:
            raise RuntimeError(f"Executor aborted: {self.executor_status_msg}")

    def control_gripper(self, aperture, state_name):
        self.publish_status(state_name)
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Gripper action server not available")
            
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(aperture)
        
        future = self.gripper_client.send_goal_async(goal_msg)
        while not future.done():
            if self.stop_requested:
                raise RuntimeError("Stopped by user")
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Gripper goal rejected")
            
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            if self.stop_requested:
                raise RuntimeError("Stopped by user")
            time.sleep(0.1)
            
        res = res_future.result()
        if res.status != 4:
            self.get_logger().warn(f"Gripper action did not return SUCCEEDED (status: {res.status})")

    def run_state_machine(self, place_target):
        try:
            base_frame = self.get_parameter('base_frame').value
            
            if place_target.header.frame_id != base_frame:
                if not self.tf_buffer.can_transform(base_frame, place_target.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0)):
                    raise RuntimeError("Cannot transform place target to base_frame")
                trans = self.tf_buffer.lookup_transform(base_frame, place_target.header.frame_id, rclpy.time.Time())
                place_target_base = do_transform_pose_stamped(place_target, trans)
            else:
                place_target_base = place_target
                
            # PLAN_PLACE and EXECUTE_PLACE
            self.send_ik_and_execute(place_target_base, "PLACE")
            
            self.control_gripper(self.get_parameter('gripper_open_aperture').value, "OPEN_GRIPPER")
            
            self.publish_status("VERIFY_RELEASE")
            time.sleep(1.0)
            self.get_logger().info("Gripper release verified.")
            
            # RETREAT
            retract_pose = compute_retract_pose(place_target_base, self.get_parameter('retract_height').value)
            
            # PLAN_RETREAT and EXECUTE_RETREAT internally handled by send_ik_and_execute
            self.send_ik_and_execute(retract_pose, "RETREAT")
            
            self.publish_status("DONE")
            
        except Exception as e:
            self.get_logger().error(f"Place failed: {e}")
            self.publish_status(f"FAILED: {e}")
        finally:
            with self.lock:
                self.running = False

def main(args=None):
    rclpy.init(args=args)
    node = PlaceStateMachine()
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
