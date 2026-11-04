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

def compute_pregrasp_pose(grasp_target, offset):
    x = grasp_target.pose.orientation.x
    y = grasp_target.pose.orientation.y
    z = grasp_target.pose.orientation.z
    w = grasp_target.pose.orientation.w
    
    x_vec = [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w)
    ]
    
    pregrasp_pose = PoseStamped()
    pregrasp_pose.header = grasp_target.header
    pregrasp_pose.pose.position.x = grasp_target.pose.position.x - offset * x_vec[0]
    pregrasp_pose.pose.position.y = grasp_target.pose.position.y - offset * x_vec[1]
    pregrasp_pose.pose.position.z = grasp_target.pose.position.z - offset * x_vec[2]
    pregrasp_pose.pose.orientation = grasp_target.pose.orientation
    return pregrasp_pose

def compute_retract_pose(grasp_target, retract_height):
    retract_pose = PoseStamped()
    retract_pose.header = grasp_target.header
    retract_pose.pose.position.x = grasp_target.pose.position.x
    retract_pose.pose.position.y = grasp_target.pose.position.y
    retract_pose.pose.position.z = grasp_target.pose.position.z + retract_height
    retract_pose.pose.orientation = grasp_target.pose.orientation
    return retract_pose

class PickStateMachine(Node):
    def __init__(self):
        super().__init__('pick_state_machine')
        
        self.declare_parameter('pregrasp_offset', 0.08)
        self.declare_parameter('retract_height', 0.10)
        self.declare_parameter('gripper_open_aperture', 0.10)
        self.declare_parameter('gripper_close_aperture', 0.0)
        self.declare_parameter('grasp_verify_threshold', -0.1)
        self.declare_parameter('state_timeout', 30.0)
        self.declare_parameter('ik_timeout', 10.0)
        self.declare_parameter('execution_timeout', 90.0)
        self.declare_parameter('base_frame', 'base_link')
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.ik_goal_pub = self.create_publisher(PoseStamped, '/ik_preview/goal_pose', 10)
        
        self.create_subscription(PoseStamped, '~/grasp_target', self.target_cb, 10, callback_group=self.cb_group)
        self.create_subscription(DiagnosticArray, '/ik_preview/status', self.ik_status_cb, 10, callback_group=self.cb_group)
        self.create_subscription(String, '/guarded_executor/status', self.executor_status_cb, 10, callback_group=self.cb_group)
        self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_cb, 10, callback_group=self.cb_group)
        
        self.pick_srv = self.create_service(Trigger, '~/pick', self.pick_cb, callback_group=self.cb_group)
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

    def pick_cb(self, request, response):
        with self.lock:
            if self.running:
                response.success = False
                response.message = "Already running"
                return response
            if self.last_target is None:
                response.success = False
                response.message = "No grasp target received"
                return response
            
            self.running = True
            self.stop_requested = False
            target = self.last_target
            
        threading.Thread(target=self.run_state_machine, args=(target,)).start()
        response.success = True
        response.message = "Pick sequence started"
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
        

        self._wait_condition(lambda: self.ik_status_time > start_time and self.ik_status_ok, ik_timeout, "IK_FAILED: solution not found")
        
        # Log q_start and q_goal
        try:
            self.get_logger().info("[IK] result = SUCCESS")
            self.get_logger().info("q_start:")
            for j in ['joint_lift', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']:
                self.get_logger().info(f"  {j}: {self.q_start.get(j, 0.0):.3f}")
            self.get_logger().info("q_goal:")
            for j in ['joint_lift', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']:
                self.get_logger().info(f"  {j}: {self.q_goal.get(j, 0.0):.3f}")
            self.get_logger().info("delta_q:")
            for j in ['joint_lift', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']:
                self.get_logger().info(f"  {j}: {self.q_goal.get(j, 0.0) - self.q_start.get(j, 0.0):.3f}")
        except Exception as e:
            pass

        self.publish_status(f"EXECUTE_{state_name}" if "PREGRASP" in state_name else state_name)

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


    def run_state_machine(self, grasp_target):
        try:
            self.publish_status("NAVIGATING")
            self.get_logger().info("NAVIGATING -> NAVIGATION_COMPLETE")
            self.publish_status("ACQUIRE_TARGET")
            self.get_logger().info("NAVIGATION_COMPLETE -> TRANSFORM_TARGET")
            base_frame = self.get_parameter('base_frame').value
            
            # Massive Log
            self.get_logger().info("================ MANIPULATION REQUEST ================")
            self.get_logger().info(f"request_id: target_received_{int(time.time())}")
            self.get_logger().info("target_map:")
            self.get_logger().info(f"  x: {grasp_target.pose.position.x}")
            self.get_logger().info(f"  y: {grasp_target.pose.position.y}")
            self.get_logger().info(f"  z: {grasp_target.pose.position.z}")
            self.get_logger().info(f"  w: {grasp_target.pose.orientation.w}")

            if grasp_target.pose.position.z < 0.1:
                raise RuntimeError("INVALID_TARGET_Z: Target Z is < 0.1. Please fix upstream or in autonomous_mission.")

            if grasp_target.header.frame_id != base_frame:
                if not self.tf_buffer.can_transform(base_frame, grasp_target.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0)):
                    raise RuntimeError("TARGET_TRANSFORM_FAILED: Cannot transform grasp target to base_frame")
                try:
                    trans = self.tf_buffer.lookup_transform(base_frame, grasp_target.header.frame_id, rclpy.time.Time())
                    # Also get base pose in map for logging
                    base_in_map = self.tf_buffer.lookup_transform('map', base_frame, rclpy.time.Time())
                    self.get_logger().info("base_pose_map:")
                    self.get_logger().info(f"  x: {base_in_map.transform.translation.x}")
                    self.get_logger().info(f"  y: {base_in_map.transform.translation.y}")
                except Exception as e:
                    self.get_logger().warn("Could not get base in map: " + str(e))

                self.get_logger().info(f"[TF] target source frame = {grasp_target.header.frame_id}")
                self.get_logger().info(f"[TF] destination frame = {base_frame}")
                self.get_logger().info("[TF] SUCCESS")
                
                grasp_target_base = do_transform_pose_stamped(grasp_target, trans)
            else:
                grasp_target_base = grasp_target
                
            self.get_logger().info("target_base_link:")
            self.get_logger().info(f"  x: {grasp_target_base.pose.position.x:.3f}")
            self.get_logger().info(f"  y: {grasp_target_base.pose.position.y:.3f}")
            self.get_logger().info(f"  z: {grasp_target_base.pose.position.z:.3f}")

            self.publish_status("COMPUTE_GRASP")
            self.get_logger().info("TRANSFORM_TARGET -> COMPUTE_PREGRASP")
            offset = self.get_parameter('pregrasp_offset').value
            
            pregrasp_pose = compute_pregrasp_pose(grasp_target_base, offset)
            
            self.get_logger().info("pregrasp_base_link:")
            self.get_logger().info(f"  x: {pregrasp_pose.pose.position.x:.3f}")
            self.get_logger().info(f"  y: {pregrasp_pose.pose.position.y:.3f}")
            self.get_logger().info(f"  z: {pregrasp_pose.pose.position.z:.3f}")
            self.get_logger().info("======================================================")

            self.control_gripper(self.get_parameter('gripper_open_aperture').value, "OPEN_GRIPPER")
            
            self.get_logger().info("COMPUTE_PREGRASP -> SOLVE_IK")
            self.send_ik_and_execute(pregrasp_pose, "PREGRASP")
            self.get_logger().info("SOLVE_IK -> PLAN -> GENERATE_TRAJECTORY -> EXECUTE_PREGRASP")
            self.publish_status("REVALIDATE")
            time.sleep(0.5)
            
            self.send_ik_and_execute(grasp_target_base, "APPROACH")
            
            self.control_gripper(self.get_parameter('gripper_close_aperture').value, "CLOSE_GRIPPER")
            
            self.publish_status("VERIFY_GRASP")
            verify_thresh = self.get_parameter('grasp_verify_threshold').value
            time.sleep(1.0)
            with self.lock:
                effort = self.current_efforts.get('joint_gripper_finger_left', 0.0)
            self.get_logger().info(f"Grasp effort verified: {effort}")
            
            retract_pose = compute_retract_pose(grasp_target_base, self.get_parameter('retract_height').value)
            self.send_ik_and_execute(retract_pose, "RETRACT")
            
            self.publish_status("DONE")
            
        except Exception as e:
            self.get_logger().error(f"Pick failed: {e}")
            self.publish_status(f"FAILED: {e}")
        finally:
            with self.lock:
                self.running = False

def main(args=None):
    rclpy.init(args=args)
    node = PickStateMachine()
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
