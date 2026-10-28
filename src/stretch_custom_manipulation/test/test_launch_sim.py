"""Launch test: one-click 5-DoF goal, duplicate click, unreachable goal and STOP in the domain-20 kinematic simulation."""
import os
import copy
import math
import time
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import launch_testing
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarkerFeedback
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

@pytest.mark.launch_test
def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = '20'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    pkg_share = get_package_share_directory('stretch_custom_manipulation')
    launch_file = os.path.join(pkg_share, 'launch', 'bringup.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'sim': 'true', 'rviz': 'false'}.items()
        ),
        launch_testing.actions.ReadyToTest()
    ])


def tcp_pose(positions):
    """Independent URDF FK oracle for measured and requested TCP transforms."""
    path = os.path.join(get_package_share_directory('stretch_custom_kinematics'),
                        'config', 'captured_robot.urdf')
    joints = {j.find('child').attrib['link']: j for j in ET.parse(path).getroot().findall('joint')}
    chain, link = [], 'link_grasp_center'
    while link != 'base_link':
        joint = joints[link]
        chain.append(joint)
        link = joint.find('parent').attrib['link']
    transform = np.eye(4)
    def rotation(axis, angle):
        axis = np.asarray(axis, dtype=float)
        axis /= np.linalg.norm(axis)
        x, y, z = axis
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew
    for joint in reversed(chain):
        origin = joint.find('origin')
        local = np.eye(4)
        if origin is not None:
            local[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            r, p, y = [float(v) for v in origin.get('rpy', '0 0 0').split()]
            local[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
        motion = np.eye(4)
        if joint.get('type') != 'fixed':
            axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            value = positions.get(joint.get('name'), 0.)
            if joint.get('type') == 'prismatic':
                motion[:3, 3] = axis * value
            else:
                motion[:3, :3] = rotation(axis, value)
        transform = transform @ local @ motion
    return transform


class TestSimulation(unittest.TestCase):
    def test_one_click_goal_and_stop(self):
        rclpy.init()
        node = rclpy.create_node('manipulation_integration_test')
        latest = {}
        def save(key, msg):
            latest[key] = msg
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subs = [node.create_subscription(JointState, '/ik_input/joint_states', lambda m: save('measured', m), 10),
                node.create_subscription(JointState, '/ik_preview/joint_states', lambda m: save('preview', m), 10),
                node.create_subscription(DiagnosticArray, '/ik_preview/status', lambda m: save('ik', m), latched),
                node.create_subscription(PoseStamped, '/ik_preview/target_pose', lambda m: save('target', m), latched),
                node.create_subscription(String, '/guarded_executor/status', lambda m: save('executor', m.data), latched)]
        publisher = node.create_publisher(InteractiveMarkerFeedback, '/ik_preview/target/feedback', 10)
        move = node.create_client(Trigger, '/guarded_executor/move_to_preview')
        stop = node.create_client(Trigger, '/guarded_executor/stop')
        def wait(predicate, seconds=30):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    return
            self.fail('Timed out: ' + str(latest.get('executor', latest)))
        def call(client):
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            future = client.call_async(Trigger.Request())
            wait(future.done)
            return future.result()
        def send(pose):
            feedback = InteractiveMarkerFeedback()
            feedback.header = latest['target'].header
            feedback.pose = pose
            feedback.client_id = 'integration_test'
            feedback.marker_name = 'target_tcp'
            feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
            stamp = node.get_clock().now().nanoseconds
            publisher.publish(feedback)
            wait(lambda: 'ik' in latest and
                 rclpy.time.Time.from_msg(latest['ik'].header.stamp).nanoseconds >= stamp and
                 abs(latest['target'].pose.position.z - pose.position.z) < 1e-8)
            # Let matching preview joint state and status reach the executor.
            end = time.monotonic() + .3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
        try:
            wait(lambda: all(k in latest for k in ('measured', 'preview', 'ik', 'target', 'executor'))
                 and latest['ik'].status[0].message == 'success')
            initial = dict(zip(latest['measured'].name, latest['measured'].position))
            requested = dict(initial)
            requested['joint_lift'] += .022
            for name in ('joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'):
                requested[name] += .018 / 4
            requested['joint_wrist_yaw'] += .07
            requested['joint_wrist_pitch'] += .06
            requested['joint_wrist_roll'] += .08
            expected = tcp_pose(requested)
            from scipy.spatial.transform import Rotation
            pose = copy.deepcopy(latest['target'].pose)
            pose.position.x, pose.position.y, pose.position.z = expected[:3, 3].tolist()
            q = Rotation.from_matrix(expected[:3, :3]).as_quat()
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q.tolist()
            send(pose)
            self.assertEqual(latest['ik'].status[0].message, 'success')
            result = call(move)
            self.assertTrue(result.success, result.message)
            self.assertFalse(call(move).success, 'Duplicate click must not start another sequence')
            wait(lambda: 'state=COMPLETE' in latest['executor'] and 'sequence_active=0' in latest['executor'], 45)
            measured = dict(zip(latest['measured'].name, latest['measured'].position))
            actual = tcp_pose(measured)
            self.assertLess(np.linalg.norm(actual[:3, 3] - expected[:3, 3]), .002)
            self.assertLess(np.linalg.norm(actual[:3, 2] - expected[:3, 2]), .003)
            for name in ('joint_lift', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll'):
                self.assertGreater(abs(measured[name] - initial[name]), .001, name)
            print('PASS: one click reached XYZ + TCP Z direction with all five axes moving')
            # A failed IK must never execute the last valid ghost.
            unreachable = copy.deepcopy(pose)
            unreachable.position.x = 10.
            unreachable.position.z = 10.
            send(unreachable)
            self.assertNotEqual(latest['ik'].status[0].message, 'success')
            self.assertFalse(call(move).success)
            # Exercise cancel/stop while the autonomous sequence is moving.
            pose.position.z += .03
            send(pose)
            self.assertEqual(latest['ik'].status[0].message, 'success')
            self.assertTrue(call(move).success)
            wait(lambda: 'tracking_synchronized=1' in latest['executor'] and 'state=EXECUTING' in latest['executor'])
            self.assertTrue(call(stop).success)
            wait(lambda: 'state=ABORTED' in latest['executor'] and 'pending_goal=0' in latest['executor'])
            stopped = list(latest['measured'].position)
            end = time.monotonic() + 1.
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
            self.assertLess(max(abs(a-b) for a,b in zip(stopped, latest['measured'].position)), 1e-6)
            print('PASS: unreachable target rejected; STOP halts motion and remaining segments')
        finally:
            node.destroy_node()
            rclpy.shutdown()
