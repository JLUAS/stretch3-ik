#!/usr/bin/env python3
"""Exercise preview marker feedback only; never publish robot commands or state."""
import argparse
import copy
import json
import math
import os
import signal
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import InteractiveMarkerFeedback
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture-pid', type=int)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') not in ('20', '42'):
        raise RuntimeError('Select domain 20 or 42')
    if args.fixture_pid:
        if os.environ.get('ROS_DOMAIN_ID') != '20':
            raise RuntimeError('Fixture pause test restricted to domain 20')
        command = Path(f'/proc/{args.fixture_pid}/cmdline').read_bytes()
        if b'/offline_seed.py' not in command:
            raise RuntimeError('PID does not identify the offline preview fixture')
    rclpy.init(args=['--ros-args', '-r', '/tf:=/ik_preview/tf', '-r', '/tf_static:=/ik_preview/tf_static'])
    node = rclpy.create_node('ik_preview_validation', namespace='/ik_preview')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest = {}
    def save(name, value):
        latest[name] = value
    subs = [node.create_subscription(DiagnosticArray, '/ik_preview/status', lambda x: save('status', x), latched),
            node.create_subscription(PoseStamped, '/ik_preview/target_pose', lambda x: save('target', x), latched),
            node.create_subscription(PoseStamped, '/ik_preview/solved_pose', lambda x: save('solved', x), latched),
            node.create_subscription(JointState, '/ik_preview/joint_states', lambda x: save('joints', x), 10)]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    pub = node.create_publisher(InteractiveMarkerFeedback, '/ik_preview/target/feedback', 10)
    assert pub.topic_name == '/ik_preview/target/feedback'
    def wait(predicate, seconds=15):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
            if predicate():
                return
        raise AssertionError('Timed out waiting for preview condition')
    def status():
        return latest['status'].status[0].message if 'status' in latest else ''
    wait(lambda: all(k in latest for k in ('target', 'solved', 'status', 'joints')) and status() == 'success')
    initial = copy.deepcopy(latest['target'].pose)
    results = {'domain': os.environ['ROS_DOMAIN_ID'], 'tests': {}}
    def feedback(pose, expected, frame='ik_preview/base_link'):
        m = InteractiveMarkerFeedback()
        m.header.frame_id = frame
        m.header.stamp = node.get_clock().now().to_msg()
        m.client_id = 'preview_validation'
        m.marker_name = 'target_tcp'
        m.control_name = 'move_2'
        m.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        m.pose = pose
        sent = node.get_clock().now().nanoseconds
        pub.publish(m)
        wait(lambda: status() == expected and
             latest['status'].header.stamp.sec*10**9 + latest['status'].header.stamp.nanosec >= sent and
             abs(latest['target'].pose.position.z-pose.position.z) < 1e-9 and
             abs(latest['target'].pose.position.x-pose.position.x) < 1e-9)
    moved = copy.deepcopy(initial)
    moved.position.z += .04
    feedback(moved, 'success')
    values = {x.key: x.value for x in latest['status'].status[0].values}
    assert float(values['position_error_m']) <= 1e-6
    assert float(values['direction_error_rad']) <= 1e-5
    results['tests']['reachable_marker_feedback'] = values
    def tf_matches():
        try:
            tf = buffer.lookup_transform('ik_preview/base_link', 'ik_preview/link_grasp_center', rclpy.time.Time())
            p, q = tf.transform.translation, latest['solved'].pose.position
            return math.sqrt((p.x-q.x)**2+(p.y-q.y)**2+(p.z-q.z)**2) < 1e-8
        except Exception:
            return False
    wait(tf_matches)
    results['tests']['ghost_tf_matches_fk'] = True
    bound_names = ['joint_lift', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
    actual = dict(zip(latest['joints'].name, latest['joints'].position))
    assert all(n in actual for n in bound_names)
    valid_positions = [actual[n] for n in bound_names]
    impossible = copy.deepcopy(initial)
    impossible.position.x = 10.
    feedback(impossible, 'unreachable target')
    actual = dict(zip(latest['joints'].name, latest['joints'].position))
    assert valid_positions == [actual[n] for n in bound_names]
    results['tests']['unreachable_preserves_last_valid_ghost'] = True
    feedback(initial, 'invalid target', 'wrong_frame')
    results['tests']['invalid_target_frame'] = True
    feedback(initial, 'success')
    if args.fixture_pid:
        try:
            os.kill(args.fixture_pid, signal.SIGSTOP)
            wait(lambda: status() == 'stale state', seconds=5)
            results['tests']['stale_seed_rejected'] = True
        finally:
            os.kill(args.fixture_pid, signal.SIGCONT)
        wait(lambda: status() == 'success')
        results['tests']['fresh_seed_recovery'] = True
    # Audit complete graph: preview/RViz nodes must not publish these robot topics.
    audits = {}
    audit_node = rclpy.create_node("preview_graph_audit", use_global_arguments=False)
    until = time.monotonic() + 2
    while time.monotonic() < until:
        rclpy.spin_once(audit_node, timeout_sec=.05)
    for topic in ('/joint_states', '/stretch/joint_states', '/tf', '/tf_static', '/joint_pose_cmd', '/stretch/cmd_vel'):
        pubs = audit_node.get_publishers_info_by_topic(topic)
        names = [p.node_namespace.rstrip('/')+'/'+p.node_name for p in pubs]
        assert not any(n.startswith('/ik_preview/') or n.startswith('/ik_input/') for n in names), (topic, names)
        audits[topic] = names
    results['robot_topic_publishers'] = audits
    preview_pubs = node.get_publisher_names_and_types_by_node('ik_preview', '/ik_preview')
    results['preview_publishers'] = [name for name, _ in preview_pubs]
    assert all(name.startswith('/ik_preview/') or name in ('/rosout', '/parameter_events') for name, _ in preview_pubs)
    results['preview_action_clients'] = node.get_action_client_names_and_types_by_node('ik_preview', '/ik_preview') if hasattr(node, 'get_action_client_names_and_types_by_node') else 'checked separately with ros2 node info'
    args.output.write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    audit_node.destroy_node()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
