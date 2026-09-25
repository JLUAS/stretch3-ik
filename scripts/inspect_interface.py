#!/usr/bin/env python3
"""Read-only ROS graph/sample capture. No publishers, control calls or SSH."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/joint_states', '/stretch/joint_states', '/robot_description', '/tf',
          '/tf_static', '/joint_limits', '/is_homed', '/is_runstopped', '/mode',
          '/tool', '/diagnostics']

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('custom_readonly_interface_inspection')
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    types = dict(node.get_topic_names_and_types())
    report, arrivals, subscriptions, static = {}, {}, [], {}

    def receive(topic, msg):
        arrivals[topic].append(time.monotonic())
        if topic == '/robot_description':
            (args.output / 'robot.urdf').write_text(msg.data)
            report[topic]['sample'] = 'robot.urdf'
        elif topic == '/tf_static':
            for transform in msg.transforms:
                static[transform.child_frame_id] = message_to_ordereddict(transform)
        elif 'sample' not in report[topic]:
            report[topic]['sample'] = message_to_ordereddict(msg)

    for topic in TOPICS:
        endpoints = node.get_publishers_info_by_topic(topic)
        report[topic] = {
            'types': types.get(topic, []),
            'publishers': [{'name': e.node_namespace.rstrip('/') + '/' + e.node_name,
                            'qos': str(e.qos_profile)} for e in endpoints]}
        arrivals[topic] = []
        if types.get(topic):
            latched = topic in ('/robot_description', '/tf_static')
            qos = QoSProfile(depth=100,
                             reliability=ReliabilityPolicy.RELIABLE if latched else ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
            subscriptions.append(node.create_subscription(
                get_message(types[topic][0]), topic,
                lambda msg, topic=topic: receive(topic, msg), qos))
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    for topic, values in report.items():
        ts = arrivals[topic]
        values['count'] = len(ts)
        values['observed_hz'] = (len(ts)-1)/(ts[-1]-ts[0]) if len(ts) > 1 else None
        print(topic, 'count=', len(ts), 'Hz=', values['observed_hz'])
    (args.output / 'interface_inspection.json').write_text(json.dumps(report, indent=2))
    (args.output / 'tf_static.json').write_text(json.dumps(list(static.values()), indent=2))
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
