#!/usr/bin/env python3
"""Phase 2 residuals: planned grasp vs FK of the IK answer (domain 20 only).

For synthetic cans across the reachable band, plans candidates with the
old candidate set (yaw 0, +-0.2) and asks the running ik_preview for grasp
and pre-grasp solutions. Records per candidate: IK success, base_dx, FK
position error, achieved-vs-planned approach and finger angles, and how much
the achieved approach turns between pre-grasp and grasp.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import grasp_planning as gp  # noqa: E402
from grasp_mission import angle, pose_from_text, pose_msg  # noqa: E402


def can(cx, cy, cz, r=0.03, h=0.12):
    a = np.linspace(0, np.pi, 30)
    z = np.linspace(cz - h / 2, cz + h / 2, 15)
    aa, zz = np.meshgrid(a, z)
    return np.column_stack([cx + r * np.cos(aa).ravel(), cy + r * np.sin(aa).ravel(), zz.ravel()])


class Sweep(Node):
    def __init__(self):
        super().__init__('ik_sweep')
        if os.environ.get('ROS_DOMAIN_ID') != '20':
            raise RuntimeError('domain 20 only')
        self.pub = self.create_publisher(PoseStamped, '/ik_preview/grasp_goal', 10)
        self.ik = {}
        self.create_subscription(DiagnosticArray, '/ik_preview/status', self.on_ik, 10)
        self.stamp = 0

    def on_ik(self, m):
        if m.status:
            self.ik = {kv.key: kv.value for kv in m.status[0].values}
            self.ik['_message'] = m.status[0].message

    def solve(self, pose):
        self.stamp = max(self.get_clock().now().nanoseconds, self.stamp + 1)
        msg = pose_msg(pose, 'base_link')
        msg.header.stamp = Time(nanoseconds=self.stamp).to_msg()
        self.pub.publish(msg)
        end = time.monotonic() + 5
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            ik = self.ik
            if ik.get('goal_stamp_ns') == str(self.stamp) and ik.get('_message') != 'stale state':
                ok = ik.get('success') == 'true'
                return {'ok': ok, 'base_dx': float(ik.get('base_dx', 'nan')),
                        'reason': ik.get('_message'),
                        'achieved': pose_from_text(ik['solved_pose']) if ok else None}
        return {'ok': False, 'reason': 'no answer', 'base_dx': math.nan, 'achieved': None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rclpy.init()
    n = Sweep()
    end = time.monotonic() + 15
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    rows = []
    for x in np.arange(-0.074, 0.006, 0.01):
        for y in (-0.60, -0.66):
            for z in (0.78, 0.90):
                obj = can(x, y, z)
                for c in gp.plan_grasps(obj, np.zeros((0, 3)), yaws=(0.0, 0.2, -0.2)):
                    g, p = n.solve(c['grasp']), n.solve(c['pregrasp'])
                    row = {'object': [round(x, 3), y, z], 'key': list(c['key']),
                           'grasp_ok': g['ok'], 'pregrasp_ok': p['ok'], 'base_dx': g['base_dx'],
                           'reason': g['reason']}
                    if g['ok']:
                        row['position_m'] = float(np.linalg.norm(g['achieved'][:3, 3] - c['grasp'][:3, 3]))
                        row['approach_rad'] = angle(g['achieved'][:3, 0], c['grasp'][:3, 0])
                        row['finger_rad'] = angle(g['achieved'][:3, 1], c['grasp'][:3, 1])
                        rot = np.eye(4)
                        rot[:3, :3] = g['achieved'][:3, :3]
                        row['fits_achieved'] = gp.fits(obj, np.zeros((0, 3)), rot) is not None
                    if g['ok'] and p['ok']:
                        row['approach_change_rad'] = angle(g['achieved'][:3, 0], p['achieved'][:3, 0])
                        # Lateral (finger-axis) drift of the TCP line between the two solutions.
                        d = p['achieved'][:3, 3] - g['achieved'][:3, 3]
                        row['tcp_x_drift_m'] = float(abs(d[0]))
                    rows.append(row)
                    print(json.dumps(row), flush=True)
    with open(a.out, 'w') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
