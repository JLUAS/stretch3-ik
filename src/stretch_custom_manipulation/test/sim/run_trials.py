#!/usr/bin/env python3
"""Seeded grasp trials against grasp_sim.launch.py (isolated, domain 20 only).

Each trial resets the mock world (fresh arrival: navigation mode, stowed arm,
random odometry), then plays the operator: ~/look, click, ~/pick, and
resubmits ~/pick after a failure up to --max-submissions times, as the
operator did on the robot. Writes one JSON record per trial.
"""
import argparse
import json
import math
import os
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


def scenario(rng, i, settle_mm, faults=None):
    return {
        'seed': i,
        'odom': [rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi)],
        'mode': 'navigation',
        'arm': [0.8, 0.0, 3.0, -0.2, 0.0],
        'aperture': 0.0,
        'object': {'center': [rng.uniform(-0.075, 0.005), rng.uniform(-0.66, -0.58),
                              rng.uniform(0.75, 0.90)],
                   'radius': rng.uniform(0.025, 0.035), 'height': rng.uniform(0.10, 0.14)},
        'neighbors': [],
        'perception_bias': [rng.uniform(-0.004, 0.004) for _ in range(3)],
        'depth_noise': 0.002,
        'faults': dict({'settle_mm': settle_mm, 'settle_s': 1.5}, **(faults or {})),
    }


class Runner(Node):
    def __init__(self):
        super().__init__('trial_runner')
        if os.environ.get('ROS_DOMAIN_ID') != '20':
            raise RuntimeError('domain 20 only')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.world_pub = self.create_publisher(String, '/mock/world', latched)
        self.status = []
        self.events = []
        self.create_subscription(String, '/grasp_mission/status',
                                 lambda m: self.status.append((time.monotonic(), m.data)), 50)
        self.create_subscription(String, '/mock/events',
                                 lambda m: self.events.append(json.loads(m.data)), 100)
        self.trig = {n: self.create_client(Trigger, n) for n in
                        ('/grasp_mission/look', '/grasp_mission/pick', '/grasp_mission/stop',
                         '/mock/click')}

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, name, timeout=10.0):
        c = self.trig[name]
        if not c.wait_for_service(timeout_sec=timeout):
            return None
        f = c.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not f.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return f.result() if f.done() else None

    def wait_status(self, since, prefixes, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for t, s in self.status:
                if t > since and s.split(':')[0].split(' ')[0] in prefixes:
                    return s
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def trial(self, world, max_submissions, attempt_timeout, auto=False):
        self.call('/grasp_mission/stop')
        self.spin_for(1.0)
        self.events.clear()
        self.world_pub.publish(String(data=json.dumps(world)))
        self.spin_for(2.5)
        t0 = time.monotonic()
        self.call('/grasp_mission/look')
        self.wait_status(t0, ('LOOKING', 'FAILED'), 15)
        t1 = time.monotonic()
        self.call('/mock/click')
        planned = self.wait_status(t1, ('PLANNED', 'PLAN', 'NOT'), 10)
        record = {'world': world, 'planned': planned, 'submissions': []}
        if auto:  # the click itself starts the mission
            final = self.wait_status(t1, ('DONE', 'FAILED', 'CANCELED'), attempt_timeout)
            record['submissions'].append({'accepted': 'starting' in (planned or ''),
                                          'response': planned, 'final': final or 'NO TERMINAL STATUS',
                                          'duration_s': round(time.monotonic() - t1, 2)})
            max_submissions = 0
        for k in range(max_submissions):
            start = time.monotonic()
            r = self.call('/grasp_mission/pick')
            sub = {'accepted': bool(r and r.success), 'response': r.message if r else 'no response'}
            if r and r.success:
                final = self.wait_status(start, ('DONE', 'FAILED', 'ABORTED', 'CANCELED',
                                                 'SUCCEEDED'), attempt_timeout)
                sub['final'] = final or 'NO TERMINAL STATUS'
            sub['duration_s'] = round(time.monotonic() - start, 2)
            record['submissions'].append(sub)
            if sub.get('final', '').startswith(('DONE', 'SUCCEEDED')):
                break
            self.spin_for(1.0)
        self.spin_for(0.5)
        record['events'] = [e for e in self.events if e['kind'] in
                            ('gripper', 'object_pushed', 'goal_rejected', 'mode', 'hang',
                             'motion_stopped', 'stop', 'base')]
        record['duration_s'] = round(time.monotonic() - t0, 2)
        return record


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=30)
    ap.add_argument('--first', type=int, default=0)
    ap.add_argument('--settle-mm', type=float, default=2.0)
    ap.add_argument('--max-submissions', type=int, default=4)
    ap.add_argument('--attempt-timeout', type=float, default=150.0)
    ap.add_argument('--faults', default='{}')
    ap.add_argument('--out', required=True)
    ap.add_argument('--auto', action='store_true', help='click starts the mission')
    a = ap.parse_args()
    rclpy.init()
    node = Runner()
    node.spin_for(8.0)  # stack startup
    with open(a.out, 'a') as out:
        for i in range(a.first, a.first + a.trials):
            world = scenario(random.Random(i), i, a.settle_mm, json.loads(a.faults))
            rec = node.trial(world, a.max_submissions, a.attempt_timeout, a.auto)
            rec['trial'] = i
            out.write(json.dumps(rec) + '\n')
            out.flush()
            subs = rec['submissions']
            print(f"trial {i}: submissions={len(subs)} final={subs[-1].get('final', subs[-1]['response'])!r}",
                  flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
