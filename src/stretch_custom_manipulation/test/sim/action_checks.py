#!/usr/bin/env python3
"""GraspObject action contract against grasp_sim.launch.py (domain 20 only):
busy, duplicate IDs, cached results, cancel with stop confirmation,
readiness rejections. Prints one PASS/FAIL line per check."""
import argparse
import json
import os
import random
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from stretch_custom_manipulation.action import GraspObject
from run_trials import scenario


class Checks(Node):
    def __init__(self):
        super().__init__('action_checks')
        if os.environ.get('ROS_DOMAIN_ID') != '20':
            raise RuntimeError('domain 20 only')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.world = self.create_publisher(String, '/mock/world', latched)
        self.client = ActionClient(self, GraspObject, '/grasp_mission/grasp')
        self.events, self.status = [], []
        self.create_subscription(String, '/mock/events', lambda m: self.events.append(json.loads(m.data)), 100)
        self.create_subscription(String, '/grasp_mission/status', lambda m: self.status.append(m.data), 50)
        self.click = self.create_client(Trigger, '/mock/click')
        self.look = self.create_client(Trigger, '/grasp_mission/look')
        self.results = []

    def spin(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not (until and until()):
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, client):
        client.wait_for_service(timeout_sec=10)
        f = client.call_async(Trigger.Request())
        self.spin(10, f.done)
        return f.result()

    def reset(self, seed, faults=None, click=True):
        self.world.publish(String(data=json.dumps(scenario(random.Random(seed), seed, 0.0, faults))))
        self.spin(2.5)
        if click:
            self.call(self.look)
            self.spin(3)
            self.call(self.click)
            self.spin(1.5)

    def send(self, mission_id, target_id='', feedback=None):
        self.client.wait_for_server(timeout_sec=10)
        f = self.client.send_goal_async(GraspObject.Goal(mission_id=mission_id, target_id=target_id),
                                        feedback_callback=feedback)
        self.spin(5, f.done)
        return f.result()

    def result(self, handle, timeout=150):
        f = handle.get_result_async()
        self.spin(timeout, f.done)
        return f.result().result if f.done() else None

    def check(self, name, ok, detail):
        line = f'{"PASS" if ok else "FAIL"} {name}: {detail}'
        print(line, flush=True)
        self.results.append({'check': name, 'pass': bool(ok), 'detail': detail})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rclpy.init()
    n = Checks()
    n.spin(8)

    n.reset(200, click=False)
    r = n.result(n.send('early'), 20)
    n.check('goal before any observation', r.error_code == 'TARGET_UNKNOWN' and not r.success,
            f'{r.error_code}: {r.message}')

    n.reset(201)
    r = n.result(n.send('wrong-target', target_id='obj-does-not-exist'), 20)
    n.check('unknown target_id', r.error_code == 'TARGET_UNKNOWN', f'{r.error_code}: {r.message}')

    stages = []
    h = n.send('A', feedback=lambda fb: stages.append(fb.feedback.stage))
    n.spin(3)
    t0 = time.monotonic()
    rb = n.result(n.send('B'), 10)
    n.check('distinct goal while busy', rb.error_code == 'BUSY' and time.monotonic() - t0 < 2,
            f'{rb.error_code} in {time.monotonic() - t0:.2f} s: {rb.message}')
    rd = n.result(n.send('A'), 10)
    n.check('same ID while running', rd.error_code == 'DUPLICATE_ACTIVE', f'{rd.error_code}')
    ra = n.result(h)
    n.check('mission A ends once with verified pickup', ra.success and ra.error_code == 'NONE',
            f'{ra.error_code}: {ra.message}; stages {sorted(set(stages))}')
    goals_before = sum(e['kind'] == 'goal_accepted' for e in n.events)
    rc = n.result(n.send('A'), 10)
    n.spin(2)
    goals_after = sum(e['kind'] == 'goal_accepted' for e in n.events)
    n.check('finished ID returns cached result, no motion',
            rc.success and rc.mission_id == 'A' and goals_after == goals_before,
            f'success={rc.success}, driver goals {goals_before}->{goals_after}')

    n.reset(202)
    stages.clear()
    h = n.send('C', feedback=lambda fb: stages.append(fb.feedback.stage))
    n.spin(90, lambda: 'MOVE_PREGRASP' in stages)
    n.spin(1.0)
    n.events.clear()
    cancel = h.cancel_goal_async()
    n.spin(5, cancel.done)
    rc = n.result(h, 20)
    stops = [e['kind'] for e in n.events if e['kind'] in ('stop', 'motion_stopped')]
    n.check('cancel during motion stops and reports CANCELED',
            rc is not None and rc.error_code == 'CANCELED' and 'stop' in stops,
            f'{rc.error_code if rc else None} at {rc.failed_stage if rc else None}; driver saw {stops}')
    n.spin(2)
    r = n.result(n.send('D'), 150)
    n.check('next mission after cancel needs no extra submission', r.success,
            f'{r.error_code}: {r.message}')

    n.reset(203, faults={'runstopped': True})
    t0 = time.monotonic()
    r = n.result(n.send('E'), 10)
    n.check('runstopped robot rejected before motion', r.error_code == 'NOT_READY' and
            time.monotonic() - t0 < 2, f'{r.error_code}: {r.message}')

    with open(a.out, 'w') as f:
        json.dump(n.results, f, indent=1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
