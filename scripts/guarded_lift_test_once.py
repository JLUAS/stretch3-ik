#!/usr/bin/env python3
"""Explicit operator command for one guarded +1 cm lift; --check sends no goal.
All arm/validation/execution/stop requests go through stretch_guarded_executor.
"""
import argparse
import fcntl
import os
import sys
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

parser = argparse.ArgumentParser(description=__doc__)
mode = parser.add_mutually_exclusive_group(required=True)
mode.add_argument('--check', action='store_true', help='arm/stage/validate, then disarm; no execution')
mode.add_argument('--execute', action='store_true', help='authorize one +0.01 m lift over 5 s')
args = parser.parse_args()
if os.environ.get('ROS_DOMAIN_ID') != '42':
    parser.error('requires ROS_DOMAIN_ID=42')
lock = open('/tmp/stretch-guarded-lift-test.lock', 'w')
try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    sys.exit('Another lift-test command is already running; no request sent.')

rclpy.init()
node = rclpy.create_node('guarded_lift_test_operator')
latest = {'text': '', 'received': 0.0}

def status(message):
    latest.update(text=message.data, received=time.monotonic())

node.create_subscription(String, '/guarded_executor/status', status,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

def field(name):
    for token in latest['text'].split():
        if token.startswith(name + '='):
            return token.split('=', 1)[1]
    return None

def call(name, service_type=Trigger, request=None):
    client = node.create_client(service_type, name)
    deadline = time.monotonic() + 5
    try:
        while not client.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not client.service_is_ready():
            raise RuntimeError(name + ' unavailable')
        future = client.call_async(request if request is not None else service_type.Request())
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise RuntimeError(name + ' timed out; never retry an execution automatically')
        response = future.result()
        if hasattr(response, 'success') and not response.success:
            raise RuntimeError(name + ': ' + response.message)
        return response
    finally:
        node.destroy_client(client)

armed_here = False
exit_code = 0
try:
    deadline = time.monotonic() + 8
    while (not latest['text'] or field('action_server_available') != '1') and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    if time.monotonic() - latest['received'] > .5:
        raise RuntimeError('Executor status unavailable/stale')
    if field('state') not in ['DISARMED', 'COMPLETE', 'ABORTED']:
        raise RuntimeError('Executor must be disarmed before this operator command: ' + latest['text'])
    for key, value in [('armed', '0'), ('pending_goal', '0'), ('stop_unconfirmed', '0'),
                       ('action_server_available', '1'), ('active_goal_present', '0'), ('controller_idle', '1')]:
        if field(key) != value:
            raise RuntimeError('Preflight requires ' + key + '=' + value + ': ' + latest['text'])
    if sum(name == 'executor' and ns == '/guarded_executor'
           for name, ns in node.get_node_names_and_namespaces()) != 1:
        raise RuntimeError('Require exactly one /guarded_executor/executor')
    request = GetParameters.Request(names=['dry_run', 'execution_enabled', 'lift_test_delta', 'trajectory_duration'])
    values = call('/guarded_executor/executor/get_parameters', GetParameters, request).values
    if (len(values) != 4 or values[0].type != 1 or values[0].bool_value or
            values[1].type != 1 or not values[1].bool_value or
            values[2].type != 3 or abs(values[2].double_value - .01) > 1e-12 or
            values[3].type != 3 or abs(values[3].double_value - 5.0) > 1e-12):
        raise RuntimeError('Require physical enabled executor configured for exactly +0.01 m / 5 s')
    call('/guarded_executor/arm', SetBool, SetBool.Request(data=True))
    armed_here = True
    call('/guarded_executor/stage_lift_test')
    approved = call('/guarded_executor/validate_preview')
    print(approved.message, flush=True)
    if args.check:
        call('/guarded_executor/disarm')
        armed_here = False
        print('Preflight PASS; executor DISARMED; no motion goal sent.')
    else:
        # Only the explicit --execute branch can request motion. Never retry.
        requested_at = time.monotonic()
        response = call('/guarded_executor/execute_preview')
        print(response.message, flush=True)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if latest['received'] <= requested_at:
                continue
            if field('state') == 'COMPLETE' and field('armed') == '0':
                armed_here = False
                print(latest['text'])
                break
            if field('state') == 'ABORTED':
                raise RuntimeError(latest['text'])
        else:
            raise RuntimeError('Terminal result not confirmed; requesting disarm/cancel')
except (Exception, KeyboardInterrupt) as error:
    print('Lift test stopped: ' + str(error), file=sys.stderr)
    exit_code = 1
finally:
    if armed_here:
        try:
            call('/guarded_executor/disarm')
        except Exception as error:
            print('Disarm/cancel NOT CONFIRMED: use physical runstop. ' + str(error), file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    lock.close()
sys.exit(exit_code)
