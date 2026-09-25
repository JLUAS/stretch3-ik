#!/usr/bin/env python3
"""Transport test. Domain 20 uses a fixture; domain 42 only subscribes to the robot.
Always launches dry_run=true. Never creates a physical action/command client.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from action_msgs.msg import GoalStatusArray, GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from std_srvs.srv import SetBool, Trigger
from rosidl_runtime_py.convert import message_to_ordereddict

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--live', action='store_true')
parser.add_argument('--output', required=True)
args = parser.parse_args()
expected_domain = '42' if args.live else '20'
assert os.environ.get('ROS_DOMAIN_ID') == expected_domain
if not args.live: os.environ['ROS_LOCALHOST_ONLY'] = '1'
rclpy.init()
node = Node('guarded_executor_dry_test' if args.live else 'stretch_driver',
            start_parameter_services=args.live)
assert not any(name == 'executor' and ns == '/guarded_executor'
               for name, ns in node.get_node_names_and_namespaces()), 'executor already running'
retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)
statuses = []
node.create_subscription(String, '/guarded_executor/status', lambda m: statuses.append(m.data), retained)
fixture = {'preview_ok': True, 'preview_delta': .005, 'publish_state': True,
           'publish_diagnostics': True, 'diagnostic_level': DiagnosticStatus.ERROR,
           'extra_diagnostic': None, 'goal_statuses': None}
if not args.live:
    goal_requests = []
    def reject_goal(request):
        goal_requests.append(request)
        return GoalResponse.REJECT
    def create_mock_server():
        return ActionServer(node, FollowJointTrajectory,
                            '/stretch_controller/follow_joint_trajectory',
                            execute_callback=lambda handle: FollowJointTrajectory.Result(),
                            goal_callback=reject_goal)
    action_server = create_mock_server()
    # No fixture publishers or controller mocks are constructed on domain 42.
    desc = node.create_publisher(String, '/robot_description', retained)
    desc.publish(String(data=(ROOT / 'docs/evidence/robot.urdf').read_text()))
    jp = node.create_publisher(JointState, '/stretch/joint_states', 10)
    hp = node.create_publisher(Bool, '/is_homed', 10)
    rp = node.create_publisher(Bool, '/is_runstopped', 10)
    mp = node.create_publisher(String, '/mode', 10)
    sp = node.create_publisher(Bool, '/is_streaming_position', 10)
    dp = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
    ap = node.create_publisher(GoalStatusArray, '/stretch_controller/follow_joint_trajectory/_action/status', retained)
    pp = node.create_publisher(JointState, '/ik_preview/joint_states', 10)
    ps = node.create_publisher(DiagnosticArray, '/ik_preview/status', retained)
    def params(req, res):
        res.values = [ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True) for _ in req.names]
        return res
    node.create_service(GetParameters, '/stretch_driver/get_parameters', params)
    def publish_fixture():
        stamp = node.get_clock().now().to_msg()
        j = JointState(); j.header.stamp = stamp
        j.name = ['joint_lift', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0',
                  'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll']
        j.position = [.5, .05, .05, .05, .05, 0., 0., 0.]
        if fixture['publish_state']: jp.publish(j)
        hp.publish(Bool(data=True)); rp.publish(Bool(data=False))
        mp.publish(String(data='trajectory')); sp.publish(Bool(data=False))
        d = DiagnosticArray(); d.header.stamp = stamp
        d.status = [DiagnosticStatus(level=fixture['diagnostic_level'],
                                    name='stretch_driver: Pimu/Cliff',
                                    message='cliff_range=75.556 out of range [-50,20]')]
        if fixture['extra_diagnostic']:
            d.status.append(DiagnosticStatus(level=DiagnosticStatus.ERROR,
                                            name=fixture['extra_diagnostic'], message='test fault'))
        if fixture['publish_diagnostics']: dp.publish(d)
        if fixture['goal_statuses'] is not None:
            status_array = GoalStatusArray()
            status_array.status_list = [GoalStatus(status=value) for value in fixture['goal_statuses']]
            ap.publish(status_array)
        j.position[0] += fixture['preview_delta']; pp.publish(j)
        d.status = [DiagnosticStatus(level=DiagnosticStatus.OK if fixture['preview_ok'] else DiagnosticStatus.ERROR,
                                    name='ik_preview', message='success' if fixture['preview_ok'] else 'unreachable target',
                                    values=[KeyValue(key='success', value='true'),
                                            KeyValue(key='seed_topic', value='/stretch/joint_states'),
                                            KeyValue(key='last_valid_joint_solution', value=f'{j.position[0]} .2 0 0 0'),
                                            KeyValue(key='seed_joint_state', value='.5 .2 0 0 0')])]
        ps.publish(d)
    node.create_timer(.04, publish_fixture)

def spin(seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.02)

def call(name, typ=Trigger, data=None):
    client = node.create_client(typ, '/guarded_executor/' + name)
    until = time.monotonic() + 10
    while not client.service_is_ready() and time.monotonic() < until: spin(.05)
    assert client.service_is_ready(), name + ' service unavailable'
    req = typ.Request()
    if data is not None: req.data = data
    f = client.call_async(req)
    while not f.done() and time.monotonic() < until: spin(.02)
    assert f.done(), name + ' timeout'
    response = f.result()
    node.destroy_client(client)
    result = dict(message_to_ordereddict(response))
    results['calls'].append({'service': name, **result})
    return response

results = {'domain': expected_domain, 'live': args.live, 'calls': []}
log_path = Path(args.output).with_suffix('.log')
log_path.parent.mkdir(parents=True, exist_ok=True)
process = None
with tempfile.TemporaryDirectory(prefix='stretch-guard-dry-') as temp, open(log_path, 'w') as logfile:
    artifact = Path(temp) / 'trajectory.json'
    command = ['ros2', 'run', 'stretch_guarded_executor', 'guarded_executor', '--ros-args',
               '-r', '__ns:=/guarded_executor',
               '--params-file', str(ROOT / 'src/stretch_guarded_executor/config/executor_real_test.yaml'),
               '-p', 'state_timeout:=0.5', '-p', 'dry_run:=true',
               '-p', f'dry_run_report_path:={artifact}']
    try:
        if not args.live:
            process = subprocess.Popen(command + ['-p', 'base_motion_enabled:=true'],
                                       stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
            until = time.monotonic() + 5
            while process.poll() is None and time.monotonic() < until: spin(.05)
            assert process.poll() is not None and process.returncode != 0, 'base-enabled profile accepted'
            assert 'Base motion is prohibited by the lift-only executor' in log_path.read_text()
            process = None
            results['base_enabled_profile_rejected'] = True
        process = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
        spin(4)
        assert process.poll() is None
        assert statuses and 'state=DISARMED' in statuses[-1]
        assert not call('arm', SetBool, True).success
        assert not call('execute_preview').success
        os.killpg(process.pid, signal.SIGINT)
        until = time.monotonic() + 5
        while process.poll() is None and time.monotonic() < until: spin(.05)
        process.wait(timeout=1); process = None
        spin(.5)
        statuses.clear()
        process = subprocess.Popen(command + ['-p', 'execution_enabled:=true'], stdout=logfile,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        spin(5)
        assert process.poll() is None
        assert statuses and 'state=DISARMED' in statuses[-1]
        assert call('arm', SetBool, True).success
        assert call('stage_lift_test').success
        validation = call('validate_preview')
        execution = call('execute_preview')
        results['manual_validation'] = validation.success
        results['manual_execution_dry'] = execution.success
        if not args.live:
            assert validation.success, validation.message
            assert execution.success, execution.message
            spin(.2)
            assert 'state=COMPLETE armed=0' in statuses[-1]
            assert 'action_server_available=1' in statuses[-1]
            assert 'status_message_received=0' in statuses[-1]
            assert 'active_goal_present=0' in statuses[-1]
            assert 'controller_idle=1' in statuses[-1]
            assert call('arm', SetBool, True).success
            assert call('stage_lift_test').success
            for active in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                           GoalStatus.STATUS_CANCELING]:
                fixture['goal_statuses'] = [active]; spin(.3)
                assert not call('validate_preview').success, 'active goal ignored'
                assert 'active_goal_present=1' in statuses[-1]
                assert 'controller_idle=0' in statuses[-1]
            fixture['goal_statuses'] = []; spin(.3)
            assert call('validate_preview').success, 'empty status blocked'
            fixture['goal_statuses'] = [GoalStatus.STATUS_SUCCEEDED,
                                       GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]
            spin(.3)
            assert call('validate_preview').success, 'terminal goals blocked'
            action_server.destroy()
            until = time.monotonic() + 4
            while 'action_server_available=0' not in statuses[-1] and time.monotonic() < until: spin(.1)
            assert 'action_server_available=0' in statuses[-1]
            assert not call('validate_preview').success, 'unavailable server ignored'
            action_server = create_mock_server()
            until = time.monotonic() + 4
            while 'action_server_available=1' not in statuses[-1] and time.monotonic() < until: spin(.1)
            assert 'action_server_available=1' in statuses[-1]
            assert call('validate_preview').success
            results['controller_preflight_checks_passed'] = True
            assert call('use_ik_preview').success
            assert call('validate_preview').success
            assert 'WARN: stretch_driver: Pimu/Cliff' in statuses[-1]
            assert 'within supervised range [-100,200]' in statuses[-1]
            assert '[-50,20]' not in statuses[-1]
            for fault in ['other_driver: sensor', 'stretch_driver: Lift/Motor',
                          'stretch_controller: follow_joint_trajectory']:
                fixture['extra_diagnostic'] = fault; spin(.2)
                assert not call('validate_preview').success, fault + ' ignored'
            fixture['extra_diagnostic'] = None
            fixture['diagnostic_level'] = DiagnosticStatus.STALE; spin(.2)
            assert not call('validate_preview').success, 'STALE cliff status downgraded'
            fixture['diagnostic_level'] = DiagnosticStatus.ERROR; spin(.2)
            assert call('validate_preview').success
            fixture['publish_diagnostics'] = False; spin(2.8)
            assert not call('validate_preview').success, 'stale diagnostics stream ignored'
            fixture['publish_diagnostics'] = True; spin(.2)
            assert call('validate_preview').success
            fixture['preview_ok'] = False; spin(.3)
            assert not call('execute_preview').success, 'failed preview executed old ghost'
            fixture['preview_ok'] = True; spin(.3)
            assert call('validate_preview').success
            fixture['preview_delta'] = .006; spin(.3)
            assert not call('execute_preview').success, 'changed preview reused approval'
            assert call('validate_preview').success
            assert call('execute_preview').success
            assert call('arm', SetBool, True).success
            assert call('stage_lift_test').success
            assert call('validate_preview').success
            fixture['publish_state'] = False; spin(.8)
            assert not call('execute_preview').success, 'stale state approved'
        if not args.live:
            log_text = log_path.read_text()
            for required in ["name='stretch_driver: Pimu/Cliff'", 'original_level=ERROR(2)',
                             'effective_level=WARN', 'cliff_range=75.556 out of range [-50,20]',
                             "policy_reason='bounded cliff exception: measured 75.556",
                             "within supervised range [-100,200]",
                             "base motion disabled"]:
                assert required in log_text, 'missing diagnostic downgrade log: ' + required
            results['diagnostic_policy_checks_passed'] = True
            assert not goal_requests, 'dry run sent a goal to the mock action server'
        results['trajectory'] = json.loads(artifact.read_text())
        results['status_before_stop'] = statuses[-1]
        pubs = node.get_publisher_names_and_types_by_node('executor', '/guarded_executor')
        clients = node.get_client_names_and_types_by_node('executor', '/guarded_executor')
        results['executor_publishers'] = pubs
        results['executor_service_clients'] = clients
        assert not any('/_action/' in n or n in ['/stop_the_robot', '/runstop'] for n, _ in clients)
        assert not any(n in ['/joint_states', '/stretch/joint_states', '/tf', '/tf_static', '/joint_pose_cmd', '/stretch/cmd_vel'] for n, _ in pubs)
        assert all('hardware_client=0' in s for s in statuses)
        assert call('stop').success
        spin(.2)
        assert 'state=ABORTED armed=0' in statuses[-1]
        assert call('disarm').success
        results['passed'] = True
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            until = time.monotonic() + 5
            while process.poll() is None and time.monotonic() < until: spin(.05)
            process.wait(timeout=1)
        Path(args.output).write_text(json.dumps(results, indent=2) + '\n')
        node.destroy_node(); rclpy.shutdown()
print(json.dumps({k: v for k, v in results.items() if k not in ['calls', 'trajectory']}, indent=2))
