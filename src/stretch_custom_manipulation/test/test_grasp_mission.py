"""Mission logic without a ROS graph: readiness, request correlation,
termination, duplicate goals, achieved-geometry checks."""
import importlib.util
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from std_msgs.msg import Header

scripts = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location('grasp_mission', scripts / 'grasp_mission.py')
gm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gm)
gp = gm.gp
M = gm.GraspMission

IDLE = ('state=COMPLETE armed=0 pending_goal=0 stop_unconfirmed=0 sequence_active=0 '
        'planning_status={REQUEST_ID=4 ENVIRONMENT=PRIMITIVE PLANNER=DIRECT} reason=done')


def can(cx, cy, z0=1.0, r=0.033, h=0.12):
    a = np.linspace(0, np.pi, 30)
    z = np.linspace(z0, z0 + h, 15)
    aa, zz = np.meshgrid(a, z)
    return np.column_stack([cx + r * np.cos(aa).ravel(), cy + r * np.sin(aa).ravel(), zz.ravel()])


def yawed(m, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.eye(4)
    r[:2, :2] = [[c, -s], [s, c]]
    out = m.copy()
    out[:3, :3] = r[:3, :3] @ m[:3, :3]
    return out


class Recorder:
    def __init__(self):
        self.events = []

    def __call__(self, name, **kw):
        self.events.append((name, kw))


def ready_node(**over):
    now = time.monotonic()
    obs = {'id': 'obj-1', 'valid': True, 'reason': '', 'confidence': 0.9, 'recv': now,
           'base_odom': (1.0, 2.0, 0.3), 'epoch': 0}
    ns = SimpleNamespace(
        fault='', obs=obs, odom=(1.0, 2.0, 0.3), odom_epoch=0, min_confidence=0.5,
        max_observation_age=120.0, joint_time=now, robot_time=now, homed=True, runstopped=False,
        ik_time=now, exec_status=IDLE, exec_time=now, dry_run=False, reobserve=True,
        camera_info=object(), mode_time=now,
        move_srv=SimpleNamespace(service_is_ready=lambda: True),
        stop_srv=SimpleNamespace(service_is_ready=lambda: True),
        traj_mode=SimpleNamespace(service_is_ready=lambda: True),
        trajectory=SimpleNamespace(server_is_ready=lambda: True),
        gripper=SimpleNamespace(server_is_ready=lambda: True),
        tf=SimpleNamespace(can_transform=lambda *a: True))
    for k, v in over.items():
        setattr(ns, k, v)
    return ns


def test_parse_executor_fields():
    e = gm.parse_executor('state=ABORTED armed=0 pending_goal=0 stop_unconfirmed=1 '
                          'sequence_active=0 planning_status={REQUEST_ID=12 X=1 '
                          'PLANNING_FAILED: no path} reason=REQUEST_ID=12 PLANNING_FAILED: no path')
    assert e['state'] == 'ABORTED' and e['request_id'] == 12 and e['stop_unconfirmed']
    assert not e['pending'] and not e['direct'] and 'PLANNING_FAILED' in e['reason']
    assert gm.executor_code(e['reason']) == 'COLLISION'
    assert gm.executor_code('controller busy') == 'CONTROLLER_REJECTED'
    assert gm.executor_code('IK_FAILED: a valid 5-DoF IK target is required') == 'IK_FAILED'


def test_readiness_passes_when_everything_is_fresh():
    assert M.readiness(ready_node(), '') is None


@pytest.mark.parametrize('over,code', [
    ({'obs': None}, 'TARGET_UNKNOWN'),
    ({'fault': 'stop after X not confirmed'}, 'NOT_READY'),
    ({'runstopped': True}, 'NOT_READY'),
    ({'joint_time': 0.0}, 'NOT_READY'),
    ({'exec_status': IDLE.replace('pending_goal=0', 'pending_goal=1')}, 'NOT_READY'),
    ({'exec_time': 0.0}, 'NOT_READY'),
    ({'gripper': SimpleNamespace(server_is_ready=lambda: False)}, 'NOT_READY'),
    ({'camera_info': None}, 'NOT_READY'),
    ({'odom': (1.1, 2.0, 0.3)}, 'STALE_OBSERVATION'),
])
def test_readiness_rejects_before_any_motion(over, code):
    problem = M.readiness(ready_node(**over), '')
    assert problem and problem[0] == code


def test_readiness_observation_checks():
    n = ready_node()
    assert M.readiness(n, 'obj-2')[0] == 'TARGET_UNKNOWN'
    n.obs = dict(n.obs, recv=time.monotonic() - 500)
    assert M.readiness(n, '')[0] == 'STALE_OBSERVATION'
    n = ready_node()
    n.obs = dict(n.obs, confidence=0.2)
    assert M.readiness(n, '')[0] == 'PERCEPTION_INVALID'
    n = ready_node(odom=(5.0, 5.0, 0.0), odom_epoch=1)  # odometry reset: age rule only
    assert M.readiness(n, '') is None


def test_admit_busy_and_duplicate_active():
    n = ready_node(lock=threading.Lock(), active=None, mission=None, stop_requested=False)
    n.claim = lambda job: M.claim(n, job)
    n.readiness = lambda t: M.readiness(n, t)
    assert M.admit(n, 'm1', '') is None and n.active == 'm1'
    n.mission = {'id': 'm1'}
    assert M.admit(n, 'm1', '')[0] == 'DUPLICATE_ACTIVE'
    assert M.admit(n, 'm2', '')[0] == 'BUSY'
    n.active, n.mission = None, None
    n.runstopped = True
    assert M.admit(n, 'm3', '')[0] == 'NOT_READY' and n.active is None  # nothing claimed


class FakeFuture:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value


def move_node(sequence):
    """Executor statuses delivered after the move_to_preview response."""
    n = SimpleNamespace(lock=threading.Lock(), exec_status=IDLE, exec_time=time.monotonic(),
                        move_timeout=3.0, max_approach_error=0.05, mission={'motion': False},
                        stop_requested=False, event=Recorder())
    n.check = lambda: None
    n.wait = lambda f, t, w, code='TIMEOUT': f.result()
    n.solve = lambda pose: {'ok': True, 'base_dx': 0.0, 'reason': 'success', 'achieved': pose}

    def call_async(_):
        def feed():
            for status in sequence:
                time.sleep(0.05)
                with n.lock:
                    n.exec_status, n.exec_time = status, time.monotonic()
        threading.Thread(target=feed, daemon=True).start()
        return FakeFuture(SimpleNamespace(success=True, message='switching to trajectory mode; '
                                                                'planning follows'))
    n.move_srv = SimpleNamespace(wait_for_service=lambda timeout_sec: True, call_async=call_async)
    return n


def test_move_reports_abort_of_async_replan_instead_of_timing_out():
    # Baseline: the executor never armed, so the old mission ignored ABORTED
    # and waited move_timeout. Now the newer REQUEST_ID ends the wait.
    aborted = ('state=ABORTED armed=0 pending_goal=0 stop_unconfirmed=0 sequence_active=0 '
               'planning_status={REQUEST_ID=6 EXECUTION_REJECTED: diagnostics stale or '
               'ERROR/STALE present} reason=REQUEST_ID=6 EXECUTION_REJECTED: diagnostics')
    n = move_node([IDLE.replace('sequence_active=0', 'sequence_active=1'), aborted])
    t0 = time.monotonic()
    with pytest.raises(gm.MissionError) as e:
        M.move_once(n, np.eye(4), 'approach')
    assert e.value.code == 'CONTROLLER_REJECTED' and 'diagnostics' in str(e.value)
    assert time.monotonic() - t0 < 1.0


def test_move_ignores_latched_complete_of_previous_request():
    done = IDLE.replace('REQUEST_ID=4', 'REQUEST_ID=5')
    n = move_node([IDLE, IDLE.replace('state=COMPLETE', 'state=EXECUTING').replace(
        'pending_goal=0', 'pending_goal=1').replace('REQUEST_ID=4', 'REQUEST_ID=5'), done])
    M.move_once(n, np.eye(4), 'pre-grasp')
    assert [e for e, _ in n.event.events][-1] == 'motion_complete'


def test_move_rejects_non_straight_approach_and_timeout():
    rrt = IDLE.replace('REQUEST_ID=4', 'REQUEST_ID=5').replace('PLANNER=DIRECT', 'PLANNER=RRT_CONNECT')
    with pytest.raises(gm.MissionError) as e:
        M.move_once(move_node([rrt]), np.eye(4), 'approach 1/2', straight=True)
    assert e.value.code == 'GRASP_GEOMETRY'
    n = move_node([])
    n.move_timeout = 0.3
    with pytest.raises(gm.MissionError) as e:
        M.move_once(n, np.eye(4), 'lift')
    assert e.value.code == 'TIMEOUT'


def test_only_preflight_rejections_are_retried_once():
    calls = []

    def attempt(code, message):
        def once(pose, label, straight=False):
            calls.append(label)
            raise gm.MissionError(code, message)
        return once
    n = SimpleNamespace(event=Recorder(), prepare_arm=lambda: None)
    n.move_once = attempt('CONTROLLER_REJECTED', 'approach: executor rejected: diagnostics stale')
    with pytest.raises(gm.MissionError):
        M.move(n, np.eye(4), 'approach')
    assert len(calls) == 2
    calls.clear()
    n.move_once = attempt('COLLISION', 'approach: executor rejected: PLANNING_FAILED')
    with pytest.raises(gm.MissionError):
        M.move(n, np.eye(4), 'approach')
    n.move_once = attempt('CONTROLLER_REJECTED', 'approach aborted: controller feedback timeout')
    with pytest.raises(gm.MissionError):
        M.move(n, np.eye(4), 'approach')
    assert len(calls) == 2  # neither collision nor an abort during motion is retried


def test_solve_matches_stamp_and_skips_stale_answers():
    n = SimpleNamespace(lock=threading.Lock(), ik={}, ik_timeout=1.0, stamp_ns=0)
    n.check = lambda: None
    n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1000))
    n.next_stamp = lambda: M.next_stamp(n)
    pose = np.eye(4)
    pose[:3, 3] = [0.0, -0.6, 0.8]

    def publish(msg):
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        old = {'goal_stamp_ns': str(ns - 1), 'success': 'true', '_message': 'success',
               'solved_pose': '0 0 0 0 0 0 1'}
        stale = {'goal_stamp_ns': str(ns), 'success': 'false', '_message': 'stale state',
                 'base_dx': '0.2'}
        fresh = {'goal_stamp_ns': str(ns), 'success': 'false', '_message': 'joint limit',
                 'base_dx': '0.04'}

        def feed():
            for ik in (old, stale, fresh):
                with n.lock:
                    n.ik = ik
                time.sleep(0.1)
        threading.Thread(target=feed, daemon=True).start()
    n.goal_pub = SimpleNamespace(publish=publish)
    a = M.solve(n, pose)
    assert not a['ok'] and a['base_dx'] == pytest.approx(0.04) and a['reason'] == 'joint limit'


def geometry_node(obj):
    n = SimpleNamespace(max_approach_error=0.05, max_approach_change=0.03, max_finger_error=0.10,
                        offset=0.10, object_base=obj)
    n.scene_base = lambda: np.zeros((0, 3))
    n.insert = lambda: 0.04
    n.max_open = lambda: gp.MAX_APERTURE
    return n


def test_geometry_rejects_achieved_frames_that_break_the_plan():
    obj = can(-0.03, -0.65)
    c = gp.plan_grasps(obj, np.zeros((0, 3)))[0]
    n = geometry_node(obj)
    ok = {'achieved': c['grasp']}
    assert M.geometry(n, c, ok, {'achieved': c['pregrasp']})[1] == ''
    r, why = M.geometry(n, c, {'achieved': yawed(c['grasp'], 0.10)}, {'achieved': c['pregrasp']})
    assert 'approach' in why and r['approach_rad'] == pytest.approx(0.10, abs=1e-6)
    r, why = M.geometry(n, c, {'achieved': yawed(c['grasp'], 0.04)},
                        {'achieved': yawed(c['pregrasp'], -0.04)})
    assert 'turns' in why


def test_approach_is_a_straight_line_in_short_steps():
    n = SimpleNamespace(approach_step=0.05)
    poses = []
    n.move = lambda pose, label, straight=False: poses.append((pose.copy(), straight))
    obj = can(-0.03, -0.65)
    c = gp.plan_grasps(obj, np.zeros((0, 3)), 0.10)[0]
    M.approach(n, c)
    assert len(poses) == 2 and all(s for _, s in poses)
    np.testing.assert_allclose(poses[-1][0], c['grasp'])
    mid = (c['pregrasp'][:3, 3] + c['grasp'][:3, 3]) / 2
    np.testing.assert_allclose(poses[0][0][:3, 3], mid, atol=1e-12)


def mission_node(pick, stopped=True):
    n = SimpleNamespace(lock=threading.Lock(), obs={'id': 'obj-1'}, dry_run=False, mission=None,
                        active='m1', results=__import__('collections').OrderedDict(),
                        stop_requested=False, event=Recorder(), fault='')
    n.say = lambda text: n.event('say', text=text)
    n.publish_result = lambda r: n.event('published', **r)
    n.result = lambda *a, **k: M.result(n, *a, **k)
    n.pick = lambda: pick(n)
    n.safe_stop = lambda why: (n.event('safe_stop', why=why), stopped)[1]
    n.perception = lambda on: n.event('perception', enabled=on)
    return n


@pytest.mark.parametrize('pick', [lambda n: (n.event('picking'), 'object held at 6.6 cm')[1],
                                  lambda n: (n.event('picking'), (_ for _ in ()).throw(
                                      gm.MissionError('IK_FAILED', 'unreachable')))])
def test_perception_is_paused_for_the_whole_mission_and_resumed_after(pick):
    n = mission_node(pick)
    M.run_mission(n, 'm1', None)
    seq = [(e, kw.get('enabled')) for e, kw in n.event.events if e in ('perception', 'picking', 'result')]
    assert seq == [('perception', False), ('picking', None), ('result', None), ('perception', True)]


def test_perception_switch_publishes_only_when_enabled():
    sent = []
    n = SimpleNamespace(pause_perception=True, event=Recorder(),
                        perception_pub=SimpleNamespace(publish=lambda m: sent.append(m.data)))
    M.perception(n, False)
    M.perception(n, True)
    n.pause_perception = False
    M.perception(n, False)
    assert sent == [False, True]


def test_failure_after_motion_stops_and_ends_exactly_once():
    def pick(n):
        n.mission['motion'] = True
        n.mission['stage'] = 'APPROACH'
        raise gm.MissionError('MOTION_ABORTED', 'approach 1/2 aborted: dynamic obstacle')
    n = mission_node(pick)
    r = M.run_mission(n, 'm1', None)
    names = [e for e, _ in n.event.events]
    assert names.count('published') == 1 and names.count('result') == 1
    assert 'safe_stop' in names and r['error_code'] == 'MOTION_ABORTED'
    assert r['failed_stage'] == 'APPROACH' and not r['success'] and n.active is None


def test_unconfirmed_stop_is_reported_and_latched():
    def pick(n):
        n.mission['motion'] = True
        raise gm.MissionError('TIMEOUT', 'lift timed out')
    n = mission_node(pick, stopped=False)
    r = M.run_mission(n, 'm1', None)
    assert r['error_code'] == 'STOP_UNCONFIRMED'


def test_success_requires_verification_and_dry_run_never_succeeds():
    n = mission_node(lambda n: 'object held at 6.6 cm')
    assert M.run_mission(n, 'm1', None)['success']
    n = mission_node(lambda n: 'object held at 1.0 cm')
    n.dry_run = True
    r = M.run_mission(n, 'm2', None)
    assert not r['success'] and r['error_code'] == 'DRY_RUN'


def test_finished_mission_id_returns_cached_result_without_motion():
    n = SimpleNamespace(results={'m1': M.result(None, 'm1', 'NONE', 'DONE', 'held')},
                        event=Recorder())
    n.run_mission = lambda *a: pytest.fail('a duplicate goal must not run again')
    n.admit = lambda *a: pytest.fail('a duplicate goal must not claim the arm')
    n.finish_handle = lambda h, r: r
    handle = SimpleNamespace(request=SimpleNamespace(mission_id='m1', target_id=''))
    assert M.execute_goal(n, handle)['success']


def test_unreachable_hint_says_which_way_to_park():
    near = gp.plan_grasps(can(-0.03, -0.43), np.zeros((0, 3)))
    assert 'too close' in M.unreachable_hint(None, near)
    far = gp.plan_grasps(can(-0.03, -0.97), np.zeros((0, 3)))
    assert 'too far' in M.unreachable_hint(None, far)
    ok = gp.plan_grasps(can(-0.03, -0.65), np.zeros((0, 3)))
    assert 'base correction' in M.unreachable_hint(None, ok)


def test_click_starts_a_mission_only_when_a_grasp_is_planned():
    started = []
    n = SimpleNamespace(reobs_wait=None, active=None, event=Recorder(),
                        candidates=[1, 2],
                        get_parameter=lambda name: SimpleNamespace(value=name == 'auto_start'))
    n.say = lambda t: n.event('say', text=t)
    n.publish_result = lambda r: None
    n.result = lambda *a, **k: M.result(n, *a, **k)
    n.admit = lambda mid, target: None
    n.run_mission = lambda mid, h: started.append(mid)
    n.base_from = lambda frame, stamp=None: np.eye(4)
    n.odom_epoch = 0
    n.plan = lambda: None
    obs = SimpleNamespace(object_id='obj-1', source='fake', valid=True, reason='', confidence=0.9,
                          header=SimpleNamespace(stamp=gm.Time().to_msg(), frame_id='base_link'),
                          cloud=gm.point_cloud2.create_cloud_xyz32(
                              Header(frame_id='base_link'), can(-0.03, -0.65).tolist()))
    M.on_observation(n, obs)
    time.sleep(0.2)
    assert len(started) == 1 and started[0].startswith('click-')

    def no_grasp():
        raise gm.MissionError('NO_FEASIBLE_GRASP', 'nothing fits')
    n.plan = no_grasp
    M.on_observation(n, obs)
    time.sleep(0.2)
    assert len(started) == 1  # nothing started without a planned grasp


URDF = (Path(__file__).parents[2] / 'stretch_custom_kinematics' / 'config' / 'captured_robot.urdf').read_text()


def q5(lift, ext, yaw, pitch, roll):
    q = {'joint_lift': lift, 'joint_wrist_yaw': yaw, 'joint_wrist_pitch': pitch,
         'joint_wrist_roll': roll}
    q.update({f'joint_arm_l{i}': ext / 4 for i in range(4)})
    return q


def test_urdf_fk_matches_independent_values():
    chain = gp.urdf_chain(URDF)
    # Values from an independent URDF walk (AUDIT/TEST_RESULTS geometry check).
    np.testing.assert_allclose(gp.fk(chain, q5(0.7, 0, 0, 0, 0))[:3, 3], [-0.0345, -0.4234, 0.7874], atol=2e-4)
    np.testing.assert_allclose(gp.fk(chain, q5(0.7, 0.2, 0, 0, 0))[:3, 3], [-0.0331, -0.6233, 0.7829], atol=2e-4)
    np.testing.assert_allclose(gp.fk(chain, q5(0.6, 0, 3.0, -0.2, 0))[:3, 3], [-0.0014, 0.1068, 0.6449], atol=2e-4)


def test_carry_goes_up_retracts_turns_wrist_and_lowers():
    moves = []
    n = SimpleNamespace(carry_up=0.05, carry_extension=0.0, carry_lift=0.4, lock=threading.Lock(),
                        joints=q5(1.0, 0.25, 0.0, -0.25, 0.0), chain=gp.urdf_chain(URDF))
    n.prepare_arm = lambda: None
    n.stage = lambda *a: None
    n.get_parameter = lambda k: SimpleNamespace(value=[0.0, 0.0, 0.0])
    n.move = lambda pose, label, straight=False: moves.append((label, pose))
    n.move_joints = lambda label, ext=None, **t: M.move_joints(n, label, ext, **t)
    c = {'grasp': gp.fk(n.chain, n.joints)}
    M.carry(n, c)
    assert [m[0] for m in moves] == ['lift', 'retract arm', 'wrist', 'lower']
    assert abs(moves[0][1][2, 3] - c['grasp'][2, 3] - 0.05) < 1e-9
    np.testing.assert_allclose(moves[1][1], gp.fk(n.chain, q5(1.0, 0.0, 0.0, -0.25, 0.0)))
    np.testing.assert_allclose(moves[3][1], gp.fk(n.chain, q5(0.4, 0.25, 0.0, -0.25, 0.0)))
