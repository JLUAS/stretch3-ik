"""Click-through of grasp_panel.py against the isolated grasp sim stack.

Presses the panel's buttons and checks what reaches ROS: auto_start parameter,
mission result, the /perception/enable pause sequence, the gripper opening.
    ws/scripts/sim_isolated.sh bash -c 'ros2 launch <this dir>/grasp_sim.launch.py &
      sleep 8; python3 <this dir>/panel_e2e.py <scripts>/grasp_panel.py /tmp/panel.png'
"""
import importlib.util, json, os, re, sys, time
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

spec = importlib.util.spec_from_file_location('grasp_panel', sys.argv[1])
gp = importlib.util.module_from_spec(spec); spec.loader.exec_module(gp)
rclpy.init()
app = QApplication([])
launched = []
gp.Panel.launch = lambda self, cmd, title: launched.append(title)
panel = gp.Panel(type('A', (), {'dryrun_cmd': 'd', 'live_cmd': 'l', 'stop_cmd': 's'})())
probe = rclpy.create_node('e2e_probe')
latched = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)
enable_seq, events = [], []
probe.create_subscription(Bool, '/perception/enable', lambda m: enable_seq.append(m.data), latched)
probe.create_subscription(String, '/grasp_mission/events', lambda m: events.append(json.loads(m.data)), 100)
fake_enabled = probe.create_publisher(Bool, '/perception/enabled', latched)
click = probe.create_client(Trigger, '/mock/click')
getp = probe.create_client(GetParameters, '/grasp_mission/get_parameters')
results = []

def pump(timeout, until=lambda: False):
    end = time.time() + timeout
    while time.time() < end:
        app.processEvents(); rclpy.spin_once(probe, timeout_sec=0.01)
        if until(): return True
    return False

def check(name, ok, detail=''):
    results.append((name, ok)); print(('PASS ' if ok else 'FAIL ') + name + (f': {detail}' if detail else ''), flush=True)

check('panel starts in the bringup-not-running state', 'not running' in panel.status.text() and not panel.pick_btn.isEnabled(), panel.status.text())
up = pump(90, lambda: panel.pick_btn.isEnabled() and panel.auto_start.isEnabled())
check('mission buttons and auto_start enable once the mission is up', up,
      f'pick={panel.pick_btn.isEnabled()} auto_start={panel.auto_start.isEnabled()}')
check('dry/live start buttons disable while the bringup runs',
      not panel.dry_btn.isEnabled() and not panel.live_btn.isEnabled())
check('auto_start checkbox reads the mission default (true)', panel.auto_start.isChecked())

def param():
    f = getp.call_async(GetParameters.Request(names=['auto_start'])); pump(5, f.done)
    return f.result().values[0].bool_value
QTest.mouseClick(panel.auto_start, Qt.LeftButton); pump(2)
check('unticking auto_start sets the mission parameter to false', param() is False)
QTest.mouseClick(panel.auto_start, Qt.LeftButton); pump(2)
check('ticking it again sets it back to true', param() is True)
QTest.mouseClick(panel.auto_start, Qt.LeftButton); pump(2)

# Same seeded world and operator steps as test/sim/run_trials.py.
import random
sys.path.insert(0, os.path.join(os.path.dirname(sys.argv[1]), '..', 'test', 'sim'))
from run_trials import scenario
world_pub = probe.create_publisher(String, '/mock/world', latched)
world_pub.publish(String(data=json.dumps(scenario(random.Random(0), 0, 2.0, {}))))
pump(3)
look = probe.create_client(Trigger, '/grasp_mission/look')
f = look.call_async(Trigger.Request()); pump(10, f.done)
pump(15, lambda: panel.status.text().startswith(('LOOKING', 'FAILED')))
f = click.call_async(Trigger.Request()); pump(10, f.done)
print('mock click:', f.result().message if f.done() and f.result() else 'no answer', flush=True)
planned = pump(30, lambda: 'call ~/pick' in panel.status.text())
check('a click with auto_start off plans and waits for Pick', planned, panel.status.text()[:120])
check('perception still on after planning (no mission yet)', enable_seq[-1:] == [True], str(enable_seq))

QTest.mouseClick(panel.pick_btn, Qt.LeftButton)
done = pump(240, lambda: panel.result.text().startswith('Last result: pick-'))
check('Pick button runs the mission to a result', done, panel.result.text()[:200])
check('mission succeeded', 'SUCCEEDED' in panel.result.text(), panel.result.text()[:200])
check('perception paused at mission start and resumed at the end',
      enable_seq[-2:] == [False, True], str(enable_seq))
stages = [e.get('stage') for e in events if e.get('event') == 'perception' and e.get('enabled') is False]
check('pause happened before grasp selection (first IK)', stages[:1] == ['ACCEPTED'], str(stages))
pre = next((e for e in events if e.get('event') == 'stage_start' and e.get('stage') == 'MOVE_PREGRASP'), {})
m = re.search(r'opening ([\d.]+) cm.*object x ([-\d.]+)\.\.([-\d.]+) y ([-\d.]+)\.\.([-\d.]+)', pre.get('detail', ''))
if m:
    opening = float(m.group(1)); width_y = (float(m.group(5)) - float(m.group(4))) * 100
    check('opening sized to the object, not 13 cm', opening < 13.0, f'opening {opening} cm, visible depth-axis extent {width_y:.1f} cm')
grip = [e for e in events if e.get('event') == 'gripper']
check('gripper commands: close for transit, open to planned width, squeeze',
      len(grip) >= 3, ', '.join(f"{g['command']*100:.1f}->{g['measured']*100:.1f}cm" for g in grip))

QTest.mouseClick(panel.release_btn, Qt.LeftButton)
check('Release button opens the gripper', pump(20, lambda: 'RELEASED' in panel.status.text()), panel.status.text())
QTest.mouseClick(panel.stop_btn, Qt.LeftButton)
check('Stop button answers', pump(5, lambda: 'stop: stop requested' in panel.log.toPlainText()))
QTest.mouseClick(panel.clear_btn, Qt.LeftButton)
check('Clear fault button answers', pump(5, lambda: 'clear_fault: fault cleared' in panel.log.toPlainText()))

fake_enabled.publish(Bool(data=True))
pump(3, lambda: panel.perception_btn.isEnabled())
n0 = len(enable_seq)
QTest.mouseClick(panel.perception_btn, Qt.LeftButton); pump(2, lambda: len(enable_seq) > n0)
check('Pause perception button publishes false', enable_seq[n0:] == [False], str(enable_seq[n0:]))
panel.grab().save(sys.argv[2])
print('screenshot', sys.argv[2])
fails = [n for n, ok in results if not ok]
print(f'{len(results) - len(fails)}/{len(results)} passed')
panel.node.destroy_node(); probe.destroy_node(); rclpy.shutdown()
sys.exit(1 if fails else 0)
