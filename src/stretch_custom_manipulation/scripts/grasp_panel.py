#!/usr/bin/env python3
"""Small operator panel for grasp tests: start/stop the bringup, pause or
resume perception, toggle auto-start, and call the mission services.

Run it on the robot desktop (domain 42 unless ROS_DOMAIN_ID is set):
    ros2 run stretch_custom_manipulation grasp_panel.py
Bringup commands: ~/grasp_test_{dryrun,run,stop}.sh when present (the test
workspace on the robot), otherwise ~/ament_ws/scripts/grasping_{dryrun,run,stop}.sh;
override them with --dryrun-cmd, --live-cmd and --stop-cmd.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '42')

import rclpy  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import GetParameters, SetParameters  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from PyQt5.QtCore import Qt, QTimer  # noqa: E402
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox,  # noqa: E402
                             QHBoxLayout, QLabel, QMessageBox, QPlainTextEdit,
                             QPushButton, QVBoxLayout, QWidget)

MISSION = '/grasp_mission'


class Panel(QWidget):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.node = rclpy.create_node('grasp_panel')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_text, self.status_time = '', 0.0
        self.perception_on = None
        self.node.create_subscription(String, MISSION + '/status', self.on_status, latched)
        self.node.create_subscription(String, MISSION + '/result', self.on_result, latched)
        self.node.create_subscription(Bool, '/perception/enabled', self.on_perception, latched)
        self.enable_pub = self.node.create_publisher(Bool, '/perception/enable', latched)
        self.services = {name: self.node.create_client(Trigger, f'{MISSION}/{name}')
                         for name in ('pick', 'stop', 'release', 'clear_fault')}
        self.get_params = self.node.create_client(GetParameters, MISSION + '/get_parameters')
        self.set_params = self.node.create_client(SetParameters, MISSION + '/set_parameters')
        self.auto_start_known = False
        self.build()
        self.timer = QTimer(self, timeout=self.spin)
        self.timer.start(50)
        self.refresh_timer = QTimer(self, timeout=self.refresh)
        self.refresh_timer.start(1000)
        self.refresh()

    # ----------------------------------------------------------------- layout
    def build(self):
        self.setWindowTitle('Grasp test panel')
        root = QVBoxLayout(self)

        self.status = QLabel('Bringup not running. Start a dry run to begin.')
        self.status.setWordWrap(True)
        self.status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        font = self.status.font()
        font.setPointSize(font.pointSize() + 3)
        font.setBold(True)
        self.status.setFont(font)
        root.addWidget(self.status)
        self.result = QLabel('No mission result yet.')
        self.result.setWordWrap(True)
        self.result.setTextInteractionFlags(Qt.TextSelectableByMouse)
        root.addWidget(self.result)

        bringup = QGroupBox('Bringup')
        row = QHBoxLayout(bringup)
        self.dry_btn = self.button('Start dry run', self.start_dry, row)
        self.live_btn = self.button('Start live run', self.start_live, row)
        self.down_btn = self.button('Shut down bringup', self.shutdown, row)
        root.addWidget(bringup)

        perception = QGroupBox('Perception')
        row = QHBoxLayout(perception)
        self.perception_label = QLabel('Perception state unknown (node not running).')
        row.addWidget(self.perception_label, 1)
        self.perception_btn = self.button('Pause perception', self.toggle_perception, row)
        root.addWidget(perception)

        mission = QGroupBox('Mission')
        grid = QGridLayout(mission)
        self.auto_start = QCheckBox('Start the mission on a valid click (auto_start)')
        self.auto_start.setCursor(Qt.PointingHandCursor)
        self.auto_start.setEnabled(False)
        self.auto_start.toggled.connect(self.set_auto_start)
        grid.addWidget(self.auto_start, 0, 0, 1, 4)
        self.pick_btn = self.button('Pick planned object', lambda: self.call('pick'))
        self.stop_btn = self.button('Stop mission', lambda: self.call('stop'))
        self.release_btn = self.button('Open gripper (release)', lambda: self.call('release'))
        self.clear_btn = self.button('Clear fault', lambda: self.call('clear_fault'))
        for i, b in enumerate((self.pick_btn, self.stop_btn, self.release_btn, self.clear_btn)):
            grid.addWidget(b, 1, i)
        root.addWidget(mission)

        root.addWidget(QLabel('Status history'))
        self.log = QPlainTextEdit(readOnly=True)
        self.log.setMaximumBlockCount(300)
        self.log.setPlaceholderText('Mission status lines and service answers appear here.')
        root.addWidget(self.log, 1)
        self.resize(760, 560)

    def button(self, text, slot, layout=None):
        b = QPushButton(text)
        b.setMinimumHeight(44)
        b.setCursor(Qt.PointingHandCursor)
        b.clicked.connect(slot)
        if layout is not None:
            layout.addWidget(b)
        return b

    # ------------------------------------------------------------------- ROS
    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def note(self, text):
        self.log.appendPlainText(time.strftime('%H:%M:%S  ') + text)

    def on_status(self, msg):
        self.status_text, self.status_time = msg.data, time.monotonic()
        self.status.setText(msg.data)
        self.note(msg.data)

    def on_result(self, msg):
        try:
            r = json.loads(msg.data)
            text = (f'Last result: {r["mission_id"]}: '
                    + ('SUCCEEDED' if r['success'] else f'{r["error_code"]} at {r["failed_stage"]}')
                    + (f', held {r["held_aperture"] * 100:.1f} cm' if r['success'] else '')
                    + f'. {r["message"]}')
        except (ValueError, KeyError):
            text = 'Last result: ' + msg.data
        self.result.setText(text)

    def on_perception(self, msg):
        self.perception_on = msg.data
        self.perception_label.setText('Perception is running: clicks are segmented.' if msg.data
                                      else 'Perception is paused: the arm owns the robot, clicks are refused.')
        self.perception_btn.setText('Pause perception' if msg.data else 'Resume perception')

    def toggle_perception(self):
        on = not self.perception_on if self.perception_on is not None else False
        self.enable_pub.publish(Bool(data=on))
        self.note(f'Perception {"resume" if on else "pause"} requested')

    def call(self, name):
        client = self.services[name]
        if not client.service_is_ready():
            self.note(f'{name}: service not available, is the bringup running?')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.note(
            f'{name}: ' + (f.result().message or ('ok' if f.result().success else 'refused'))
            if f.result() else f'{name}: no answer'))

    def refresh(self):
        up = self.services['pick'].service_is_ready()
        for name, b in (('pick', self.pick_btn), ('stop', self.stop_btn),
                        ('release', self.release_btn), ('clear_fault', self.clear_btn)):
            b.setEnabled(self.services[name].service_is_ready())
            b.setToolTip('' if b.isEnabled() else 'Waiting for the grasp mission (bringup not running)')
        self.dry_btn.setEnabled(not up)
        self.live_btn.setEnabled(not up)
        self.perception_btn.setEnabled(self.perception_on is not None)
        if not up:
            self.auto_start.setEnabled(False)
            self.auto_start_known = False
            if time.monotonic() - self.status_time > 5:
                self.status.setText('Bringup not running. Start a dry run to begin.')
        elif not self.auto_start_known and self.get_params.service_is_ready():
            self.auto_start_known = True
            req = GetParameters.Request(names=['auto_start'])
            self.get_params.call_async(req).add_done_callback(self.on_auto_start)

    def on_auto_start(self, future):
        values = future.result().values if future.result() else []
        if values:
            self.auto_start.blockSignals(True)
            self.auto_start.setChecked(values[0].bool_value)
            self.auto_start.blockSignals(False)
            self.auto_start.setEnabled(True)
        else:
            self.auto_start_known = False

    def set_auto_start(self, on):
        param = Parameter(name='auto_start',
                          value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=on))
        self.set_params.call_async(SetParameters.Request(parameters=[param])).add_done_callback(
            lambda f: self.note(f'auto_start set to {on}' if f.result() and f.result().results[0].successful
                                else 'auto_start could not be set'))

    # --------------------------------------------------------------- bringup
    def launch(self, cmd, title):
        cmd = os.path.expanduser(cmd)
        if not os.path.exists(cmd.split()[0]):
            self.note(f'{title}: {cmd} not found')
            return
        if shutil.which('xterm'):
            subprocess.Popen(['setsid', '-f', 'xterm', '-T', title, '-e', cmd])
        else:
            subprocess.Popen(['setsid', '-f', 'bash', '-c', cmd], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        self.note(f'{title}: started {cmd}')

    def start_dry(self):
        self.launch(self.args.dryrun_cmd, 'grasp dry run')

    def start_live(self):
        answer = QMessageBox.question(
            self, 'Start live run',
            'The arm, gripper and base will move on a valid click (auto_start) or on Pick. '
            'Stand at the runstop. Start the live run?')
        if answer == QMessageBox.Yes:
            self.launch(self.args.live_cmd, 'grasp live run')

    def shutdown(self):
        self.launch(self.args.stop_cmd, 'grasp shutdown')


def default_cmd(kind):
    test = os.path.expanduser(f'~/grasp_test_{kind}.sh')
    return test if os.path.exists(test) else f'~/ament_ws/scripts/grasping_{kind}.sh'


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--dryrun-cmd', default=default_cmd('dryrun'))
    ap.add_argument('--live-cmd', default=default_cmd('run'))
    ap.add_argument('--stop-cmd', default=default_cmd('stop'))
    args, rest = ap.parse_known_args()
    rclpy.init(args=rest)
    app = QApplication(sys.argv[:1])
    # rclpy owns SIGINT and Qt never returns to Python for it: Ctrl-C and
    # `kill` would leave the window running. The 50 ms ROS timer lets these run.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    panel = Panel(args)
    panel.show()
    try:
        app.exec_()
    finally:
        panel.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
