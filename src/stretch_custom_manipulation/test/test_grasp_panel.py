"""Headless smoke test for the operator panel. No command is executed."""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

import rclpy
from PyQt5.QtWidgets import QApplication, QMessageBox


class Future:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class Client:
    def __init__(self, result, requests=None):
        self.result = result
        self.requests = requests

    def service_is_ready(self):
        return True

    def call_async(self, request):
        if self.requests is not None:
            self.requests.append(request)
        return Future(self.result)


def test_panel_controls_are_wired_and_live_run_requires_confirmation(monkeypatch):
    path = Path(__file__).parents[1] / 'scripts' / 'grasp_panel.py'
    spec = importlib.util.spec_from_file_location('grasp_panel', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    launched = []
    monkeypatch.setattr(module.Panel, 'launch',
                        lambda self, command, title: launched.append((command, title)))
    monkeypatch.setattr(module.QMessageBox, 'question', lambda *args: QMessageBox.No)

    rclpy.init()
    app = QApplication.instance() or QApplication([])
    args = SimpleNamespace(dryrun_cmd='dry', live_cmd='live', stop_cmd='stop')
    panel = module.Panel(args)
    try:
        panel.dry_btn.click()
        panel.live_btn.click()
        monkeypatch.setattr(module.QMessageBox, 'question', lambda *args: QMessageBox.Yes)
        panel.live_btn.click()
        panel.down_btn.click()
        panel.services = {name: Client(SimpleNamespace(success=True, message='done'))
                          for name in panel.services}
        parameter_requests = []
        panel.set_params = Client(SimpleNamespace(
            results=[SimpleNamespace(successful=True)]), parameter_requests)
        published = []
        panel.enable_pub = SimpleNamespace(publish=published.append)
        panel.refresh()
        for button in (panel.pick_btn, panel.stop_btn, panel.release_btn, panel.clear_btn):
            button.click()
        panel.auto_start.setEnabled(True)
        panel.auto_start.click()
        panel.toggle_perception()
        app.processEvents()
        assert launched == [('dry', 'grasp dry run'), ('live', 'grasp live run'),
                            ('stop', 'grasp shutdown')]
        assert panel.dry_btn.minimumHeight() >= 44
        for name in panel.services:
            assert f'{name}: done' in panel.log.toPlainText()
        assert parameter_requests[0].parameters[0].value.bool_value
        assert published[-1].data is False
        assert 'Perception pause requested' in panel.log.toPlainText()
    finally:
        panel.close()
        panel.node.destroy_node()
        rclpy.shutdown()
