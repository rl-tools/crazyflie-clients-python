"""Run with: python -m unittest discover -s tests -v (no drone required)."""
import atexit
from contextlib import ExitStack
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
from urllib.request import urlopen

from websockets.sync.client import connect


class JoystreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import cfclient
        cls.config_dir = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.config_dir.cleanup)
        cls.config_patch = patch.object(cfclient, "config_path", cls.config_dir.name)
        cls.config_patch.start()
        cls.addClassCleanup(cls.config_patch.stop)
        with patch.dict(os.environ, JOYSTREAM="0"):
            from cfclient.utils.input import JoystickReader, inputreaders
            from cfclient.utils.config import Config
            from cfclient.utils.input.inputreaders.joystream import JoystickReader as PhoneReader
        cls.InputReader = JoystickReader
        cls.readers = inputreaders
        cls.PhoneReader = PhoneReader
        cls.Config = Config

    def setUp(self):
        self.Config().set("device_config_mapping", {})
        self.Config().set("input_device", "")
        with patch.dict(os.environ, JOYSTREAM_HOST="127.0.0.1", JOYSTREAM_PORT="0"):
            self.phone = self.PhoneReader()
        self.addCleanup(atexit.unregister, self.phone.receiver.close)
        self.addCleanup(self.phone.receiver.close)
        self.connections = ExitStack()
        self.addCleanup(self.connections.close)
        self.device = self.readers.InputDevice("joystream", 0, self.phone)
        device_patch = patch.object(self.readers, "devices", return_value=[self.device])
        device_patch.start()
        self.addCleanup(device_patch.stop)
        self.input = self.InputReader(do_device_discovery=False)
        self.input._read_timer = Mock()
        self.device.input = self.input
        self.updates, self.errors = [], []
        self.input.input_updated.add_callback(lambda *values: self.updates.append(values))
        self.input.device_error.add_callback(self.errors.append)
        self.url = f"http://127.0.0.1:{self.phone.receiver.port}"

    def start_input(self):
        self.input.set_input_map("joystream", "joystream")
        self.input.start_input("joystream")

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail("Timed out waiting for input")
            time.sleep(0.005)

    def connect_phone(self, **state):
        ws = self.connections.enter_context(connect(
            self.url.replace("http:", "ws:") + "/ws",
            open_timeout=2, close_timeout=0.2))
        ws.send(json.dumps(state))
        self.wait_for(lambda: self.phone.receiver.read().connected)
        return ws

    def test_library_only_loads_when_enabled(self):
        script = """
import os, sys
from cfclient.utils.input import inputreaders
enabled = os.environ.get('JOYSTREAM') == '1'
assert ('joystream_input' in sys.modules) == enabled
assert any(r.name == 'joystream' for r in inputreaders.initialized_readers) == enabled
"""
        for value in (None, "0", "true", "1"):
            with self.subTest(value=value):
                env = dict(os.environ, JOYSTREAM_HOST="127.0.0.1", JOYSTREAM_PORT="0")
                env.pop("JOYSTREAM", None)
                if value is not None:
                    env["JOYSTREAM"] = value
                result = subprocess.run([sys.executable, "-c", script], env=env,
                                        capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_browser_page_is_served(self):
        with urlopen(self.url, timeout=2) as response:
            self.assertEqual(response.status, 200)
            self.assertEqual(response.headers.get_content_type(), "text/html")
            self.assertIn(b"/ws", response.read())

    def test_missing_dependency_reports_install_command(self):
        script = """
import sys
sys.modules['joystream_input'] = None
from cfclient.utils.input import inputreaders
assert not any(r.name == 'joystream' for r in inputreaders.initialized_readers)
"""
        result = subprocess.run([sys.executable, "-c", script],
                                env=dict(os.environ, JOYSTREAM="1"),
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("pip install 'cfclient[joystream]'", result.stderr)

    def test_waiting_never_generates_thrust_even_with_non_springy_mapping(self):
        self.start_input()
        self.input.springy_throttle = False
        self.input.read_input()
        self.assertEqual(self.updates[-1], (0, 0, 0, 0))
        self.assertEqual(self.errors, [])
        self.input._read_timer.stop.assert_not_called()
        axes, buttons, data = self.device.read(include_raw=True)
        self.assertEqual((axes, buttons, data.thrust), ([], [], 0))

    def test_axes_and_buttons_use_default_mapping(self):
        self.start_input()
        self.connect_phone(lx=0.6, ly=-0.7, rx=0.4, ry=-0.5, a=1, b=1, l1=1, r1=1)
        self.device.limit_rp = self.device.limit_yaw = self.device.limit_thrust = False
        self.device.set_dead_band(0)
        data = self.device.read()
        self.assertEqual((data.roll, data.pitch, data.yaw, data.thrust), (0.4, 0.5, 0.6, 0.7))
        self.assertTrue(all((data.arm, data.estop, data.assistedControl, data.alt1)))

    def test_input_callback_receives_scaled_controls(self):
        self.start_input()
        self.device.set_dead_band(0)
        self.connect_phone(lx=0.6, ly=-0.7, rx=0.4, ry=-0.5)
        self.input.read_input()
        for actual, expected in zip(self.updates[-1], (12, 15, 100, 45500)):
            self.assertAlmostEqual(actual, expected)

    def test_disconnect_keeps_input_open_and_reconnects_automatically(self):
        self.start_input()
        ws = self.connect_phone(ly=-0.8)
        self.input.read_input()
        self.assertGreater(self.updates[-1][3], 0)
        ws.close()
        self.wait_for(lambda: not self.phone.receiver.read().connected)
        for _ in range(3):
            self.input.read_input()
            self.assertEqual(self.updates[-1], (0, 0, 0, 0))
        self.assertEqual(self.errors, [])
        self.input._read_timer.stop.assert_not_called()
        ws = self.connect_phone()
        self.input.read_input()
        self.assertEqual(self.updates[-1][3], 0)
        ws.send(json.dumps({"ly": -0.8}))
        self.wait_for(lambda: self.phone.receiver.read().ly == -0.8)
        self.input.read_input()
        self.assertGreater(self.updates[-1][3], 0)
        self.input._read_timer.start.assert_called_once()

    def test_silent_connection_is_neutral_and_resumes_on_next_packet(self):
        self.start_input()
        ws = self.connect_phone(ly=-0.8)
        self.input.read_input()
        self.wait_for(lambda: not self.phone.receiver.read().connected)
        self.input.read_input()
        self.assertEqual(self.updates[-1], (0, 0, 0, 0))
        self.assertEqual(self.errors, [])
        self.input._read_timer.stop.assert_not_called()
        ws.send(json.dumps({}))
        self.wait_for(lambda: self.phone.receiver.read().connected)
        self.input.read_input()
        self.assertEqual(self.updates[-1][3], 0)

    def test_disconnect_bypasses_non_springy_thrust_mapping(self):
        self.start_input()
        self.input.springy_throttle = False
        ws = self.connect_phone(ly=-1)
        self.input.read_input()
        self.assertGreater(self.updates[-1][3], 0)
        ws.close()
        self.wait_for(lambda: not self.phone.receiver.read().connected)
        self.input.read_input()
        self.assertEqual(self.updates[-1], (0, 0, 0, 0))
        self.assertEqual(self.errors, [])

    def test_disconnect_releases_assisted_control(self):
        self.start_input()
        self.input.set_assisted_control(self.input.ASSISTED_CONTROL_POSHOLD)
        assisted = []
        self.input.assisted_input_updated.add_callback(lambda *values: assisted.append(values))
        ws = self.connect_phone(ly=-0.8, l1=1)
        self.input.read_input()
        self.assertEqual(len(assisted), 1)
        self.assertFalse(self.device.limit_thrust)
        ws.close()
        self.wait_for(lambda: not self.phone.receiver.read().connected)
        self.input.read_input()
        self.assertEqual(len(assisted), 1)
        self.assertTrue(self.device.limit_thrust)
        self.assertTrue(self.device.limit_rp)
        self.assertEqual(self.updates[-1], (0, 0, 0, 0))
        self.assertEqual(self.errors, [])

    def check_button_release(self, timeout=False):
        self.start_input()
        # Include buttons outside the default map, and exercise every phone
        # button through cfclient's state and event paths.
        for index, key in ((2, "alt2"), (3, "pitchPos"), (6, "rollNeg"), (7, "muxswitch")):
            self.device.input_map[f"Input.BUTTON-{index}"] = {
                "type": "Input.BUTTON", "key": key,
            }
        events = {name: [] for name in (
            "arm", "alt1", "alt2", "assisted_control", "emergency_stop")}
        for name, values in events.items():
            getattr(self.input, name + "_updated").add_callback(values.append)
        pressed = dict.fromkeys(("a", "b", "x", "y", "l1", "r1", "select", "start"), 1)
        ws = self.connect_phone(**pressed)
        self.input.read_input()
        self.assertEqual(sum(self.device.data._prev_btn_values.values()), 8)
        self.assertTrue(all(values and values[-1] is True for values in events.values()))
        for values in events.values():
            values.clear()

        if not timeout:
            ws.close()
        self.wait_for(lambda: not self.phone.receiver.read().connected)
        self.input.read_input()
        self.assertEqual(self.phone.receiver.read().buttons, (0,) * 8)
        self.assertEqual(self.phone.receiver._state.buttons, (0,) * 8)
        data = self.device.data
        self.assertFalse(any(data.get(key) for key in data._buttons))
        self.assertFalse(any(data._prev_btn_values.values()))
        self.assertEqual(sum(data.toggled.values()), 8)
        # Arm is intentionally a press-only action in cfclient.
        self.assertEqual(events["arm"], [])
        for name in ("alt1", "alt2", "assisted_control", "emergency_stop"):
            self.assertEqual(events[name], [False])
        releases = {name: list(values) for name, values in events.items()}
        for _ in range(5):
            self.input.read_input()
        self.assertEqual(events, releases)
        self.assertFalse(any(data.toggled.values()))

        if timeout:
            ws.send("{}")
            self.wait_for(lambda: self.phone.receiver.read().connected)
        else:
            ws = self.connect_phone()
        self.input.read_input()
        self.assertEqual(events, releases)
        self.assertFalse(any(data._prev_btn_values.values()))
        ws.send(json.dumps(pressed))
        self.wait_for(lambda: self.phone.receiver.read().a == 1)
        self.input.read_input()
        self.assertTrue(all(values[-1] is True for values in events.values()))
        self.assertEqual(self.errors, [])
        self.input._read_timer.stop.assert_not_called()

    def test_disconnect_clears_all_buttons_caches_and_repeated_presses(self):
        self.check_button_release()

    def test_timeout_clears_all_buttons_caches_and_repeated_presses(self):
        self.check_button_release(timeout=True)

    def test_pausing_keeps_listener_available(self):
        self.start_input()
        self.input.pause_input()
        self.connect_phone(ly=-0.8)
        self.input.resume_input()
        self.input.read_input()
        self.assertGreater(self.updates[-1][3], 0)

    def test_saved_mapping_takes_precedence(self):
        self.assertEqual(self.input.get_saved_device_mapping("joystream"), "joystream")
        self.Config().get("device_config_mapping")["joystream"] = "custom"
        self.assertEqual(self.input.get_saved_device_mapping("joystream"), "custom")

    def test_busy_port_reports_initialization_failure(self):
        env = dict(os.environ, JOYSTREAM="1", JOYSTREAM_HOST="127.0.0.1",
                   JOYSTREAM_PORT=str(self.phone.receiver.port))
        script = """
from cfclient.utils.input import inputreaders
assert not any(r.name == 'joystream' for r in inputreaders.initialized_readers)
"""
        result = subprocess.run([sys.executable, "-c", script], env=env,
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Could not initialize [joystream]", result.stderr)

    def test_phone_disconnect_keeps_menu_selected_and_radio_link_open(self):
        # Exercise the real menu handlers without constructing a Crazyflie or
        # starting the scanner, radio, Vicon, or the rest of the main window.
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PyQt6.QtGui import QAction
        from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QMenu
        from cfclient.ui.main import MainUI

        app = QApplication.instance() or QApplication([])

        class InputMenu(QMainWindow):
            device_discovery = MainUI.device_discovery
            _inputdevice_selected = MainUI._inputdevice_selected
            _inputconfig_selected = MainUI._inputconfig_selected
            _mux_selected = MainUI._mux_selected
            _get_dev_status = MainUI._get_dev_status
            _update_input_device_footer = MainUI._update_input_device_footer
            _display_input_device_error = MainUI._display_input_device_error

        window = InputMenu()
        window.cf = Mock()
        window.joystickReader = self.input
        window._available_devices = ()
        window._statusbar_label = QLabel(window)
        role = QMenu("Device", window)
        mux = QAction("Normal", window, checkable=True)
        mux.setData((self.input._selected_mux, [role]))
        mux.toggled.connect(window._mux_selected)
        window._all_role_menus = [{"rolemenu": role, "muxmenu": mux}]
        window._all_mux_nodes = [mux]
        window.device_discovery([self.device])
        action = role.actions()[0]
        self.assertTrue(action.isChecked())
        self.assertEqual(self.device.input_map_name, "joystream")
        self.input.device_error.add_callback(window._display_input_device_error)
        ws = self.connect_phone(ly=-0.8)
        self.input.read_input()
        ws.close()
        self.wait_for(lambda: not self.phone.receiver.read().connected)
        with patch("cfclient.ui.main.QMessageBox.critical") as popup:
            self.input.read_input()
        popup.assert_not_called()
        window.cf.close_link.assert_not_called()
        self.assertEqual(self.updates[-1], (0, 0, 0, 0))
        self.assertTrue(action.isChecked())
        self.input._read_timer.stop.assert_not_called()
        self.connect_phone(ly=-0.8)
        self.input.read_input()
        self.assertGreater(self.updates[-1][3], 0)
        self.input._read_timer.start.assert_called_once()
        window.close()
        app.processEvents()


if __name__ == "__main__":
    unittest.main()
