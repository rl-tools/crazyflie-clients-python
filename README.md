> **⚠️ Note**: This repo is a fork of [https://github.com/bitcraze/crazyflie-clients-python](https://github.com/bitcraze/crazyflie-clients-python)
> It contains minimal changes that crudely hack mocap and policy switch trigger (for the RLtools/Learning to Fly in Seconds controller) support into the `cfclient` UI. It is embarrasingly crude and I didn't want to publish it but multiple people asked for it, so here it is

# Installation

Create venv and then install this package into it:
```
pip install -e .
```

# Main diff

```diff
diff --git a/src/cfclient/ui/main.py b/src/cfclient/ui/main.py
index 74b3f43..57d9612 100644
--- a/src/cfclient/ui/main.py
+++ b/src/cfclient/ui/main.py
@@ -29,6 +29,7 @@ The main file for the Crazyflie control application.
 import logging
 import sys
 import usb
+import json
 
 import cfclient
 from cfclient.ui.pose_logger import PoseLogger
@@ -65,11 +66,36 @@ from PyQt6.QtWidgets import QLabel
 from PyQt6.QtWidgets import QMenu
 from PyQt6.QtWidgets import QMessageBox
 
+
+import cflib.crtp
+from cflib.crazyflie import Crazyflie
+from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
+from cflib.utils import uri_helper
+from cflib.crtp.crtpstack import CRTPPacket
+from cflib.crtp.crtpstack import CRTPPort
+from cflib.crazyflie.commander import SET_SETPOINT_CHANNEL, META_COMMAND_CHANNEL, TYPE_HOVER 
+
 from .dialogs.cf2config import Cf2ConfigDialog
 from .dialogs.inputconfigdialogue import InputConfigDialogue
 from .dialogs.logconfigdialogue import LogConfigDialogue
 
 
+
+import os
+send_vicon_pos = "VICON_POSE_TOPIC" in os.environ
+if send_vicon_pos:
+    try: 
+        import roslibpy
+        from PyQt6.QtCore import QTimer, QThread, pyqtSignal, QMutex
+    except:
+        send_vicon_pos = False
+import os
+import random
+import struct
+import time
+
+
+
 __author__ = 'Bitcraze AB'
 __all__ = ['MainUI']
 
@@ -79,6 +105,12 @@ logger = logging.getLogger(__name__)
  main_windows_base_class) = (uic.loadUiType(cfclient.module_path +
                                             '/ui/main.ui'))
 
+def send_learned_policy_packet(cf):
+    pk = CRTPPacket()
+    pk.port = CRTPPort.COMMANDER_GENERIC
+    pk.channel = META_COMMAND_CHANNEL
+    pk.data = struct.pack('<B', 1)
+    cf.send_packet(pk)
 
 class UIState:
     DISCONNECTED = 0
@@ -91,6 +123,41 @@ class BatteryStates:
     BATTERY, CHARGING, CHARGED, LOW_POWER = list(range(4))
 
 
+class ROSWorker(QThread):
+    vicon_data_signal = pyqtSignal(dict)
+    
+    def __init__(self):
+        super().__init__()
+        self.mutex = QMutex()
+        self.running = True
+
+    def run(self):
+        try:
+            ros = roslibpy.Ros(host='localhost', port=9090)
+            ros.run(timeout=5)
+            
+            listener = roslibpy.Topic(ros, os.environ.get("VICON_POSE_TOPIC", "/vicon/crazyflie/pose"), 'geometry_msgs/PoseStamped')
+            listener.subscribe(self.process_vicon_message)
+            
+            while self.running and ros.is_connected:
+                time.sleep(0.001)
+                
+        except Exception as e:
+            print(f"ROS Error: {e}")
+        finally:
+            if ros.is_connected:
+                ros.close()
+
+    def process_vicon_message(self, message):
+        self.mutex.lock()
+        self.vicon_data_signal.emit(message)
+        self.mutex.unlock()
+
+    def stop(self):
+        self.running = False
+        self.wait()
+
+
 class MainUI(QtWidgets.QMainWindow, main_window_class):
     connectionLostSignal = pyqtSignal(str, str)
     connectionInitiatedSignal = pyqtSignal(str)
@@ -205,6 +272,16 @@ class MainUI(QtWidgets.QMainWindow, main_window_class):
             lambda *args: self._disable_input or
             self.cf.commander.send_zdistance_setpoint(*args))
 
+        self.learned_controller_counter = 0
+        def learned_controller_callback(*args):
+            if self.learned_controller_counter % 10 == 0:
+                print("learned controller callback")
+                self._disable_input or send_learned_policy_packet(self.cf) #.commander.send_learned_controller()
+            self.learned_controller_counter += 1
+            return True
+
+        self.joystickReader.alt1_updated.add_callback(learned_controller_callback)
+
         self.joystickReader.hover_input_updated.add_callback(
             self.cf.commander.send_hover_setpoint)
 
@@ -320,6 +397,14 @@ class MainUI(QtWidgets.QMainWindow, main_window_class):
         # We only want to warn about USB permission once
         self._permission_warned = False
 
+        if send_vicon_pos:
+            self.ros_worker = ROSWorker()
+            self.ros_worker.vicon_data_signal.connect(self.handle_vicon_data)
+            self.ros_worker.start()
+
+        self.vicon_counter = 0
+        self.last_vicon_message = None
+
     def create_tab_toolboxes(self, tabs_menu_item, toolboxes_menu_item, tab_widget):
         loaded_tab_toolboxes = {}
 
@@ -888,6 +973,34 @@ class MainUI(QtWidgets.QMainWindow, main_window_class):
         self.close()
         sys.exit(0)
 
+    def handle_vicon_data(self, message):
+        # print("Received Vicon data:", message)
+        if self.uiState == UIState.CONNECTED:
+            try:
+                now = time.time()
+                if self.last_vicon_message is None or (now - self.last_vicon_message) > 0.01:
+                    self.last_vicon_message = now
+                    pose = message['pose']
+                    self.cf.extpos.send_extpose(
+                        pose['position']['x'],
+                        pose['position']['y'],
+                        pose['position']['z'],
+                        pose['orientation']['x'],
+                        pose['orientation']['y'],
+                        pose['orientation']['z'],
+                        pose['orientation']['w']
+                    )
+                    self.vicon_counter += 1
+                    if self.vicon_counter % 10 == 0:
+                        print(f'Sending pose: {pose["position"]} orientation: {pose["orientation"]}')
+                
+            except Exception as e:
+                print(f"Error sending pose: {e}")
+
+    def closeEvent(self, event):
+        if hasattr(self, 'ros_worker'):
+            self.ros_worker.stop()
+
 
 class ScannerThread(QThread):
 
diff --git a/src/cfclient/utils/input/__init__.py b/src/cfclient/utils/input/__init__.py
index 6d066e0..041e8ff 100644
--- a/src/cfclient/utils/input/__init__.py
+++ b/src/cfclient/utils/input/__init__.py
@@ -413,7 +413,6 @@ class JoystickReader(object):
                                 "Exception while doing callback from "
                                 "input-device for assited "
                                 "control: {}".format(e))
-
                 if data.toggled.estop:
                     try:
                         self.emergency_stop_updated.call(data.estop)
@@ -432,6 +431,12 @@ class JoystickReader(object):
                     except Exception as e:
                         logger.warning("Exception while doing callback from"
                                        "input-device for alt1: {}".format(e))
+                if data.alt1:
+                    try:
+                        self.alt1_updated.call(data.alt1)
+                    except Exception as e:
+                        logger.warning("Exception while doing callback from"
+                                       "input-device for alt1: {}".format(e))
                 if data.toggled.alt2:
                     try:
                         self.alt2_updated.call(data.alt2)
@@ -453,7 +458,7 @@ class JoystickReader(object):
                     vx = data.roll
                     vy = data.pitch
                     vz = data.thrust
-                    yawrate = data.yaw
+                    yawrate = -data.yaw
                     # The odd use of vx and vy is to map forward on the
                     # physical joystick to positive X-axis
                     self.assisted_input_updated.call(vy, -vx, vz, yawrate)
@@ -473,7 +478,7 @@ class JoystickReader(object):
                     if self._target_height < MIN_HOVER_HEIGHT:
                         self._target_height = MIN_HOVER_HEIGHT
 
-                    yawrate = data.yaw
+                    yawrate = -data.yaw
                     # The odd use of vx and vy is to map forward on the
                     # physical joystick to positive X-axis
                     self.hover_input_updated.call(vy, -vx, yawrate,
@@ -499,7 +504,7 @@ class JoystickReader(object):
                             and data.assistedControl:
                         roll = data.roll + self.trim_roll
                         pitch = data.pitch + self.trim_pitch
-                        yawrate = data.yaw
+                        yawrate = -data.yaw
                         # Scale thrust to a value between -1.0 to 1.0
                         vz = (data.thrust - 32767) / 32767.0
                         # Integrate velocity setpoint

```

# Learning to Fly in Seconds

## Input mapping

Please do the input configuration as normal and assign `alt1` to e.g. one of the shoulder buttons. `alt1` is used to activate the policy mid-flight


## Motion Capturing

```
roscore
```

```
roslaunch mocap_vicon vicon.launch
```

```
roslaunch rosbridge_server rosbridge_websocket.launch
```


=> `.vscode/launch.json`



### "Too many Packets Lost"

If this error comes up, probably the connection was interrupted by e.g. rebooting the CF without disconnecting from the UI (continuing to send Mocap/Extpos messages). Restart the UI, Crazyflie and CrazyRadio in random orders and it should work again...