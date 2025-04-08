> **⚠️ Note**: This repo is a fork of [https://github.com/bitcraze/crazyflie-clients-python](https://github.com/bitcraze/crazyflie-clients-python)
> It contains minimal changes that crudely hack mocap and policy switch trigger (for the RLtools/Learning to Fly in Seconds controller) support into the `cfclient` UI. It is embarrasingly crude and I didn't want to publish it but multiple people asked for it, so here it is

# Main diff

```diff
diff --git a/src/cfclient/ui/main.py b/src/cfclient/ui/main.py
index b3a3b97..556f315 100644
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
+            listener = roslibpy.Topic(ros, os.environ.get("VICON_POSE_TOPIC", "/vicon/crazyflie/pose"), 'geometry_msgs/PoseStamped', throttle_rate=50)
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
+        self.joystickReader.learned_controller_input_updated.add_callback(learned_controller_callback)
+
         self.joystickReader.hover_input_updated.add_callback(
             self.cf.commander.send_hover_setpoint)
 
@@ -320,6 +397,13 @@ class MainUI(QtWidgets.QMainWindow, main_window_class):
         # We only want to warn about USB permission once
         self._permission_warned = False
 
+        if send_vicon_pos:
+            self.ros_worker = ROSWorker()
+            self.ros_worker.vicon_data_signal.connect(self.handle_vicon_data)
+            self.ros_worker.start()
+
+        self.vicon_counter = 0
+
     def create_tab_toolboxes(self, tabs_menu_item, toolboxes_menu_item, tab_widget):
         loaded_tab_toolboxes = {}
 
@@ -888,6 +972,31 @@ class MainUI(QtWidgets.QMainWindow, main_window_class):
         self.close()
         sys.exit(0)
 
+    def handle_vicon_data(self, message):
+        # print("Received Vicon data:", message)
+        if self.uiState == UIState.CONNECTED:
+            try:
+                pose = message['pose']
+                self.cf.extpos.send_extpose(
+                    pose['position']['x'],
+                    pose['position']['y'],
+                    pose['position']['z'],
+                    pose['orientation']['x'],
+                    pose['orientation']['y'],
+                    pose['orientation']['z'],
+                    pose['orientation']['w']
+                )
+                self.vicon_counter += 1
+                if self.vicon_counter % 10 == 0:
+                    print(f'Sending pose: {pose["position"]} orientation: {pose["orientation"]}')
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
index 6d066e0..a723b46 100644
--- a/src/cfclient/utils/input/__init__.py
+++ b/src/cfclient/utils/input/__init__.py
@@ -178,6 +178,7 @@ class JoystickReader(object):
         self.assisted_control_updated = Caller()
         self.alt1_updated = Caller()
         self.alt2_updated = Caller()
+        self.learned_controller_input_updated = Caller()
 
         # Call with 3 bools (rp_limiting, yaw_limiting, thrust_limiting)
         self.limiting_updated = Caller()
@@ -413,6 +414,20 @@ class JoystickReader(object):
                                 "Exception while doing callback from "
                                 "input-device for assited "
                                 "control: {}".format(e))
+                if data.toggled.learnedController:
+                    print("toggled learned controller")
+                    try:
+                        self.learned_controller_input_updated.call(data.learnedController)
+                    except Exception as e:
+                        logger.warning("Exception while doing callback from"
+                                       "input-device for learned_controller: {}".format(e))
+                if data.learnedController:
+                    print(" learned controller")
+                    try:
+                        self.learned_controller_input_updated.call(data.learnedController)
+                    except Exception as e:
+                        logger.warning("Exception while doing callback from"
+                                       "input-device for learned_controller: {}".format(e))
 
                 if data.toggled.estop:
                     try:
diff --git a/src/cfclient/version.json b/src/cfclient/version.json
new file mode 100644
index 0000000..e5374a0
--- /dev/null
+++ b/src/cfclient/version.json
@@ -0,0 +1 @@
+{"version": "2023.10rc1.post44.dev0+61be0d8"}
\ No newline at end of file

```

# Learning to Fly in Seconds

## Input mapping


Do the normal assignment, then use `python3 test_gamepad_inputs.py` to check the id of an additional button and insert it into the config under `learnedController` as described in the following.

Change the mapping in `~/.config/cfclient/input/xxx.json` to include `learnedController`, e.g.:
Note: newer versions appear to save the config in `/Users/jonas/Library/Application Support/cfclient/config.json`

```
{
  "inputconfig": {
    "inputdevice": {
      "axis": [
        {
          "id": 1,
          "scale": -1.0,
          "key": "thrust",
          "name": "thrust",
          "type": "Input.AXIS"
        },
        {
          "id": 4,
          "scale": -1.0,
          "key": "pitch",
          "name": "pitch",
          "type": "Input.AXIS"
        },
        {
          "id": 3,
          "scale": 1.0,
          "key": "roll",
          "name": "roll",
          "type": "Input.AXIS"
        },
        {
          "id": 0,
          "scale": 1.0,
          "key": "yaw",
          "name": "yaw",
          "type": "Input.AXIS"
        },
        {
          "id": 4,
          "scale": 1.0,
          "key": "assistedControl",
          "name": "assistedControl",
          "type": "Input.BUTTON"
        },
        {
          "id": 5,
          "scale": 1.0,
          "key": "learnedController",
          "name": "learnedController",
          "type": "Input.BUTTON"
        }
      ],
      "name": "PS3_Mode_1_8bitdo",
      "updateperiod": 10
    }
  }
}
```

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
