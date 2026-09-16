"""Phone input through joystream, enabled by JOYSTREAM=1."""
import atexit
import logging
import os

try:
    from joystream_input import Receiver
except ImportError as error:
    raise ImportError(
        "Install phone input with: pip install 'cfclient[joystream]'") from error

MODULE_MAIN = "JoystickReader"
MODULE_NAME = "joystream"
logger = logging.getLogger(__name__)


class JoystickReader:
    name = MODULE_NAME
    default_mapping = "joystream"

    def __init__(self):
        self.receiver = Receiver(
            os.environ.get("JOYSTREAM_HOST", "0.0.0.0"),
            int(os.environ.get("JOYSTREAM_PORT", "8000")),
        ).start()
        atexit.register(self.receiver.close)
        logger.info("joystream listening on %s:%s; connect the phone to /ws",
                    self.receiver.host, self.receiver.port)

    def devices(self):
        return [{"id": 0, "name": self.name}]

    def open(self, device_id):
        if device_id != 0:
            raise ValueError("Unknown joystream device")

    def close(self, device_id):
        # Keep listening while input is paused or another device is selected.
        pass

    def read(self, device_id):
        if device_id != 0:
            raise ValueError("Unknown joystream device")
        state = self.receiver.read()
        if not state.connected:
            # Keep the cfclient device open and neutral until input resumes.
            return None
        return [list(state.axes), list(state.buttons)]
