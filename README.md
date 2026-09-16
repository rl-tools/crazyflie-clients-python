> Note: This repo is a fork of
> https://github.com/bitcraze/crazyflie-clients-python.
> It contains minimal changes for mocap forwarding and a learned-policy
> trigger for the RLtools/Learning to Fly in Seconds controller.

# Installation

Create a venv and install this package into it:

```
pip install -e .
```

# Custom additions

This fork adds:

- Vicon DataStream external-pose forwarding into `cf.extpos.send_extpose(...)`
- `alt1` controller callback support for triggering the learned policy
- Optional joystream phone input, without a system joystick or HID entitlement

## Phone input

Install with `pip install -e '.[joystream]'` (or install your local joystream
checkout into the same environment), then run `JOYSTREAM=1 cfclient`.
Connect the iPhone app to `<computer-ip>:8000`, or open
`http://<computer-ip>:8000` in a phone browser. Select **joystream** under
**Input device** if another controller is selected.

The default Mode 2 mapping uses left stick for yaw/thrust, right for roll/pitch,
A to arm, B for emergency stop, L1 for assisted control, and R1 for `alt1`.
Saved mappings take precedence. Disconnect or 0.5s without input sends neutral
controls while keeping the Crazyflie connection open; input resumes automatically
when the phone sends again. `JOYSTREAM_HOST` and
`JOYSTREAM_PORT` override the default listener (`0.0.0.0:8000`).

# Learning to Fly in Seconds

## Input mapping

Configure the input device as normal and assign `alt1` to one of the
shoulder buttons. `alt1` activates the policy mid-flight.

## Motion Capturing

The client can forward Vicon DataStream poses directly to the Crazyflie.
ROS, `mocap_vicon`, and `rosbridge_server` are not required for this path.

Enable forwarding with:

```
VICON_DATASTREAM=1 cfclient
```

By default this connects to Vicon at `192.154.4.123`, reads the object named
`crazyflie`, uses its root segment, converts Vicon millimeters to meters, and
forwards at 100 Hz.

Runtime overrides:

```
VICON_DATASTREAM_HOST=192.154.4.123
VICON_OBJECT_NAME=crazyflie
VICON_SEGMENT_NAME=crazyflie
VICON_POSITION_SCALE=0.001
VICON_FORWARD_RATE_HZ=100
```

If `VICON_SEGMENT_NAME` is omitted, the root segment for `VICON_OBJECT_NAME`
is used. Set `VICON_DATASTREAM=0` to disable forwarding even when other Vicon
environment variables are present.

### "Too many Packets Lost"

If this error appears, the connection was probably interrupted while the UI
continued to send mocap/extpos messages. Restart the UI, Crazyflie, and
Crazyradio.
