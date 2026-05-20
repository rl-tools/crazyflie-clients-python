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

This fork keeps two small additions on top of the normal `cfclient` UI:

- Vicon DataStream external-pose forwarding into `cf.extpos.send_extpose(...)`
- `alt1` controller callback support for triggering the learned policy

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
