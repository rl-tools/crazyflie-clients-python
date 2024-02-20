# Crazyflie PC client [![CI](https://github.com/bitcraze/crazyflie-clients-python/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-clients-python/actions?query=workflow%3ACI) [![cfclient](https://snapcraft.io//cfclient/badge.svg)](https://snapcraft.io/cfclient)


The Crazyflie PC client enables flashing and controlling the Crazyflie.
It implements the user interface and high-level control (for example gamepad handling).
The communication with Crazyflie and the implementation of the CRTP protocol to control the Crazyflie is handled by the [cflib](https://github.com/bitcraze/crazyflie-lib-python) project.

## Installation
See the [installation instructions](docs/installation/install.md) in the GitHub docs folder.

## Official Documentation

Check out the [Bitcraze crazyflie-client-python documentation](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/) on our website.

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

### Test code for contribution
Run the automated build locally to test your code

	python3 tools/build/build

## Input mapping


Do the normal assignment, then go to [https://hardwaretester.com/gamepad](https://hardwaretester.com/gamepad) to check the id of an additional button.

Change the mapping in `~/.config/cfclient/input/xxx.json` to include `learnedController`, e.g.:

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