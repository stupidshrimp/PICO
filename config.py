import json
import os
from copy import deepcopy

DEFAULT_CONFIG = {
    "joystick": {
        "port": "COM14",
        "baudrate": 9600,
        "deadzone": 0,
        "sensitivity": 100,
        "smoothing": 0,
    },
    "crsf": {
        "port": "COM3",
        "baudrate": 921600,
        # Interval in ms (~300 Hz)
        "packet_interval": 3,
    },
    # ``device_index`` is optional so that automatic capture device detection
    # can run when no explicit index is configured.
    "vtx": {},
    "warnings": {
        # Trigger when airspeed < stall_airspeed and altitude > stall_altitude
        "stall_airspeed": 10,
        "stall_altitude": 50,
        # Trigger when altitude < altitude_alarm_altitude and airspeed > altitude_alarm_airspeed
        "altitude_alarm_airspeed": 30,
        "altitude_alarm_altitude": 20,
        # Trigger when |roll| > roll_angle
        "roll_angle": 45,
        # Enable/disable individual telemetry alarms
        "stall_alarm_enabled": True,
        "altitude_alarm_enabled": True,
        "bank_angle_alarm_enabled": True,
    },
    "map": {
        # Initial center [lat, lon] and zoom level for the offline map
        "center": [0, 0],
        "zoom": 2,
    },
}

def load_config(path: str = "config.json"):
    """Load configuration from a JSON file and environment variables.

    Environment variables take precedence over file values which in turn
    override the hard-coded defaults.
    """
    config = deepcopy(DEFAULT_CONFIG)

    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as fh:
                file_config = json.load(fh)
            for section, values in file_config.items():
                if section in config and isinstance(values, dict):
                    config[section].update(values)
                else:
                    config[section] = values
        except Exception as exc:
            print(f"Failed to read config file '{path}': {exc}")

    # Environment variable overrides
    joystick_port = os.getenv("JOYSTICK_PORT")
    joystick_baud = os.getenv("JOYSTICK_BAUDRATE")
    crsf_port = os.getenv("CRSF_PORT")
    crsf_baud = os.getenv("CRSF_BAUDRATE")

    if joystick_port:
        config["joystick"]["port"] = joystick_port
    if joystick_baud:
        try:
            config["joystick"]["baudrate"] = int(joystick_baud)
        except ValueError:
            print(f"Invalid JOYSTICK_BAUDRATE '{joystick_baud}', using default {config['joystick']['baudrate']}")

    if crsf_port:
        config["crsf"]["port"] = crsf_port
    if crsf_baud:
        try:
            config["crsf"]["baudrate"] = int(crsf_baud)
        except ValueError:
            print(f"Invalid CRSF_BAUDRATE '{crsf_baud}', using default {config['crsf']['baudrate']}")

    return config


def save_config(config, path: str = "config.json"):
    """Persist configuration to a JSON file."""
    try:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(config, fh, indent=4)
    except Exception as exc:
        print(f"Failed to write config file '{path}': {exc}")
