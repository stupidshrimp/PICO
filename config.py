import json
import os
from copy import deepcopy

DEFAULT_CONFIG = {
    "joystick": {"port": "COM14", "baudrate": 9600},
    "crsf": {"port": "COM3", "baudrate": 921600}
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
