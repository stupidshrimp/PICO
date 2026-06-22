import json
import os
from copy import deepcopy

ALLOWED_ATTITUDE_PACKET_RATES_HZ = (100, 250, 500)
DEFAULT_ATTITUDE_PACKET_RATE_HZ = 250

# Sentinel shown in the port dropdowns when no serial device is selected.
NOT_CONNECTED = "Not connected"


def resolve_port_selection(current, desired, available_ports):
    """Decide how a serial-port dropdown should react to the live device list.

    ``current`` is the port the dropdown is presently showing, ``desired`` is
    the port the operator last intentionally selected (remembered across an
    unplug), and ``available_ports`` are the device names currently reported by
    the OS.  Returns an ``(action, target)`` tuple:

    * ``("keep", current)`` -- the active port is still present; leave it alone.
    * ``("reconnect", desired)`` -- the desired port has (re)appeared, so the
      caller should reconnect to it.  This is what lets a joystick/transmitter
      USB re-plug restore the link automatically, without the operator having to
      reopen the config page and reselect the port.
    * ``("disconnect", None)`` -- the desired port is gone; the caller should
      show "Not connected" and drop the live link while remembering ``desired``
      for a later automatic reconnect.
    """

    if current != NOT_CONNECTED and current in available_ports:
        return ("keep", current)
    if desired != NOT_CONNECTED and desired in available_ports:
        return ("reconnect", desired)
    return ("disconnect", None)


def packet_interval_ms_from_rate(rate_hz: int) -> int:
    """Return the CRSF RC packet interval for a supported attitude rate."""

    try:
        rate = int(rate_hz)
    except (TypeError, ValueError):
        rate = DEFAULT_ATTITUDE_PACKET_RATE_HZ
    if rate not in ALLOWED_ATTITUDE_PACKET_RATES_HZ:
        rate = DEFAULT_ATTITUDE_PACKET_RATE_HZ
    return max(1, int(round(1000 / rate)))


def packet_rate_hz_from_interval(interval_ms: int) -> int:
    """Return the supported attitude packet rate nearest to an interval."""

    try:
        interval = max(1, int(interval_ms))
    except (TypeError, ValueError):
        return DEFAULT_ATTITUDE_PACKET_RATE_HZ

    actual_rate = 1000 / interval
    return min(
        ALLOWED_ATTITUDE_PACKET_RATES_HZ,
        key=lambda supported_rate: abs(supported_rate - actual_rate),
    )


def normalise_packet_interval_ms(interval_ms: int) -> int:
    """Clamp a configured CRSF interval to one of the supported rates."""

    return packet_interval_ms_from_rate(packet_rate_hz_from_interval(interval_ms))


DEFAULT_CONFIG = {
    "joystick": {
        "port": "COM14",
        "baudrate": 9600,
        "deadzone": 0,
        "sensitivity": 100,
        "yaw_sensitivity": 100,
        "smoothing": 0,
    },
    "crsf": {
        "port": "COM3",
        "baudrate": 921600,
        # Worker-thread RC frame interval in ms (10/4/2 ms = 100/250/500 Hz)
        "packet_interval": packet_interval_ms_from_rate(DEFAULT_ATTITUDE_PACKET_RATE_HZ),
        # GUI/control-input polling interval; the worker repeats the latest
        # channel state at packet_interval so UI load cannot lower RC frame rate.
        "channel_update_interval": 8,
        # How long the CRSF worker will tolerate missing GUI channel refreshes
        # before it stops replaying the last command.  Keep this comfortably
        # above normal desktop/Qt scheduling hiccups; the FC still enforces its
        # own 250 ms packet-age failsafe if serial packets actually stop.
        "channel_stale_timeout_s": 2.0,
    },
    "throttle": {
        # Auto-throttle target sent to the FC on CH3 when Auto Throttle is active.
        # PID gains and stale-data timeouts live in flight_controller/Main.ino.
        "target_airspeed_mph": 20.0,
    },
    "fbw": {
        # Ground-station authority limits for Fly-By-Wire attitude commands.
        # The flight controller still clamps at 80 degrees as a redundant
        # safety limit; these values determine the actual commanded envelope.
        "max_roll_angle_deg": 45.0,
        "max_pitch_angle_deg": 30.0,
    },
    "osd": {
        # Percentage weight applied to new samples for the attitude indicator
        "attitude_smoothing": 20,
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
        # Trigger when descent rate exceeds sink_rate_threshold_fps
        "sink_rate_threshold_fps": 10.0,
        # Enable/disable all telemetry alarms or individual alarm groups
        "warning_alarms_enabled": True,
        "stall_alarm_enabled": True,
        "altitude_alarm_enabled": True,
        "bank_angle_alarm_enabled": True,
        "sink_rate_alarm_enabled": True,
    },
    "airborne": {
        # By default the takeoff detector derives its airspeed threshold from
        # the stall warning speed instead of maintaining an independent value.
        "takeoff_airspeed_multiplier": 1.2,
        "takeoff_altitude_ft": 15.0,
        "landed_airspeed_mph": 7.0,
        "landed_altitude_ft": 5.0,
        "takeoff_hold_s": 2.0,
        "landing_hold_s": 5.0,
        "gps_fresh_timeout_s": 2.0,
    },
    "map": {
        # Initial center [lat, lon] and zoom level for the offline map
        "center": [0.0, 0.0],
        "zoom": 8,
        "enabled": True,
        "follow": True,
    },
    "aircraft": {
        "battery_cells": "3s",
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

    crsf_config = config.setdefault("crsf", {})
    crsf_config["packet_interval"] = normalise_packet_interval_ms(
        crsf_config.get("packet_interval")
    )
    try:
        crsf_config["channel_stale_timeout_s"] = max(
            0.0, float(crsf_config.get("channel_stale_timeout_s", 2.0))
        )
    except (TypeError, ValueError):
        crsf_config["channel_stale_timeout_s"] = 2.0
    # Normalize the GUI control-poll interval so a malformed value (null or a
    # non-numeric string) cannot crash the Configuration page or the transmit
    # timer when it is later coerced to an int.
    try:
        crsf_config["channel_update_interval"] = max(
            1, int(float(crsf_config.get("channel_update_interval", 8)))
        )
    except (TypeError, ValueError):
        crsf_config["channel_update_interval"] = 8

    # Remove legacy GS-side throttle PID/stale-timeout keys.  The FC owns these
    # safety-critical control-loop values in flight_controller/Main.ino.
    throttle_config = config.setdefault("throttle", {})
    for legacy_key in (
        "pid_kp",
        "pid_ki",
        "pid_kd",
        "airspeed_stale_timeout_s",
    ):
        throttle_config.pop(legacy_key, None)

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
