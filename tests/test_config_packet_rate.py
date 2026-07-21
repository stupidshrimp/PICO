import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from config import (
    DEFAULT_ATTITUDE_PACKET_RATE_HZ,
    load_config,
    normalise_packet_interval_ms,
    packet_interval_ms_from_rate,
    packet_rate_hz_from_interval,
)


def test_supported_attitude_packet_rates_map_to_rc_intervals():
    assert packet_interval_ms_from_rate(100) == 10
    assert packet_interval_ms_from_rate(250) == 4
    assert packet_interval_ms_from_rate(500) == 2


def test_packet_interval_maps_back_to_nearest_supported_rate():
    assert packet_rate_hz_from_interval(10) == 100
    assert packet_rate_hz_from_interval(4) == 250
    assert packet_rate_hz_from_interval(2) == 500
    assert packet_rate_hz_from_interval(8) == 100
    assert packet_rate_hz_from_interval(None) == DEFAULT_ATTITUDE_PACKET_RATE_HZ


def test_packet_interval_normalisation_uses_supported_rates():
    assert normalise_packet_interval_ms(10) == 10
    assert normalise_packet_interval_ms(4) == 4
    assert normalise_packet_interval_ms(2) == 2
    assert normalise_packet_interval_ms("bad") == 4


def test_load_config_removes_legacy_ground_station_throttle_pid_keys(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(
        """{
            \"throttle\": {
                \"target_airspeed_mph\": 22.0,
                \"pid_kp\": 1.0,
                \"pid_ki\": 0.1,
                \"pid_kd\": 0.2,
                \"airspeed_stale_timeout_s\": 1.0
            }
        }""",
        encoding="utf-8",
    )

    config = load_config(str(config_path))

    assert config["throttle"] == {"target_airspeed_mph": 22.0}


def test_load_config_defaults_crsf_channel_stale_timeout(tmp_path):
    config_path = tmp_path / "missing.json"

    config = load_config(str(config_path))

    assert config["crsf"]["channel_stale_timeout_s"] == 2.0


def test_load_config_normalises_crsf_channel_stale_timeout(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(
        """{
            \"crsf\": {
                \"channel_stale_timeout_s\": -5
            }
        }""",
        encoding="utf-8",
    )

    config = load_config(str(config_path))

    assert config["crsf"]["channel_stale_timeout_s"] == 0.0


def test_load_config_falls_back_on_invalid_crsf_channel_stale_timeout(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(
        """{
            \"crsf\": {
                \"channel_stale_timeout_s\": \"bad\"
            }
        }""",
        encoding="utf-8",
    )

    config = load_config(str(config_path))

    assert config["crsf"]["channel_stale_timeout_s"] == 2.0


def test_load_config_ignores_malformed_known_sections(tmp_path, monkeypatch):
    config_path = tmp_path / "config.json"
    config_path.write_text(
        """{
            \"joystick\": \"bad\",
            \"crsf\": null,
            \"throttle\": 7
        }""",
        encoding="utf-8",
    )
    monkeypatch.setenv("JOYSTICK_PORT", "COM42")
    monkeypatch.setenv("CRSF_PORT", "COM43")

    config = load_config(str(config_path))

    assert config["joystick"]["port"] == "COM42"
    assert config["joystick"]["baudrate"] == 9600
    assert config["crsf"]["port"] == "COM43"
    assert config["crsf"]["packet_interval"] == 4
    assert config["throttle"] == {"target_airspeed_mph": 20.0}


def test_load_config_normalises_serial_baudrates(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(
        """{
            \"joystick\": {\"baudrate\": \"bad\"},
            \"crsf\": {\"baudrate\": 0}
        }""",
        encoding="utf-8",
    )

    config = load_config(str(config_path))

    assert config["joystick"]["baudrate"] == 9600
    assert config["crsf"]["baudrate"] == 921600


def test_load_config_accepts_numeric_string_serial_baudrates(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(
        """{
            \"joystick\": {\"baudrate\": \"115200\"},
            \"crsf\": {\"baudrate\": \"420000\"}
        }""",
        encoding="utf-8",
    )

    config = load_config(str(config_path))

    assert config["joystick"]["baudrate"] == 115200
    assert config["crsf"]["baudrate"] == 420000
