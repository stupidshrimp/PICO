"""Ground-station compass-calibration channel semantics (modules/compass_cal.py).

CH7/AUX3 is a three-band channel: low = manual throttle, high = auto
throttle, center band = the FC's on-ground magnetometer-calibration request.
These tests pin the transmitted values against the FC-side contract in
docs/protocol_contract.md (request band 891..1091, auto-throttle threshold
1550) and the start-guard reasons the configuration-page button reports.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from modules.compass_cal import (
    COMPASS_CAL_CHANNEL_VALUE,
    THROTTLE_MODE_AUTO_VALUE,
    THROTTLE_MODE_MANUAL_VALUE,
    compass_cal_start_blockers,
    throttle_mode_channel_value,
)

# FC-side contract values (flight_controller/Main.ino).
FC_RC_INPUT_MIN = 172
FC_RC_INPUT_MAX = 1811
FC_RC_INPUT_CENTER = (FC_RC_INPUT_MIN + FC_RC_INPUT_MAX) // 2
FC_MAG_CAL_REQUEST_MIN = FC_RC_INPUT_CENTER - 100
FC_MAG_CAL_REQUEST_MAX = FC_RC_INPUT_CENTER + 100
FC_THROTTLE_MODE_AUTO_MIN = 1700 - 150


def test_throttle_mode_values_unchanged_without_calibration():
    assert throttle_mode_channel_value("Manual", False) == THROTTLE_MODE_MANUAL_VALUE == 400
    assert throttle_mode_channel_value("Auto Throttle", False) == THROTTLE_MODE_AUTO_VALUE == 1700


def test_calibration_value_lands_inside_fc_request_band():
    value = throttle_mode_channel_value("Manual", True)
    assert value == COMPASS_CAL_CHANNEL_VALUE
    assert FC_MAG_CAL_REQUEST_MIN <= value <= FC_MAG_CAL_REQUEST_MAX


def test_calibration_value_overrides_auto_throttle_and_stays_manual_to_fc():
    # Even if the GS is somehow left in Auto Throttle, an active calibration
    # must transmit the request band -- which the FC's throttle-mode logic
    # reads as Manual Throttle, so the throttle PID can never engage.
    value = throttle_mode_channel_value("Auto Throttle", True)
    assert value == COMPASS_CAL_CHANNEL_VALUE
    assert value < FC_THROTTLE_MODE_AUTO_MIN


def test_normal_mode_values_stay_outside_the_request_band():
    for mode in ("Manual", "Auto Throttle"):
        value = throttle_mode_channel_value(mode, False)
        assert not (FC_MAG_CAL_REQUEST_MIN <= value <= FC_MAG_CAL_REQUEST_MAX)


def _ready_kwargs(**overrides):
    kwargs = dict(
        transmission_active=True,
        airborne_state="grounded",
        control_mode="Manual",
        throttle_mode="Manual",
        throttle_percent=0.0,
    )
    kwargs.update(overrides)
    return kwargs


def test_start_allowed_when_grounded_manual_and_idle_throttle():
    assert compass_cal_start_blockers(**_ready_kwargs()) == []


def test_start_blockers_each_reported():
    assert compass_cal_start_blockers(**_ready_kwargs(transmission_active=False))
    assert compass_cal_start_blockers(**_ready_kwargs(airborne_state="airborne"))
    assert compass_cal_start_blockers(**_ready_kwargs(control_mode="Fly-By-Wire"))
    assert compass_cal_start_blockers(**_ready_kwargs(throttle_mode="Auto Throttle"))
    assert compass_cal_start_blockers(**_ready_kwargs(throttle_percent=25.0))


def test_start_blockers_accumulate():
    reasons = compass_cal_start_blockers(
        transmission_active=False,
        airborne_state="airborne",
        control_mode="Fly-By-Wire",
        throttle_mode="Auto Throttle",
        throttle_percent=50.0,
    )
    assert len(reasons) == 5
