"""Ground-station CH3 semantics for the FC airspeed-hold loop.

CH3 carries manual throttle percent in Manual Throttle and the FC's target
airspeed in Auto Throttle, over the same ``172..1811`` span. These tests pin the
transmitted values against the FC-side converters in
``flight_controller/Main.ino`` (``mapRcToPercent`` and
``mapRcToAutoThrottleTargetMph``) and against the mode-change guard that keeps a
GS/FC mode disagreement from commanding a phantom throttle setting.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from modules.auto_throttle import (
    AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MIN,
    THROTTLE_MODE_CHANGE_GUARD_S,
    clamp_target_airspeed,
    mode_change_guard_active,
    throttle_channel_value,
)

# FC-side contract (flight_controller/Main.ino).
FC_RC_INPUT_MIN = 172
FC_RC_INPUT_MAX = 1811
FC_AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH = 100.0


def fc_map_rc_to_percent(value: int) -> float:
    """Mirror of the FC's ``mapRcToPercent``."""
    value = max(FC_RC_INPUT_MIN, min(FC_RC_INPUT_MAX, value))
    return (value - FC_RC_INPUT_MIN) * 100.0 / (FC_RC_INPUT_MAX - FC_RC_INPUT_MIN)


def fc_map_rc_to_target_mph(value: int) -> float:
    """Mirror of the FC's ``mapRcToAutoThrottleTargetMph``."""
    return (
        fc_map_rc_to_percent(value) / 100.0
    ) * FC_AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH


def _auto(target, **overrides):
    kwargs = dict(
        throttle_mode="Auto Throttle",
        throttle_percent=0.0,
        target_airspeed_mph=target,
    )
    kwargs.update(overrides)
    return throttle_channel_value(**kwargs)


def _manual(percent, **overrides):
    kwargs = dict(
        throttle_mode="Manual",
        throttle_percent=percent,
        target_airspeed_mph=20.0,
    )
    kwargs.update(overrides)
    return throttle_channel_value(**kwargs)


def test_gs_scale_matches_the_fc_constant():
    assert AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH == FC_AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH
    assert (CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX) == (FC_RC_INPUT_MIN, FC_RC_INPUT_MAX)


def test_auto_throttle_target_round_trips_through_the_fc_converter():
    for target in (0.0, 5.0, 17.5, 20.0, 25.0, 45.0, 100.0):
        value = _auto(target)
        assert CRSF_CHANNEL_MIN <= value <= CRSF_CHANNEL_MAX
        # The int() truncation in the channel build costs at most one CRSF step,
        # which is 100/1639 mph.
        assert abs(fc_map_rc_to_target_mph(value) - target) < 0.1


def test_manual_throttle_percent_round_trips_through_the_fc_converter():
    for percent in (0, 1, 25, 50, 75, 99, 100):
        value = _manual(percent)
        assert CRSF_CHANNEL_MIN <= value <= CRSF_CHANNEL_MAX
        assert abs(fc_map_rc_to_percent(value) - percent) < 0.1


def test_endpoints_are_exactly_the_channel_limits():
    assert _manual(0) == CRSF_CHANNEL_MIN
    assert _manual(100) == CRSF_CHANNEL_MAX
    assert _auto(0.0) == CRSF_CHANNEL_MIN
    assert _auto(AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH) == CRSF_CHANNEL_MAX


def test_setpoint_read_as_manual_throttle_is_the_bug_the_guard_prevents():
    # This is why the guard exists. CH3 and CH7/AUX3 do not reach the FC
    # together (ELRS sends AUX channels round-robin), so for a few packets after
    # a toggle the FC applies the manual converter to the airspeed setpoint. A
    # 20 mph target then reads as a rock-steady 20% throttle that ignores
    # airspeed entirely.
    assert abs(fc_map_rc_to_percent(_auto(20.0, seconds_since_mode_change=None)) - 20.0) < 0.1


def test_mode_change_guard_sends_minimum_then_the_real_value():
    # Inside the window: minimum, whichever mode is being entered.
    for elapsed in (0.0, 0.05, THROTTLE_MODE_CHANGE_GUARD_S - 0.01):
        assert _auto(25.0, seconds_since_mode_change=elapsed) == CRSF_CHANNEL_MIN
        assert _manual(60, seconds_since_mode_change=elapsed) == CRSF_CHANNEL_MIN
    # Once the window closes the channel carries the real value again.
    after = THROTTLE_MODE_CHANGE_GUARD_S + 0.01
    assert _auto(25.0, seconds_since_mode_change=after) == _auto(25.0)
    assert _manual(60, seconds_since_mode_change=after) == _manual(60)


def test_guard_value_is_safe_under_both_fc_converters():
    # Minimum is the one CH3 value that is harmless no matter which converter
    # the FC applies: throttle cut, or a 0 mph target it decays toward idle.
    guarded = _auto(100.0, seconds_since_mode_change=0.0)
    assert guarded == CRSF_CHANNEL_MIN
    assert fc_map_rc_to_percent(guarded) == 0.0
    assert fc_map_rc_to_target_mph(guarded) == 0.0


def test_guard_window_covers_the_elrs_aux_round_robin():
    # Seven AUX slots at the slowest common ELRS packet rate the GS is used
    # with: the guard has to outlast CH7's worst-case arrival delay.
    worst_case_aux_delay_s = 7 / 50.0  # 7 packets at 50 Hz
    assert THROTTLE_MODE_CHANGE_GUARD_S > worst_case_aux_delay_s


def test_compass_cal_request_masks_ch3_in_either_mode():
    assert _auto(80.0, compass_cal_active=True) == CRSF_CHANNEL_MIN
    assert _manual(100, compass_cal_active=True) == CRSF_CHANNEL_MIN


def test_unknown_mode_string_falls_back_to_manual_percent():
    # Anything that is not exactly "Auto Throttle" must use the manual
    # converter, matching the FC's "explicit high AUX3 or Manual" rule.
    assert throttle_channel_value(
        throttle_mode="auto throttle",
        throttle_percent=0,
        target_airspeed_mph=50.0,
    ) == CRSF_CHANNEL_MIN


def test_malformed_inputs_cannot_put_junk_on_the_channel():
    assert _manual(None) == CRSF_CHANNEL_MIN
    assert _manual("loud") == CRSF_CHANNEL_MIN
    assert _manual(float("nan")) == CRSF_CHANNEL_MIN
    assert _manual(-50) == CRSF_CHANNEL_MIN
    assert _manual(900) == CRSF_CHANNEL_MAX
    # A malformed setpoint falls back to the documented 20 mph default.
    assert _auto(float("nan")) == _auto(20.0)
    assert _auto(None) == _auto(20.0)
    assert _auto(-10.0) == CRSF_CHANNEL_MIN
    assert _auto(10_000.0) == CRSF_CHANNEL_MAX
    # A nonsensical scale must not divide by zero or send a high value.
    assert _auto(20.0, speed_channel_max_mph=0.0) == CRSF_CHANNEL_MIN


def test_clamp_target_airspeed_bounds_and_fallback():
    assert clamp_target_airspeed(25.0) == 25.0
    assert clamp_target_airspeed(-1.0) == 0.0
    assert clamp_target_airspeed(150.0) == AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH
    assert clamp_target_airspeed(float("inf")) == 20.0
    assert clamp_target_airspeed("fast") == 20.0
    assert clamp_target_airspeed(40.0, 30.0) == 30.0


def test_guard_is_closed_when_no_mode_change_has_happened():
    assert mode_change_guard_active(None) is False
    assert mode_change_guard_active(THROTTLE_MODE_CHANGE_GUARD_S) is False
    assert mode_change_guard_active(0.0) is True
    # An age that could never expire must fail CLOSED: a guard that never lifts
    # would mask CH3 to minimum forever and leave the aircraft with no throttle.
    assert mode_change_guard_active(-1.0) is False
    assert mode_change_guard_active(float("nan")) is False
    assert mode_change_guard_active(float("inf")) is False
    assert mode_change_guard_active("soon") is False
