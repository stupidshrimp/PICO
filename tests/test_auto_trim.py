"""Unit tests for the ground-station auto-trim calculation.

These verify the pure maths (modules/auto_trim.py) headlessly -- no PySide6 --
covering the attitude averaging, the flight gating helpers, the sign
conventions that tie a measured attitude bias to a trim correction, and the
convergence / clamping behaviour of both the one-shot suggestion and the
rate-limited apply step.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from modules.auto_trim import (
    AUTO_TRIM_DEAD_BAND_DEG,
    AUTO_TRIM_MIN_SAMPLES,
    AUTO_TRIM_SAMPLE_WINDOW_S,
    AUTO_TRIM_STICK_CENTER_TOLERANCE,
    DEFAULT_TRIM_MAX_LEVEL,
    AttitudeAverager,
    apply_trim_step,
    is_trimmed,
    recommended_trim_levels,
    sticks_centered,
)


# --- sticks_centered --------------------------------------------------------
def test_sticks_centered_within_and_outside_tolerance():
    tol = AUTO_TRIM_STICK_CENTER_TOLERANCE
    assert sticks_centered(0.0, 0.0)
    assert sticks_centered(tol, -tol)
    assert not sticks_centered(tol * 2, 0.0)
    assert not sticks_centered(0.0, tol * 2)


def test_sticks_centered_none_is_not_centered():
    # No joystick reading must never count as hands-off.
    assert not sticks_centered(None, 0.0)
    assert not sticks_centered(0.0, None)
    assert not sticks_centered(None, None)


# --- is_trimmed -------------------------------------------------------------
def test_is_trimmed_dead_band():
    band = AUTO_TRIM_DEAD_BAND_DEG
    assert is_trimmed(band, -band)
    assert is_trimmed(0.0, 0.0)
    assert not is_trimmed(band + 0.5, 0.0)
    assert not is_trimmed(0.0, -(band + 0.5))


def test_is_trimmed_handles_bad_input():
    assert not is_trimmed(None, 0.0)


# --- recommended_trim_levels (suggest) --------------------------------------
def test_recommend_within_dead_band_leaves_trim_alone():
    ail, elev = recommended_trim_levels(0.5, -0.5, current_aileron_level=3,
                                        current_elevator_level=-2)
    assert ail == 3
    assert elev == -2


def test_recommend_sign_opposes_the_measured_bias():
    # Positive roll (banked left) needs NEGATIVE aileron trim; positive pitch
    # (nose high) needs NEGATIVE elevator trim -- correction opposes the bias.
    ail, elev = recommended_trim_levels(10.0, 8.0)
    assert ail < 0
    assert elev < 0
    # A negative bias flips both corrections positive.
    ail_neg, elev_neg = recommended_trim_levels(-10.0, -8.0)
    assert ail_neg > 0
    assert elev_neg > 0


def test_recommend_adds_to_existing_trim_and_clamps():
    # Huge bias saturates at the trim-level ceiling rather than overshooting.
    ail, elev = recommended_trim_levels(999.0, 999.0)
    assert ail == -DEFAULT_TRIM_MAX_LEVEL
    assert elev == -DEFAULT_TRIM_MAX_LEVEL
    ail_hi, elev_hi = recommended_trim_levels(-999.0, -999.0)
    assert ail_hi == DEFAULT_TRIM_MAX_LEVEL
    assert elev_hi == DEFAULT_TRIM_MAX_LEVEL


# --- apply_trim_step (closed loop) ------------------------------------------
def test_apply_step_is_zero_inside_dead_band():
    assert apply_trim_step(0.5, -0.5) == (0, 0)


def test_apply_step_is_rate_limited_and_signed():
    # A large bias still moves at most one level per axis per cycle, opposing
    # the bias (negative correction for a positive attitude error).
    ail, elev = apply_trim_step(30.0, 30.0)
    assert ail == -1
    assert elev == -1
    ail_neg, elev_neg = apply_trim_step(-30.0, -30.0)
    assert ail_neg == 1
    assert elev_neg == 1


def test_apply_step_moves_at_least_one_level_just_outside_dead_band():
    # A small-but-real bias must not round to a stalled zero step.
    bias = AUTO_TRIM_DEAD_BAND_DEG + 0.2
    ail, elev = apply_trim_step(bias, -bias)
    assert ail == -1
    assert elev == 1


def test_apply_step_converges_a_simulated_bias_to_level():
    # Model the aircraft: each aileron level cancels ~1 deg of the initial
    # bias. Stepping repeatedly should drive the residual into the dead-band.
    residual = 6.0
    level = 0
    for _ in range(50):
        if is_trimmed(residual, 0.0):
            break
        step, _ = apply_trim_step(residual, 0.0)
        level += step
        residual = 6.0 + level  # positive bias, negative steps reduce it
    assert is_trimmed(residual, 0.0)
    assert level < 0  # trimmed in the correcting direction


# --- AttitudeAverager -------------------------------------------------------
def test_averager_not_ready_until_window_and_count_met():
    avg = AttitudeAverager(window_s=2.0, min_samples=4)
    assert not avg.ready()
    assert avg.averages() is None
    # Enough samples but too short a time span -> still not ready.
    for i in range(10):
        avg.add(i * 0.01, 1.0, 2.0)
    assert not avg.ready()


def test_averager_ready_and_means_correct():
    avg = AttitudeAverager(window_s=2.0, min_samples=4)
    # Span the full window with a steady bias.
    for i in range(21):
        avg.add(i * 0.1, 3.0, -1.0)
    assert avg.ready()
    mean_roll, mean_pitch = avg.averages()
    assert abs(mean_roll - 3.0) < 1e-9
    assert abs(mean_pitch - (-1.0)) < 1e-9


def test_averager_expires_old_samples_outside_window():
    avg = AttitudeAverager(window_s=1.0, min_samples=1)
    avg.add(0.0, 10.0, 10.0)   # stale, should expire once newer samples arrive
    # Fill a full 1.0 s window of level samples; the last add expires t=0.
    for i in range(11):
        avg.add(2.0 + i * 0.1, 0.0, 0.0)  # window ends spanning [2.0, 3.0]
    mean_roll, mean_pitch = avg.averages()
    # The stale 10-degree sample must not skew the mean.
    assert abs(mean_roll) < 1e-9
    assert abs(mean_pitch) < 1e-9


def test_averager_clear_resets_state():
    avg = AttitudeAverager(window_s=1.0, min_samples=1)
    avg.add(0.0, 5.0, 5.0)
    avg.clear()
    assert avg.count == 0
    assert avg.averages() is None


def test_module_constants_are_sane():
    assert AUTO_TRIM_SAMPLE_WINDOW_S > 0
    assert AUTO_TRIM_MIN_SAMPLES > 0
    assert DEFAULT_TRIM_MAX_LEVEL == 25  # keep in lockstep with main.TRIM_MAX_LEVEL
