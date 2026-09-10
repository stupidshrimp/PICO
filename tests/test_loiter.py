"""Ground-station loiter state machine (modules/loiter.py).

Loiter v1 is a fixed-bank orbit commanded through the existing Fly-By-Wire
path, engaged by HOLDING the control-mode toggle and dropped by pressing it
again, moving the stick, or losing any of the conditions it depends on.

These tests pin the parts that are easy to get subtly wrong and expensive to
discover in the air: that a short tap still performs the ordinary mode toggle
(and a hold does not *also* toggle), that every documented disengage path
fires, and that the commanded bank is expressed against the FC's 80 deg hard
limit rather than the stick's scaled range -- the arithmetic that decides
whether the aircraft banks 20 degrees or 11.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from modules.loiter import (
    EVENT_DISENGAGED,
    EVENT_ENGAGED,
    EVENT_REFUSED,
    LOITER_ARMING,
    LOITER_DISENGAGED,
    LOITER_ENGAGED,
    REASON_ATTITUDE_STALE,
    REASON_NOT_FBW,
    REASON_NO_JOYSTICK,
    REASON_STICK,
    REASON_TIMEOUT,
    REASON_TOGGLE,
    LoiterController,
    LoiterGates,
    bank_to_channel_norm,
    stick_break_exceeded,
)

# FC-side contract value (flight_controller/Main.ino FBW_MAX_ROLL_ANGLE_DEG).
FC_HARD_LIMIT_DEG = 80.0
# Ground-station default envelope (config.py "fbw").
GS_ROLL_LIMIT_DEG = 45.0


def _ready_gates(stick_roll=0.0, stick_pitch=0.0):
    """Gates with everything loiter needs satisfied."""

    return LoiterGates(
        fbw_active=True,
        attitude_fresh=True,
        joystick_present=True,
        stick_roll=stick_roll,
        stick_pitch=stick_pitch,
    )


def _engage(controller, gates=None, t0=0.0):
    """Drive a controller through a completed engage hold; return the event."""

    gates = gates if gates is not None else _ready_gates()
    controller.press(t0)
    return controller.poll(t0 + controller.hold_seconds, gates)


# ---------------------------------------------------------------------------
# Hold to engage
# ---------------------------------------------------------------------------


def test_hold_engages_after_the_full_duration():
    c = LoiterController()
    gates = _ready_gates()

    c.press(0.0)
    assert c.state == LOITER_ARMING
    # Still arming right up to the threshold.
    assert c.poll(1.99, gates) is None
    assert not c.engaged

    event = c.poll(2.0, gates)
    assert event.kind == EVENT_ENGAGED
    assert event.reason is None
    assert c.engaged


def test_short_tap_falls_through_to_the_ordinary_mode_toggle():
    c = LoiterController()
    gates = _ready_gates()

    c.press(0.0)
    c.poll(0.4, gates)
    assert c.release(0.5) == REASON_TOGGLE
    assert c.state == LOITER_DISENGAGED


def test_completed_hold_does_not_also_toggle_on_release():
    """The hold has already acted; its release must not flip Manual/FBW too."""

    c = LoiterController()
    gates = _ready_gates()

    _engage(c, gates)
    assert c.engaged
    assert c.release(2.1) is None
    assert c.engaged


def test_refused_hold_does_not_toggle_on_release_either():
    c = LoiterController()
    manual = LoiterGates(
        fbw_active=False, attitude_fresh=True, joystick_present=True,
        stick_roll=0.0, stick_pitch=0.0,
    )

    c.press(0.0)
    event = c.poll(2.0, manual)
    assert event.kind == EVENT_REFUSED
    assert event.reason == REASON_NOT_FBW
    assert not c.engaged
    assert c.release(2.1) is None


def test_hold_progress_reports_the_arming_fraction():
    c = LoiterController()
    gates = _ready_gates()

    assert c.hold_progress(0.0) == 0.0
    c.press(0.0)
    assert c.hold_progress(1.0) == 0.5
    assert c.hold_progress(5.0) == 1.0


def test_cancel_press_abandons_the_hold_without_toggling():
    c = LoiterController()
    gates = _ready_gates()

    c.press(0.0)
    c.cancel_press()
    assert c.state == LOITER_DISENGAGED
    assert c.poll(5.0, gates) is None


# ---------------------------------------------------------------------------
# Engage gates
# ---------------------------------------------------------------------------


def test_engage_is_refused_without_fly_by_wire():
    c = LoiterController()
    gates = LoiterGates(
        fbw_active=False, attitude_fresh=True, joystick_present=True
    )
    assert _engage(c, gates).reason == REASON_NOT_FBW


def test_engage_is_refused_on_stale_attitude():
    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=False, joystick_present=True
    )
    assert _engage(c, gates).reason == REASON_ATTITUDE_STALE


def test_engage_is_refused_without_a_joystick():
    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_present=False
    )
    assert _engage(c, gates).reason == REASON_NO_JOYSTICK


# ---------------------------------------------------------------------------
# Disengage paths
# ---------------------------------------------------------------------------


def test_press_while_engaged_disengages_immediately():
    c = LoiterController()
    gates = _ready_gates()
    _engage(c, gates)

    event = c.press(10.0)
    assert event.kind == EVENT_DISENGAGED
    assert event.reason == REASON_TOGGLE
    assert not c.engaged
    # ...and that press's release must not toggle the flight mode as well.
    assert c.release(10.1) is None


def test_stick_movement_beyond_the_threshold_disengages():
    c = LoiterController()
    _engage(c, _ready_gates(stick_roll=0.1, stick_pitch=-0.1))

    # A nudge inside the threshold keeps the orbit.
    assert c.poll(3.0, _ready_gates(stick_roll=0.3, stick_pitch=-0.1)) is None
    assert c.engaged

    event = c.poll(4.0, _ready_gates(stick_roll=0.5, stick_pitch=-0.1))
    assert event.kind == EVENT_DISENGAGED
    assert event.reason == REASON_STICK


def test_stick_baseline_is_the_position_at_engage_not_centre():
    """A pilot holding an offset stick must not instantly break their own orbit."""

    c = LoiterController()
    offset = _ready_gates(stick_roll=0.6, stick_pitch=0.0)
    _engage(c, offset)

    assert c.poll(3.0, offset) is None
    assert c.engaged


def test_losing_fly_by_wire_disengages():
    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(fbw_active=False, attitude_fresh=True, joystick_present=True),
    )
    assert event.reason == REASON_NOT_FBW


def test_stale_attitude_disengages():
    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(fbw_active=True, attitude_fresh=False, joystick_present=True),
    )
    assert event.reason == REASON_ATTITUDE_STALE


def test_losing_the_joystick_disengages():
    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(fbw_active=True, attitude_fresh=True, joystick_present=False),
    )
    assert event.reason == REASON_NO_JOYSTICK


def test_orbit_times_out():
    c = LoiterController(max_duration_s=60.0)
    gates = _ready_gates()
    _engage(c, gates)

    assert c.poll(2.0 + 59.0, gates) is None
    event = c.poll(2.0 + 60.0, gates)
    assert event.reason == REASON_TIMEOUT
    assert not c.engaged


def test_disengaging_clears_state_so_the_next_hold_starts_clean():
    c = LoiterController()
    gates = _ready_gates()
    _engage(c, gates)
    c.press(10.0)
    c.release(10.1)

    assert c.state == LOITER_DISENGAGED
    assert c.elapsed(20.0) == 0.0
    # A fresh hold engages again.
    c.press(20.0)
    assert c.poll(22.0, gates).kind == EVENT_ENGAGED


# ---------------------------------------------------------------------------
# Stick-break helper
# ---------------------------------------------------------------------------


def test_missing_samples_never_break_loiter_on_their_own():
    assert stick_break_exceeded(None, 1.0, 1.0) is False
    assert stick_break_exceeded((0.0, 0.0), None, None) is False
    # A live axis still breaks even when the other axis has no sample.
    assert stick_break_exceeded((0.0, 0.0), 0.9, None) is True


def test_stick_break_is_symmetric_about_the_baseline():
    assert stick_break_exceeded((0.5, 0.0), 0.9, 0.0, threshold=0.25) is True
    assert stick_break_exceeded((0.5, 0.0), 0.1, 0.0, threshold=0.25) is True
    assert stick_break_exceeded((0.5, 0.0), 0.6, 0.0, threshold=0.25) is False


# ---------------------------------------------------------------------------
# Commanded attitude arithmetic
# ---------------------------------------------------------------------------


def test_bank_is_expressed_against_the_fc_hard_limit():
    """20 deg of bank must normalize against 80, not against the GS limit.

    Normalizing against the 45 deg GS limit instead would command 45 * (20/45)
    through the scaled stick path and fly an 11 degree turn.
    """

    norm = bank_to_channel_norm(20.0, FC_HARD_LIMIT_DEG, GS_ROLL_LIMIT_DEG)
    assert norm == 20.0 / 80.0


def test_bank_request_is_clamped_into_the_operator_envelope():
    norm = bank_to_channel_norm(70.0, FC_HARD_LIMIT_DEG, GS_ROLL_LIMIT_DEG)
    assert norm == GS_ROLL_LIMIT_DEG / FC_HARD_LIMIT_DEG


def test_bank_normalisation_is_signed_and_bounded():
    assert bank_to_channel_norm(-20.0, FC_HARD_LIMIT_DEG) == -0.25
    assert bank_to_channel_norm(500.0, FC_HARD_LIMIT_DEG) == 1.0
    assert bank_to_channel_norm(20.0, 0.0) == 0.0


def test_command_angles_respect_the_configured_limits_and_direction():
    c = LoiterController(bank_angle_deg=20.0, bank_direction=-1.0)
    roll, pitch = c.command_angles(GS_ROLL_LIMIT_DEG, 30.0)
    assert roll == -20.0
    assert pitch == 0.0

    tight = LoiterController(bank_angle_deg=60.0)
    roll, _ = tight.command_angles(GS_ROLL_LIMIT_DEG, 30.0)
    assert roll == GS_ROLL_LIMIT_DEG
