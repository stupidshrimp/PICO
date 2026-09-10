"""Ground-station half of loiter (modules/loiter.py).

The orbit is flown by the FC (flight_controller/loiter_nav.h, covered by
tests/test_loiter_nav.py); this module owns the operator interface -- the
2 s hold that engages it, the conditions the ground station can see, and
handing control back.

These tests pin the parts that are easy to get subtly wrong and expensive to
discover in the air: that a short tap still performs the ordinary mode toggle
while a hold does not *also* toggle, that every documented disengage path
fires, and that CH10 only ever carries the FC's explicit request value.
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
    loiter_channel_value,
    stick_break_exceeded,
)
from modules.loiter import (
    LOITER_CHANNEL_INDEX,
    LOITER_CHANNEL_OFF_VALUE,
    LOITER_CHANNEL_REQUEST_VALUE,
)

# FC-side contract values (flight_controller/loiter_nav.h).
FC_LOITER_REQUEST_MIN = 1700 - 150
# CH8/CH9 carry the board-alignment trim, so loiter is CH10/AUX6.
FC_LOITER_CHANNEL_INDEX = 9


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


def test_cancel_press_cannot_strand_a_running_orbit():
    """Cancelling after the hold matured must not arm a late release.

    Focus loss while the key is still down cancels the press. If that cleared
    the consumed flag on an engaged orbit, the eventual release would fire the
    ordinary mode toggle and drop the aircraft out of Fly-By-Wire mid-orbit.
    """

    c = LoiterController()
    gates = _ready_gates()
    _engage(c, gates)

    c.cancel_press()
    assert c.engaged, "cancelling a matured press must not end the orbit"
    assert c.release(3.0) is None, "a late release must not toggle the flight mode"
    assert c.engaged


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
# CH10 request encoding
# ---------------------------------------------------------------------------


def test_channel_values_match_the_firmware_threshold():
    """The request value must clear the FC gate; the off value must not."""

    assert loiter_channel_value(True) >= FC_LOITER_REQUEST_MIN
    assert loiter_channel_value(False) < FC_LOITER_REQUEST_MIN
    assert loiter_channel_value(True) == LOITER_CHANNEL_REQUEST_VALUE
    assert loiter_channel_value(False) == LOITER_CHANNEL_OFF_VALUE


def test_channel_index_is_ch10():
    """CH8/CH9 belong to the board-alignment trim."""

    assert LOITER_CHANNEL_INDEX == FC_LOITER_CHANNEL_INDEX


def test_off_value_is_not_merely_centre():
    """A centred channel must read as off to the FC, and so must our off value.

    The FC treats anything below its threshold as off, so this is really a
    guard against someone "simplifying" the off value up into the request band.
    """

    assert LOITER_CHANNEL_OFF_VALUE < FC_LOITER_REQUEST_MIN
    assert 992 < FC_LOITER_REQUEST_MIN
