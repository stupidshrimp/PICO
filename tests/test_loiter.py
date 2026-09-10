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
    REASON_GROUNDED,
    REASON_NOT_TRANSMITTING,
    REASON_NOT_FBW,
    REASON_NO_JOYSTICK,
    REASON_STICK,
    REASON_TIMEOUT,
    REASON_TOGGLE,
    LOITER_HOLD_SECONDS,
    LOITER_MAX_STICK_BREAK_NORM,
    LOITER_MIN_HOLD_SECONDS,
    PRESS_SOURCE_JOYSTICK,
    PRESS_SOURCE_KEY,
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
        joystick_live=True,
        airborne=True,
        transmitting=True,
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
        fbw_active=False, attitude_fresh=True, joystick_live=True,
        airborne=True, stick_roll=0.0, stick_pitch=0.0,
    )

    c.press(0.0)
    event = c.poll(2.0, manual)
    assert event.kind == EVENT_REFUSED
    assert event.reason == REASON_NOT_FBW
    assert not c.engaged
    assert c.release(2.1) is None


def test_unmatched_release_does_not_toggle():
    """A release with no preceding press must be ignored.

    The joystick can genuinely deliver one: connect or reconnect while button
    13 is already held and the parser forwards the eventual "RELEASED" with no
    "PRESSED" before it. Treating that as a tap would flip Manual/Fly-By-Wire
    with nobody having pressed anything.
    """

    c = LoiterController()
    assert c.release(0.0) is None
    assert c.state == LOITER_DISENGAGED
    # And a second one, in case the stream repeats.
    assert c.release(0.1) is None


def test_release_is_consumed_once_only():
    """A duplicated release must not toggle twice off one press."""

    c = LoiterController()
    c.press(0.0)
    assert c.release(0.2) == REASON_TOGGLE
    assert c.release(0.3) is None


def test_cancelled_press_release_is_ignored_not_treated_as_a_tap():
    c = LoiterController()
    c.press(0.0)
    c.cancel_press()
    assert c.release(0.5) is None


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
        fbw_active=False, attitude_fresh=True, joystick_live=True, airborne=True
    )
    assert _engage(c, gates).reason == REASON_NOT_FBW


def test_engage_is_refused_on_stale_attitude():
    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=False, joystick_live=True, airborne=True
    )
    assert _engage(c, gates).reason == REASON_ATTITUDE_STALE


def test_engage_is_refused_when_not_transmitting():
    """Intent to transmit is not enough; the uplink must actually be up.

    Terminating transmission stops RC frames while telemetry keeps arriving,
    so every other gate stays satisfied with nothing reaching the aircraft.
    Engaging then would arm CH10 locally, and the next "start transmitting"
    click would carry it high as a fresh FC-side rising edge -- making that
    click, not a loiter gesture, the thing that starts the orbit.
    """

    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_live=True,
        airborne=True, transmitting=False,
    )
    assert _engage(c, gates).reason == REASON_NOT_TRANSMITTING
    assert not c.engaged


def test_stopping_transmission_drops_a_running_orbit():
    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(
            fbw_active=True, attitude_fresh=True, joystick_live=True,
            airborne=True, transmitting=False,
        ),
    )
    assert event.reason == REASON_NOT_TRANSMITTING
    assert not c.engaged


def test_engage_is_refused_on_the_ground():
    """A hold made on the ground must be refused at the gesture.

    Without this the GS would raise CH10 while grounded and the FC -- whose
    airborne check is a continuous predicate -- would begin the orbit the
    moment its own latch set after takeoff, with no further operator action.
    The firmware now also requires a fresh request edge, so this is the
    outer half of a two-sided fix.
    """

    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_live=True, airborne=False
    )
    assert _engage(c, gates).reason == REASON_GROUNDED
    assert not c.engaged


def test_landing_during_an_orbit_hands_control_back():
    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(
            fbw_active=True, attitude_fresh=True, joystick_live=True, airborne=False
        ),
    )
    assert event.reason == REASON_GROUNDED
    assert not c.engaged


def test_engage_is_refused_without_a_joystick():
    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_live=False, airborne=True
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
        LoiterGates(fbw_active=False, attitude_fresh=True, joystick_live=True, airborne=True),
    )
    assert event.reason == REASON_NOT_FBW


def test_stale_attitude_disengages():
    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(fbw_active=True, attitude_fresh=False, joystick_live=True, airborne=True),
    )
    assert event.reason == REASON_ATTITUDE_STALE


def test_losing_the_joystick_disengages():
    """Covers a silently stalled serial stream, not just an unplugged stick.

    The handler keeps returning cached axis values when the stream dies, so
    the caller must compute joystick_live from sample freshness. If it did
    not, stick-break would be dead exactly when the pilot most needs it.
    """

    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(fbw_active=True, attitude_fresh=True, joystick_live=False, airborne=True),
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


# ---------------------------------------------------------------------------
# Configuration robustness
# ---------------------------------------------------------------------------


def test_nonpositive_hold_falls_back_to_the_default():
    """config.json is hand-editable and 0 or negative would defeat the gesture.

    With a non-positive hold the very first poll tick satisfies
    `now - press_start >= hold_seconds`, so an ordinary tap spanning one GUI
    refresh (~14 ms) would engage an autonomous mode.
    """

    gates = _ready_gates()
    for bad in (0.0, -5.0):
        c = LoiterController(hold_seconds=bad)
        assert c.hold_seconds == LOITER_HOLD_SECONDS
        c.press(0.0)
        assert c.poll(0.014, gates) is None, "a tap must not engage an orbit"
        assert not c.engaged


def test_tiny_hold_is_floored():
    """A small positive hold fails the same way, so the guard is a floor."""

    c = LoiterController(hold_seconds=0.01)
    assert c.hold_seconds == LOITER_MIN_HOLD_SECONDS
    c.press(0.0)
    assert c.poll(0.014, _ready_gates()) is None
    assert not c.engaged


def test_stick_break_threshold_is_bounded():
    """Too large is the dangerous direction: it disables the way out."""

    c = LoiterController(stick_break_norm=99.0)
    assert c.stick_break_norm == LOITER_MAX_STICK_BREAK_NORM
    _engage(c, _ready_gates(stick_roll=0.0))
    # Full deflection must still break out at the clamped threshold.
    event = c.poll(3.0, _ready_gates(stick_roll=-1.0))
    assert event is not None and event.reason == REASON_STICK

    # Non-positive falls back to the default rather than breaking instantly.
    c2 = LoiterController(stick_break_norm=-1.0)
    assert c2.stick_break_norm > 0.0
    _engage(c2, _ready_gates())
    assert c2.poll(3.0, _ready_gates()) is None


def test_negative_max_duration_normalises_to_disabled():
    c = LoiterController(max_duration_s=-10.0)
    assert c.max_duration_s == 0.0
    _engage(c, _ready_gates())
    assert c.poll(10_000.0, _ready_gates()) is None
    assert c.engaged


# ---------------------------------------------------------------------------
# Input source matching
# ---------------------------------------------------------------------------


def test_release_from_the_other_source_is_ignored():
    """An orphan joystick release must not cancel a keyboard hold.

    The joystick can deliver an unmatched release after a reconnect. Sharing
    one press flag between both controls let that release consume the Ctrl+M
    press and toggle Manual/Fly-By-Wire with the key still held down.
    """

    c = LoiterController()
    c.press(0.0, PRESS_SOURCE_KEY)

    assert c.release(0.5, PRESS_SOURCE_JOYSTICK) is None
    assert c.state == LOITER_ARMING, "the keyboard hold must survive"

    # The keyboard's own release still resolves normally.
    assert c.release(0.6, PRESS_SOURCE_KEY) == REASON_TOGGLE


def test_keyboard_hold_still_matures_despite_a_foreign_release():
    c = LoiterController()
    gates = _ready_gates()
    c.press(0.0, PRESS_SOURCE_KEY)
    c.release(0.5, PRESS_SOURCE_JOYSTICK)
    assert c.poll(2.0, gates).kind == EVENT_ENGAGED


def test_a_press_from_either_source_can_disengage():
    """Disengaging must never be blocked by which control started the orbit."""

    c = LoiterController()
    _engage(c, _ready_gates())
    c.release(2.1, PRESS_SOURCE_KEY)

    event = c.press(5.0, PRESS_SOURCE_JOYSTICK)
    assert event.kind == EVENT_DISENGAGED
    assert event.reason == REASON_TOGGLE
    assert not c.engaged
