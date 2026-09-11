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

from config import DEFAULT_CONFIG

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
    attitude_gap_exceeded,
    FC_AIRBORNE_ENGAGE_AIRSPEED_MPH,
    FC_AIRBORNE_ENGAGE_HEIGHT_FT,
    fc_airborne_engage_ok,
    fc_airborne_latched,
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
        engage_airborne=True,
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
        fbw_active=False, attitude_fresh=True, joystick_live=True, airborne=True,
        engage_airborne=True,
    )
    assert _engage(c, gates).reason == REASON_NOT_FBW


def test_engage_is_refused_on_stale_attitude():
    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=False, joystick_live=True, airborne=True,
        engage_airborne=True,
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
        airborne=True, engage_airborne=True, transmitting=False,
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
            airborne=True, engage_airborne=True, transmitting=False,
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
        fbw_active=True, attitude_fresh=True, joystick_live=True,
        airborne=False, engage_airborne=False,
    )
    assert _engage(c, gates).reason == REASON_GROUNDED
    assert not c.engaged


def test_landing_during_an_orbit_hands_control_back():
    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(
            fbw_active=True, attitude_fresh=True, joystick_live=True,
            airborne=False, engage_airborne=False,
        ),
    )
    assert event.reason == REASON_GROUNDED
    assert not c.engaged


def test_engage_is_refused_without_a_joystick():
    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_live=False, airborne=True,
        engage_airborne=True,
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
        LoiterGates(fbw_active=False, attitude_fresh=True, joystick_live=True, airborne=True, engage_airborne=True),
    )
    assert event.reason == REASON_NOT_FBW


def test_stale_attitude_disengages():
    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.poll(
        3.0,
        LoiterGates(fbw_active=True, attitude_fresh=False, joystick_live=True, airborne=True, engage_airborne=True),
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
        LoiterGates(fbw_active=True, attitude_fresh=True, joystick_live=False, airborne=True, engage_airborne=True),
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


# ---------------------------------------------------------------------------
# Synchronous abort (teardown the periodic gates cannot observe)
# ---------------------------------------------------------------------------


def test_abort_disengages_a_running_orbit():
    """Teardown inside one GUI callback never shows the gates an unhealthy tick.

    A CRSF transport or joystick handler torn down and rebuilt synchronously
    looks continuously healthy to poll(), so a running orbit would survive the
    swap -- and the rebuilt transport would be seeded with CH10 already high
    for a reconnecting FC to read as a fresh request edge.
    """

    c = LoiterController()
    _engage(c, _ready_gates())

    event = c.abort(REASON_NOT_TRANSMITTING)
    assert event is not None
    assert event.kind == EVENT_DISENGAGED
    assert event.reason == REASON_NOT_TRANSMITTING
    assert not c.engaged
    assert c.state == LOITER_DISENGAGED


def test_abort_cancels_a_pending_hold_and_its_release():
    c = LoiterController()
    c.press(0.0)
    assert c.abort(REASON_NO_JOYSTICK) is None, "nothing was flying, nothing to announce"
    assert c.state == LOITER_DISENGAGED
    # The hold must not mature afterwards...
    assert c.poll(5.0, _ready_gates()) is None
    assert not c.engaged
    # ...and its release must not read as a tap.
    assert c.release(5.1) is None


def test_abort_when_idle_is_silent():
    c = LoiterController()
    assert c.abort(REASON_NO_JOYSTICK) is None
    assert c.state == LOITER_DISENGAGED


def test_abort_leaves_the_controller_reusable():
    c = LoiterController()
    _engage(c, _ready_gates())
    c.abort(REASON_NO_JOYSTICK)

    c.press(10.0)
    assert c.poll(12.0, _ready_gates()).kind == EVENT_ENGAGED


# ---------------------------------------------------------------------------
# Airborne agreement between the two detectors
# ---------------------------------------------------------------------------

_MAIN_INO = pathlib.Path(__file__).resolve().parents[1] / "flight_controller" / "Main.ino"


def _firmware_define(name):
    """Read a numeric #define straight out of the firmware.

    Comparing the Python mirror against another Python literal would detect
    nothing: editing the firmware constant would leave such a test passing
    while the GS/FC mismatch silently reopened. This reads the real value.
    """

    import re

    match = re.search(
        r"^#define\s+" + re.escape(name) + r"\s+\(([-+0-9.eE]+)f?\)",
        _MAIN_INO.read_text(),
        re.MULTILINE,
    )
    assert match is not None, f"{name} not found in {_MAIN_INO.name}"
    return float(match.group(1))


_LOITER_NAV_H = (
    pathlib.Path(__file__).resolve().parents[1] / "flight_controller" / "loiter_nav.h"
)
_CONFIG_JSON = pathlib.Path(__file__).resolve().parents[1] / "config.json"


def _header_define(path, name):
    """Read a bare numeric ``#define NAME value`` (no surrounding parentheses)."""

    import re

    match = re.search(
        r"^#define\s+" + re.escape(name) + r"\s+([-+0-9.eE]+)f?\s*$",
        path.read_text(),
        re.MULTILINE,
    )
    assert match is not None, f"{name} not found in {path.name}"
    return float(match.group(1))


def _firmware_const_float(name):
    """Read a ``const float NAME = value;`` out of Main.ino."""

    import re

    match = re.search(
        r"^const\s+float\s+" + re.escape(name) + r"\s*=\s*([-+0-9.eE]+)f?\s*;",
        _MAIN_INO.read_text(),
        re.MULTILINE,
    )
    assert match is not None, f"{name} not found in {_MAIN_INO.name}"
    return float(match.group(1))


def test_loiter_airspeed_floor_sits_between_stall_and_cruise():
    """stall < floor < cruise, checked across all three files that set them.

    Both halves are load-bearing and the shipped config once violated the
    first: an 18 mph floor under a 20 mph configured stall protects nothing,
    because the wing is already gone before the floor trips. A floor at or
    above the cruise target is the opposite failure -- the hold is abandoned
    permanently and loiter silently degrades to a descending un-held orbit.

    Every value is read from the file that owns it, so this cannot pass on a
    stale literal, and it covers the config actually flown rather than only
    the built-in defaults.
    """

    import json

    floor = _header_define(_LOITER_NAV_H, "LOITER_MIN_AIRSPEED_MPH")
    cruise = _firmware_const_float("AUTO_THROTTLE_DEFAULT_TARGET_MPH")

    shipped = json.loads(_CONFIG_JSON.read_text())
    stall_values = [
        DEFAULT_CONFIG["warnings"]["stall_airspeed"],
        shipped.get("warnings", {}).get("stall_airspeed"),
    ]
    for stall in stall_values:
        if stall is None:
            continue
        assert floor > float(stall), (
            f"LOITER_MIN_AIRSPEED_MPH ({floor}) must exceed the configured "
            f"stall airspeed ({stall}); a floor at or below stall cannot "
            "protect the wing it exists to protect"
        )

    assert floor < cruise, (
        f"LOITER_MIN_AIRSPEED_MPH ({floor}) must stay below the auto-throttle "
        f"cruise target ({cruise}); otherwise the altitude hold is abandoned "
        "on every orbit and loiter quietly becomes a descending circle"
    )


def test_attitude_gap_is_detected_regardless_of_when_the_gs_polls():
    """An outage that recovers between two polls must still be caught.

    This is the race that equal thresholds alone do not close. The FC decides
    at its 8 ms control rate; the ground station polls loiter on a 14 ms timer
    and a resumed packet immediately refreshes the arrival timestamp. So an
    outage only slightly longer than the threshold can trip the FC and then
    recover before the next poll, leaving every poll to observe a fresh age.

    The simulation below is the concrete failure: a 210 ms gap, with polls on a
    14 ms grid. Sampling the current age never sees it; the inter-arrival gap
    always does.
    """

    threshold = 0.2
    poll_period = 0.014

    # Attitude packets arrive at ~125 Hz, stop, then resume 210 ms later --
    # long enough for the FC (200 ms) to have dropped the orbit.
    outage_start = 1.000
    outage_end = outage_start + 0.210
    arrivals = [outage_start - 0.008, outage_start, outage_end]

    # What the OLD code did: compare the current age at each poll tick.
    stale_seen_by_polling = False
    poll_time = outage_start
    while poll_time <= outage_end + poll_period:
        last_arrival = max((a for a in arrivals if a <= poll_time), default=None)
        if last_arrival is not None and (poll_time - last_arrival) > threshold:
            stale_seen_by_polling = True
        poll_time += poll_period
    assert not stale_seen_by_polling, (
        "this test is meant to reproduce the race where polling misses the "
        "outage; if polling now catches it the scenario needs rebuilding"
    )

    # What the gap latch does: measure between consecutive arrivals.
    gaps = [
        attitude_gap_exceeded(previous, current, threshold)
        for previous, current in zip(arrivals, arrivals[1:])
    ]
    assert any(gaps), "the 210 ms gap must be detected at the packet callback"


def test_attitude_gap_ignores_normal_arrival_jitter():
    """Ordinary 125 Hz arrivals, and the first packet, must not trip it."""

    assert not attitude_gap_exceeded(None, 5.0, 0.2), (
        "the first attitude packet has no predecessor and is not a gap"
    )
    assert not attitude_gap_exceeded(1.000, 1.008, 0.2), "8 ms is the normal period"
    assert not attitude_gap_exceeded(1.000, 1.199, 0.2), "just inside the window"
    assert attitude_gap_exceeded(1.000, 1.201, 0.2), "just outside must trip"


_MAIN_PY = pathlib.Path(__file__).resolve().parents[1] / "main.py"


def _firmware_constexpr(name):
    """Read a numeric ``constexpr`` straight out of the firmware.

    Same reasoning as _firmware_define, for the constants declared that way.
    """

    import re

    match = re.search(
        r"^constexpr\s+\w+\s+" + re.escape(name) + r"\s*=\s*([0-9.eE+-]+)[uUlLfF]*\s*;",
        _MAIN_INO.read_text(),
        re.MULTILINE,
    )
    assert match is not None, f"{name} not found in {_MAIN_INO.name}"
    return float(match.group(1))


def _main_py_class_constant(name):
    """Read a numeric class constant out of main.py without importing it.

    main.py needs PySide6, which the headless test environment does not have,
    so the value is read from source -- and reading it is the point anyway: a
    Python literal restated here would detect no drift at all.
    """

    import re

    match = re.search(
        r"^\s+" + re.escape(name) + r"\s*=\s*([0-9.eE+-]+)\s*$",
        _MAIN_PY.read_text(),
        re.MULTILINE,
    )
    assert match is not None, f"{name} not found in {_MAIN_PY.name}"
    return float(match.group(1))


def test_loiter_attitude_window_is_no_laxer_than_the_firmware():
    """The GS must not stay engaged through an outage that trips the FC.

    The FC drops the orbit once its attitude estimate is ATTITUDE_STALE_TIMEOUT_US
    stale, and the rising-edge rule then refuses to restart it until CH10 cycles.
    A laxer ground-station window leaves a band of outage lengths that drop the
    FC's orbit while the GS holds CH10 high and never cycles it, stranding the
    request with nothing announcing the handover.

    Both sides are read from source so this cannot pass on stale literals.
    """

    fc_stale_s = _firmware_constexpr("ATTITUDE_STALE_TIMEOUT_US") * 1e-6
    gs_stale_s = _main_py_class_constant("LOITER_ATTITUDE_STALE_S")
    assert gs_stale_s <= fc_stale_s + 1e-9, (
        f"main.py LOITER_ATTITUDE_STALE_S ({gs_stale_s} s) is laxer than the "
        f"FC's ATTITUDE_STALE_TIMEOUT_US ({fc_stale_s} s): an attitude outage "
        "between the two drops the FC's orbit while the GS keeps CH10 high, "
        "and the rising-edge rule then strands the request"
    )


def test_mirrored_airspeed_tracks_the_real_firmware_constant():
    """The mirror must follow AIRBORNE_ENGAGE_AIRSPEED_MPS, read from source."""

    fc_mps = _firmware_define("AIRBORNE_ENGAGE_AIRSPEED_MPS")
    assert abs(FC_AIRBORNE_ENGAGE_AIRSPEED_MPH - fc_mps * 2.23694) < 0.05, (
        "modules/loiter.py FC_AIRBORNE_ENGAGE_AIRSPEED_MPH has drifted from "
        f"AIRBORNE_ENGAGE_AIRSPEED_MPS ({fc_mps} m/s) in Main.ino"
    )


def test_mirrored_height_tracks_the_real_firmware_constant():
    """The height half of the engage condition must track the firmware too."""

    fc_m = _firmware_define("AIRBORNE_ENGAGE_HEIGHT_M")
    assert abs(FC_AIRBORNE_ENGAGE_HEIGHT_FT - fc_m * 3.28084) < 0.05, (
        "modules/loiter.py FC_AIRBORNE_ENGAGE_HEIGHT_FT has drifted from "
        f"AIRBORNE_ENGAGE_HEIGHT_M ({fc_m} m) in Main.ino"
    )


def test_the_drift_guard_would_actually_catch_a_change():
    """Guard the guard: prove the extraction reads real values, not defaults."""

    assert _firmware_define("AIRBORNE_ENGAGE_AIRSPEED_MPS") > 0.0
    assert _firmware_define("AIRBORNE_ENGAGE_HEIGHT_M") > 0.0


def test_gs_cannot_latch_below_the_fc_airborne_threshold():
    """The GS airborne detector can be the laxer of the two.

    With the default warning config the GS latches airborne at 12 mph
    (stall_airspeed 10 x takeoff multiplier 1.2) while the FC waits for 8 m/s
    (17.9 mph). A hold in that window raises CH10, the FC refuses it as
    grounded, and its rising-edge rule means it stays refused even once the FC
    does latch airborne -- so the orbit never flies while the operator has been
    told it is.
    """

    assert fc_airborne_latched(False, True, 12.0, 100.0) is False, "the GS-only window must refuse"
    assert fc_airborne_latched(False, True, FC_AIRBORNE_ENGAGE_AIRSPEED_MPH, 100.0) is True
    assert fc_airborne_latched(False, True, 30.0, 100.0) is True


def test_the_latch_survives_a_slowdown():
    """The firmware latches: airspeed SETS the flag and never clears it.

    Only the disengage height does. Re-testing the engage threshold every
    cycle would drop a running orbit on any slow-down -- and the FC keeps
    orbiting below its airspeed floor by design, abandoning only altitude
    hold. This is the regression the previous commit introduced.
    """

    latched = fc_airborne_latched(False, True, 30.0, 100.0)
    assert latched is True

    # Slowing well below the engage threshold must NOT clear it.
    assert fc_airborne_latched(latched, True, 5.0, 100.0) is True
    assert fc_airborne_latched(latched, True, 0.0, 100.0) is True
    # Nor must losing the airspeed reading entirely.
    assert fc_airborne_latched(latched, True, None, 100.0) is True


def test_only_going_grounded_clears_the_latch():
    latched = fc_airborne_latched(False, True, 30.0, 100.0)
    assert fc_airborne_latched(latched, False, 30.0, 100.0) is False
    # ...and once cleared it needs the engage threshold again, not just airborne.
    assert fc_airborne_latched(False, True, 12.0, 100.0) is False


def test_missing_airspeed_cannot_set_the_latch():
    """Without airspeed the GS cannot establish the FC's engage condition."""

    assert fc_airborne_latched(False, True, None, 100.0) is False
    assert fc_airborne_latched(False, True, "fast", 100.0) is False


def test_engage_is_refused_inside_the_disagreement_window():
    """End to end: the controller must refuse, not engage-and-never-fly."""

    c = LoiterController()
    gates = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_live=True,
        transmitting=True,
        airborne=True,
        engage_airborne=fc_airborne_engage_ok(True, 12.0, 100.0),
    )
    assert _engage(c, gates).reason == REASON_GROUNDED
    assert not c.engaged


# ---------------------------------------------------------------------------
# Engage and hold read airborne state differently, on purpose
# ---------------------------------------------------------------------------


def test_a_stale_latch_cannot_authorise_a_new_request():
    """The recovery branch the GS cannot observe.

    After a watchdog reset in flight the firmware runs a different airborne
    branch, where airspeed below AIRBORNE_RECOVERY_DISENGAGE_AIRSPEED_MPS
    clears the flag. The GS cannot see watchdogRecoveryBoot, so its latch can
    read true while the FC has dropped the orbit. Accepting a NEW request on
    that stale belief would strand it permanently -- the FC refuses it and its
    rising-edge rule never retries. Engagement therefore re-derives the
    condition instead of trusting the latch.
    """

    latched_but_slow = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_live=True,
        transmitting=True,
        airborne=True,                                    # latch says yes...
        engage_airborne=fc_airborne_engage_ok(True, 5.0, 100.0),  # ...the FC would not
    )
    c = LoiterController()
    assert _engage(c, latched_but_slow).reason == REASON_GROUNDED
    assert not c.engaged


def test_a_running_orbit_survives_what_would_refuse_a_new_one():
    """The other half of the asymmetry.

    A false negative on the hold path interrupts flight on a ground-station
    guess, which is the worse error -- and the FC keeps orbiting below its own
    airspeed floor by design. So the hold path reads the latch, and a
    slow-down that would refuse a fresh request must not drop a running one.
    """

    c = LoiterController()
    _engage(c, _ready_gates())
    assert c.engaged

    slowed = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_live=True,
        transmitting=True,
        airborne=fc_airborne_latched(True, True, 5.0, 100.0),     # latch holds
        engage_airborne=fc_airborne_engage_ok(True, 5.0, 100.0),  # would refuse anew
        stick_roll=0.0, stick_pitch=0.0,
    )
    assert slowed.engage_airborne is False, "a fresh request would be refused here"
    assert slowed.airborne is True, "but the latch holds"
    assert c.poll(3.0, slowed) is None, "the running orbit must continue"
    assert c.engaged


def test_landing_still_ends_a_running_orbit():
    """The latch does clear when the GS's own landing detector says so."""

    c = LoiterController()
    _engage(c, _ready_gates())

    landed = LoiterGates(
        fbw_active=True, attitude_fresh=True, joystick_live=True,
        transmitting=True,
        airborne=fc_airborne_latched(True, False, 5.0, 100.0),
        engage_airborne=False,
        stick_roll=0.0, stick_pitch=0.0,
    )
    assert landed.airborne is False
    assert c.poll(3.0, landed).reason == REASON_GROUNDED
    assert not c.engaged


def test_a_fast_low_pass_cannot_authorise_a_new_request():
    """The height half of the divergence, separate from watchdog recovery.

    The FC clears its latch at AIRBORNE_DISENGAGE_HEIGHT_M (1.5 m) and will not
    set it again below 3 m, while the GS needs low speed AND low altitude
    sustained for its landing debounce. Through a fast low pass or a
    touch-and-go the GS stays airborne at full cruise speed while the FC has
    already gone grounded -- so checking airspeed alone would approve a request
    the firmware rejects and never retries.
    """

    # Cruise speed, but below the FC's engage height: must refuse.
    assert fc_airborne_engage_ok(True, 30.0, 5.0) is False
    assert fc_airborne_engage_ok(True, 30.0, FC_AIRBORNE_ENGAGE_HEIGHT_FT - 0.1) is False
    # At the height, with speed: allowed.
    assert fc_airborne_engage_ok(True, 30.0, FC_AIRBORNE_ENGAGE_HEIGHT_FT) is True


def test_both_halves_of_the_engage_condition_are_required():
    """Neither airspeed nor height alone may authorise a request."""

    fast_low = fc_airborne_engage_ok(True, 30.0, 1.0)
    slow_high = fc_airborne_engage_ok(True, 5.0, 500.0)
    assert fast_low is False, "speed without height must refuse"
    assert slow_high is False, "height without speed must refuse"
    assert fc_airborne_engage_ok(True, 30.0, 500.0) is True


def test_unknown_height_refuses_like_unknown_airspeed():
    """No baseline means no AGL, and the GS cannot establish the condition."""

    assert fc_airborne_engage_ok(True, 30.0, None) is False
    assert fc_airborne_engage_ok(True, 30.0, "high") is False


def test_a_low_pass_does_not_drop_a_running_orbit():
    """The asymmetry still holds: only engagement re-derives."""

    latched = fc_airborne_latched(False, True, 30.0, 100.0)
    assert latched is True
    # Descending through the FC engage height must not clear the GS latch.
    assert fc_airborne_latched(latched, True, 30.0, 1.0) is True


def test_stale_flight_metrics_refuse_like_missing_ones():
    """Cached-but-stale telemetry must not authorise a request.

    When GPS stops while attitude and the uplink stay live, the GS freezes its
    airborne state and the cached speed/altitude rather than guessing. If the
    aircraft lands during that outage those cached values still read airborne,
    and a request raised on them would be rejected by the FC and never retried.
    The caller passes None for stale readings, which must refuse exactly as a
    missing reading does.
    """

    # Fresh and flying: allowed.
    assert fc_airborne_engage_ok(True, 30.0, 100.0) is True
    # The same values, but stale, arrive as None from the caller.
    assert fc_airborne_engage_ok(True, None, None) is False
    assert fc_airborne_engage_ok(True, 30.0, None) is False
    assert fc_airborne_engage_ok(True, None, 100.0) is False
