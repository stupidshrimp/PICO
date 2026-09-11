"""Ground-station half of loiter: the operator interface to the FC orbit.

The orbit itself is flown by the flight controller (see
``flight_controller/loiter_nav.h``), which substitutes a fixed bank and level
pitch for the pilot's stick inside its own Fly-By-Wire branch.  This module
owns only what the FC cannot: the engage gesture, the conditions the ground
station can see, and handing control back.  All it transmits is a request on
CH10 -- it never commands an attitude.

That division is deliberate.  A pilot reaches for loiter when the model is far
away and they need a moment, which is exactly when the link is weakest, so the
loop that holds the wings over has to live on the aircraft.  What stays here is
everything involving a joystick, which the FC knows nothing about.

Engagement is a *hold* of the control-mode toggle (Ctrl+M or the joystick's
control-mode button) for ``hold_seconds``.  A short tap of the same control
still performs the ordinary Manual/Fly-By-Wire toggle, so the tap can only be
resolved on the release edge -- see ``release()``.

Disengagement is deliberately easy and happens on any of:

* another press of the toggle control,
* the stick moving away from where it sat at engage time,
* the FC no longer being commanded Fly-By-Wire,
* attitude telemetry going stale (the GS cannot see whether the FC has
  dropped to its limited-authority pass-through, so a stale estimate is
  treated as loss of the attitude loop this mode depends on),
* the joystick disappearing OR its sample stream going stale,
* packet transmission being terminated,
* a bounded maximum duration elapsing.

Engaging additionally requires BOTH airborne detectors to agree, so a hold made
on the ground -- or in the window where the GS has latched airborne and the FC
has not -- is refused at the gesture rather than quietly arming something the
firmware will reject and then never retry.

Note what "engaged" means here: the ground station has accepted the gesture and
is REQUESTING the orbit on CH10.  It is not a statement that the aircraft is
orbiting.  Nothing in the downlink reports the FC's nav state, so the ground
station cannot know that, and callers must not present it as though it does --
the confirmation is watching the aircraft turn.

The FC enforces its own gates independently and continuously -- RC freshness,
Fly-By-Wire, a converged attitude estimate, and the airborne latch -- so
dropping the request here is one of two ways the orbit ends, not the only one.

Kept free of Qt imports so the state machine is unit-testable headless (the
package initializer is deliberately empty for the same reason).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------

# How long the control-mode toggle must be held before loiter engages.
LOITER_HOLD_SECONDS = 2.0

# Floor for the configured hold.  config.json is hand-editable, and a zero or
# negative value would make the very first poll tick satisfy the hold -- so an
# ordinary tap spanning one GUI refresh would engage an autonomous mode, which
# is exactly what the hold gesture exists to prevent.  A tiny positive value
# fails the same way, so this is a floor rather than a sign check.
LOITER_MIN_HOLD_SECONDS = 0.5

# Bounds for the configured stick-break threshold.  Too large is the dangerous
# direction: it would disable the pilot's primary way out of the orbit.  Too
# small only makes the mode twitchy.
#
# The maximum is deliberately BELOW 1.0.  A centred stick has exactly 1.0 of
# travel available and the comparison is strictly greater-than, so a cap of 1.0
# would still leave full deflection unable to break out -- the very failure the
# cap exists to prevent.  0.75 guarantees breakout from any baseline within a
# quarter of centre.
LOITER_MIN_STICK_BREAK_NORM = 0.02
LOITER_MAX_STICK_BREAK_NORM = 0.75

# The FC's own airborne-latch ENGAGE airspeed, mirrored here so the ground
# station can refuse to accept a request the firmware would reject.  Authority
# is AIRBORNE_ENGAGE_AIRSPEED_MPS (8.0 m/s) in flight_controller/Main.ino; the
# test suite reads that macro out of the firmware so the mirror cannot drift.
#
# ENGAGE is the operative word: the firmware latches, and only the disengage
# HEIGHT clears the flag afterwards.  See fc_airborne_latched().
#
# The two airborne detectors are independent and use different thresholds, and
# the GS one can be the LAXER of the pair: with the default warning config it
# latches at 12 mph while the FC waits for 17.9. A hold in that window raises
# CH10, the FC refuses it as grounded, and -- because a refused request needs a
# fresh rising edge -- it stays refused even after the FC does latch airborne.
# The orbit then never flies while the ground station believes it is flying.
FC_AIRBORNE_ENGAGE_AIRSPEED_MPH = 17.9

# The other half of that engage condition: the firmware requires height above
# its ground reference as well as airspeed.  Authority is
# AIRBORNE_ENGAGE_HEIGHT_M (3.0 m) in flight_controller/Main.ino; the test suite
# reads that macro too.
#
# Height is where the two detectors diverge most sharply.  The FC clears its
# latch the moment height falls to AIRBORNE_DISENGAGE_HEIGHT_M (1.5 m) and will
# not set it again below 3 m, while the ground station needs low speed AND low
# altitude sustained for its landing debounce.  Through a fast low pass or a
# touch-and-go the GS therefore stays airborne at full cruise speed while the
# FC has already gone grounded -- so an engage check on airspeed alone approves
# a request the firmware will reject and never retry.
FC_AIRBORNE_ENGAGE_HEIGHT_FT = 9.84

# Which input delivered a press.  Releases are matched against it so an
# unmatched edge from one control cannot consume the other's press.
PRESS_SOURCE_KEY = "key"
PRESS_SOURCE_JOYSTICK = "joystick"

# CH10/AUX6 carries the loiter request.  CH8/CH9 are taken by the
# board-alignment trim.  Values mirror the CH6/CH7 encoding: an explicit high
# requests the mode, and the low value means off.
LOITER_CHANNEL_INDEX = 9
LOITER_CHANNEL_REQUEST_VALUE = 1700
LOITER_CHANNEL_OFF_VALUE = 400

# How far the stick must move from where it sat at engage time before loiter
# hands control back.  Normalized stick units (-1..1), so 0.25 is a quarter of
# full deflection on either axis.  Large enough that link/serial jitter and a
# resting hand cannot trip it, small enough that a deliberate nudge does.
LOITER_STICK_BREAK_NORM = 0.25

# Upper bound on a single loiter run.  A mode that flies the aircraft with
# nobody touching the controls should not do so indefinitely.
LOITER_MAX_DURATION_S = 300.0

# States
LOITER_DISENGAGED = "disengaged"
LOITER_ARMING = "arming"
LOITER_ENGAGED = "engaged"

# Event kinds emitted to the caller
EVENT_ENGAGED = "engaged"
EVENT_REFUSED = "refused"
EVENT_DISENGAGED = "disengaged"

# Reasons, carried on refuse/disengage events for annunciation and logging
REASON_TOGGLE = "toggle"
REASON_STICK = "stick"
REASON_NOT_FBW = "not_fbw"
REASON_ATTITUDE_STALE = "attitude_stale"
REASON_NO_JOYSTICK = "no_joystick"
REASON_GROUNDED = "grounded"
REASON_NOT_TRANSMITTING = "not_transmitting"
REASON_TIMEOUT = "timeout"


@dataclass(frozen=True)
class LoiterGates:
    """Everything the controller needs to decide whether loiter may run.

    ``joystick_live`` must mean "present AND producing fresh samples", not
    merely that a handler object exists.  The serial reader hands back its last
    cached axis values when the stream stalls, so an object-existence check
    would keep loiter engaged while stick movement and button releases could no
    longer be observed -- disabling the pilot's primary way out of the orbit.

    ``transmitting`` must reflect a genuinely open uplink, not merely the
    intent to transmit.  Terminating transmission from the configuration page
    stops the RC frames while leaving telemetry flowing, so every other gate
    here can stay satisfied with nothing reaching the aircraft.  Engaging then
    would arm CH10 locally and the NEXT "start transmitting" click would carry
    it high as a fresh edge -- making the click that restores control, rather
    than a loiter gesture, the thing that starts the orbit.

    ``airborne`` and ``engage_airborne`` are deliberately separate, because the
    ground station's knowledge of the FC's airborne latch is asymmetric.
    ``engage_airborne`` is the FC's engage condition re-derived NOW and gates a
    new request: accepting one the FC would refuse strands it permanently,
    since a refused request is never retried.  ``airborne`` is a latched
    estimate and only decides whether a RUNNING orbit continues, where a false
    negative is the worse error -- it would interrupt flight on a guess.

    ``stick_roll``/``stick_pitch`` are normalized (-1..1) axis values, or
    ``None`` when the joystick has not produced a sample.
    """

    fbw_active: bool
    attitude_fresh: bool
    joystick_live: bool
    airborne: bool = False
    engage_airborne: bool = False
    transmitting: bool = True
    stick_roll: Optional[float] = None
    stick_pitch: Optional[float] = None


@dataclass(frozen=True)
class LoiterEvent:
    """One state transition worth annunciating."""

    kind: str
    reason: Optional[str] = None


def fc_airborne_engage_ok(
    gs_airborne: bool,
    airspeed_mph: Optional[float],
    height_agl_ft: Optional[float],
) -> bool:
    """The FC's airborne ENGAGE condition, evaluated fresh, in FULL.

    Used to authorise a NEW request, never to keep an existing one running.
    A request the FC refuses is never retried -- its rising-edge rule keeps it
    refused even once the FC does latch airborne -- so accepting one on a stale
    or partial belief leaves the orbit permanently unflown while the operator is
    told otherwise.  Re-deriving the condition rather than trusting a latch is
    what makes that impossible.

    BOTH halves of the firmware's condition are required.  Airspeed alone is
    not enough: the FC also needs height above its ground reference, and that
    is where the two detectors diverge most -- through a fast low pass the
    ground station stays airborne at cruise speed while the FC has already
    cleared its latch on height.

    A missing reading of either refuses.  The GS cannot establish the FC's
    condition without both, and refusing is the safe answer: it costs the
    operator a repeated gesture, where accepting costs them an orbit that
    never flies.
    """

    if not gs_airborne or airspeed_mph is None or height_agl_ft is None:
        return False
    try:
        speed = float(airspeed_mph)
        height = float(height_agl_ft)
    except (TypeError, ValueError):
        return False
    return (
        speed >= FC_AIRBORNE_ENGAGE_AIRSPEED_MPH
        and height >= FC_AIRBORNE_ENGAGE_HEIGHT_FT
    )


def fc_airborne_latched(
    previous: bool,
    gs_airborne: bool,
    airspeed_mph: Optional[float],
    height_agl_ft: Optional[float],
) -> bool:
    """Track whether the FC probably still considers the aircraft airborne.

    Used to decide whether a RUNNING orbit may continue, and deliberately NOT
    to authorise a new one -- see fc_airborne_engage_ok.

    In normal flight the firmware latches: airspeed is checked only to SET the
    flag, and once set only falling below the disengage HEIGHT clears it (see
    ``updateAirborneState``).  Re-testing the engage threshold every cycle would
    be a different condition entirely: an airborne aircraft that slowed below it
    would have its orbit dropped, which the FC would not do -- it keeps orbiting
    below its own airspeed floor by design and abandons only altitude hold.
    Interrupting flight on a ground-station guess is the worse error, so this
    holds through a slow-down and clears only when the GS's own landing
    detector says the aircraft is down.

    This mirror is KNOWN TO BE IMPERFECT and cannot be made exact.  After a
    watchdog reset in flight the firmware runs a different branch, where no
    ground reference exists and airspeed below
    AIRBORNE_RECOVERY_DISENGAGE_AIRSPEED_MPS (6 m/s) does clear the flag.  The
    ground station cannot see ``watchdogRecoveryBoot`` -- nothing in the
    downlink reports it -- so in that case this can read true while the FC has
    dropped the orbit.  The consequence is a stale display, which is the same
    blind spot that makes the indicator read "Loiter req" rather than "Loiter";
    it cannot authorise anything, because engagement goes through
    fc_airborne_engage_ok instead.
    """

    if not gs_airborne:
        return False
    if previous:
        return True
    return fc_airborne_engage_ok(gs_airborne, airspeed_mph, height_agl_ft)


def loiter_channel_value(engaged: bool) -> int:
    """Return the CH10/AUX6 value for the current loiter state.

    The FC treats anything below its threshold as off, so the low value is not
    merely conventional: it is what stops a centred or defaulted channel from
    ever reading as a request.
    """

    return LOITER_CHANNEL_REQUEST_VALUE if engaged else LOITER_CHANNEL_OFF_VALUE


def stick_break_exceeded(
    baseline: Optional[tuple[Optional[float], Optional[float]]],
    stick_roll: Optional[float],
    stick_pitch: Optional[float],
    threshold: float = LOITER_STICK_BREAK_NORM,
) -> bool:
    """True when the stick has moved far enough from ``baseline`` to break loiter.

    A missing baseline or a missing current sample never breaks loiter on its
    own: a stalled joystick stream is handled by the ``joystick_live`` gate,
    and treating "no sample" as movement would drop the mode on serial jitter.
    """

    if baseline is None:
        return False
    base_roll, base_pitch = baseline
    for base, current in ((base_roll, stick_roll), (base_pitch, stick_pitch)):
        if base is None or current is None:
            continue
        if abs(float(current) - float(base)) > threshold:
            return True
    return False


class LoiterController:
    """Hold-to-engage state machine for the fixed-bank loiter orbit.

    The controller owns no Qt objects and reads no globals; the caller feeds it
    press/release edges plus a ``LoiterGates`` snapshot and acts on the events
    it returns.  ``now`` is a monotonic seconds value supplied by the caller so
    tests can drive time directly.
    """

    def __init__(
        self,
        *,
        hold_seconds: float = LOITER_HOLD_SECONDS,
        stick_break_norm: float = LOITER_STICK_BREAK_NORM,
        max_duration_s: float = LOITER_MAX_DURATION_S,
    ) -> None:
        # Validate rather than trust: these come from a hand-editable config
        # file, and the failure modes are asymmetric -- a bad hold or a bad
        # break threshold makes an autonomous mode easier to enter or harder to
        # escape.  Guarding here rather than at the call site keeps every
        # caller safe, including tests and any future one.
        hold = float(hold_seconds)
        if not (hold > 0.0):
            hold = LOITER_HOLD_SECONDS
        self.hold_seconds = max(LOITER_MIN_HOLD_SECONDS, hold)

        break_norm = float(stick_break_norm)
        if not (break_norm > 0.0):
            break_norm = LOITER_STICK_BREAK_NORM
        self.stick_break_norm = min(
            LOITER_MAX_STICK_BREAK_NORM, max(LOITER_MIN_STICK_BREAK_NORM, break_norm)
        )

        # A non-positive duration disables the timeout, which is documented and
        # intentional; normalise negatives to 0 so the intent reads clearly.
        duration = float(max_duration_s)
        self.max_duration_s = duration if duration > 0.0 else 0.0

        self._state = LOITER_DISENGAGED
        self._press_start: Optional[float] = None
        self._engaged_at: Optional[float] = None
        self._stick_baseline: Optional[tuple[Optional[float], Optional[float]]] = None
        # Set whenever the current press has already done its job (engaged
        # loiter, been refused, or disengaged a running orbit).  The matching
        # release must then NOT also fire the ordinary mode toggle.
        self._release_consumed = False
        # Which input owns the outstanding press, so a release from the other
        # one cannot consume it.
        self._press_source: Optional[str] = None
        # True between a press edge and its matching release.  Guards against
        # an UNMATCHED release, which the joystick really can deliver: connect
        # or reconnect while the button is already held and the parser forwards
        # the eventual "RELEASED" with no "PRESSED" before it.  Treating that as
        # a tap would flip Manual/Fly-By-Wire with nobody having pressed
        # anything.  (The Ctrl+M path guards this in the event filter; the
        # joystick path has no equivalent, so it belongs here.)
        self._press_active = False

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------
    @property
    def state(self) -> str:
        return self._state

    @property
    def engaged(self) -> bool:
        return self._state == LOITER_ENGAGED

    @property
    def arming(self) -> bool:
        return self._state == LOITER_ARMING

    def hold_progress(self, now: float) -> float:
        """Fraction of the engage hold completed (0..1); 0 when not arming."""

        if self._state != LOITER_ARMING or self._press_start is None:
            return 0.0
        if self.hold_seconds <= 0.0:
            return 1.0
        elapsed = float(now) - self._press_start
        return max(0.0, min(1.0, elapsed / self.hold_seconds))

    def elapsed(self, now: float) -> float:
        """Seconds the current orbit has been engaged; 0 when disengaged."""

        if self._engaged_at is None:
            return 0.0
        return max(0.0, float(now) - self._engaged_at)

    # ------------------------------------------------------------------
    # Edges
    # ------------------------------------------------------------------
    def press(
        self, now: float, source: str = PRESS_SOURCE_KEY
    ) -> Optional[LoiterEvent]:
        """Register a press edge of the control-mode toggle.

        While an orbit is running this is the pilot's primary way out, so it
        disengages immediately on the press edge rather than waiting for the
        release.  Otherwise it starts the engage hold; the gates are not
        consulted until the hold matures in ``poll()``, so that a refusal is
        announced when the operator has actually asked for loiter rather than
        the instant they touch the control.
        """

        # A press is always honoured, whichever control it came from: it is the
        # disengage path, and refusing one because another control is mid-hold
        # would leave the operator unable to stop a running orbit.  The most
        # recent press simply owns the gesture from here.
        self._press_active = True
        self._press_source = source

        if self._state == LOITER_ENGAGED:
            self._release_consumed = True
            return self._disengage(REASON_TOGGLE)

        self._press_start = float(now)
        self._state = LOITER_ARMING
        self._release_consumed = False
        return None

    def release(
        self, now: float, source: str = PRESS_SOURCE_KEY
    ) -> Optional[str]:
        """Register a release edge.

        Returns ``"toggle"`` when the press was a short tap that should perform
        the ordinary Manual/Fly-By-Wire toggle, and ``None`` when the press has
        already been consumed by engaging, refusing, or disengaging loiter, or
        when no press was outstanding at all.
        """

        if not self._press_active or source != self._press_source:
            # Either no press was observed at all, or this release belongs to
            # the other control.  Both must be ignored: an unmatched joystick
            # release arriving while Ctrl+M is held would otherwise cancel the
            # keyboard hold and toggle Manual/Fly-By-Wire with the key still
            # down.
            return None
        self._press_active = False
        self._press_source = None

        consumed = self._release_consumed
        self._release_consumed = False
        self._press_start = None
        if self._state == LOITER_ARMING:
            self._state = LOITER_DISENGAGED
        if consumed:
            return None
        return REASON_TOGGLE

    def cancel_press(self) -> None:
        """Forget an in-flight engage hold without toggling (focus loss, disconnect).

        Only an *arming* press can be cancelled.  Once a press has matured into
        an engage there is nothing in flight to cancel, and clearing the
        consumed flag then would leave a late release free to fire the ordinary
        mode toggle -- flipping a running orbit out of Fly-By-Wire.  Callers
        that also swallow the release edge mask that, but the state machine
        must not depend on them doing so.
        """

        if self._state != LOITER_ARMING:
            return

        # The abandoned press has no matching action left, so its eventual
        # release must be ignored rather than read as a tap.
        self._press_active = False
        self._press_source = None
        self._press_start = None
        self._release_consumed = False
        self._state = LOITER_DISENGAGED

    def abort(self, reason: str) -> Optional[LoiterEvent]:
        """Force loiter off immediately, outside the poll cycle.

        The gates in ``poll()`` are periodic, so they only catch a condition
        that is still false when the next tick runs.  A transport or handler
        being torn down and rebuilt inside a single GUI callback never presents
        such a tick: the link looks continuously healthy across the swap, and a
        running orbit would survive a device change that removed the very input
        the pilot would take over with -- or, worse, seed the replacement
        transport with CH10 already high, which a freshly connected FC reads as
        a rising edge and flies.

        Callers must therefore abort synchronously at the teardown itself.
        Cancels any outstanding press too, so the eventual release is ignored
        rather than read as a tap.
        """

        self._press_active = False
        self._press_source = None
        self._press_start = None
        self._release_consumed = False
        if self._state == LOITER_ENGAGED:
            return self._disengage(reason)
        self._state = LOITER_DISENGAGED
        return None

    # ------------------------------------------------------------------
    # Periodic update
    # ------------------------------------------------------------------
    def poll(self, now: float, gates: LoiterGates) -> Optional[LoiterEvent]:
        """Advance the hold timer and re-check the gates for a running orbit."""

        now = float(now)

        if self._state == LOITER_ARMING:
            if self._press_start is None:
                self._state = LOITER_DISENGAGED
                return None
            if now - self._press_start < self.hold_seconds:
                return None
            # The hold completed: engage, or refuse and say why.  Either way
            # the press has done its job, so the release must not also toggle.
            self._release_consumed = True
            refusal = self._refusal_reason(gates)
            if refusal is not None:
                self._state = LOITER_DISENGAGED
                return LoiterEvent(EVENT_REFUSED, refusal)
            self._state = LOITER_ENGAGED
            self._engaged_at = now
            self._stick_baseline = (gates.stick_roll, gates.stick_pitch)
            return LoiterEvent(EVENT_ENGAGED)

        if self._state == LOITER_ENGAGED:
            reason = self._hold_failure_reason(now, gates)
            if reason is not None:
                return self._disengage(reason)

        return None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    @staticmethod
    def _refusal_reason(gates: LoiterGates) -> Optional[str]:
        """Why loiter may not engage right now, or ``None`` when it may."""

        if not gates.transmitting:
            return REASON_NOT_TRANSMITTING
        if not gates.joystick_live:
            return REASON_NO_JOYSTICK
        if not gates.fbw_active:
            return REASON_NOT_FBW
        if not gates.attitude_fresh:
            return REASON_ATTITUDE_STALE
        if not gates.engage_airborne:
            return REASON_GROUNDED
        return None

    def _hold_failure_reason(self, now: float, gates: LoiterGates) -> Optional[str]:
        """Why a running orbit must stop, or ``None`` when it may continue."""

        if not gates.transmitting:
            # Nothing is reaching the aircraft, so the local "engaged" state is
            # a fiction. Dropping it here also stops CH10 being left high for
            # the next start-transmitting click to deliver as a fresh edge.
            return REASON_NOT_TRANSMITTING
        if not gates.joystick_live:
            return REASON_NO_JOYSTICK
        if not gates.fbw_active:
            return REASON_NOT_FBW
        if not gates.attitude_fresh:
            return REASON_ATTITUDE_STALE
        if not gates.airborne:
            # The FC drops the orbit on its own airborne latch, and its
            # rising-edge requirement means it will NOT resume when the latch
            # returns.  Following it here keeps the two sides telling the
            # operator the same story: without this the indicator would still
            # read "Loiter" while the aircraft had quietly handed the stick
            # back.  The two detectors use different thresholds, so whichever
            # calls "grounded" first wins -- which is the safe direction.
            return REASON_GROUNDED
        if stick_break_exceeded(
            self._stick_baseline,
            gates.stick_roll,
            gates.stick_pitch,
            self.stick_break_norm,
        ):
            return REASON_STICK
        if self.max_duration_s > 0.0 and self.elapsed(now) >= self.max_duration_s:
            return REASON_TIMEOUT
        return None

    def _disengage(self, reason: str) -> LoiterEvent:
        self._state = LOITER_DISENGAGED
        self._engaged_at = None
        self._stick_baseline = None
        self._press_start = None
        return LoiterEvent(EVENT_DISENGAGED, reason)
