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
* a bounded maximum duration elapsing.

Engaging additionally requires the ground station to believe the aircraft is
airborne, so a hold made on the ground is refused at the gesture rather than
quietly arming something the FC would later fly.

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
REASON_TIMEOUT = "timeout"


@dataclass(frozen=True)
class LoiterGates:
    """Everything the controller needs to decide whether loiter may run.

    ``joystick_live`` must mean "present AND producing fresh samples", not
    merely that a handler object exists.  The serial reader hands back its last
    cached axis values when the stream stalls, so an object-existence check
    would keep loiter engaged while stick movement and button releases could no
    longer be observed -- disabling the pilot's primary way out of the orbit.

    ``airborne`` is the ground station's own airborne estimate.  The FC gates on
    its own latch regardless; checking here as well is what lets a hold made on
    the ground be REFUSED audibly at the gesture instead of silently doing
    nothing.

    ``stick_roll``/``stick_pitch`` are normalized (-1..1) axis values, or
    ``None`` when the joystick has not produced a sample.
    """

    fbw_active: bool
    attitude_fresh: bool
    joystick_live: bool
    airborne: bool = False
    stick_roll: Optional[float] = None
    stick_pitch: Optional[float] = None


@dataclass(frozen=True)
class LoiterEvent:
    """One state transition worth annunciating."""

    kind: str
    reason: Optional[str] = None


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
        self.hold_seconds = float(hold_seconds)
        self.stick_break_norm = float(stick_break_norm)
        self.max_duration_s = float(max_duration_s)

        self._state = LOITER_DISENGAGED
        self._press_start: Optional[float] = None
        self._engaged_at: Optional[float] = None
        self._stick_baseline: Optional[tuple[Optional[float], Optional[float]]] = None
        # Set whenever the current press has already done its job (engaged
        # loiter, been refused, or disengaged a running orbit).  The matching
        # release must then NOT also fire the ordinary mode toggle.
        self._release_consumed = False
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
    def press(self, now: float) -> Optional[LoiterEvent]:
        """Register a press edge of the control-mode toggle.

        While an orbit is running this is the pilot's primary way out, so it
        disengages immediately on the press edge rather than waiting for the
        release.  Otherwise it starts the engage hold; the gates are not
        consulted until the hold matures in ``poll()``, so that a refusal is
        announced when the operator has actually asked for loiter rather than
        the instant they touch the control.
        """

        self._press_active = True

        if self._state == LOITER_ENGAGED:
            self._release_consumed = True
            return self._disengage(REASON_TOGGLE)

        self._press_start = float(now)
        self._state = LOITER_ARMING
        self._release_consumed = False
        return None

    def release(self, now: float) -> Optional[str]:
        """Register a release edge.

        Returns ``"toggle"`` when the press was a short tap that should perform
        the ordinary Manual/Fly-By-Wire toggle, and ``None`` when the press has
        already been consumed by engaging, refusing, or disengaging loiter, or
        when no press was outstanding at all.
        """

        if not self._press_active:
            # Unmatched release (see _press_active): no press was observed, so
            # there is no tap to act on.
            return None
        self._press_active = False

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
        self._press_start = None
        self._release_consumed = False
        self._state = LOITER_DISENGAGED

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

        if not gates.joystick_live:
            return REASON_NO_JOYSTICK
        if not gates.fbw_active:
            return REASON_NOT_FBW
        if not gates.attitude_fresh:
            return REASON_ATTITUDE_STALE
        if not gates.airborne:
            return REASON_GROUNDED
        return None

    def _hold_failure_reason(self, now: float, gates: LoiterGates) -> Optional[str]:
        """Why a running orbit must stop, or ``None`` when it may continue."""

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
