"""Ground-station loiter mode: a fixed-bank orbit flown over the CRSF link.

Loiter v1 deliberately performs NO navigation.  It commands a constant bank
angle and level pitch through the existing Fly-By-Wire path: the flight
controller closes its 125 Hz attitude loop exactly as it does for stick input,
and the aircraft simply circles where it is.  There is no GPS dependency, no
position loop, and no new firmware on the FC.

Two consequences of that choice, both deliberate for a first autonomous mode:

* The circle drifts downwind.  Holding it over a fixed point needs a position
  loop, which is the next stage rather than this one.
* Loiter ends the moment the link drops.  The FC's own RC failsafe
  (``RC_FAILSAFE_TIMEOUT_US``, 250 ms) takes over and the model glides, which
  is the existing proven behaviour.  This mode therefore can never become a
  link-loss return-to-home; that requires the loop to live on the FC.

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
* the joystick disappearing,
* a bounded maximum duration elapsing.

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

# Commanded bank angle for the orbit.  Well inside the default GS Fly-By-Wire
# roll limit (45 deg) and far inside the FC's 80 deg hard clamp, so both
# existing envelopes stay redundant rather than active.
LOITER_BANK_ANGLE_DEG = 20.0

# Commanded pitch for the orbit.  Level: this version holds no altitude, it
# only stops the nose wandering while the bank does the work.
LOITER_PITCH_ANGLE_DEG = 0.0

# Sign of the commanded bank.  This follows the FC's roll convention rather
# than a compass direction -- per docs/protocol_contract.md the firmware
# treats left rolls as positive -- so confirm which way the aircraft actually
# circles on the first flight and flip this if it turns the wrong way.
LOITER_BANK_DIRECTION = 1.0

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
REASON_TIMEOUT = "timeout"


@dataclass(frozen=True)
class LoiterGates:
    """Everything the controller needs to decide whether loiter may run.

    ``stick_roll``/``stick_pitch`` are normalized (-1..1) axis values, or
    ``None`` when the joystick has not produced a sample.
    """

    fbw_active: bool
    attitude_fresh: bool
    joystick_present: bool
    stick_roll: Optional[float] = None
    stick_pitch: Optional[float] = None


@dataclass(frozen=True)
class LoiterEvent:
    """One state transition worth annunciating."""

    kind: str
    reason: Optional[str] = None


def bank_to_channel_norm(
    bank_deg: float, fc_limit_deg: float, gs_limit_deg: Optional[float] = None
) -> float:
    """Return the normalized channel value that commands ``bank_deg`` at the FC.

    The FC reads roll/pitch channels as ``normalized * FC hard limit``, so a
    loiter command that wants a true angle has to be expressed against that
    hard limit directly rather than against the stick's scaled range.  When
    ``gs_limit_deg`` is supplied the request is first clamped into the
    operator's own Fly-By-Wire envelope, so loiter can never command a
    steeper attitude than the configured limits allow by hand.
    """

    if not fc_limit_deg or fc_limit_deg <= 0.0:
        return 0.0
    target = float(bank_deg)
    if gs_limit_deg is not None:
        limit = abs(float(gs_limit_deg))
        target = max(-limit, min(limit, target))
    normalized = target / float(fc_limit_deg)
    return max(-1.0, min(1.0, normalized))


def stick_break_exceeded(
    baseline: Optional[tuple[Optional[float], Optional[float]]],
    stick_roll: Optional[float],
    stick_pitch: Optional[float],
    threshold: float = LOITER_STICK_BREAK_NORM,
) -> bool:
    """True when the stick has moved far enough from ``baseline`` to break loiter.

    A missing baseline or a missing current sample never breaks loiter on its
    own: a stalled joystick stream is handled by the ``joystick_present`` gate,
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
        bank_angle_deg: float = LOITER_BANK_ANGLE_DEG,
        pitch_angle_deg: float = LOITER_PITCH_ANGLE_DEG,
        bank_direction: float = LOITER_BANK_DIRECTION,
        stick_break_norm: float = LOITER_STICK_BREAK_NORM,
        max_duration_s: float = LOITER_MAX_DURATION_S,
    ) -> None:
        self.hold_seconds = float(hold_seconds)
        self.bank_angle_deg = float(bank_angle_deg)
        self.pitch_angle_deg = float(pitch_angle_deg)
        self.bank_direction = 1.0 if float(bank_direction) >= 0.0 else -1.0
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
    # Commanded attitude
    # ------------------------------------------------------------------
    def command_angles(
        self, gs_roll_limit_deg: float, gs_pitch_limit_deg: float
    ) -> tuple[float, float]:
        """Return the (roll, pitch) degrees this orbit wants, already clamped."""

        roll = self.bank_angle_deg * self.bank_direction
        roll_limit = abs(float(gs_roll_limit_deg))
        pitch_limit = abs(float(gs_pitch_limit_deg))
        roll = max(-roll_limit, min(roll_limit, roll))
        pitch = max(-pitch_limit, min(pitch_limit, self.pitch_angle_deg))
        return roll, pitch

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
        already been consumed by engaging, refusing, or disengaging loiter.
        """

        consumed = self._release_consumed
        self._release_consumed = False
        self._press_start = None
        if self._state == LOITER_ARMING:
            self._state = LOITER_DISENGAGED
        if consumed:
            return None
        return REASON_TOGGLE

    def cancel_press(self) -> None:
        """Forget any in-flight press without toggling (focus loss, disconnect)."""

        self._press_start = None
        self._release_consumed = False
        if self._state == LOITER_ARMING:
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

        if not gates.joystick_present:
            return REASON_NO_JOYSTICK
        if not gates.fbw_active:
            return REASON_NOT_FBW
        if not gates.attitude_fresh:
            return REASON_ATTITUDE_STALE
        return None

    def _hold_failure_reason(self, now: float, gates: LoiterGates) -> Optional[str]:
        """Why a running orbit must stop, or ``None`` when it may continue."""

        if not gates.joystick_present:
            return REASON_NO_JOYSTICK
        if not gates.fbw_active:
            return REASON_NOT_FBW
        if not gates.attitude_fresh:
            return REASON_ATTITUDE_STALE
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
