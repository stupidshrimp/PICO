"""Ground-station side of the auto-trim helper.

Auto-calculates the surface (elevator/aileron) trim from the attitude telemetry
the FC already streams back at ~125 Hz. It complements the manual joystick trim
(``_set_trim_level`` in main.py): instead of the operator nudging the trim by
hand while watching the aircraft drift, the GS averages the residual bank/pitch
over a short window of straight-and-level flight and works out how much trim is
needed to null it.

Two behaviours, selected on the command page:

* Suggest (default): it reports the recommended trim levels but changes nothing
  -- the operator reads the numbers and applies them (or not).
* Apply: a gentle closed loop that rate-limits the trim toward level, then
  latches once the aircraft holds level hands-off.

The calculation only makes sense when the pilot is flying manually, wings level,
sticks centred, fast enough to have real control authority, and the attitude
telemetry is fresh -- otherwise it would chase turns and gusts. The gating
inputs live in main.py; this module keeps the maths Qt-free so it stays
unit-testable headless, mirroring modules/board_align_trim.py and
modules/compass_cal.py.

Sign conventions (derived from main.py, keep in lockstep):

* ``_apply_trim_to_channels`` adds a positive ``aileron_trim_level`` onto the
  roll channel (CH1) and a positive ``elevator_trim_level`` onto the pitch
  channel (CH2). Per the trim-state comment in main.py, a positive aileron trim
  commands LEFT-bank trim and a positive elevator trim commands NOSE-UP trim,
  matching the FC convention that the attitude telemetry also uses.
* So to correct a measured *positive* roll (aircraft banked left) back to level
  we need *negative* aileron trim, and to correct a measured *positive* pitch
  (nose high) we need *negative* elevator trim. Both corrections are therefore
  proportional to the NEGATED average attitude error.
"""

from __future__ import annotations

from collections import deque
from typing import Optional

# Trim-level ceiling. Must match TRIM_MAX_LEVEL in main.py (the manual trim uses
# the same clamp); kept here so the recommendation never suggests a level the
# transmit path cannot honour. Same "keep in lockstep with main.py" pattern as
# the FBW / board-align limits mirrored elsewhere.
DEFAULT_TRIM_MAX_LEVEL = 25

# --- Gating thresholds ------------------------------------------------------
# Minimum airspeed (mph) before the surfaces have enough authority for the
# measured attitude to reflect trim rather than a stall mush.
AUTO_TRIM_MIN_AIRSPEED_MPH = 15.0

# How close to centre (fraction of full stick travel, 0..1) both sticks must sit
# for the flight to count as "hands-off". 0.06 ~= 6% of travel.
AUTO_TRIM_STICK_CENTER_TOLERANCE = 0.06

# Attitude telemetry older than this (seconds) is treated as stale -- sampling
# pauses rather than trimming on a frozen number.
AUTO_TRIM_ATTITUDE_STALE_S = 0.5

# A joystick sample older than this (seconds) is treated as stale. The stick
# cache in main.py holds its last good value when a read fails, so the hands-off
# check must confirm the reading is live rather than a frozen centred sample.
AUTO_TRIM_STICK_STALE_S = 0.5

# --- Averaging window -------------------------------------------------------
# Length of the rolling window (seconds) and the minimum sample count within it
# before an average is trusted. At ~125 Hz attitude the count is easily met; the
# floor guards the case where telemetry is sparse.
AUTO_TRIM_SAMPLE_WINDOW_S = 2.0
AUTO_TRIM_MIN_SAMPLES = 8

# --- Controller tuning ------------------------------------------------------
# Aircraft is considered level (no trim change) when the averaged roll and pitch
# are both within this dead-band (degrees). Also the convergence/latch test.
AUTO_TRIM_DEAD_BAND_DEG = 1.0

# One-shot suggestion gain (trim levels per degree of residual attitude). Turns
# the measured bias straight into a recommended trim level.
AUTO_TRIM_SUGGEST_GAIN = 0.6

# Closed-loop gain and per-update ceiling. The apply loop deliberately moves at
# most a level or two each cycle so the trim eases toward level over a few
# seconds instead of snapping and oscillating.
AUTO_TRIM_APPLY_GAIN = 0.4
AUTO_TRIM_MAX_STEP_PER_UPDATE = 1


def sticks_centered(
    roll_norm: Optional[float],
    pitch_norm: Optional[float],
    tolerance: float = AUTO_TRIM_STICK_CENTER_TOLERANCE,
) -> bool:
    """Return whether both sticks sit within ``tolerance`` of centre.

    ``roll_norm`` / ``pitch_norm`` are the normalised (-1..1) stick positions
    cached by ``_capture_stick_state`` in main.py. ``None`` (no joystick / no
    reading) counts as not centred so the loop never trims blind.
    """
    if roll_norm is None or pitch_norm is None:
        return False
    return abs(roll_norm) <= tolerance and abs(pitch_norm) <= tolerance


def is_trimmed(
    avg_roll_deg: float,
    avg_pitch_deg: float,
    dead_band_deg: float = AUTO_TRIM_DEAD_BAND_DEG,
) -> bool:
    """Return whether the averaged attitude is level within the dead-band."""
    try:
        return abs(float(avg_roll_deg)) <= dead_band_deg and (
            abs(float(avg_pitch_deg)) <= dead_band_deg
        )
    except (TypeError, ValueError):
        return False


def _clamp_int(value: float, low: int, high: int) -> int:
    return int(max(low, min(high, value)))


def recommended_trim_levels(
    avg_roll_deg: float,
    avg_pitch_deg: float,
    current_aileron_level: int = 0,
    current_elevator_level: int = 0,
    gain: float = AUTO_TRIM_SUGGEST_GAIN,
    dead_band_deg: float = AUTO_TRIM_DEAD_BAND_DEG,
    max_level: int = DEFAULT_TRIM_MAX_LEVEL,
) -> tuple[int, int]:
    """Return the recommended ABSOLUTE (aileron, elevator) trim levels.

    Turns the residual attitude straight into the trim levels that should null
    it: the required change is ``-gain * error`` (see the sign note at the top
    of the module), added to whatever trim is already in effect and clamped to
    the transmissible range. Within the dead-band no change is recommended, so
    an already-level aircraft is left alone.
    """
    try:
        roll = float(avg_roll_deg)
        pitch = float(avg_pitch_deg)
        ail = int(current_aileron_level)
        elev = int(current_elevator_level)
    except (TypeError, ValueError):
        return current_aileron_level, current_elevator_level

    ail_delta = 0 if abs(roll) <= dead_band_deg else round(-gain * roll)
    elev_delta = 0 if abs(pitch) <= dead_band_deg else round(-gain * pitch)

    new_ail = _clamp_int(ail + ail_delta, -max_level, max_level)
    new_elev = _clamp_int(elev + elev_delta, -max_level, max_level)
    return new_ail, new_elev


def apply_trim_step(
    avg_roll_deg: float,
    avg_pitch_deg: float,
    gain: float = AUTO_TRIM_APPLY_GAIN,
    dead_band_deg: float = AUTO_TRIM_DEAD_BAND_DEG,
    max_step: int = AUTO_TRIM_MAX_STEP_PER_UPDATE,
) -> tuple[int, int]:
    """Return the rate-limited (aileron, elevator) level DELTA for one cycle.

    The closed-loop counterpart of :func:`recommended_trim_levels`: it nudges by
    at most ``max_step`` levels per update in the direction that reduces the
    residual attitude, and returns ``(0, 0)`` once inside the dead-band so the
    trim latches instead of hunting. The sign is ``-gain * error`` for the same
    reason documented at the top of the module.
    """
    try:
        roll = float(avg_roll_deg)
        pitch = float(avg_pitch_deg)
    except (TypeError, ValueError):
        return 0, 0

    if abs(roll) <= dead_band_deg:
        ail_step = 0
    else:
        ail_step = _clamp_int(round(-gain * roll), -max_step, max_step)
        # Never let rounding stall a real correction at zero movement.
        if ail_step == 0:
            ail_step = 1 if roll < 0 else -1

    if abs(pitch) <= dead_band_deg:
        elev_step = 0
    else:
        elev_step = _clamp_int(round(-gain * pitch), -max_step, max_step)
        if elev_step == 0:
            elev_step = 1 if pitch < 0 else -1

    return ail_step, elev_step


class AttitudeAverager:
    """Rolling time-windowed mean of the roll/pitch attitude samples.

    Feeds the auto-trim controller a smoothed attitude so a single gust does not
    move the trim. Samples are timestamped (monotonic seconds) and expired once
    older than ``window_s``; the average is only offered once the window is both
    time-spanned and has at least ``min_samples`` points.
    """

    def __init__(
        self,
        window_s: float = AUTO_TRIM_SAMPLE_WINDOW_S,
        min_samples: int = AUTO_TRIM_MIN_SAMPLES,
    ) -> None:
        self.window_s = float(window_s)
        self.min_samples = int(min_samples)
        self._samples: deque[tuple[float, float, float]] = deque()

    def clear(self) -> None:
        """Drop all buffered samples (call when gating breaks)."""
        self._samples.clear()

    def add(self, timestamp: float, roll_deg: float, pitch_deg: float) -> None:
        """Append one attitude sample and expire anything past the window."""
        try:
            t = float(timestamp)
            roll = float(roll_deg)
            pitch = float(pitch_deg)
        except (TypeError, ValueError):
            return
        self._samples.append((t, roll, pitch))
        # Keep one sample straddling the cutoff so the retained buffer provably
        # spans the full window. Dropping every sample older than the cutoff
        # would leave the oldest retained sample just inside it, so with real
        # (jittered) 200 ms ticks the span sits perpetually a tick under
        # window_s and ready() would never fire. Only discard samples[0] once
        # samples[1] is itself old enough to start the window.
        cutoff = t - self.window_s
        while len(self._samples) > 1 and self._samples[1][0] < cutoff:
            self._samples.popleft()

    @property
    def count(self) -> int:
        return len(self._samples)

    def ready(self) -> bool:
        """True once the buffer spans the window and has enough samples."""
        if len(self._samples) < self.min_samples:
            return False
        span = self._samples[-1][0] - self._samples[0][0]
        return span >= self.window_s

    def averages(self) -> Optional[tuple[float, float]]:
        """Return ``(mean_roll, mean_pitch)`` once :meth:`ready`, else ``None``."""
        if not self.ready():
            return None
        n = len(self._samples)
        roll_sum = sum(s[1] for s in self._samples)
        pitch_sum = sum(s[2] for s in self._samples)
        return roll_sum / n, pitch_sum / n
