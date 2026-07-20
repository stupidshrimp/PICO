"""Ground-station side of the live board-alignment (level) trim.

Interim keyboard tuning aid for the flight controller's board-alignment offset
(see flight_controller/board_align.h and the FC_BOARD_ALIGN_TRIM_* block in
flight_controller/Main.ino). The keyboard arrow keys nudge a small roll/pitch
offset DELTA that is carried to the FC on two spare aux channels:

* CH8/AUX4 (index 7)  -> roll offset delta,
* CH9/AUX5 (index 8)  -> pitch offset delta.

Channel center means zero delta, so the FC keeps its compiled
FC_BOARD_ALIGN_*_DEG baseline until the operator trims. The FC adds the decoded
delta to that baseline and rebuilds its alignment live -- but only on the ground
with fresh RC -- and does NOT persist it. This is a tool to FIND the offset
while watching the attitude telemetry read level; once happy, the value is baked
into FC_BOARD_ALIGN_*_DEG and reflashed.

Kept free of Qt imports so the channel semantics are unit-testable headless,
mirroring modules/compass_cal.py.
"""

from __future__ import annotations

# CRSF channel units (min 172 / center 992 / max 1811). Mirrors
# pico_modules.pico_transmitpackets.CRSF_CHANNEL_*, which cannot be imported
# here without dragging in PySide6.
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX = 1811

# Symmetric half-span so +-full deflection maps cleanly around center. The FC
# decodes with the same center and half-span (see boardAlignTrimChannelToDeg in
# flight_controller/Main.ino).
_CRSF_HALF_SPAN = min(
    CRSF_CHANNEL_CENTER - CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX - CRSF_CHANNEL_CENTER
)

# Spare aux channels carrying the roll/pitch offset delta (zero-based indices).
# CH5/AUX1 (arm), CH6/AUX2 (mode), CH7/AUX3 (throttle mode + compass cal) are
# taken; these two are free.
BOARD_ALIGN_ROLL_CHANNEL_INDEX = 7   # CH8/AUX4
BOARD_ALIGN_PITCH_CHANNEL_INDEX = 8  # CH9/AUX5

# +-full deflection corresponds to this many degrees of trim delta. Must match
# FC_BOARD_ALIGN_TRIM_RANGE_DEG in flight_controller/Main.ino.
BOARD_ALIGN_TRIM_RANGE_DEG = 10.0

# Degrees adjusted per arrow-key press.
BOARD_ALIGN_TRIM_STEP_DEG = 0.1

# Compiled firmware baseline (FC_BOARD_ALIGN_*_DEG in flight_controller/Main.ino).
# The keyboard trim is a DELTA the FC ADDS to this baseline, so the effective
# offset -- the value to bake back into the firmware once the aircraft reads
# level -- is baseline + delta, not the delta alone. Keep these in lockstep with
# the firmware defaults (same "keep in lockstep with Main.ino" pattern as the
# FBW / auto-throttle limits mirrored in main.py).
FC_BOARD_ALIGN_BASELINE_ROLL_DEG = -6.9
FC_BOARD_ALIGN_BASELINE_PITCH_DEG = -4.3


def effective_board_align_offset(baseline_deg: float, delta_deg: float) -> float:
    """Return the effective offset to bake into the firmware: baseline + delta."""
    try:
        return float(baseline_deg) + float(delta_deg)
    except (TypeError, ValueError):
        return 0.0


def clamp_board_align_offset(offset_deg: float) -> float:
    """Clamp a roll/pitch trim delta to the transmissible +-range."""
    try:
        value = float(offset_deg)
    except (TypeError, ValueError):
        return 0.0
    return max(-BOARD_ALIGN_TRIM_RANGE_DEG, min(BOARD_ALIGN_TRIM_RANGE_DEG, value))


def step_board_align_offset(offset_deg: float, steps: int) -> float:
    """Return the trim delta after ``steps`` arrow presses (sign = direction)."""
    try:
        count = int(steps)
    except (TypeError, ValueError):
        count = 0
    return clamp_board_align_offset(offset_deg + count * BOARD_ALIGN_TRIM_STEP_DEG)


def board_align_offset_to_channel(offset_deg: float) -> int:
    """Map a roll/pitch trim delta (degrees) to a CRSF channel value.

    Channel center is zero delta; +-BOARD_ALIGN_TRIM_RANGE_DEG maps to the
    symmetric channel extremes.
    """
    frac = clamp_board_align_offset(offset_deg) / BOARD_ALIGN_TRIM_RANGE_DEG
    return int(round(CRSF_CHANNEL_CENTER + frac * _CRSF_HALF_SPAN))


def board_align_channel_to_offset(value: int) -> float:
    """Inverse of :func:`board_align_offset_to_channel` (the FC's decode).

    Exposed so the round trip can be verified headlessly and the same formula
    documented on both sides of the link.
    """
    try:
        channel = int(value)
    except (TypeError, ValueError):
        return 0.0
    channel = max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, channel))
    return (channel - CRSF_CHANNEL_CENTER) / _CRSF_HALF_SPAN * BOARD_ALIGN_TRIM_RANGE_DEG
