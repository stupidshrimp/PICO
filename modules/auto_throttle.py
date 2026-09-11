"""Ground-station side of the FC airspeed-hold loop ("Auto Throttle").

CH3 carries two different quantities, selected by CH7/AUX3 (see
``modules/compass_cal.py`` and ``docs/protocol_contract.md``):

* Manual Throttle -> throttle percent, ``0..100 %`` over ``172..1811``;
* Auto Throttle   -> the target airspeed the FC's throttle PID holds,
  ``0..100 mph`` over the same ``172..1811``.

The two meanings share the same channel values, so a ground station and a
flight controller that momentarily disagree about the mode do not fail
loudly -- the FC just applies the wrong converter and commands a plausible
but wrong throttle. A 20 mph setpoint read as a manual throttle percent is a
rock-steady 20 % throttle that ignores airspeed entirely, which is exactly
what "auto throttle does nothing, the motor just sits at one speed" looks
like from the cockpit.

That disagreement is unavoidable for a short window on every toggle, because
CH3 and CH7 do not arrive together. ELRS packs CH1-CH4 into every RF packet
but sends the AUX channels ROUND-ROBIN, roughly one AUX per packet (see
``GenerateChannelDataHybrid8`` / ``GenerateChannelDataHybridWide`` in
``flight_controller/references/ExpressLRS-master/src/lib/OTA/OTA.cpp``): with
seven AUX slots that is about seven packet intervals, ~140 ms at a 50 Hz
packet rate and proportionally longer below it. So the FC reliably sees the
new CH3 value several packets before the new CH7 that says how to read it.

``throttle_channel_value`` closes that window by holding CH3 at minimum for
``THROTTLE_MODE_CHANGE_GUARD_S`` after a mode change. The FC then reads
minimum under either converter -- throttle cut, or a 0 mph target it decays
toward idle -- so a mode change can never command a phantom throttle setting,
and an engage that the FC never accepts shows up as a motor that will not
spool instead of one holding a mystery cruise setting.

Kept free of Qt imports so the channel semantics are unit-testable headless.
"""

from __future__ import annotations

import math

# CRSF channel units. Mirrors pico_modules.pico_transmitpackets, which cannot be
# imported here without dragging in PySide6.
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_MAX = 1811
CRSF_CHANNEL_SPAN = CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN

# Keep in lockstep with AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH in
# flight_controller/Main.ino: CH3 auto-throttle setpoints are scaled by this
# fixed range on both the GS and the FC.
AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH = 100.0

DEFAULT_TARGET_AIRSPEED_MPH = 20.0

# How long CH3 is held at minimum across a throttle-mode change. Covers the
# worst-case AUX round-robin above (~140 ms at 50 Hz) with enough margin for a
# slower packet rate, while staying short enough that engaging auto throttle in
# flight is a brief power interruption rather than a glide.
THROTTLE_MODE_CHANGE_GUARD_S = 0.4


def clamp_target_airspeed(
    speed_mph, speed_channel_max_mph: float = AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH
) -> float:
    """Clamp an auto-throttle setpoint to the CH3 converter range.

    Falls back to the default target for a value that is not a finite number so
    a malformed config cannot put a NaN on the channel.
    """
    try:
        speed = float(speed_mph)
    except (TypeError, ValueError):
        speed = DEFAULT_TARGET_AIRSPEED_MPH
    if not math.isfinite(speed):
        speed = DEFAULT_TARGET_AIRSPEED_MPH
    return max(0.0, min(float(speed_channel_max_mph), speed))


def mode_change_guard_active(seconds_since_mode_change) -> bool:
    """Whether CH3 is still inside the post-mode-change guard window.

    Only a finite, non-negative age can open the guard. The age is always a
    difference of two ``time.monotonic()`` reads, so nothing else should reach
    here -- but an unreadable one must fail CLOSED (guard off), because a value
    that can never expire would mask CH3 to minimum forever and leave the
    aircraft with no throttle at all.
    """
    if seconds_since_mode_change is None:
        return False
    try:
        elapsed = float(seconds_since_mode_change)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(elapsed) or elapsed < 0.0:
        return False
    return elapsed < THROTTLE_MODE_CHANGE_GUARD_S


def throttle_channel_value(
    *,
    throttle_mode: str,
    throttle_percent: float,
    target_airspeed_mph: float,
    speed_channel_max_mph: float = AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH,
    compass_cal_active: bool = False,
    seconds_since_mode_change=None,
) -> int:
    """Return the CH3 value to transmit.

    ``seconds_since_mode_change`` is the monotonic age of the last throttle-mode
    change (``None`` when no change has happened). Minimum is sent while a
    compass calibration is requested, and while the mode-change guard is open.
    """
    if compass_cal_active or mode_change_guard_active(seconds_since_mode_change):
        return CRSF_CHANNEL_MIN

    if throttle_mode == "Auto Throttle":
        speed_max = float(speed_channel_max_mph)
        if speed_max <= 0.0:
            return CRSF_CHANNEL_MIN
        fraction = clamp_target_airspeed(target_airspeed_mph, speed_max) / speed_max
    else:
        try:
            percent = float(throttle_percent)
        except (TypeError, ValueError):
            percent = 0.0
        if not math.isfinite(percent):
            percent = 0.0
        fraction = max(0.0, min(100.0, percent)) / 100.0

    return int(fraction * CRSF_CHANNEL_SPAN + CRSF_CHANNEL_MIN)
