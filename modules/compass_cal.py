"""Ground-station side of the in-field magnetometer (compass) calibration.

CH7/AUX3 carries three bands instead of two (see flight_controller/Main.ino,
"In-field magnetometer calibration", and docs/protocol_contract.md):

* low  (400)  -> manual throttle (unchanged),
* high (1700) -> FC auto throttle (unchanged),
* center (992, ``COMPASS_CAL_CHANNEL_VALUE``) -> compass-calibration request.

The FC only honors the request on the ground (not airborne-latched, Manual
control mode, throttle stick at minimum) after the band is held for a full
second, holds the control surfaces in a distinctive pose while the operator
rotates the aircraft, and ends the run when the channel leaves the band: a
valid fit is applied live, saved to the FC's flash, and acknowledged with a
slow full-travel sweep; an invalid fit keeps the previous calibration and is
signalled with a rapid surface flutter.

Kept free of Qt imports so the channel semantics are unit-testable headless
(the package initializer is deliberately empty for the same reason).
"""

from __future__ import annotations

# CRSF channel units (min 172 / center 992 / max 1811). Mirrors
# pico_modules.pico_transmitpackets.CRSF_CHANNEL_CENTER, which cannot be
# imported here without dragging in PySide6.
COMPASS_CAL_CHANNEL_VALUE = 992

# The two-band values _build_control_channels has always sent on CH7.
THROTTLE_MODE_AUTO_VALUE = 1700
THROTTLE_MODE_MANUAL_VALUE = 400


def throttle_mode_channel_value(throttle_mode: str, compass_cal_active: bool) -> int:
    """Return the CH7/AUX3 value for the current throttle mode.

    While a compass calibration is active the channel holds the center band
    instead of either throttle-mode value; the band reads as "manual
    throttle" to the FC's mode logic, so a calibration request can never
    engage the throttle PID.
    """
    if compass_cal_active:
        return COMPASS_CAL_CHANNEL_VALUE
    if throttle_mode == "Auto Throttle":
        return THROTTLE_MODE_AUTO_VALUE
    return THROTTLE_MODE_MANUAL_VALUE


def compass_cal_start_blockers(
    *,
    transmission_active: bool,
    airborne_state: str,
    control_mode: str,
    throttle_mode: str,
    throttle_percent: float,
) -> list[str]:
    """Return the reasons a compass calibration cannot start right now.

    Empty list means starting is allowed. These mirror the FC's own entry
    gates so the button fails fast with an explanation instead of silently
    sending a request the FC will ignore. The FC remains authoritative: it
    re-checks every gate itself.
    """
    reasons: list[str] = []
    if not transmission_active:
        reasons.append("packet transmission is stopped")
    if airborne_state != "grounded":
        reasons.append("the aircraft is not on the ground")
    if control_mode != "Manual":
        reasons.append("the control mode is not Manual")
    if throttle_mode != "Manual":
        reasons.append("the throttle mode is not Manual")
    if throttle_percent > 0:
        reasons.append("the throttle is not at zero")
    return reasons
