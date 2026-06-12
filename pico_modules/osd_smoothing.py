"""Shared time-scaled smoothing helper for the OSD widgets.

The OSD widgets smooth incoming telemetry with an exponential moving average
(``value = value * (1 - w) + sample * w``).  Historically the blend weight
``w`` was applied once per GUI refresh tick, so the *effective* smoothing
depended on the refresh rate: doubling the refresh rate made the same ``w``
converge roughly twice as fast in wall-clock terms, weakening the smoothing.
It also coupled the smoothing to the (user-selectable) telemetry arrival rate
whenever packets arrived slower than the GUI repainted.

To decouple the perceived smoothing from both rates, the weight is
reparametrised against the elapsed time since the previous update.  The blend
was originally tuned against the ~30 Hz refresh timer (a 33 ms period), so that
period is used as the reference interval.  For an arbitrary ``dt`` the
equivalent weight is::

    alpha = 1 - (1 - w) ** (dt / REFERENCE_INTERVAL_S)

which is identical to applying the fixed-weight blend ``dt / REFERENCE_INTERVAL_S``
times.  At ``dt == REFERENCE_INTERVAL_S`` this reduces to ``alpha == w`` so the
behaviour exactly matches the legacy 30 Hz tuning; at higher refresh rates each
individual blend is correspondingly gentler, keeping the smoothing time
constant fixed.
"""

# Reference refresh period (seconds) the OSD smoothing weights were tuned at.
REFERENCE_INTERVAL_S = 0.033


def time_scaled_weight(
    weight: float, dt: float, reference: float = REFERENCE_INTERVAL_S
) -> float:
    """Return ``weight`` rescaled for an ``dt``-second gap since the last blend.

    Parameters
    ----------
    weight:
        The per-call EMA weight tuned at the ``reference`` interval, in
        ``[0, 1]``.
    dt:
        Seconds elapsed since the previous smoothing update.
    reference:
        Interval (seconds) the ``weight`` was tuned against.  Defaults to the
        legacy ~30 Hz refresh period.

    Notes
    -----
    A non-positive ``dt`` yields ``0.0`` (no movement); a non-positive
    ``reference`` falls back to applying ``weight`` unchanged.  Weights at or
    beyond the ``[0, 1]`` bounds short-circuit to those bounds.
    """

    if dt <= 0.0:
        return 0.0
    if weight <= 0.0:
        return 0.0
    if weight >= 1.0:
        return 1.0
    if reference <= 0.0:
        return weight
    return 1.0 - (1.0 - weight) ** (dt / reference)
