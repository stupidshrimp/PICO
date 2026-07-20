"""Post-flight analysis of sortie (blackbox) CSV recordings.

This module turns a recorded sortie log into a set of automated flight-test
findings.  It is deliberately pure standard-library Python so it can run on a
background thread without touching Qt, and so the analysis can be exercised
headlessly (e.g. in tests or from a script) with nothing but a CSV file.

The tests implemented here are the diagnostic half of the ground station's
monitoring story: the live ``check_warnings`` path in ``main.py`` covers
alarms that demand pilot action *now*, while these run once the blackbox
recording stops and look at the whole flight at once:

* Telemetry continuity  - per-stream packet gaps and effective rates.
* Frozen sensors        - attitude/GPS values that stop changing while the
                          link keeps delivering packets.
* Control response      - cross-correlation of recorded stick commands
                          against the attitude the aircraft actually flew.
* Fly-By-Wire tracking  - RMS error, offset, ringing, envelope compliance,
                          and saturation of the flown attitude against the
                          recorded FBW setpoints.
* Link budget           - RSSI modelled against distance from the first GPS
                          fix, antenna A/B divergence, and link-quality lows.
* Estimator continuity  - non-physical attitude jumps that betray EKF
                          innovation-gate re-acquisitions (or telemetry
                          corruption).
* EKF attitude vs GPS   - a tuning advisor: cross-checks the attitude estimate
                          against GPS (which the maiden-bring-up build does not
                          fuse for roll/pitch/yaw) and turns the disagreement
                          into "which firmware constant, which direction"
                          guidance for roll/pitch bias and heading trust.

Each test degrades gracefully: if the flight did not exercise the data a test
needs (no stick activity, no GPS fix, no steady flight) the test reports
``no_data`` instead of guessing.

The EKF-tuning advisor is deliberately telemetry-only: it works from the
estimator *output* plus GPS, so it can flag a symptom and the constant to
reach for, but it cannot solve for a provably-optimal value (that needs the
EKF's internal innovations/covariance, which are not downlinked). It never
escalates past ``warn`` - a tunable bias is a hint, not a failed flight.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

# Ordered worst-first for combining statuses. ``no_data`` never worsens the
# overall verdict: an unexercised test is not a failed flight.
_STATUS_SEVERITY = {"fail": 3, "warn": 2, "pass": 1, "no_data": 0}

# Sample grid used for the control-response cross-correlation. 20 Hz keeps a
# whole flight cheap to correlate in pure Python while resolving lags well
# below human/airframe response times.
_CORRELATION_DT_S = 0.05
_CORRELATION_MAX_LAG_S = 1.5


@dataclass
class Finding:
    """One test's verdict: a status, a one-line summary, and detail lines."""

    name: str
    status: str  # "pass" | "warn" | "fail" | "no_data"
    summary: str
    details: list[str] = field(default_factory=list)


@dataclass
class SortieReport:
    """All findings for one sortie file."""

    path: str
    findings: list[Finding] = field(default_factory=list)
    error: Optional[str] = None

    def overall_status(self) -> str:
        if self.error:
            return "fail"
        worst = "no_data"
        for finding in self.findings:
            if _STATUS_SEVERITY.get(finding.status, 0) > _STATUS_SEVERITY[worst]:
                worst = finding.status
        return worst


@dataclass
class _Stream:
    """Rows of one telemetry packet type, as parallel per-field columns."""

    times: list[float] = field(default_factory=list)
    fields: dict[str, list[float]] = field(default_factory=dict)

    def column(self, name: str) -> list[float]:
        return self.fields.get(name, [])


# Fields captured per packet type. Rows snapshot the whole telemetry state,
# so a stream only samples the columns that its own packet actually updates -
# reading e.g. attitude off a link_stats row would just repeat stale values.
_STREAM_FIELDS = {
    "attitude": (
        "pitch",
        "roll",
        "yaw",
        "stick_pitch",
        "stick_roll",
        "fbw_setpoint_roll",
        "fbw_setpoint_pitch",
        "fbw_limit_roll",
        "fbw_limit_pitch",
    ),
    "gps": (
        "latitude",
        "longitude",
        "altitude_ft",
        "airspeed_mph",
        "ground_course",
        "satellites",
    ),
    "link_stats": (
        "rssi_a",
        "rssi_b",
        "link_quality",
        "snr",
        "downlink_quality",
        "downlink_snr",
    ),
}


def _parse_float(value: Optional[str]) -> float:
    if value is None:
        return math.nan
    value = value.strip()
    if not value or value.lower() == "none":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def _parse_timestamp(raw: str) -> Optional[datetime]:
    try:
        return datetime.strptime(raw, "%Y-%m-%d %H:%M:%S.%f")
    except ValueError:
        try:
            return datetime.fromisoformat(raw)
        except ValueError:
            return None


def load_sortie_streams(path: str) -> dict[str, _Stream]:
    """Parse a sortie CSV into per-packet-type column streams."""

    streams: dict[str, _Stream] = {name: _Stream() for name in _STREAM_FIELDS}
    start: Optional[datetime] = None

    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            raw_timestamp = row.get("timestamp")
            if not raw_timestamp:
                continue
            timestamp = _parse_timestamp(raw_timestamp)
            if timestamp is None:
                continue
            if start is None:
                start = timestamp
            packet_type = (row.get("packet_type") or "").strip().lower()
            stream = streams.get(packet_type)
            if stream is None:
                continue
            stream.times.append((timestamp - start).total_seconds())
            for name in _STREAM_FIELDS[packet_type]:
                stream.fields.setdefault(name, []).append(
                    _parse_float(row.get(name))
                )
            if packet_type == "attitude":
                # control_mode is a string column; encode it numerically so
                # it rides in the same float streams (1 = Fly-By-Wire,
                # 0 = Manual, NaN = not recorded / pre-FBW-logging file).
                mode_raw = (row.get("control_mode") or "").strip().lower()
                if not mode_raw:
                    mode_value = math.nan
                elif mode_raw.startswith("fly"):
                    mode_value = 1.0
                else:
                    mode_value = 0.0
                stream.fields.setdefault("fbw_active", []).append(mode_value)

    return streams


# ----------------------------------------------------------------------
# Small numeric helpers (kept dependency-free on purpose)
# ----------------------------------------------------------------------
def _finite(values: list[float]) -> list[float]:
    return [v for v in values if math.isfinite(v)]


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else math.nan


def _std(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    mean = _mean(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def _median(values: list[float]) -> float:
    ordered = sorted(v for v in values if math.isfinite(v))
    n = len(ordered)
    if n == 0:
        return math.nan
    mid = n // 2
    if n % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    """Least-squares ``y = a + b*x``; returns (a, b, residual std)."""

    n = len(xs)
    mean_x = _mean(xs)
    mean_y = _mean(ys)
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx <= 0.0:
        return mean_y, 0.0, _std(ys)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    residuals = [y - (intercept + slope * x) for x, y in zip(xs, ys)]
    return intercept, slope, math.sqrt(sum(r * r for r in residuals) / n)


def _resample(
    times: list[float], values: list[float], dt: float
) -> list[float]:
    """Linearly resample an irregular series onto a uniform ``dt`` grid.

    Non-finite samples are skipped, so short telemetry dropouts are bridged
    by interpolation instead of poisoning the correlation with NaNs.
    """

    pairs = [
        (t, v) for t, v in zip(times, values) if math.isfinite(v)
    ]
    if len(pairs) < 2:
        return []
    grid: list[float] = []
    index = 0
    t = pairs[0][0]
    end = pairs[-1][0]
    while t <= end:
        while index + 1 < len(pairs) and pairs[index + 1][0] < t:
            index += 1
        t0, v0 = pairs[index]
        t1, v1 = pairs[min(index + 1, len(pairs) - 1)]
        if t1 <= t0:
            grid.append(v0)
        else:
            frac = (t - t0) / (t1 - t0)
            frac = min(max(frac, 0.0), 1.0)
            grid.append(v0 + (v1 - v0) * frac)
        t += dt
    return grid


def _peak_lagged_correlation(
    command: list[float], response: list[float], max_lag_samples: int
) -> tuple[float, int]:
    """Best Pearson correlation of ``response`` delayed 0..max_lag samples."""

    best_corr = 0.0
    best_lag = 0
    for lag in range(0, max_lag_samples + 1):
        cmd = command[: len(command) - lag] if lag else command
        resp = response[lag:]
        n = min(len(cmd), len(resp))
        if n < 20:
            break
        cmd = cmd[:n]
        resp = resp[:n]
        std_cmd = _std(cmd)
        std_resp = _std(resp)
        if std_cmd <= 0.0 or std_resp <= 0.0:
            continue
        mean_cmd = _mean(cmd)
        mean_resp = _mean(resp)
        cov = sum(
            (c - mean_cmd) * (r - mean_resp) for c, r in zip(cmd, resp)
        ) / n
        corr = cov / (std_cmd * std_resp)
        if corr > best_corr:
            best_corr = corr
            best_lag = lag
    return best_corr, best_lag


def _derivative(values: list[float], dt: float) -> list[float]:
    if len(values) < 2:
        return []
    return [
        (values[i + 1] - values[i]) / dt for i in range(len(values) - 1)
    ]


def _longest_constant_run(
    times: list[float], columns: list[list[float]]
) -> tuple[float, float]:
    """Longest span where every column repeats the exact same value.

    Returns ``(duration_s, start_time_s)`` of the longest such run.
    """

    best_duration = 0.0
    best_start = 0.0
    run_start_index = 0
    for i in range(1, len(times)):
        unchanged = all(
            (
                column[i] == column[i - 1]
                or (math.isnan(column[i]) and math.isnan(column[i - 1]))
            )
            for column in columns
        )
        if not unchanged:
            run_start_index = i
            continue
        duration = times[i] - times[run_start_index]
        if duration > best_duration:
            best_duration = duration
            best_start = times[run_start_index]
    return best_duration, best_start


# ----------------------------------------------------------------------
# Individual tests
# ----------------------------------------------------------------------
def _test_continuity(streams: dict[str, _Stream]) -> Finding:
    """Scan every stream for reception gaps and report effective rates.

    Gaps are measured against the whole sortie's time span, not just each
    stream's own first/last packet — otherwise a stream that dies mid-flight
    (or starts late) would report a healthy rate and no dropout while it was
    actually silent for most of the recording.
    """

    details: list[str] = []
    worst_status = "pass"
    worst_gap = 0.0
    worst_stream = ""

    populated = [stream.times for stream in streams.values() if stream.times]
    sortie_start = min(times[0] for times in populated)
    sortie_end = max(times[-1] for times in populated)
    sortie_duration = sortie_end - sortie_start

    for name in ("attitude", "gps", "link_stats"):
        stream = streams[name]
        times = stream.times
        if len(times) < 2:
            if worst_status != "fail":
                worst_status = "warn"
            details.append(
                f"{name}: fewer than two packets recorded — stream "
                "missing or recording was too short."
            )
            continue
        rate = (
            (len(times) - 1) / sortie_duration if sortie_duration > 0 else 0.0
        )
        leading_gap = times[0] - sortie_start
        trailing_gap = sortie_end - times[-1]
        inter_gap = max(
            times[i] - times[i - 1] for i in range(1, len(times))
        )
        max_gap = max(inter_gap, leading_gap, trailing_gap)
        long_gaps = sum(
            1
            for i in range(1, len(times))
            if times[i] - times[i - 1] > 1.0
        )
        long_gaps += sum(1 for gap in (leading_gap, trailing_gap) if gap > 1.0)
        details.append(
            f"{name}: {len(times)} packets at {rate:.1f} Hz average, "
            f"longest gap {max_gap:.2f} s, {long_gaps} gap(s) over 1 s."
        )
        if trailing_gap > 1.0 and trailing_gap == max_gap:
            details.append(
                f"{name}: went silent for the last {trailing_gap:.0f} s of "
                "the sortie."
            )
        elif leading_gap > 1.0 and leading_gap == max_gap:
            details.append(
                f"{name}: first packet only arrived {leading_gap:.0f} s into "
                "the sortie."
            )
        if max_gap > worst_gap:
            worst_gap = max_gap
            worst_stream = name
        if max_gap > 5.0:
            worst_status = "fail"
        elif max_gap > 1.0 and worst_status != "fail":
            worst_status = "warn"

    if worst_status == "pass":
        summary = (
            f"No telemetry dropouts; longest gap {worst_gap:.2f} s"
            f" ({worst_stream})."
            if worst_stream
            else "No telemetry dropouts."
        )
    else:
        summary = (
            f"Telemetry dropped out: longest gap {worst_gap:.1f} s on the "
            f"{worst_stream} stream."
        )
    return Finding("Telemetry continuity", worst_status, summary, details)


def _test_frozen_sensors(streams: dict[str, _Stream]) -> Finding:
    """Detect values that stopped changing while packets kept arriving."""

    details: list[str] = []
    status = "pass"

    attitude = streams["attitude"]
    if len(attitude.times) < 10:
        return Finding(
            "Frozen sensors",
            "no_data",
            "Not enough attitude packets to check for frozen sensors.",
        )

    duration, start = _longest_constant_run(
        attitude.times,
        [attitude.column("pitch"), attitude.column("roll"), attitude.column("yaw")],
    )
    if duration > 2.0:
        status = "fail"
        details.append(
            f"Attitude froze for {duration:.1f} s starting at t={start:.0f} s "
            "while packets kept arriving — IMU or flight-controller loop "
            "likely hung."
        )
    else:
        details.append(
            f"Attitude never froze (longest identical stretch {duration:.2f} s)."
        )

    gps = streams["gps"]
    if len(gps.times) >= 10:
        duration, start = _longest_constant_run(
            gps.times,
            [gps.column("altitude_ft"), gps.column("airspeed_mph")],
        )
        if duration > 30.0:
            if status != "fail":
                status = "warn"
            details.append(
                f"Altitude and airspeed were both frozen for {duration:.0f} s "
                f"starting at t={start:.0f} s — plausible if parked on the "
                "ground, suspicious in flight."
            )
        else:
            details.append(
                "Altitude/airspeed never froze "
                f"(longest identical stretch {duration:.1f} s)."
            )

    summary = (
        "All sensor streams kept changing while data flowed."
        if status == "pass"
        else "A sensor stream stopped changing while the link stayed up."
    )
    return Finding("Frozen sensors", status, summary, details)


def _test_control_response(streams: dict[str, _Stream]) -> Finding:
    """Correlate recorded stick commands against the flown attitude."""

    attitude = streams["attitude"]
    if len(attitude.times) < 100:
        return Finding(
            "Control response",
            "no_data",
            "Not enough attitude telemetry to evaluate control response.",
        )

    dt = _CORRELATION_DT_S
    max_lag = int(_CORRELATION_MAX_LAG_S / dt)
    details: list[str] = []
    worst_corr: Optional[float] = None
    evaluated_any = False

    for axis, stick_field, attitude_field in (
        ("roll", "stick_roll", "roll"),
        ("pitch", "stick_pitch", "pitch"),
    ):
        stick = _resample(attitude.times, attitude.column(stick_field), dt)
        actual = _resample(attitude.times, attitude.column(attitude_field), dt)
        n = min(len(stick), len(actual))
        stick, actual = stick[:n], actual[:n]
        if n < max_lag + 40:
            details.append(f"{axis}: series too short to correlate.")
            continue
        if _std(stick) < 3.0:
            details.append(
                f"{axis}: stick was essentially idle (moved less than ±3°), "
                "so response cannot be judged."
            )
            continue

        evaluated_any = True
        # Manual mode maps stick roughly to attitude *rate*; Fly-By-Wire maps
        # it to the attitude itself. Score against both and keep the better,
        # so the test works in either control mode.
        corr_angle, lag_angle = _peak_lagged_correlation(stick, actual, max_lag)
        rate = _derivative(actual, dt)
        corr_rate, lag_rate = _peak_lagged_correlation(stick, rate, max_lag)
        if corr_angle >= corr_rate:
            corr, lag, mode = corr_angle, lag_angle, "attitude"
        else:
            corr, lag, mode = corr_rate, lag_rate, "attitude rate"
        details.append(
            f"{axis}: peak correlation {corr:.2f} against {mode} at "
            f"{lag * dt:.2f} s lag."
        )
        if worst_corr is None or corr < worst_corr:
            worst_corr = corr

    if not evaluated_any:
        return Finding(
            "Control response",
            "no_data",
            "Sticks were not moved enough this flight to grade control "
            "response.",
            details,
        )

    if worst_corr >= 0.5:
        status = "pass"
        summary = (
            "Aircraft attitude tracked stick commands "
            f"(weakest axis correlation {worst_corr:.2f})."
        )
    elif worst_corr >= 0.3:
        status = "warn"
        summary = (
            "Attitude only loosely followed stick commands "
            f"(weakest axis correlation {worst_corr:.2f}) — check control "
            "linkages, trim, and link latency."
        )
    else:
        status = "fail"
        summary = (
            "Attitude did not follow stick commands "
            f"(weakest axis correlation {worst_corr:.2f}) — possible control "
            "surface, servo, or uplink failure."
        )
    return Finding("Control response", status, summary, details)


# Keep in lockstep with FBW_FC_MAX_*_ANGLE_DEG in main.py / Main.ino: attitude
# beyond this while in Fly-By-Wire means the FC-side limiter is not working.
_FBW_FC_ABSOLUTE_LIMIT_DEG = 80.0
# Ignore this long after each FBW engagement so the engage transient does not
# count against steady tracking quality (it is graded separately).
_FBW_ENGAGE_SETTLE_S = 1.0


def _test_fbw_tracking(streams: dict[str, _Stream]) -> Finding:
    """Grade how well the aircraft flew its Fly-By-Wire attitude setpoints."""

    attitude = streams["attitude"]
    times = attitude.times
    mode = attitude.column("fbw_active")
    if len(mode) != len(times) or not any(
        v == 1.0 for v in mode if math.isfinite(v)
    ):
        return Finding(
            "Fly-By-Wire tracking",
            "no_data",
            "No Fly-By-Wire flight was recorded (never engaged, or the log "
            "predates FBW setpoint logging).",
        )

    # Contiguous runs of FBW samples, as (start, end) index pairs.
    segments: list[tuple[int, int]] = []
    start_index: Optional[int] = None
    for i, value in enumerate(mode):
        if value == 1.0:
            if start_index is None:
                start_index = i
        elif start_index is not None:
            segments.append((start_index, i - 1))
            start_index = None
    if start_index is not None:
        segments.append((start_index, len(mode) - 1))

    fbw_time = sum(times[end] - times[start] for start, end in segments)
    details = [
        f"{len(segments)} Fly-By-Wire segment(s) totalling {fbw_time:.0f} s."
    ]
    if fbw_time < 10.0:
        return Finding(
            "Fly-By-Wire tracking",
            "no_data",
            "Fly-By-Wire was engaged too briefly to grade tracking.",
            details,
        )

    dt = _CORRELATION_DT_S
    max_lag = int(1.0 / dt)
    status = "pass"
    worst_rms: Optional[float] = None

    for axis, setpoint_field, limit_field in (
        ("roll", "fbw_setpoint_roll", "fbw_limit_roll"),
        ("pitch", "fbw_setpoint_pitch", "fbw_limit_pitch"),
    ):
        setpoints = attitude.column(setpoint_field)
        actuals = attitude.column(axis)
        limits = attitude.column(limit_field)
        if len(limits) != len(times):
            limits = [math.nan] * len(times)
        if len(setpoints) != len(times):
            details.append(f"{axis}: setpoints were not recorded.")
            continue

        # Resample each segment (minus the engage settle window) onto a
        # uniform grid so tracking error can be measured at the aircraft's
        # actual response lag instead of penalising that lag as error.
        grids: list[tuple[list[float], list[float]]] = []
        max_abs_setpoint = 0.0
        max_abs_actual = 0.0
        max_abs_actual_all = 0.0
        saturated = 0
        limited_total = 0
        last_limit = math.nan
        engage_bump = 0.0
        for start, end in segments:
            settle_end = times[start] + _FBW_ENGAGE_SETTLE_S
            seg_times: list[float] = []
            seg_sp: list[float] = []
            seg_att: list[float] = []
            for i in range(start, end + 1):
                sp, att = setpoints[i], actuals[i]
                if not (math.isfinite(sp) and math.isfinite(att)):
                    continue
                # The FC hard ceiling is a safety envelope: every FBW sample
                # counts, including the engage transient skipped below.
                max_abs_actual_all = max(max_abs_actual_all, abs(att))
                if times[i] < settle_end:
                    engage_bump = max(engage_bump, abs(att - sp))
                    continue
                seg_times.append(times[i])
                seg_sp.append(sp)
                seg_att.append(att)
                max_abs_setpoint = max(max_abs_setpoint, abs(sp))
                max_abs_actual = max(max_abs_actual, abs(att))
                # Saturation is judged against the recorded configured FBW
                # limit, not the flight's own maximum command — holding a
                # steady moderate bank must not read as "pinned at the limit".
                limit = limits[i]
                if math.isfinite(limit) and limit >= 5.0:
                    limited_total += 1
                    last_limit = limit
                    if abs(sp) >= 0.95 * limit:
                        saturated += 1
            sp_grid = _resample(seg_times, seg_sp, dt)
            att_grid = _resample(seg_times, seg_att, dt)
            n = min(len(sp_grid), len(att_grid))
            if n >= 40:
                grids.append((sp_grid[:n], att_grid[:n]))

        if not grids:
            details.append(
                f"{axis}: FBW segments were too short or sparse to grade."
            )
            continue

        # Pick the response lag that minimises RMS error, then report the
        # error statistics at that lag.
        best: Optional[tuple[float, int, list[float]]] = None
        for lag in range(0, max_lag + 1):
            errors: list[float] = []
            for sp_grid, att_grid in grids:
                span = len(sp_grid) - lag
                if span < 40:
                    continue
                errors.extend(
                    att_grid[i + lag] - sp_grid[i] for i in range(span)
                )
            if len(errors) < 40:
                continue
            rms = math.sqrt(sum(e * e for e in errors) / len(errors))
            if best is None or rms < best[0]:
                best = (rms, lag, errors)
        if best is None:
            details.append(
                f"{axis}: FBW segments were too short or sparse to grade."
            )
            continue

        rms, lag, errors = best
        offset = _mean(errors)
        details.append(
            f"{axis}: RMS tracking error {rms:.1f}° at {lag * dt:.2f} s "
            f"response lag, steady offset {offset:+.1f}°."
        )
        if worst_rms is None or rms > worst_rms:
            worst_rms = rms

        if abs(offset) > 5.0:
            if status == "pass":
                status = "warn"
            details.append(
                f"{axis}: persistent {offset:+.1f}° offset from the "
                "commanded attitude — check trim, rigging, or CG."
            )

        # Ringing: how often the (demeaned) error swings through zero by
        # more than a degree. Frequent swings with real amplitude mean the
        # FC angle PID is oscillating around the setpoint.
        flips = 0
        previous = 0.0
        for error in errors:
            centred = error - offset
            if abs(centred) < 1.0:
                continue
            if previous and (centred > 0) != (previous > 0):
                flips += 1
            previous = centred
        duration = len(errors) * dt
        flip_rate = flips / duration if duration > 0 else 0.0
        if flip_rate > 1.5 and rms > 3.0:
            if status == "pass":
                status = "warn"
            details.append(
                f"{axis}: error oscillated {flip_rate:.1f} times/s around "
                "the setpoint — possible over-tuned FC angle PID."
            )

        if max_abs_actual_all > _FBW_FC_ABSOLUTE_LIMIT_DEG:
            status = "fail"
            details.append(
                f"{axis}: attitude reached {max_abs_actual_all:.0f}° in FBW "
                f"— beyond the FC's {_FBW_FC_ABSOLUTE_LIMIT_DEG:.0f}° hard "
                "ceiling; the FC-side limiter is not enforcing the envelope."
            )
        elif max_abs_actual > max_abs_setpoint + 10.0:
            if status != "fail":
                status = "warn"
            details.append(
                f"{axis}: attitude reached {max_abs_actual:.0f}° while only "
                f"{max_abs_setpoint:.0f}° was commanded — the aircraft flew "
                "outside its commanded envelope."
            )

        if limited_total > 0:
            saturation = saturated / limited_total
            if saturation > 0.3:
                if status == "pass":
                    status = "warn"
                details.append(
                    f"{axis}: setpoint sat at the configured "
                    f"{last_limit:.0f}° limit {saturation * 100.0:.0f}% of "
                    "the time — the FBW angle limits may be too tight for "
                    "how the aircraft is being flown."
                )

        if engage_bump > 20.0:
            if status == "pass":
                status = "warn"
            details.append(
                f"{axis}: up to {engage_bump:.0f}° of attitude error during "
                "FBW engagement — the mode switch is not bumpless."
            )

    if worst_rms is None:
        return Finding(
            "Fly-By-Wire tracking",
            "no_data",
            "Fly-By-Wire setpoints were not recorded for the engaged "
            "segments.",
            details,
        )

    if status == "fail":
        summary = (
            "Fly-By-Wire flew outside its commanded envelope "
            f"(worst-axis RMS error {worst_rms:.1f}°)."
        )
    elif worst_rms > 10.0:
        status = "fail"
        summary = (
            "Fly-By-Wire tracked its setpoints poorly "
            f"(worst-axis RMS error {worst_rms:.1f}°)."
        )
    elif status == "warn" or worst_rms > 5.0:
        status = "warn"
        summary = (
            "Fly-By-Wire tracking needs attention "
            f"(worst-axis RMS error {worst_rms:.1f}°)."
        )
    else:
        summary = (
            "Fly-By-Wire tracked its commanded attitude "
            f"(worst-axis RMS error {worst_rms:.1f}°)."
        )
    return Finding("Fly-By-Wire tracking", status, summary, details)


_FEET_TO_METERS = 0.3048
_METERS_PER_DEG_LAT = 111_132.0
_METERS_PER_DEG_LON_EQUATOR = 111_320.0


def _gps_distances(gps: _Stream) -> tuple[list[float], list[float]]:
    """3D distance (m) from the first valid fix, per GPS packet."""

    lats = gps.column("latitude")
    lons = gps.column("longitude")
    alts = gps.column("altitude_ft")
    home: Optional[tuple[float, float, float]] = None
    times: list[float] = []
    distances: list[float] = []
    for t, lat, lon, alt in zip(gps.times, lats, lons, alts):
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        if lat == 0.0 and lon == 0.0:
            continue
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            continue
        alt_m = alt * _FEET_TO_METERS if math.isfinite(alt) else 0.0
        if home is None:
            home = (lat, lon, alt_m)
        dy = (lat - home[0]) * _METERS_PER_DEG_LAT
        dx = (
            (lon - home[1])
            * _METERS_PER_DEG_LON_EQUATOR
            * math.cos(math.radians(home[0]))
        )
        dz = alt_m - home[2]
        times.append(t)
        distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    return times, distances


def _interp_at(times: list[float], values: list[float], t: float) -> float:
    """Linear interpolation with edge clamping (times must be sorted)."""

    if not times:
        return math.nan
    if t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    lo, hi = 0, len(times) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    span = times[hi] - times[lo]
    if span <= 0:
        return values[lo]
    frac = (t - times[lo]) / span
    return values[lo] + (values[hi] - values[lo]) * frac


def _test_link_budget(streams: dict[str, _Stream]) -> Finding:
    """Model RSSI against distance from home and check link health."""

    link = streams["link_stats"]
    if len(link.times) < 5:
        return Finding(
            "Link budget",
            "no_data",
            "No link statistics were recorded.",
        )

    details: list[str] = []
    status = "pass"

    # Uplink quality lows are the closest thing to "how near did we get to
    # failsafe" the record contains.
    qualities = _finite(link.column("link_quality"))
    if qualities:
        lq_min = min(qualities)
        if lq_min < 30.0:
            status = "fail"
            details.append(
                f"Uplink link quality fell to {lq_min:.0f}% — the link came "
                "close to failsafe."
            )
        elif lq_min < 50.0:
            status = "warn"
            details.append(
                f"Uplink link quality fell to {lq_min:.0f}% — margin was thin."
            )
        else:
            details.append(f"Uplink link quality never fell below {lq_min:.0f}%.")

    # Diversity antenna comparison: both antennas reporting means a sustained
    # spread points at a damaged/shadowed antenna.
    rssi_a = link.column("rssi_a")
    rssi_b = link.column("rssi_b")
    spreads = [
        abs(a - b)
        for a, b in zip(rssi_a, rssi_b)
        if math.isfinite(a) and math.isfinite(b)
    ]
    if spreads:
        mean_spread = _mean(spreads)
        if mean_spread > 15.0:
            if status != "fail":
                status = "warn"
            details.append(
                f"RSSI antennas disagreed by {mean_spread:.0f} dB on average "
                "— check for a damaged or shadowed diversity antenna."
            )
        else:
            details.append(
                f"Diversity antennas tracked each other (avg spread "
                f"{mean_spread:.0f} dB)."
            )

    # Path-loss model: best-antenna RSSI against log10(distance from home).
    gps_times, distances = _gps_distances(streams["gps"])
    max_distance = max(distances) if distances else 0.0
    if not distances:
        details.append(
            "No valid GPS fix was recorded, so RSSI could not be modelled "
            "against distance."
        )
    elif max_distance < 50.0:
        details.append(
            f"Flight stayed within {max_distance:.0f} m of home — too close "
            "to model RSSI against distance."
        )
    else:
        xs: list[float] = []
        ys: list[float] = []
        for t, a, b in zip(link.times, rssi_a, rssi_b):
            best = max(
                (v for v in (a, b) if math.isfinite(v)), default=math.nan
            )
            if not math.isfinite(best):
                continue
            d = _interp_at(gps_times, distances, t)
            if math.isfinite(d) and d > 20.0:
                xs.append(math.log10(d))
                ys.append(best)
        if len(xs) >= 20:
            _, slope, residual_std = _linear_fit(xs, ys)
            exponent = -slope / 10.0
            details.append(
                f"Max distance {max_distance:.0f} m; fitted path-loss "
                f"exponent {exponent:.1f} (≈2 is clean line-of-sight), "
                f"residual spread {residual_std:.1f} dB."
            )
            if exponent > 3.5:
                if status != "fail":
                    status = "warn"
                details.append(
                    "Signal fell off with distance much faster than free "
                    "space — check antenna placement, connectors, and "
                    "polarisation."
                )
            if residual_std > 6.0:
                if status != "fail":
                    status = "warn"
                details.append(
                    "RSSI scattered widely at similar distances — often "
                    "antenna blanking on certain headings or multipath."
                )
        else:
            details.append(
                "Too few overlapping RSSI/GPS samples beyond 20 m to fit a "
                "path-loss model."
            )

    summary = {
        "pass": "Radio link stayed healthy for the whole flight.",
        "warn": "Radio link showed degradation worth investigating.",
        "fail": "Radio link came dangerously close to failsafe.",
    }[status]
    return Finding("Link budget", status, summary, details)


# ----------------------------------------------------------------------
# EKF tuning advisor: cross-check the estimator against GPS
# ----------------------------------------------------------------------
# GPS is an independent truth reference the attitude EKF does not fuse for
# roll/pitch/yaw on the maiden-bring-up build, so its disagreement with the
# estimate is a clean tuning signal. These tests turn that disagreement into
# "which constant, which direction" guidance. They are advisory: a detected
# bias is a hint, never a failed flight, so nothing here escalates past "warn".

_GRAVITY_MS2 = 9.80665
_FLIGHT_FRAME_DT_S = 0.1  # 10 Hz analysis grid: fine enough for turn rates,
# coarse enough to keep a whole flight cheap in pure Python.
# Above walking/taxi speed: the aircraft is airborne and translating, so a
# GPS ground track is meaningful. Groundspeed is derived from position, so it
# is independent of both the (unproven) pitot and the EKF under test.
_FLYING_GROUNDSPEED_MS = 3.0
_MIN_STRAIGHT_SEG_S = 3.0  # shortest straight-and-level run worth averaging
_MIN_DRIFT_SEG_S = 6.0  # shortest run long enough to fit a yaw-drift slope
_SEG_SETTLE_S = 0.5  # discard each segment's entry transient
_STRAIGHT_TURN_RATE_DPS = 3.0  # |track rate| below this reads as "straight"
_LEVEL_CLIMB_MS = 1.5  # |climb rate| below this reads as "level"
_TURN_RATE_MIN_DPS = 6.0  # |track rate| above this reads as a real turn
# Advisory thresholds (degrees unless noted).
_ROLL_BIAS_WARN_DEG = 3.0
_PITCH_LEVEL_WARN_DEG = 8.0
_HEADING_OFFSET_WARN_DEG = 8.0
_HEADING_CONSISTENT_STD_DEG = 6.0  # per-leg offset spread below this => not wind
_YAW_DRIFT_WARN_DPS = 1.0
# Estimator-continuity ("snap") detector. Sortie rows are stamped at
# ground-station receive time, so the nominal attitude interval follows the
# link/packet rate and jitter, not the fixed 125 Hz FC cadence. Rather than
# hard-code an interval, judge a pair only when it is within a small multiple
# of the log's own median interval (so normal cadence and a couple of missed
# frames count as fresh-to-fresh, but a real dropout is skipped).
_SNAP_DT_MEDIAN_FACTOR = 3.0
_SNAP_DT_FLOOR_S = 0.03  # minimum acceptance window (very high-rate logs)
_SNAP_DT_CEIL_S = 0.25  # never bridge a gap longer than the RC failsafe window
_SNAP_RATE_DPS = 500.0  # no fixed-wing slews this fast; a jump this steep is
# an estimator re-acquisition or a corrupted telemetry sample, not real motion.
_SNAP_WARN_PER_MIN = 6.0


def _wrap180(delta: float) -> float:
    """Fold a degree difference into ``[-180, 180]``."""

    return (delta + 180.0) % 360.0 - 180.0


def _angdiff_deg(a: float, b: float) -> float:
    """Smallest signed ``a - b`` in degrees, wrap-aware."""

    if not (math.isfinite(a) and math.isfinite(b)):
        return math.nan
    return _wrap180(a - b)


def _unwrap_deg(values: list[float]) -> list[float]:
    """Remove 360° wraps from an angle series, preserving NaN gaps."""

    out = [math.nan] * len(values)
    offset = 0.0
    prev: Optional[float] = None
    for i, v in enumerate(values):
        if not math.isfinite(v):
            continue
        if prev is not None:
            while (v + offset) - prev > 180.0:
                offset -= 360.0
            while (v + offset) - prev < -180.0:
                offset += 360.0
        out[i] = v + offset
        prev = out[i]
    return out


def _smooth(values: list[float], window: int) -> list[float]:
    """Centred moving average that skips NaNs (window in samples)."""

    if window <= 1:
        return list(values)
    n = len(values)
    out = [math.nan] * n
    half = window // 2
    for i in range(n):
        chunk = [
            v
            for v in values[max(0, i - half) : min(n, i + half + 1)]
            if math.isfinite(v)
        ]
        if chunk:
            out[i] = sum(chunk) / len(chunk)
    return out


def _grid_derivative(values: list[float], dt: float) -> list[float]:
    """Centred first difference on a uniform grid; NaN where undefined."""

    n = len(values)
    out = [math.nan] * n
    for i in range(1, n - 1):
        a, b = values[i - 1], values[i + 1]
        if math.isfinite(a) and math.isfinite(b):
            out[i] = (b - a) / (2.0 * dt)
    if n >= 2:
        if math.isfinite(values[0]) and math.isfinite(values[1]):
            out[0] = (values[1] - values[0]) / dt
        if math.isfinite(values[-1]) and math.isfinite(values[-2]):
            out[-1] = (values[-1] - values[-2]) / dt
    return out


def _sample_on_grid(
    times: list[float], values: list[float], grid: list[float]
) -> list[float]:
    """Interpolate a series onto ``grid``; NaN outside the series' own span."""

    pairs = [(t, v) for t, v in zip(times, values) if math.isfinite(v)]
    if len(pairs) < 2:
        return [math.nan] * len(grid)
    ts = [p[0] for p in pairs]
    vs = [p[1] for p in pairs]
    lo, hi = ts[0], ts[-1]
    return [
        (_interp_at(ts, vs, t) if lo <= t <= hi else math.nan) for t in grid
    ]


def _bool_runs(flags: list[bool]) -> list[tuple[int, int]]:
    """Contiguous ``True`` spans as inclusive ``(start, end)`` index pairs."""

    runs: list[tuple[int, int]] = []
    start: Optional[int] = None
    for i, flag in enumerate(flags):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(flags) - 1))
    return runs


def _flight_frame(streams: dict[str, _Stream]) -> Optional[dict[str, list[float]]]:
    """Resample attitude + GPS onto one 10 Hz grid with GPS-derived motion.

    Returns None unless there is a usable overlap of attitude telemetry and
    valid GPS fixes. Groundspeed and ground track are built from GPS position
    alone, so they stay independent of the pitot and of the attitude estimate
    the advisor is grading.
    """

    att = streams["attitude"]
    gps = streams["gps"]
    if len(att.times) < 50 or len(gps.times) < 10:
        return None

    lats = gps.column("latitude")
    lons = gps.column("longitude")
    alts = gps.column("altitude_ft")
    courses = gps.column("ground_course")
    fix_t: list[float] = []
    east: list[float] = []
    north: list[float] = []
    fix_course: list[float] = []
    fix_alt: list[float] = []
    home: Optional[tuple[float, float]] = None
    for i, t in enumerate(gps.times):
        lat = lats[i] if i < len(lats) else math.nan
        lon = lons[i] if i < len(lons) else math.nan
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        if lat == 0.0 and lon == 0.0:
            continue
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            continue
        if home is None:
            home = (lat, lon)
        fix_t.append(t)
        north.append((lat - home[0]) * _METERS_PER_DEG_LAT)
        east.append(
            (lon - home[1])
            * _METERS_PER_DEG_LON_EQUATOR
            * math.cos(math.radians(home[0]))
        )
        c = courses[i] if i < len(courses) else math.nan
        fix_course.append(c)
        a = alts[i] if i < len(alts) else math.nan
        fix_alt.append(a * _FEET_TO_METERS if math.isfinite(a) else math.nan)

    if len(fix_t) < 5:
        return None
    t0 = max(att.times[0], fix_t[0])
    t1 = min(att.times[-1], fix_t[-1])
    if t1 - t0 < _MIN_STRAIGHT_SEG_S:
        return None

    dt = _FLIGHT_FRAME_DT_S
    count = int((t1 - t0) / dt) + 1
    grid = [t0 + i * dt for i in range(count)]

    east_g = _sample_on_grid(fix_t, east, grid)
    north_g = _sample_on_grid(fix_t, north, grid)
    alt_g = _sample_on_grid(fix_t, fix_alt, grid)
    # Unwrap heading angles *before* interpolating: interpolating across a
    # 359°->1° wrap would otherwise synthesise a phantom ~180° midpoint.
    yaw_g = _sample_on_grid(att.times, _unwrap_deg(att.column("yaw")), grid)
    roll_g = _sample_on_grid(att.times, att.column("roll"), grid)
    pitch_g = _sample_on_grid(att.times, att.column("pitch"), grid)

    v_east = _grid_derivative(east_g, dt)
    v_north = _grid_derivative(north_g, dt)
    speed = _smooth(
        [
            math.hypot(ve, vn)
            if math.isfinite(ve) and math.isfinite(vn)
            else math.nan
            for ve, vn in zip(v_east, v_north)
        ],
        11,  # ~1 s: tame GPS position-differencing noise
    )

    # Prefer the receiver's own Doppler course (low noise); fall back to the
    # position-derived track only where the receiver did not report one.
    logged = _sample_on_grid(fix_t, _unwrap_deg(fix_course), grid)
    course: list[float] = []
    for i in range(count):
        c = logged[i]
        if not math.isfinite(c):
            if (
                math.isfinite(v_east[i])
                and math.isfinite(v_north[i])
                and math.isfinite(speed[i])
                and speed[i] > _FLYING_GROUNDSPEED_MS
            ):
                c = math.degrees(math.atan2(v_east[i], v_north[i]))
            else:
                c = math.nan
        course.append(c)
    course = _unwrap_deg(course)

    return {
        "grid": grid,
        "yaw": yaw_g,
        "roll": roll_g,
        "pitch": pitch_g,
        "speed": speed,
        "course": course,
        "alt": alt_g,
    }


def _test_estimator_continuity(streams: dict[str, _Stream]) -> Finding:
    """Flag non-physical attitude jumps between consecutive fresh samples."""

    att = streams["attitude"]
    times = att.times
    if len(times) < 100:
        return Finding(
            "Estimator continuity",
            "no_data",
            "Not enough attitude telemetry to check estimator continuity.",
        )

    # Acceptance window scaled to this log's own cadence (see constants).
    intervals = [
        times[i] - times[i - 1]
        for i in range(1, len(times))
        if times[i] - times[i - 1] > 0.0
    ]
    median_dt = _median(intervals)
    if median_dt > 0.0:
        max_dt = min(
            _SNAP_DT_CEIL_S, max(_SNAP_DT_FLOOR_S, _SNAP_DT_MEDIAN_FACTOR * median_dt)
        )
    else:
        max_dt = _SNAP_DT_FLOOR_S

    details: list[str] = []
    total_snaps = 0
    worst_rate = 0.0
    for axis in ("roll", "pitch", "yaw"):
        values = att.column(axis)
        if axis == "yaw":
            values = _unwrap_deg(values)
        snaps = 0
        for i in range(1, len(times)):
            dt = times[i] - times[i - 1]
            if not 0.0 < dt <= max_dt:
                continue  # a dropout gap, not a fresh-to-fresh step
            a, b = values[i - 1], values[i]
            if not (math.isfinite(a) and math.isfinite(b)):
                continue
            rate = abs(b - a) / dt
            if rate > _SNAP_RATE_DPS:
                snaps += 1
                worst_rate = max(worst_rate, rate)
        total_snaps += snaps
        details.append(
            f"{axis}: {snaps} non-physical jump(s) "
            f"(> {_SNAP_RATE_DPS:.0f}°/s between fresh samples)."
        )
    details.append(
        f"(judged pairs up to {max_dt * 1000.0:.0f} ms apart; "
        f"median attitude interval {median_dt * 1000.0:.0f} ms.)"
    )

    duration = times[-1] - times[0]
    per_min = total_snaps / (duration / 60.0) if duration > 0 else 0.0
    if per_min > _SNAP_WARN_PER_MIN:
        status = "warn"
        summary = (
            f"{total_snaps} non-physical attitude jump(s) "
            f"({per_min:.1f}/min, worst {worst_rate:.0f}°/s)."
        )
        details.append(
            "Frequent snaps are usually EKF innovation-gate re-acquisitions "
            "(a gate too tight — MAG_YAW_INNOVATION_GATE for yaw, "
            "ACCEL_INNOVATION_GATE for roll/pitch) or corrupted telemetry; "
            "cross-check the link-budget finding to tell them apart."
        )
    else:
        status = "pass"
        summary = (
            "Attitude evolved continuously (no non-physical jumps)."
            if total_snaps == 0
            else f"{total_snaps} isolated attitude jump(s) — within noise."
        )
    return Finding("Estimator continuity", status, summary, details)


def _test_attitude_vs_gps(streams: dict[str, _Stream]) -> Finding:
    """Cross-check the attitude estimate against GPS and advise on tuning."""

    frame = _flight_frame(streams)
    if frame is None:
        return Finding(
            "EKF attitude vs GPS",
            "no_data",
            "No usable overlap of attitude telemetry and valid GPS fixes to "
            "cross-check the estimator.",
        )

    dt = _FLIGHT_FRAME_DT_S
    grid = frame["grid"]
    yaw, roll, pitch = frame["yaw"], frame["roll"], frame["pitch"]
    speed, course, alt = frame["speed"], frame["course"], frame["alt"]
    n = len(grid)
    turn = _smooth(_grid_derivative(course, dt), 5)  # deg/s
    climb = _smooth(_grid_derivative(alt, dt), 11)  # m/s

    flying = [
        math.isfinite(speed[i])
        and speed[i] > _FLYING_GROUNDSPEED_MS
        and math.isfinite(yaw[i])
        and math.isfinite(roll[i])
        and math.isfinite(pitch[i])
        and math.isfinite(course[i])
        for i in range(n)
    ]
    if sum(flying) < int(_MIN_STRAIGHT_SEG_S / dt):
        return Finding(
            "EKF attitude vs GPS",
            "no_data",
            "The aircraft never translated far enough under GPS for a "
            "cross-check (mostly stationary or no fix).",
        )

    straight = [
        flying[i]
        and abs(turn[i]) < _STRAIGHT_TURN_RATE_DPS
        and (not math.isfinite(climb[i]) or abs(climb[i]) < _LEVEL_CLIMB_MS)
        for i in range(n)
    ]

    # Every level-leg metric is compared against what a *correct* estimate
    # would read, not against zero: even a leg that passes the straight gate
    # can carry a small residual turn (up to _STRAIGHT_TURN_RATE_DPS) or climb.
    # A coordinated turn at rate omega genuinely banks the aircraft by
    # atan(V*omega/g) and swings its heading at omega, so subtracting the
    # expected bank / flight-path angle / GPS course slope keeps a clean
    # estimator from being reported as a roll bias or an R_INIT_YAW drift on a
    # gently curving leg.
    roll_vals: list[float] = []
    pitch_vals: list[float] = []
    seg_offsets: list[float] = []  # mean(yaw - course) per straight leg
    worst_drift = 0.0
    for start, end in _bool_runs(straight):
        if grid[end] - grid[start] < _MIN_STRAIGHT_SEG_S:
            continue
        settle = grid[start] + _SEG_SETTLE_S
        idx = [i for i in range(start, end + 1) if grid[i] >= settle]
        if len(idx) < 2:
            continue
        for i in idx:
            expected_bank = -math.degrees(
                math.atan2(speed[i] * math.radians(turn[i]), _GRAVITY_MS2)
            )
            roll_vals.append(roll[i] - expected_bank)
            if math.isfinite(climb[i]):
                gamma = math.degrees(math.atan2(climb[i], speed[i]))
            else:
                gamma = 0.0
            pitch_vals.append(pitch[i] - gamma)
        # Heading *error* (estimate minus GPS track), not raw yaw: its mean is
        # the heading offset and its slope is the estimator's yaw drift with the
        # GPS course change already removed.
        herr = _unwrap_deg([_angdiff_deg(yaw[i], course[i]) for i in idx])
        finite_herr = [(grid[i], h) for i, h in zip(idx, herr) if math.isfinite(h)]
        if finite_herr:
            seg_offsets.append(_mean([h for _, h in finite_herr]))
        if (
            len(finite_herr) >= 2
            and finite_herr[-1][0] - finite_herr[0][0] >= _MIN_DRIFT_SEG_S
        ):
            _, slope, _ = _linear_fit(
                [t for t, _ in finite_herr], [h for _, h in finite_herr]
            )
            worst_drift = max(worst_drift, abs(slope))

    if not roll_vals:
        return Finding(
            "EKF attitude vs GPS",
            "no_data",
            "No sustained straight-and-level flight was found to average the "
            "estimator against GPS.",
        )

    details: list[str] = []
    status = "pass"

    # --- Roll bias (turn-compensated level roll should be ~0) --------------
    roll_bias = _mean(roll_vals)
    details.append(
        f"Turn-compensated level roll averages {roll_bias:+.1f}° over "
        f"{len(roll_vals)} samples (target ~0°)."
    )
    if abs(roll_bias) > _ROLL_BIAS_WARN_DEG:
        status = "warn"
        details.append(
            f"→ Steady roll bias: trim the board-alignment roll by about "
            f"{-roll_bias:+.1f}° (FC_BOARD_ALIGN_ROLL_DEG), or suspect "
            "R_INIT_ACC over-trusting the accelerometer."
        )

    # --- Pitch in level flight (= angle of attack + any bias) --------------
    pitch_level = _mean(pitch_vals)
    details.append(
        f"Level-flight pitch averages {pitch_level:+.1f}° nose-up "
        "(a few degrees of angle of attack is normal)."
    )
    if abs(pitch_level) > _PITCH_LEVEL_WARN_DEG:
        status = "warn"
        details.append(
            f"→ {pitch_level:+.1f}° is large for level cruise: check "
            "R_INIT_ACC (specific force during climb/accel biases pitch) or "
            "the board-alignment pitch trim (FC_BOARD_ALIGN_PITCH_DEG)."
        )

    # --- Heading: yaw vs GPS ground track ---------------------------------
    if seg_offsets:
        offset_mean = _mean(seg_offsets)
        offset_std = _std(seg_offsets) if len(seg_offsets) >= 2 else 0.0
        details.append(
            f"Heading vs GPS track offset {offset_mean:+.1f}° across "
            f"{len(seg_offsets)} leg(s) (spread {offset_std:.1f}°)."
        )
        if abs(offset_mean) > _HEADING_OFFSET_WARN_DEG:
            if len(seg_offsets) >= 2 and offset_std < _HEADING_CONSISTENT_STD_DEG:
                status = "warn"
                details.append(
                    f"→ Consistent {offset_mean:+.1f}° offset across legs: a "
                    "magnetic declination (FC_MAG_DECLINATION_RAD) or "
                    "compass-calibration error, not R_INIT_YAW."
                )
            else:
                details.append(
                    "→ Offset varies between legs — likely wind crab rather "
                    "than a calibration error; interpret with caution."
                )

    if worst_drift > _YAW_DRIFT_WARN_DPS:
        status = "warn"
        details.append(
            f"→ Heading drifted up to {worst_drift:.1f}°/s on a straight leg "
            "while the GPS track held steady: the compass is under-weighted — "
            "lower R_INIT_YAW (or check for a reject-happy "
            "MAG_YAW_INNOVATION_GATE)."
        )

    # --- Coordinated-turn cross-check (sign + scale of roll) --------------
    predicted: list[float] = []
    observed: list[float] = []
    for i in range(n):
        if not (flying[i] and abs(turn[i]) > _TURN_RATE_MIN_DPS):
            continue
        # Coordinated-turn load balance: tan(bank) = V * omega / g. A right
        # turn (track increasing) needs right bank, which is negative roll in
        # this firmware's convention, hence the leading minus.
        bank = -math.degrees(
            math.atan2(speed[i] * math.radians(turn[i]), _GRAVITY_MS2)
        )
        if abs(bank) < 75.0:
            predicted.append(bank)
            observed.append(roll[i])
    if len(predicted) >= 30 and (max(predicted) - min(predicted)) > 10.0:
        _, slope, _ = _linear_fit(predicted, observed)
        details.append(
            f"Roll vs coordinated-turn bank: slope {slope:.2f} over "
            f"{len(predicted)} turning samples (expect ~1)."
        )
        if slope < 0.0:
            status = "warn"
            details.append(
                "→ Roll tracks GPS-derived bank with inverted sign — a roll "
                "sign-convention problem, not a tuning value."
            )
        elif abs(slope - 1.0) > 0.4:
            status = "warn"
            details.append(
                f"→ Roll is scaled {slope:.2f}× versus the coordinated-turn "
                "bank — check the attitude/board-alignment scaling."
            )

    details.append(
        "Note: if heading is accurate in level flight but degrades in turns, "
        "suspect the magnetic inclination constant (FC_MAG_INCLINATION_RAD)."
    )

    if status == "warn":
        summary = (
            "Estimator shows a tunable bias against the GPS cross-check — see "
            "the constant guidance below."
        )
    else:
        summary = "Attitude estimate agreed with the GPS cross-check."
    return Finding("EKF attitude vs GPS", status, summary, details)


# ----------------------------------------------------------------------
# Entry points
# ----------------------------------------------------------------------
def analyze_sortie(path: str) -> SortieReport:
    """Run every post-flight test over one sortie CSV."""

    report = SortieReport(path=path)
    try:
        streams = load_sortie_streams(path)
    except OSError as exc:
        report.error = f"Could not read sortie file: {exc}"
        return report

    if not any(stream.times for stream in streams.values()):
        report.error = "The sortie file contains no telemetry rows."
        return report

    report.findings.append(_test_continuity(streams))
    report.findings.append(_test_frozen_sensors(streams))
    report.findings.append(_test_control_response(streams))
    report.findings.append(_test_fbw_tracking(streams))
    report.findings.append(_test_link_budget(streams))
    report.findings.append(_test_estimator_continuity(streams))
    report.findings.append(_test_attitude_vs_gps(streams))
    return report


_STATUS_ICONS = {"pass": "✅", "warn": "⚠️", "fail": "❌", "no_data": "ℹ️"}

_OVERALL_SUMMARIES = {
    "pass": "All post-flight checks passed.",
    "warn": "Post-flight checks passed with warnings.",
    "fail": "Post-flight checks found problems.",
    "no_data": "Not enough telemetry was recorded to run the checks.",
}


def report_html(report: SortieReport) -> str:
    """Build the rich-text post-flight report shown after a recording stops."""

    name = os.path.basename(report.path)
    if report.error:
        return f"<b>Post-flight analysis of {name} failed.</b><p>{report.error}</p>"

    overall = report.overall_status()
    parts = [f"<b>{_OVERALL_SUMMARIES[overall]}</b>", "<ul>"]
    for finding in report.findings:
        icon = _STATUS_ICONS.get(finding.status, "•")
        parts.append(f"<li>{icon} <b>{finding.name}:</b> {finding.summary}")
        if finding.details:
            parts.append("<ul>")
            parts.extend(f"<li>{detail}</li>" for detail in finding.details)
            parts.append("</ul>")
        parts.append("</li>")
    parts.append("</ul>")
    return "".join(parts)


def report_text(report: SortieReport) -> str:
    """Plain-text rendering of a report (headless/scripted use)."""

    name = os.path.basename(report.path)
    if report.error:
        return f"Post-flight analysis of {name} failed: {report.error}"
    lines = [f"Post-flight report — {name}: {_OVERALL_SUMMARIES[report.overall_status()]}"]
    for finding in report.findings:
        lines.append(f"[{finding.status.upper():7}] {finding.name}: {finding.summary}")
        lines.extend(f"    - {detail}" for detail in finding.details)
    return "\n".join(lines)


# The Qt worker is optional so the analysis above stays importable in headless
# environments (tests, scripts) where PySide6 is not installed.
try:
    from PySide6.QtCore import QThread, Signal

    class SortieAnalysisWorker(QThread):
        """Run ``analyze_sortie`` off the GUI thread and emit the report."""

        report_ready = Signal(object)

        def __init__(self, path: str, parent=None):
            super().__init__(parent)
            self._path = path

        def run(self) -> None:  # pragma: no cover - thin Qt wrapper
            self.report_ready.emit(analyze_sortie(self._path))

except ImportError:  # pragma: no cover - headless analysis still works
    SortieAnalysisWorker = None
