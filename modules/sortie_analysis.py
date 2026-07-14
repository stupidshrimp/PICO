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

Each test degrades gracefully: if the flight did not exercise the data a test
needs (no stick activity, no GPS fix) the test reports ``no_data`` instead of
guessing.
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
