"""Tests for the GPS-truth EKF tuning advisor in ``sortie_analysis``.

The advisor cross-checks the attitude estimate against GPS (which the
maiden-bring-up firmware does not fuse for roll/pitch/yaw), so these tests
synthesise a physically consistent flight — position integrated from a heading
schedule, roll set to the coordinated-turn bank — and confirm the advisor both
stays quiet on a clean flight and points at the right firmware constant when a
known bias or drift is injected.
"""

import csv
import math
import pathlib
import sys
from datetime import datetime, timedelta

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from modules.sortie_analysis import (  # noqa: E402
    _test_attitude_vs_gps,
    _test_estimator_continuity,
    analyze_sortie,
    load_sortie_streams,
)

_G = 9.80665
_SPEED_MS = 20.0
_HOME_LAT = 34.0
_HOME_LON = -117.0
_M_PER_DEG_LAT = 111_132.0
_M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(_HOME_LAT))
_MS_TO_MPH = 2.236_936

# (start_s, end_s, turn_rate_deg_per_s): straight, right turn, straight, left
# turn (a gentler rate), straight. The two different turn rates give the
# coordinated-turn regression a real range of bank angles to fit, and each
# straight leg is long enough (>6 s after the entry settle) to fit a yaw drift.
_PHASES = [(0.0, 8.0, 0.0), (8.0, 13.0, 15.0), (13.0, 21.0, 0.0),
           (21.0, 26.0, -8.0), (26.0, 34.0, 0.0)]
# Roll onto/off each turn at a finite rate so the synthetic roll trace ramps
# like a real airframe instead of stepping (an instantaneous step would read,
# correctly, as a non-physical jump to the continuity detector).
_OMEGA_RATE_DPS2 = 40.0
_FULL_FIELDS = [
    "pitch", "roll", "yaw", "stick_pitch", "stick_roll", "stick_yaw",
    "stick_throttle", "control_mode", "fbw_setpoint_roll",
    "fbw_setpoint_pitch", "fbw_limit_roll", "fbw_limit_pitch", "latitude",
    "longitude", "altitude_ft", "airspeed_mph", "ground_course", "satellites",
    "rssi_a", "rssi_b", "link_quality", "snr", "downlink_quality",
    "downlink_snr", "link_piggyback_count", "battery_voltage",
    "battery_current", "battery_capacity", "battery_percent",
]


def _omega_dps(t: float, phases) -> float:
    for start, end, omega in phases:
        if start <= t < end:
            return omega
    return 0.0


def _write_flight(path, *, roll_bias=0.0, pitch_bias=0.0, yaw_drift_dps=0.0,
                  with_gps=True, yaw_snaps=0, initial_hdg=90.0, att_dt=0.02,
                  phases=None, course_value=None, gps_outage=None,
                  wind_ms=(0.0, 0.0)):
    """Integrate the flight and write it as a sortie CSV.

    roll/pitch/yaw are the *estimator's* values, so the injected bias/drift is
    the estimator error the advisor should recover; the GPS ground track stays
    truth.
    """

    phases = phases if phases is not None else _PHASES
    base = datetime(2026, 7, 20, 12, 0, 0)
    total = phases[-1][1]
    sim_dt = 0.01
    lat, lon, hdg = _HOME_LAT, _HOME_LON, initial_hdg
    omega = 0.0  # ramps toward the phase target so roll never steps

    rows = []
    last_att = -1.0
    last_gps = -1.0
    att_index = 0
    snap_indices = set()
    if yaw_snaps:
        # spread the spikes across the flight; each is one bad attitude sample
        att_total = int(total / att_dt)
        step = max(1, att_total // (yaw_snaps + 1))
        snap_indices = {step * (k + 1) for k in range(yaw_snaps)}

    steps = int(total / sim_dt)
    max_domega = _OMEGA_RATE_DPS2 * sim_dt
    for step in range(steps + 1):
        t = step * sim_dt
        target = _omega_dps(t, phases)
        omega += max(-max_domega, min(max_domega, target - omega))
        hdg_rad = math.radians(hdg)
        # coordinated bank for the current (air-relative) turn rate
        bank = -math.degrees(
            math.atan2(_SPEED_MS * math.radians(omega), _G)
        )
        # Ground velocity = air velocity (along the physical heading) + wind, so
        # in wind the GPS ground track crabs off the heading and its rate is not
        # the air-relative yaw rate.
        wind_e, wind_n = wind_ms
        gvE = _SPEED_MS * math.sin(hdg_rad) + wind_e
        gvN = _SPEED_MS * math.cos(hdg_rad) + wind_n
        ground_course = math.degrees(math.atan2(gvE, gvN)) % 360.0
        ts = (base + timedelta(seconds=t)).strftime("%Y-%m-%d %H:%M:%S.%f")

        if t - last_att >= att_dt - 1e-9:
            last_att = t
            yaw = hdg + yaw_drift_dps * t
            if att_index in snap_indices:
                yaw += 40.0  # one non-physical spike
            att_index += 1
            rows.append((ts, "attitude", {
                "pitch": f"{2.0 + pitch_bias:.4f}",
                "roll": f"{bank + roll_bias:.4f}",
                "yaw": f"{yaw % 360.0:.4f}",
                "stick_pitch": "0", "stick_roll": "0", "stick_yaw": "0",
                "stick_throttle": "0", "control_mode": "Manual",
            }))

        in_outage = (
            gps_outage is not None and gps_outage[0] <= t < gps_outage[1]
        )
        if with_gps and not in_outage and t - last_gps >= 0.1 - 1e-9:
            last_gps = t
            rows.append((ts, "gps", {
                "latitude": f"{lat:.7f}",
                "longitude": f"{lon:.7f}",
                "altitude_ft": "300.0",
                "airspeed_mph": f"{_SPEED_MS * _MS_TO_MPH:.2f}",
                "ground_course": (
                    f"{course_value:.2f}" if course_value is not None
                    else f"{ground_course:.2f}"
                ),
                "satellites": "10",
            }))

        # integrate position (ground velocity) and heading for the next step
        lat += gvN / _M_PER_DEG_LAT * sim_dt
        lon += gvE / _M_PER_DEG_LON * sim_dt
        hdg += omega * sim_dt

    header = ["timestamp", "packet_type", *_FULL_FIELDS]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=header)
        writer.writeheader()
        for ts, packet_type, values in rows:
            row = {"timestamp": ts, "packet_type": packet_type}
            row.update(values)
            writer.writerow(row)


def test_clean_flight_passes(tmp_path):
    path = tmp_path / "clean.csv"
    _write_flight(path)
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "pass", finding.details
    # The level-flight roll should read essentially zero.
    roll_line = next(d for d in finding.details if d.startswith("Turn-compensated level roll"))
    assert "+0.0°" in roll_line or "-0.0°" in roll_line
    # A clean flight must not falsely flag a heading or drift problem.
    assert not any("declination" in d for d in finding.details)
    assert not any("under-weighted" in d for d in finding.details)


def test_roll_bias_points_at_board_align(tmp_path):
    path = tmp_path / "roll_bias.csv"
    _write_flight(path, roll_bias=5.0)
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "warn"
    assert any("Turn-compensated level roll averages +5" in d for d in finding.details)
    # A +5° residual must recommend *increasing* FC_BOARD_ALIGN_ROLL_DEG by
    # +5° (it is the reported-at-level offset, boardAlignInit stores its
    # inverse) — decreasing it would double the mount bias.
    guidance = next(d for d in finding.details if "FC_BOARD_ALIGN_ROLL_DEG" in d)
    assert "increase" in guidance and "+5.0°" in guidance
    # The coordinated-turn regression should still see the right sign/scale.
    slope_line = next(d for d in finding.details if "coordinated-turn bank" in d)
    assert "slope 1." in slope_line


def test_yaw_drift_points_at_r_init_yaw(tmp_path):
    path = tmp_path / "yaw_drift.csv"
    _write_flight(path, yaw_drift_dps=1.5)
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "warn"
    assert any(
        "under-weighted" in d and "R_INIT_YAW" in d for d in finding.details
    )


def test_heading_wrap_across_north_is_clean(tmp_path):
    # Start at 350° so the first turn carries the heading through 360°/0°.
    # If course/yaw were interpolated before unwrapping, the wrap would inject
    # a phantom ~180° swing and corrupt every cross-check.
    path = tmp_path / "wrap.csv"
    _write_flight(path, initial_hdg=350.0)
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "pass", finding.details
    roll_line = next(d for d in finding.details if d.startswith("Turn-compensated level roll"))
    assert "+0.0°" in roll_line or "-0.0°" in roll_line


def test_shallow_turn_not_flagged(tmp_path):
    # A continuous gentle 2°/s turn sits below the straight-leg gate, so its
    # samples feed the level averages. The aircraft is genuinely banked ~4° and
    # its heading tracks GPS at 2°/s — a clean estimator must not be reported as
    # a roll board-alignment bias or an R_INIT_YAW drift.
    path = tmp_path / "shallow.csv"
    _write_flight(path, phases=[(0.0, 34.0, 2.0)])
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "pass", finding.details
    assert not any("board-alignment roll" in d for d in finding.details)
    assert not any("under-weighted" in d for d in finding.details)


def test_snap_detection_adapts_to_slow_cadence(tmp_path):
    # Rows are stamped at ground-station receive time. At a ~22 Hz received
    # cadence (45 ms) the detector must still judge consecutive pairs instead of
    # skipping every one as a "gap".
    clean = tmp_path / "slow_clean.csv"
    _write_flight(clean, att_dt=0.045)
    assert _test_estimator_continuity(load_sortie_streams(str(clean))).status == "pass"

    snappy = tmp_path / "slow_snappy.csv"
    _write_flight(snappy, att_dt=0.045, yaw_snaps=8)
    finding = _test_estimator_continuity(load_sortie_streams(str(snappy)))
    assert finding.status == "warn", finding.details
    assert any("non-physical" in d for d in finding.details)


def test_placeholder_gps_course_uses_derived_track(tmp_path):
    # A GGA-only receiver logs a constant 0° ground course while lat/lon are
    # valid and the aircraft is turning. Trusting it would make every leg read
    # as course 0° (false heading offsets, no turns detected). The advisor must
    # fall back to the position-derived track and stay clean on a clean flight.
    path = tmp_path / "ggaonly.csv"
    _write_flight(path, course_value=0.0)
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "pass", finding.details
    # It should still have found real turns via the derived track.
    assert any("coordinated-turn bank" in d for d in finding.details)
    assert not any("declination" in d for d in finding.details)


def test_placeholder_course_rejected_even_when_mostly_northbound(tmp_path):
    # A constant-0 placeholder course *agrees on average* with the true track
    # when the flight is mostly northbound, so the median-disagreement guard
    # alone would trust it. The variation guard must still reject it: a real
    # turn excursion means the track varies while the logged course stays flat.
    path = tmp_path / "north_placeholder.csv"
    _write_flight(
        path,
        course_value=0.0,
        initial_hdg=0.0,
        phases=[(0.0, 12.0, 0.0), (12.0, 16.0, 20.0),
                (16.0, 20.0, -20.0), (20.0, 32.0, 0.0)],
    )
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "pass", finding.details
    # Turns must be seen via the derived track — impossible if the flat course
    # had been trusted (its turn rate would be ~0 everywhere).
    assert any("coordinated-turn bank" in d for d in finding.details)


def test_gps_outage_span_not_bridged(tmp_path):
    # The aircraft turns (banks ~-35°) during a 6 s GPS outage. If the resampler
    # bridged the gap, the interpolated straight-line chord would understate the
    # turn — the real bank then looks mis-scaled against the (bogus) GPS course,
    # and the advisor emits false roll scale/heading guidance for a span that
    # had no GPS truth. The gap must be excluded instead, leaving it clean.
    path = tmp_path / "outage.csv"
    _write_flight(
        path,
        phases=[(0.0, 8.0, 0.0), (8.0, 14.0, 15.0), (14.0, 24.0, 0.0)],
        gps_outage=(8.0, 14.0),
    )
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "pass", finding.details
    assert not any("→" in d for d in finding.details)  # no warn guidance lines


def test_wind_does_not_trigger_false_roll_scale(tmp_path):
    # Healthy estimator (roll = true air-relative coordinated bank) flown in a
    # strong crosswind, with legs at different headings so the crab varies. The
    # GPS ground-speed turn model then predicts a bank scaled well off the true
    # one, but that is wind, not a roll scaling error: the advisor must detect
    # the wind (varying crab) and not emit a scale warning.
    # A 180° turn flown in a strong crosswind: the air-relative bank is constant
    # but the GPS ground-speed turn model swings, so the regression slope lands
    # far from 1 (~0.26 here) purely from wind. The two straight legs point
    # opposite ways, so their crab differs sharply (large spread) — wind is
    # detected and the scale warning must be suppressed.
    path = tmp_path / "wind.csv"
    _write_flight(
        path,
        initial_hdg=0.0,
        phases=[(0.0, 4.0, 0.0), (4.0, 16.0, 15.0), (16.0, 26.0, 0.0)],
        wind_ms=(14.0, 0.0),
    )
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "pass", finding.details
    # Slope deviates from 1, but it is reported with the wind caveat, not a
    # scale warning.
    assert not any("check the attitude/board-alignment scaling" in d
                   for d in finding.details), finding.details
    assert any("deviates from 1" in d and "wind crab" in d
               for d in finding.details), finding.details


def test_no_gps_is_no_data(tmp_path):
    path = tmp_path / "no_gps.csv"
    _write_flight(path, with_gps=False)
    finding = _test_attitude_vs_gps(load_sortie_streams(str(path)))
    assert finding.status == "no_data"


def test_estimator_continuity_flags_snaps(tmp_path):
    clean = tmp_path / "clean.csv"
    _write_flight(clean)
    assert _test_estimator_continuity(load_sortie_streams(str(clean))).status == "pass"

    snappy = tmp_path / "snappy.csv"
    _write_flight(snappy, yaw_snaps=8)
    finding = _test_estimator_continuity(load_sortie_streams(str(snappy)))
    assert finding.status == "warn"
    assert any("non-physical" in d for d in finding.details)


def test_full_report_includes_advisor(tmp_path):
    path = tmp_path / "roll_bias.csv"
    _write_flight(path, roll_bias=5.0)
    report = analyze_sortie(str(path))
    names = {finding.name for finding in report.findings}
    assert "EKF attitude vs GPS" in names
    assert "Estimator continuity" in names
