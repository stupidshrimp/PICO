import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pico_modules.osd_smoothing import REFERENCE_INTERVAL_S, time_scaled_weight


def test_reference_interval_preserves_legacy_weight():
    # At the tuned reference interval the scaled weight must equal the
    # configured per-call weight so behaviour matches the legacy ~30 Hz feel.
    for weight in (0.05, 0.2, 0.5):
        assert abs(time_scaled_weight(weight, REFERENCE_INTERVAL_S) - weight) < 1e-12


def test_smoothing_is_rate_independent():
    # Blending at a faster refresh rate must compound to the same response over
    # one reference interval, i.e. the perceived smoothing is rate-independent.
    weight = 0.2
    fast_dt = 1 / 70.0
    fast_alpha = time_scaled_weight(weight, fast_dt)
    ticks = REFERENCE_INTERVAL_S / fast_dt
    compounded = 1 - (1 - fast_alpha) ** ticks
    assert abs(compounded - weight) < 1e-9


def test_faster_rate_uses_gentler_per_call_weight():
    weight = 0.2
    assert time_scaled_weight(weight, 1 / 70.0) < weight


def test_large_gap_snaps_toward_latest_sample():
    # A long pause between samples should converge essentially fully.
    assert time_scaled_weight(0.2, 5.0) > 0.999


def test_edge_cases():
    assert time_scaled_weight(0.2, 0.0) == 0.0
    assert time_scaled_weight(0.2, -1.0) == 0.0
    assert time_scaled_weight(1.0, 0.01) == 1.0
    assert time_scaled_weight(0.0, 0.01) == 0.0
    # A non-positive reference falls back to the unscaled weight.
    assert time_scaled_weight(0.2, 0.01, reference=0.0) == 0.2
