"""Unit tests for the ground-station board-alignment trim channel mapping.

These verify the pure channel semantics (modules/board_align_trim.py) headlessly
-- no PySide6 needed -- and that the encode/decode round trip agrees with the
formula the FC uses in flight_controller/Main.ino.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from modules.board_align_trim import (
    BOARD_ALIGN_PITCH_CHANNEL_INDEX,
    BOARD_ALIGN_ROLL_CHANNEL_INDEX,
    BOARD_ALIGN_TRIM_RANGE_DEG,
    BOARD_ALIGN_TRIM_STEP_DEG,
    CRSF_CHANNEL_CENTER,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MIN,
    FC_BOARD_ALIGN_BASELINE_PITCH_DEG,
    FC_BOARD_ALIGN_BASELINE_ROLL_DEG,
    board_align_channel_to_offset,
    board_align_offset_to_channel,
    clamp_board_align_offset,
    effective_board_align_offset,
    step_board_align_offset,
)


def test_zero_offset_is_channel_center():
    assert board_align_offset_to_channel(0.0) == CRSF_CHANNEL_CENTER
    assert board_align_channel_to_offset(CRSF_CHANNEL_CENTER) == 0.0


def test_full_range_maps_to_symmetric_extremes():
    high = board_align_offset_to_channel(BOARD_ALIGN_TRIM_RANGE_DEG)
    low = board_align_offset_to_channel(-BOARD_ALIGN_TRIM_RANGE_DEG)
    # Symmetric about center and within the CRSF envelope.
    assert CRSF_CHANNEL_MIN <= low < CRSF_CHANNEL_CENTER < high <= CRSF_CHANNEL_MAX
    assert (high - CRSF_CHANNEL_CENTER) == (CRSF_CHANNEL_CENTER - low)


def test_offsets_beyond_range_are_clamped():
    assert clamp_board_align_offset(999.0) == BOARD_ALIGN_TRIM_RANGE_DEG
    assert clamp_board_align_offset(-999.0) == -BOARD_ALIGN_TRIM_RANGE_DEG
    # Encoding a clamped offset never leaves the channel envelope.
    assert CRSF_CHANNEL_MIN <= board_align_offset_to_channel(50.0) <= CRSF_CHANNEL_MAX
    assert CRSF_CHANNEL_MIN <= board_align_offset_to_channel(-50.0) <= CRSF_CHANNEL_MAX


def test_encode_decode_round_trip_within_one_channel_step():
    # One channel unit is worth ~0.012 deg, so the round trip is exact to well
    # under the 0.1 deg key step.
    tolerance = BOARD_ALIGN_TRIM_RANGE_DEG / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) * 2
    for tenths in range(-100, 101):
        offset = tenths * 0.1
        decoded = board_align_channel_to_offset(board_align_offset_to_channel(offset))
        assert abs(decoded - clamp_board_align_offset(offset)) <= tolerance


def test_step_accumulates_and_clamps():
    assert step_board_align_offset(0.0, 1) == BOARD_ALIGN_TRIM_STEP_DEG
    assert step_board_align_offset(0.0, -1) == -BOARD_ALIGN_TRIM_STEP_DEG
    # Stepping past the range saturates instead of overshooting.
    many = int(round(BOARD_ALIGN_TRIM_RANGE_DEG / BOARD_ALIGN_TRIM_STEP_DEG)) + 50
    assert step_board_align_offset(0.0, many) == BOARD_ALIGN_TRIM_RANGE_DEG
    assert step_board_align_offset(0.0, -many) == -BOARD_ALIGN_TRIM_RANGE_DEG


def test_effective_offset_is_baseline_plus_delta():
    # The FC adds the keyboard delta to its compiled baseline, so the value to
    # bake back into FC_BOARD_ALIGN_*_DEG is baseline + delta, not the delta.
    assert effective_board_align_offset(-6.9, 0.0) == -6.9
    assert abs(effective_board_align_offset(-6.9, 2.3) - (-4.6)) < 1e-9
    assert effective_board_align_offset(FC_BOARD_ALIGN_BASELINE_PITCH_DEG, 0.0) == (
        FC_BOARD_ALIGN_BASELINE_PITCH_DEG
    )
    # A zero baseline makes the effective offset equal to the delta.
    assert effective_board_align_offset(0.0, 1.5) == 1.5


def test_channels_are_the_free_auxes():
    # CH5/AUX1=4 (arm), CH6/AUX2=5 (mode), CH7/AUX3=6 (throttle/compass) are used.
    assert BOARD_ALIGN_ROLL_CHANNEL_INDEX == 7   # CH8/AUX4
    assert BOARD_ALIGN_PITCH_CHANNEL_INDEX == 8  # CH9/AUX5
    assert BOARD_ALIGN_ROLL_CHANNEL_INDEX not in (4, 5, 6)
    assert BOARD_ALIGN_PITCH_CHANNEL_INDEX not in (4, 5, 6)
    assert BOARD_ALIGN_ROLL_CHANNEL_INDEX != BOARD_ALIGN_PITCH_CHANNEL_INDEX
