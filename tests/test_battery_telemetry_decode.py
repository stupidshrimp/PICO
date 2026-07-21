import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pico_modules.pico_transmitpackets import CRSFPacketProcessor


class _SignalCapture:
    def __init__(self):
        self.values = []

    def emit(self, value):
        self.values.append(value)


def _processor_for_decode():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.telemetry_ready = _SignalCapture()
    return processor


def test_decode_crsf_battery_payload_uses_big_endian_u24_capacity():
    processor = _processor_for_decode()
    payload = bytes([
        0x00, 0x7B,  # 12.3 V in 0.1 V units
        0x00, 0x2D,  # 4.5 A in 0.1 A units
        0x01, 0x00, 0x2C,  # 65,580 mAh (requires u24, not u16)
        87,
    ])

    consumed = processor._decode_payload(0x08, payload)

    assert consumed == 8
    assert processor.telemetry_ready.values == [
        ("battery", 12.3, 4.5, 65580, 87.0)
    ]


def test_decode_crsf_battery_payload_accepts_missing_percent_byte():
    processor = _processor_for_decode()
    payload = bytes([
        0x00, 0x7B,
        0x00, 0x2D,
        0x00, 0x04, 0xD2,  # 1,234 mAh
    ])

    consumed = processor._decode_payload(0x08, payload)

    assert consumed == 7
    assert processor.telemetry_ready.values == [("battery", 12.3, 4.5, 1234)]
