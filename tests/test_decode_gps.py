import struct
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from pico_modules.pico_transmitpackets import CRSFPacketProcessor


class DummySignal:
    def __init__(self):
        self.emitted = None

    def emit(self, value):
        self.emitted = value


def test_decode_gps_scaling():
    proc = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    proc.telemetry_ready = DummySignal()

    lat_deg = 37.7749
    lon_deg = -122.4194
    speed_mps = 30.0
    course_deg = 90.0
    alt_m = 1500
    sats = 10

    lat_raw = int(lat_deg * 10_000_000)
    lon_raw = int(lon_deg * 10_000_000)
    speed_raw = int((speed_mps * 36 + 50) / 100)
    course_raw = int(course_deg * 100)
    alt_raw = int(alt_m + 1000)

    payload = struct.pack(
        ">iiHHHB", lat_raw, lon_raw, speed_raw, course_raw, alt_raw, sats
    )
    packet_type = 0x02
    length = len(payload) + 2  # type + payload + crc
    crc = CRSFPacketProcessor.crc8_data(bytes([packet_type]) + payload)
    packet = bytes([0xEA, length, packet_type]) + payload + bytes([crc])

    proc.decode_gps(packet)
    name, lat, lon, speed_mph, course, alt_ft, sats_out = proc.telemetry_ready.emitted

    assert name == "gps"
    assert abs(lat - lat_deg) < 1e-7
    assert abs(lon - lon_deg) < 1e-7

    expected_mph = ((speed_raw * 100 - 50) / 36.0) * 2.23694
    assert abs(speed_mph - expected_mph) < 1e-5

    assert abs(course - course_deg) < 1e-2

    expected_alt_ft = alt_m * 3.28084
    assert abs(alt_ft - expected_alt_ft) < 1e-2

    assert sats_out == sats
