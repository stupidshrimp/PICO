import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pico_modules.pico_transmitpackets import CRSFPacketProcessor


class DummySignal:
    def __init__(self):
        self.emitted = []

    def emit(self, payload):
        self.emitted.append(payload)


class DummySerial:
    def __init__(self, frame: bytes):
        self._buffer = bytearray(frame)

    def bytesAvailable(self) -> int:
        return len(self._buffer)

    def readAll(self) -> bytes:
        data = bytes(self._buffer)
        self._buffer.clear()
        return data


def _build_handset_frame(rate: int, offset: int) -> bytes:
    subtype = 0x10
    dest = 0xEA
    orig = 0xEE
    payload = bytes([dest, orig, subtype]) + rate.to_bytes(4, "big") + offset.to_bytes(4, "big")
    frame = bytearray([CRSFPacketProcessor.TELEMETRY_SYNC, len(payload) + 2, 0x3A])
    frame.extend(payload)
    crc = CRSFPacketProcessor.crc8_data(frame[2:])
    frame.append(crc)
    return bytes(frame)


@pytest.mark.parametrize("rate, offset", [(1000, 200), (5000, 0)])
def test_handset_frame_not_dropped(rate, offset):
    frame = _build_handset_frame(rate, offset)

    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.serial = DummySerial(frame)
    processor.serial_data = DummySignal()
    processor.telemetry_ready = DummySignal()
    processor.error = DummySignal()
    processor._rx_buffer = bytearray()

    processor.read_serial_data()

    assert processor.serial_data.emitted, "Serial bytes should be forwarded"
    assert processor.telemetry_ready.emitted, "Handset telemetry should not be dropped"

    event = processor.telemetry_ready.emitted[0]
    assert event[0] == "handset_timing"
    assert event[1] == 0x10
    assert event[2] == rate
    assert event[3] == offset
    assert event[4] == 0xEA
    assert event[5] == 0xEE


def test_decode_handset_piggyback_payload():
    subtype = 0x10
    rate = 2000
    offset = 50
    payload = bytes([subtype]) + rate.to_bytes(4, "big") + offset.to_bytes(4, "big")

    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.telemetry_ready = DummySignal()

    consumed = processor._decode_payload(0x3A, payload)

    assert consumed == len(payload)
    assert processor.telemetry_ready.emitted == [
        ("handset_timing", subtype, rate, offset, None, None)
    ]

class DummyWritableSerial:
    def __init__(self):
        self.writes = []

    def write(self, payload: bytes) -> int:
        self.writes.append(payload)
        return len(payload)

    def errorString(self) -> str:
        return "dummy serial error"


def test_channel_update_only_refreshes_latest_channels():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.error = DummySignal()
    processor._ensure_tx_timer = lambda: None

    result = processor.update_and_send_packet([172, 1811, 1000])

    assert result == "Good"
    assert processor.channels[:3] == [172, 1811, 1000]
    assert processor.channels[3:] == [1500] * 13


def test_worker_transmit_writes_current_packet():
    serial = DummyWritableSerial()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [1500] * 16
    processor.serial = serial
    processor.error = DummySignal()
    processor.packet_sent = DummySignal()
    processor._tx_enabled = True
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    result = processor.send_current_packet()

    assert result == "Good"
    assert len(serial.writes) == 1
    assert serial.writes[0] == bytes(processor.create_packet())
    assert processor.packet_sent.emitted == [[1500] * 16]


class FailingWritableSerial:
    def write(self, payload: bytes) -> int:
        return -1

    def errorString(self) -> str:
        return "dummy serial error"


def test_worker_transmit_does_not_emit_packet_sent_on_failed_write():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [1500] * 16
    processor.serial = FailingWritableSerial()
    processor.error = DummySignal()
    processor.packet_sent = DummySignal()
    processor._tx_enabled = True
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    result = processor.send_current_packet()

    assert result.startswith("Error:")
    assert processor.packet_sent.emitted == []
    assert processor.error.emitted


class DummyTimer:
    def __init__(self):
        self.stopped = False
        self.started_with = []

    def stop(self):
        self.stopped = True

    def start(self, interval):
        self.started_with.append(interval)

    def isActive(self):
        return bool(self.started_with) and not self.stopped


def test_set_transmission_enabled_false_stops_worker_timer():
    timer = DummyTimer()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor._tx_timer = timer

    processor.set_transmission_enabled(False)

    assert processor._tx_enabled is False
    assert timer.stopped is True


def test_update_channels_and_enable_refreshes_before_starting_timer():
    timer = DummyTimer()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.error = DummySignal()
    processor._tx_enabled = False
    processor._tx_timer = timer
    processor._tx_interval_ms = 4

    result = processor.update_channels_and_enable([172, 1811, 1000])

    assert result == "Good"
    assert processor.channels[:3] == [172, 1811, 1000]
    assert processor.channels[3:] == [1500] * 13
    assert processor._tx_enabled is True
    assert timer.started_with == [4]
