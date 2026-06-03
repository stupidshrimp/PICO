import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pico_modules.pico_transmitpackets import (
    CRSF_CHANNEL_CENTER,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MIN,
    CRSFPacketProcessor,
    _HighResolutionTransmitPacer,
)


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
    assert processor.channels[3:] == [CRSF_CHANNEL_CENTER] * 13


def test_channel_normalisation_clamps_and_centers_invalid_values():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)

    channels = processor._normalise_channels([0, 2000, None, "bad", 1000])

    assert channels[:5] == [
        CRSF_CHANNEL_MIN,
        CRSF_CHANNEL_MAX,
        CRSF_CHANNEL_CENTER,
        CRSF_CHANNEL_CENTER,
        1000,
    ]
    assert channels[5:] == [CRSF_CHANNEL_CENTER] * 11


def test_worker_transmit_writes_current_packet():
    serial = DummyWritableSerial()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [CRSF_CHANNEL_CENTER] * 16
    processor.serial = serial
    processor.error = DummySignal()
    processor._tx_enabled = True
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    result = processor.send_current_packet()

    assert result == "Good"
    assert len(serial.writes) == 1
    assert serial.writes[0] == bytes(processor.create_packet())


class DummyPacer:
    def __init__(self, active=False):
        self.active = active
        self.stopped = False
        self.start_count = 0
        self.reschedule_count = 0
        self.complete_count = 0

    def stop(self):
        self.stopped = True
        self.active = False

    def start(self):
        self.start_count += 1
        self.active = True

    def reschedule(self):
        self.reschedule_count += 1

    def mark_send_complete(self):
        self.complete_count += 1

    def isActive(self):
        return self.active


def test_set_transmission_enabled_false_stops_worker_timer():
    pacer = DummyPacer(active=True)
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor._tx_pacer = pacer

    processor.set_transmission_enabled(False)

    assert processor._tx_enabled is False
    assert pacer.stopped is True


def test_update_channels_and_enable_refreshes_before_starting_timer():
    pacer = DummyPacer()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.error = DummySignal()
    processor._tx_enabled = False
    processor._tx_pacer = pacer
    processor._tx_interval_ms = 4

    result = processor.update_channels_and_enable([172, 1811, 1000])

    assert result == "Good"
    assert processor.channels[:3] == [172, 1811, 1000]
    assert processor.channels[3:] == [CRSF_CHANNEL_CENTER] * 13
    assert processor._tx_enabled is True
    assert pacer.start_count == 1


def test_packet_interval_change_reschedules_active_pacer():
    pacer = DummyPacer(active=True)
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor._tx_pacer = pacer

    processor.set_packet_interval(8)

    assert processor._tx_interval_ms == 8
    assert pacer.reschedule_count == 1


def test_high_resolution_pacer_waits_until_next_deadline(monkeypatch):
    class StopEvent:
        def __init__(self):
            self.stopped = False

        def is_set(self):
            return self.stopped

        def set(self):
            self.stopped = True

    class RescheduleEvent:
        def __init__(self, stop_event):
            self.stop_event = stop_event
            self.waits = []

        def wait(self, timeout):
            self.waits.append(timeout)
            self.stop_event.set()
            return False

        def clear(self):
            pass

        def set(self):
            pass

    processor = type("DummyProcessor", (), {})()
    processor._tx_interval_ms = 4
    processor._tx_enabled = True
    pacer = _HighResolutionTransmitPacer(processor)
    stop_event = StopEvent()
    reschedule_event = RescheduleEvent(stop_event)
    pacer._stop_event = stop_event
    pacer._reschedule_event = reschedule_event

    times = iter([100.0, 99.996])
    monkeypatch.setattr(
        "pico_modules.pico_transmitpackets.time.perf_counter",
        lambda: next(times),
    )
    def fail_if_send_queued(*args):
        if len(args) > 1 and args[1] == "send_current_packet":
            pytest.fail("pacer should wait instead of busy-polling")
        return True

    monkeypatch.setattr(
        "pico_modules.pico_transmitpackets.QMetaObject.invokeMethod",
        fail_if_send_queued,
    )

    pacer._run()

    assert reschedule_event.waits == [pytest.approx(0.004)]


def test_high_resolution_pacer_queues_send_at_deadline(monkeypatch):
    class StopEvent:
        def __init__(self):
            self.stopped = False

        def is_set(self):
            return self.stopped

        def set(self):
            self.stopped = True

    processor = type("DummyProcessor", (), {})()
    processor._tx_interval_ms = 4
    processor._tx_enabled = True
    pacer = _HighResolutionTransmitPacer(processor)
    stop_event = StopEvent()
    pacer._stop_event = stop_event

    queued = []

    def fake_invoke(*args):
        queued.append(args)
        stop_event.set()
        return True

    times = iter([100.0, 100.0, 100.0])
    monkeypatch.setattr(
        "pico_modules.pico_transmitpackets.time.perf_counter",
        lambda: next(times),
    )
    monkeypatch.setattr(
        "pico_modules.pico_transmitpackets.QMetaObject.invokeMethod",
        fake_invoke,
    )

    pacer._run()

    assert len(queued) == 1
    assert queued[0][0] is processor
    assert queued[0][1] == "send_current_packet"


def test_worker_transmit_marks_pacer_send_complete():
    serial = DummyWritableSerial()
    pacer = DummyPacer()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [CRSF_CHANNEL_CENTER] * 16
    processor.serial = serial
    processor.error = DummySignal()
    processor._tx_enabled = True
    processor._tx_pacer = pacer
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    result = processor.send_current_packet()

    assert result == "Good"
    assert pacer.complete_count == 1


def test_high_resolution_pacer_drops_ticks_while_send_in_flight(monkeypatch):
    class StopEvent:
        def __init__(self, limit):
            self.calls = 0
            self.limit = limit

        def is_set(self):
            self.calls += 1
            return self.calls > self.limit

        def set(self):
            self.calls = self.limit + 1

    processor = type("DummyProcessor", (), {})()
    processor._tx_interval_ms = 4
    processor._tx_enabled = True
    pacer = _HighResolutionTransmitPacer(processor)
    pacer._stop_event = StopEvent(limit=4)

    queued = []

    def fake_invoke(*args):
        queued.append(args)
        # Deliberately do not call mark_send_complete(); this simulates a slow
        # worker-thread serial write/reconnect that has not serviced the queued
        # call yet. Additional elapsed deadlines should be dropped/coalesced.
        return True

    times = iter(
        [100.0, 100.0, 100.0, 100.004, 100.008, 100.012, 100.016, 100.020, 100.024]
    )
    monkeypatch.setattr(
        "pico_modules.pico_transmitpackets.time.perf_counter",
        lambda: next(times),
    )
    monkeypatch.setattr(
        "pico_modules.pico_transmitpackets.QMetaObject.invokeMethod",
        fake_invoke,
    )

    pacer._run()

    assert len(queued) == 1


def test_high_resolution_pacer_can_queue_after_send_completion():
    processor = type("DummyProcessor", (), {})()
    processor._tx_interval_ms = 4
    processor._tx_enabled = True
    pacer = _HighResolutionTransmitPacer(processor)

    assert pacer._claim_send_slot() is True
    assert pacer._claim_send_slot() is False

    pacer.mark_send_complete()

    assert pacer._claim_send_slot() is True


def test_mark_send_complete_does_not_reschedule_deadline():
    processor = type("DummyProcessor", (), {})()
    processor._tx_interval_ms = 4
    processor._tx_enabled = True
    pacer = _HighResolutionTransmitPacer(processor)
    pacer._send_in_flight = True

    class RescheduleEvent:
        def __init__(self):
            self.set_count = 0

        def set(self):
            self.set_count += 1

    reschedule_event = RescheduleEvent()
    pacer._reschedule_event = reschedule_event

    pacer.mark_send_complete()

    assert pacer._send_in_flight is False
    assert reschedule_event.set_count == 0
