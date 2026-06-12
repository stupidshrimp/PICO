import pathlib
import sys
import time

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
    processor._raw_serial_debug_enabled = True

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



def test_raw_serial_data_not_emitted_when_debug_disabled():
    frame = _build_handset_frame(1000, 200)

    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.serial = DummySerial(frame)
    processor.serial_data = DummySignal()
    processor.telemetry_ready = DummySignal()
    processor.error = DummySignal()
    processor._rx_buffer = bytearray()
    processor._raw_serial_debug_enabled = False

    processor.read_serial_data()

    assert processor.serial_data.emitted == []
    assert processor.telemetry_ready.emitted, "Telemetry should still decode with raw debug disabled"


def test_raw_serial_debug_flag_can_be_toggled():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)

    processor.set_raw_serial_debug_enabled(True)
    assert processor._raw_serial_debug_enabled is True

    processor.set_raw_serial_debug_enabled(False)
    assert processor._raw_serial_debug_enabled is False

def test_decode_compact_handset_payload():
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


class _DummyConnectable:
    """Stand-in for a Qt signal exposing connect/disconnect."""

    def connect(self, *args):
        pass

    def disconnect(self, *args):
        pass


class DummyDroppableSerial:
    """Writable serial whose write fails and that tracks close()."""

    def __init__(self, write_result: int = -1):
        self.write_result = write_result
        self.writes = []
        self.closed = False
        self._open = True
        self.readyRead = _DummyConnectable()

    def write(self, payload: bytes) -> int:
        self.writes.append(payload)
        return self.write_result

    def errorString(self) -> str:
        return "device removed"

    def isOpen(self) -> bool:
        return self._open

    def close(self):
        self.closed = True
        self._open = False


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


def test_reconnect_state_initialised_before_worker_thread_start():
    source = pathlib.Path("pico_modules/pico_transmitpackets.py").read_text()
    processor_start = source.index("class CRSFPacketProcessor")
    worker_start = source.index("self._thread.start()", processor_start)

    assert source.index("self._last_port_check = 0.0", processor_start) < worker_start
    assert source.index("self._last_reconnect_attempt = 0.0", processor_start) < worker_start


def test_read_single_parameter_assembles_chunked_text_selection():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.parameter_query_update = DummySignal()

    field_id = 7
    full_payload = (
        bytes([0, 9])
        + b"Telem Ratio\x00"
        + b"Std;Off;1:128;1:64;1:32;1:16;1:8;1:4;1:2;Race\x00"
        + bytes([8, 0, 9, 0])
        + b"\x00"
    )
    chunk_payloads = [full_payload[:24], full_payload[24:50], full_payload[50:]]
    requested_chunks = []

    def read_chunk(requested_field_id, chunk_index):
        requested_chunks.append((requested_field_id, chunk_index))
        if requested_field_id != field_id or chunk_index >= len(chunk_payloads):
            return None
        return {
            "field_id": requested_field_id,
            "chunks_remaining": len(chunk_payloads) - chunk_index - 1,
            "chunk_payload": chunk_payloads[chunk_index],
        }

    processor._read_parameter_chunk = read_chunk

    entry = processor._read_single_parameter(field_id)

    assert requested_chunks == [(field_id, 0), (field_id, 1), (field_id, 2)]
    assert entry["name"] == "Telem Ratio"
    assert entry["value"] == 8
    assert entry["selected"] == "1:2"
    assert entry["options"][-2:] == ["1:2", "Race"]
    assert entry["chunks_remaining"] == 0


def test_golden_fc_to_gs_attitude_frame_decodes_contract_values():
    # FC telemetryWriteAttitude(roll=123 ddeg, pitch=45 ddeg, yaw=900 ddeg)
    # is serialized by CRSF as pitch, roll, yaw signed BE radians*10000.
    frame = bytes.fromhex("c8081efcef08633d5c83")

    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.serial = DummySerial(frame)
    processor.serial_data = DummySignal()
    processor.telemetry_ready = DummySignal()
    processor.error = DummySignal()
    processor._rx_buffer = bytearray()
    processor._raw_serial_debug_enabled = False
    processor._parameter_query_active = False

    processor.read_serial_data()

    assert processor.telemetry_ready.emitted
    packet_type, pitch, roll, yaw = processor.telemetry_ready.emitted[0]
    assert packet_type == "attitude"
    assert pitch == pytest.approx(4.497, abs=0.01)
    assert roll == pytest.approx(12.301, abs=0.01)
    assert yaw == pytest.approx(89.999, abs=0.01)


def test_golden_fc_to_gs_gps_frame_decodes_contract_values():
    # GPS payload: 37.7749, -122.4194, speed raw 724, course 123.45 deg,
    # altitude 100 m (+1000 m CRSF offset), 10 satellites.
    frame = bytes.fromhex("c811021683fe08b708483002d43039044c0a89")

    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.serial = DummySerial(frame)
    processor.serial_data = DummySignal()
    processor.telemetry_ready = DummySignal()
    processor.error = DummySignal()
    processor._rx_buffer = bytearray()
    processor._raw_serial_debug_enabled = False
    processor._parameter_query_active = False

    processor.read_serial_data()

    assert processor.telemetry_ready.emitted == [
        (
            "gps",
            pytest.approx(37.7749),
            pytest.approx(-122.4194),
            pytest.approx(328.084),
            pytest.approx(44.987, abs=0.001),
            pytest.approx(123.45),
            10,
        )
    ]


def test_send_current_packet_skips_write_when_channels_stale():
    # When the GUI thread stops refreshing channels the pacer keeps ticking,
    # but the watchdog must stop writing so the FC's own RC-fresh failsafe can
    # engage instead of replaying the last commanded values forever.
    serial = DummyWritableSerial()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [CRSF_CHANNEL_CENTER] * 16
    processor.serial = serial
    processor.error = DummySignal()
    processor._tx_enabled = True
    processor._channel_stale_timeout_s = 0.2
    # Last fresh channel update is well beyond the watchdog window.
    processor._last_channel_update_perf = time.perf_counter() - 1.0
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    result = processor.send_current_packet()

    assert result == "Stale"
    assert serial.writes == []
    assert processor._tx_stale_skips == 1
    assert processor._diag_tx_stale == 1


def test_send_current_packet_writes_when_channels_fresh():
    serial = DummyWritableSerial()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [CRSF_CHANNEL_CENTER] * 16
    processor.serial = serial
    processor.error = DummySignal()
    processor._tx_enabled = True
    processor._channel_stale_timeout_s = 0.2
    processor._last_channel_update_perf = time.perf_counter()
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    result = processor.send_current_packet()

    assert result == "Good"
    assert len(serial.writes) == 1
    assert processor._tx_stale_skips == 0


def test_zero_stale_timeout_disables_watchdog():
    # A non-positive timeout opts out of the watchdog entirely.
    serial = DummyWritableSerial()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [CRSF_CHANNEL_CENTER] * 16
    processor.serial = serial
    processor.error = DummySignal()
    processor._tx_enabled = True
    processor._channel_stale_timeout_s = 0.0
    processor._last_channel_update_perf = time.perf_counter() - 100.0
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    result = processor.send_current_packet()

    assert result == "Good"
    assert len(serial.writes) == 1


def test_channel_update_marks_channels_fresh():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.error = DummySignal()
    processor._ensure_tx_timer = lambda: None
    processor._channel_stale_timeout_s = 0.2
    processor._last_channel_update_perf = time.perf_counter() - 5.0

    assert processor._channels_are_stale() is True

    processor.update_and_send_packet([CRSF_CHANNEL_CENTER] * 4)

    assert processor._channels_are_stale() is False


def test_update_channels_and_enable_marks_channels_fresh():
    pacer = DummyPacer()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.error = DummySignal()
    processor._tx_enabled = False
    processor._tx_pacer = pacer
    processor._tx_interval_ms = 4
    processor._channel_stale_timeout_s = 0.2
    processor._last_channel_update_perf = time.perf_counter() - 5.0

    processor.update_channels_and_enable([CRSF_CHANNEL_CENTER] * 4)

    assert processor._channels_are_stale() is False


def test_set_transmission_enabled_true_grants_grace_period():
    pacer = DummyPacer()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor._tx_pacer = pacer
    processor._tx_interval_ms = 4
    processor._channel_stale_timeout_s = 0.2
    processor._last_channel_update_perf = time.perf_counter() - 5.0

    processor.set_transmission_enabled(True)

    assert processor._tx_enabled is True
    assert processor._channels_are_stale() is False


def test_golden_gs_to_fc_channel_packet_matches_contract_bytes():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [
        CRSF_CHANNEL_MIN,      # CH1 roll min
        CRSF_CHANNEL_CENTER,   # CH2 pitch center
        CRSF_CHANNEL_MAX,      # CH3 throttle / target max
        400,                   # CH4 yaw low test value
        CRSF_CHANNEL_MAX,      # CH5 ELRS arm keepalive high
        1700,                  # CH6 Fly-By-Wire high
        400,                   # CH7 Manual throttle low
        *([CRSF_CHANNEL_CENTER] * 9),
    ]

    packet = bytes(processor.create_packet())

    assert packet.hex() == "c81816ac00dfc42133715243067ce0031ff8c0073ef0810f7c03"


def test_seconds_since_last_tx_none_until_first_write():
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    assert processor.seconds_since_last_tx() is None


def test_send_current_packet_records_last_write_time():
    serial = DummyWritableSerial()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [CRSF_CHANNEL_CENTER] * 16
    processor.serial = serial
    processor.error = DummySignal()
    processor._tx_enabled = True
    processor._channel_stale_timeout_s = 0.0  # disable staleness watchdog
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    assert processor.seconds_since_last_tx() is None

    assert processor.send_current_packet() == "Good"

    age = processor.seconds_since_last_tx()
    assert age is not None and age >= 0.0


def test_write_failure_drops_serial_for_reconnect():
    # A failed write must clear self.serial so the throttled reconnect path
    # re-opens the port; otherwise TX silently dies after a USB glitch.
    serial = DummyDroppableSerial(write_result=-1)
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.channels = [CRSF_CHANNEL_CENTER] * 16
    processor.serial = serial
    processor.error = DummySignal()
    processor._tx_enabled = True
    processor._channel_stale_timeout_s = 0.0
    processor.check_usb_connection = lambda: True
    processor.is_connected = lambda: True

    result = processor.send_current_packet()

    assert str(result).startswith("Error")
    assert processor.serial is None
    assert serial.closed is True
    assert processor._tx_write_errors == 1
    assert processor.error.emitted  # failure surfaced to the GUI


def test_handle_serial_error_drops_port_on_resource_error():
    from PySide6.QtSerialPort import QSerialPort

    serial = DummyDroppableSerial()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.serial = serial

    processor._handle_serial_error(QSerialPort.SerialPortError.ResourceError)

    assert processor.serial is None
    assert serial.closed is True


def test_handle_serial_error_keeps_port_on_nonfatal_error():
    from PySide6.QtSerialPort import QSerialPort

    serial = DummyDroppableSerial()
    processor = CRSFPacketProcessor.__new__(CRSFPacketProcessor)
    processor.serial = serial

    # A transient write/read-class error is logged but must not drop the port.
    processor._handle_serial_error(QSerialPort.SerialPortError.WriteError)

    assert processor.serial is serial
    assert serial.closed is False
