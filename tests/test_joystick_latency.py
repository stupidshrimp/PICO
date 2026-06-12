import pathlib
import sys
import threading
from queue import Queue

import serial as pyserial

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pico_modules.pico_joystick2state import JoystickRawHandler, _SerialReader


class _DummySignal:
    def __init__(self):
        self.emitted = []

    def emit(self, payload):
        self.emitted.append(payload)


class _FakeSerial:
    def __init__(self, on_readline, is_open=True, in_waiting=1):
        self._on_readline = on_readline
        self.is_open = is_open
        self.in_waiting = in_waiting

    def readline(self):
        return self._on_readline()


def _reader_with_serial(stop, fake_serial):
    reader = _SerialReader.__new__(_SerialReader)
    reader.serial_connection = fake_serial
    reader._stop = stop
    reader.data_queue = Queue(maxsize=8)
    reader.button_queue = Queue()
    reader.error = _DummySignal()
    return reader


def _handler_with_queue(*lines, smoothing=0):
    handler = JoystickRawHandler.__new__(JoystickRawHandler)
    handler.data_queue = Queue(maxsize=8)
    handler.button_queue = Queue()
    for line in lines:
        handler.data_queue.put_nowait(line)
    handler.roll = 512
    handler.pitch = 512
    handler.deadzone = 0
    handler.sensitivity = 100
    handler.smoothing = smoothing
    return handler


def test_get_raw_values_uses_latest_valid_joystick_sample_only():
    handler = _handler_with_queue("X=0 Y=0", "button=pressed", "X=1023 Y=1023")

    pitch, roll = handler.get_raw_values()

    assert pitch == 1023
    assert roll == 1023
    assert handler.data_queue.empty()


def test_get_raw_values_ignores_invalid_lines_without_losing_last_state():
    handler = _handler_with_queue("noise", "button=pressed")

    pitch, roll = handler.get_raw_values()

    assert pitch == 512
    assert roll == 512
    assert handler.data_queue.empty()


def test_get_raw_values_buffers_button_events_between_axis_samples():
    handler = _handler_with_queue(
        "X=100 Y=200",
        "Button 13 PRESSED",
        "Button 14 RELEASED",
        "X=300 Y=400",
    )

    pitch, roll = handler.get_raw_values()

    assert pitch == 400
    assert roll == 300
    assert handler.consume_button_events() == [(13, True), (14, False)]
    assert handler.button_states == {13: True, 14: False}
    assert handler.consume_button_events() == []


def test_button_event_parsing_is_case_insensitive():
    handler = _handler_with_queue("button 1 pressed", "BUTTON 15 RELEASED")

    pitch, roll = handler.get_raw_values()

    assert pitch == 512
    assert roll == 512
    assert handler.consume_button_events() == [(1, True), (15, False)]


def test_serial_reader_keeps_button_edges_out_of_droppable_axis_queue():
    data_queue = Queue(maxsize=2)
    button_queue = Queue()
    reader = _SerialReader.__new__(_SerialReader)
    reader.data_queue = data_queue
    reader.button_queue = button_queue

    reader._queue_raw_line("X=1 Y=1")
    reader._queue_raw_line("Button 14 PRESSED")
    reader._queue_raw_line("X=2 Y=2")
    reader._queue_raw_line("X=3 Y=3")
    reader._queue_raw_line("Button 14 RELEASED")

    assert list(button_queue.queue) == [(14, True), (14, False)]
    assert list(data_queue.queue) == ["X=2 Y=2", "X=3 Y=3"]


def test_serial_reader_suppresses_error_during_intentional_stop():
    # close() sets _stop and then closes the port, which interrupts a blocking
    # readline().  The reader must treat that as a clean shutdown and not emit an
    # error (which would trip the cut-throttle failsafe during a normal reselect).
    stop = threading.Event()

    def on_readline():
        stop.set()
        raise pyserial.SerialException("port closed")

    reader = _reader_with_serial(stop, _FakeSerial(on_readline))
    reader.run()

    assert reader.error.emitted == []


def test_serial_reader_emits_error_on_unexpected_loss():
    # An unexpected disconnect (stop not set) is a genuine joystick loss and must
    # be surfaced so handle_worker_error can cut throttle and alarm.
    stop = threading.Event()

    def on_readline():
        raise pyserial.SerialException("device unplugged")

    reader = _reader_with_serial(stop, _FakeSerial(on_readline))
    reader.run()

    assert reader.error.emitted
