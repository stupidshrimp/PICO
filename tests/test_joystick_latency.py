import pathlib
import sys
import threading
from queue import Queue

import serial

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pico_modules.pico_joystick2state import JoystickRawHandler, _SerialReader


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


class _RaisingFromInWaitingSerial:
    """Fake serial port whose ``in_waiting`` raises on the first poll, the way
    pyserial reports a USB unplug. ``readline`` is never reached on this path."""

    is_open = True

    @property
    def in_waiting(self):
        raise serial.SerialException("device reports readiness to read but returned no data")

    def readline(self):  # pragma: no cover - should never be called
        raise AssertionError("readline must not run once in_waiting has failed")


def _run_reader_to_completion(serial_connection):
    captured = []
    reader = _SerialReader.__new__(_SerialReader)
    reader.serial_connection = serial_connection
    reader._stop = threading.Event()
    reader.data_queue = Queue(maxsize=8)
    reader.button_queue = Queue()
    # ``error`` is a Qt Signal on the real class; stub it so run() works headless.
    reader.error = type("_Sig", (), {"emit": lambda self, msg: captured.append(msg)})()
    reader.run()
    return captured


def test_serial_unplug_via_in_waiting_triggers_joystick_loss_failsafe():
    # main.py keys its joystick-loss failsafe (centre roll/pitch, cut throttle)
    # on the substring "Serial connection error". A disconnect raised from
    # in_waiting must surface under that exact wording so the failsafe engages
    # instead of leaving frozen stick commands in flight.
    messages = _run_reader_to_completion(_RaisingFromInWaitingSerial())

    assert len(messages) == 1
    assert "Serial connection error" in messages[0]
