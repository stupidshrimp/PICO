import pathlib
import sys
import threading
from queue import Queue

import pytest
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


def test_smoothing_converges_independently_of_poll_rate(monkeypatch):
    # The joystick EMA weight is now rescaled by the elapsed time since the last
    # sample, so the converged value after a fixed wall-clock interval must not
    # depend on how many times get_raw_values() is polled in between. Without the
    # time scaling, polling more often would smooth faster (the original bug).
    import pico_modules.pico_joystick2state as joymod

    clock = {"t": 1000.0}
    monkeypatch.setattr(joymod.time, "monotonic", lambda: clock["t"])

    def converge_over(total_s, steps):
        handler = _handler_with_queue(smoothing=60)
        # Warm-start the smoothing clock so every measured step is time-scaled
        # (the very first sample with no prior timestamp uses the raw weight).
        handler._smoothing_last_update = clock["t"]
        dt = total_s / steps
        for _ in range(steps):
            clock["t"] += dt
            handler.data_queue.put_nowait("X=1023 Y=1023")
            handler.get_raw_values()
        return handler.roll

    coarse = converge_over(0.2, 4)
    fine = converge_over(0.2, 40)

    # Same time constant regardless of poll rate, and partially (not fully)
    # converged from the 512 centre toward the 1023 step input.
    assert coarse == pytest.approx(fine, rel=1e-6)
    assert 512 < coarse < 1023


def test_smoothing_disabled_passes_latest_sample_through(monkeypatch):
    # smoothing=0 must remain an exact pass-through of the newest sample
    # regardless of timing, matching the legacy behaviour relied on elsewhere.
    import pico_modules.pico_joystick2state as joymod

    monkeypatch.setattr(joymod.time, "monotonic", lambda: 1234.0)
    handler = _handler_with_queue("X=1023 Y=1023", smoothing=0)

    pitch, roll = handler.get_raw_values()

    assert pitch == 1023
    assert roll == 1023


def test_smoothing_disabled_passes_through_with_zero_dt(monkeypatch):
    # Regression: with smoothing disabled, a second sample sharing the previous
    # monotonic timestamp (dt == 0) must still pass through rather than being
    # dropped. time_scaled_weight(1.0, 0, ...) returns 0.0 (its dt<=0 guard runs
    # before the full-weight guard), so the disabled case is handled directly.
    import pico_modules.pico_joystick2state as joymod

    monkeypatch.setattr(joymod.time, "monotonic", lambda: 5000.0)  # frozen clock
    handler = _handler_with_queue(smoothing=0)

    handler.data_queue.put_nowait("X=0 Y=0")
    handler.get_raw_values()  # first poll sets _smoothing_last_update
    handler.data_queue.put_nowait("X=1023 Y=1023")
    pitch, roll = handler.get_raw_values()  # dt == 0 against the frozen clock

    assert pitch == 1023
    assert roll == 1023


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
