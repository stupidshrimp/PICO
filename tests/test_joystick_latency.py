import pathlib
import sys
from queue import Queue

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from pico_modules.pico_joystick2state import JoystickRawHandler


def _handler_with_queue(*lines, smoothing=0):
    handler = JoystickRawHandler.__new__(JoystickRawHandler)
    handler.data_queue = Queue(maxsize=8)
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
