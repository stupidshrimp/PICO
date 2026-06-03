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
