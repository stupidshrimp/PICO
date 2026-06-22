import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from config import resolve_port_selection


def test_keeps_active_port_when_still_present():
    # A live joystick/transmitter that is still plugged in must not be disturbed.
    assert resolve_port_selection("COM3", "COM3", ["COM3", "COM4"]) == (
        "keep",
        "COM3",
    )


def test_drops_link_and_remembers_port_on_unplug():
    # The selected device vanished from the OS list -> disconnect, but the
    # caller keeps "COM3" as the desired port for a later automatic reconnect.
    assert resolve_port_selection("COM3", "COM3", ["COM4"]) == ("disconnect", None)


def test_reconnects_to_remembered_port_on_replug():
    # After an unplug the dropdown sits at "Not connected" while the desired
    # port is remembered.  When that port reappears we reconnect automatically.
    assert resolve_port_selection("Not connected", "COM3", ["COM3", "COM4"]) == (
        "reconnect",
        "COM3",
    )


def test_stays_disconnected_while_remembered_port_absent():
    assert resolve_port_selection("Not connected", "COM3", ["COM4"]) == (
        "disconnect",
        None,
    )


def test_no_reconnect_when_user_intentionally_disconnected():
    # If the operator explicitly selected "Not connected", there is no desired
    # port to restore, so a newly appearing device must not be auto-selected.
    assert resolve_port_selection(
        "Not connected", "Not connected", ["COM3"]
    ) == ("disconnect", None)


def test_unplug_of_other_device_does_not_steal_focus():
    # Desired port is still present even though some other port disappeared.
    assert resolve_port_selection("COM3", "COM3", ["COM3"]) == ("keep", "COM3")
