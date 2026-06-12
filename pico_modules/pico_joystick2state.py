import re
import serial
import threading
from queue import Empty, Full, Queue
from PySide6.QtCore import QObject, Signal, QThread


BUTTON_EVENT_RE = re.compile(r"Button\s+(\d+)\s+(PRESSED|RELEASED)", re.IGNORECASE)


def parse_button_event(raw_line):
    """Return a ``(button, pressed)`` tuple for joystick button event lines."""
    match = BUTTON_EVENT_RE.search(raw_line)
    if not match:
        return None
    return int(match.group(1)), match.group(2).upper() == "PRESSED"


class _SerialReader(QThread):
    """Background thread that pulls lines from the serial port."""

    error = Signal(str)

    def __init__(self, serial_connection, stop_event, data_queue, button_queue):
        super().__init__()
        self.serial_connection = serial_connection
        self._stop = stop_event
        self.data_queue = data_queue
        self.button_queue = button_queue

    def _queue_raw_line(self, raw):
        """Queue button edges losslessly and axis/debug lines through axis coalescing."""
        button_event = parse_button_event(raw)
        if button_event is not None:
            self.button_queue.put_nowait(button_event)
            return

        try:
            self.data_queue.put_nowait(raw)
        except Full:
            try:
                self.data_queue.get_nowait()
            except Empty:
                pass
            try:
                self.data_queue.put_nowait(raw)
            except Full:
                pass

    def run(self):
        try:
            while not self._stop.is_set():
                if self.serial_connection.is_open and self.serial_connection.in_waiting > 0:
                    try:
                        raw = self.serial_connection.readline().decode("utf-8").strip()
                    except (serial.SerialException, OSError) as exc:
                        # close() closes the port to interrupt a blocking read;
                        # suppress the resulting error so an intentional shutdown
                        # is not reported as a joystick loss (which would trip the
                        # cut-throttle failsafe during a normal port reselect).
                        if not self._stop.is_set():
                            self.error.emit(f"Serial connection error: {exc}")
                        break
                    self._queue_raw_line(raw)
                else:
                    self.msleep(5)
        except Exception as exc:  # pragma: no cover - serial read errors
            if not self._stop.is_set():
                self.error.emit(f"Error reading serial data: {exc}")


class JoystickRawHandler(QObject):
    """Read raw joystick values over a serial connection.

    The microcontroller is expected to stream lines similar to the Arduino
    sketch used for the custom joystick, e.g. ``Hall X=123  Y=456``. Raw
    values are already mapped to the ``0-1023`` HID range on the device, so
    the handler simply constrains the numbers and returns them unchanged.
    Use :meth:`get_mapped_values` to convert roll and pitch to the CRSF
    channel range (``172-1811``).
    """

    error = Signal(str)

    def __init__(self, port, baudrate=9600, deadzone=0, sensitivity=100, smoothing=0):
        super().__init__()
        try:
            self.serial_connection = serial.Serial(port, baudrate=baudrate, timeout=1)
            print(f"Connected to joystick on {port} at {baudrate} baud.")
        except serial.SerialException as e:  # pragma: no cover - serial init
            raise RuntimeError(f"Failed to connect to joystick on {port}: {e}")

        # Keep only a small backlog of joystick samples.  Control updates should
        # use the newest stick position rather than replaying stale serial lines
        # after a short burst or GUI hiccup.
        self.data_queue = Queue(maxsize=8)
        # Button edges are intentionally separate from the droppable axis queue so
        # short GUI stalls cannot lose a mode toggle or yaw release.
        self.button_queue = Queue()
        self.roll = 512
        self.pitch = 512
        self.button_states = {}
        self._button_events = []
        self.deadzone = deadzone  # percent
        self.sensitivity = sensitivity  # percent
        self.smoothing = smoothing  # percent

        # Event used to signal the reading thread to stop
        self._stop = threading.Event()

        # Start background QThread to continually read data from the serial port
        self.reading_thread = _SerialReader(
            self.serial_connection, self._stop, self.data_queue, self.button_queue
        )
        self.reading_thread.error.connect(self.error)
        self.reading_thread.start()

    def set_deadzone(self, percent):
        self.deadzone = max(0, min(100, int(percent)))

    def set_sensitivity(self, percent):
        self.sensitivity = max(1, int(percent))

    def set_smoothing(self, percent):
        self.smoothing = max(0, min(100, int(percent)))

    # ------------------------------------------------------------------
    # Serial helpers
    # ------------------------------------------------------------------
    def connect_serial(self):
        """Reconnect the serial port if it has been closed."""
        if not self.serial_connection.is_open:
            self.serial_connection.open()

    def close(self):
        """Stop background reading and close the serial connection."""
        self._stop.set()
        # Close the port first so a blocking readline() returns promptly instead
        # of stalling close() for up to the 1 s read timeout.  The reader thread
        # treats the resulting exception as an intentional shutdown (it checks
        # ``_stop``) and exits without emitting an error.
        if self.serial_connection.is_open:
            try:
                self.serial_connection.close()
            except Exception:
                pass
        if self.reading_thread.isRunning():
            self.reading_thread.wait()

    # ------------------------------------------------------------------
    # Data processing
    # ------------------------------------------------------------------
    def _parse_line(self, raw_line):
        """Parse a single line of serial data into raw roll/pitch values."""
        match = re.search(r"X\s*=\s*(\d+)\s*Y\s*=\s*(\d+)", raw_line)
        if match:
            x, y = map(int, match.groups())
        elif "," in raw_line:
            x, y = map(int, raw_line.split(",", 1))
        else:
            raise ValueError

        return x, y

    def _store_button_event(self, button, pressed):
        """Update button state and append a pending event for consumers."""
        if not hasattr(self, "button_states"):
            self.button_states = {}
        if not hasattr(self, "_button_events"):
            self._button_events = []
        self.button_states[button] = pressed
        self._button_events.append((button, pressed))

    def _parse_button_line(self, raw_line):
        """Parse and store a button event line from the joystick sketch."""
        button_event = parse_button_event(raw_line)
        if button_event is None:
            return False
        self._store_button_event(*button_event)
        return True

    def _drain_button_queue(self):
        """Move losslessly queued serial button edges into local pending events."""
        if not hasattr(self, "button_queue"):
            return
        while True:
            try:
                button, pressed = self.button_queue.get_nowait()
            except Empty:
                break
            self._store_button_event(button, pressed)

    def consume_button_events(self):
        """Return pending joystick button events and clear the event buffer."""
        self._drain_button_queue()
        if not hasattr(self, "_button_events"):
            self._button_events = []
        events = list(self._button_events)
        self._button_events.clear()
        return events

    def _apply_deadzone_sensitivity(self, value):
        """Apply deadzone and sensitivity to a raw axis value."""
        center = 512
        delta = value - center
        max_delta = 512
        dz = self.deadzone / 100 * max_delta
        if abs(delta) <= dz:
            delta = 0
        else:
            sign = 1 if delta > 0 else -1
            delta = sign * ((abs(delta) - dz) / (max_delta - dz) * max_delta)
        delta *= self.sensitivity / 100
        delta = max(-max_delta, min(max_delta, delta))
        return center + delta

    def get_raw_values(self):
        """Return the most recent processed pitch and roll values."""
        latest_sample = None
        while True:
            try:
                raw_line = self.data_queue.get_nowait()
            except Empty:
                break
            try:
                latest_sample = self._parse_line(raw_line)
            except ValueError:
                # Live button lines are routed into ``button_queue`` before this
                # lossy axis queue.  Keep this fallback for tests or callers that
                # inject raw button lines directly into ``data_queue``.
                self._parse_button_line(raw_line)
                continue

        if latest_sample is not None:
            raw_roll, raw_pitch = latest_sample
            proc_roll = self._apply_deadzone_sensitivity(raw_roll)
            proc_pitch = self._apply_deadzone_sensitivity(raw_pitch)
            alpha = 1.0 - (self.smoothing / 100.0)
            self.roll += alpha * (proc_roll - self.roll)
            self.pitch += alpha * (proc_pitch - self.pitch)

        return self.pitch, self.roll  # pitch first for consistency with callers

    @staticmethod
    def _map_to_crsf(value, in_min=0, in_max=1023, out_min=172, out_max=1811):
        """Linearly map ``value`` from one range to another."""
        value = max(in_min, min(in_max, value))
        scale = (out_max - out_min) / (in_max - in_min)
        return int((value - in_min) * scale + out_min)

    def get_mapped_values(self):
        """Return roll and pitch values mapped to the CRSF channel range."""
        pitch, roll = self.get_raw_values()
        mapped_roll = self._map_to_crsf(roll)
        mapped_pitch = self._map_to_crsf(pitch)
        return mapped_roll, mapped_pitch


# ----------------------------------------------------------------------
# Basic manual test
# ----------------------------------------------------------------------
if __name__ == "__main__":  # pragma: no cover - manual test only
    PORT = "COM14"  # Replace with your COM port
    BAUDRATE = 9600

    joystick = JoystickRawHandler(port=PORT, baudrate=BAUDRATE)
    print("Starting joystick listener. Press Ctrl+C to exit.")
    try:
        while True:
            pitch, roll = joystick.get_raw_values()
            print(f"Pitch={pitch} Roll={roll}")
    except KeyboardInterrupt:
        pass
    finally:
        joystick.close()
        print("Serial connection closed.")

