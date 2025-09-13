import re
import serial
import threading
from queue import Queue
from PySide6.QtCore import QObject, Signal, QThread


class _SerialReader(QThread):
    """Background thread that pulls lines from the serial port."""

    error = Signal(str)

    def __init__(self, serial_connection, stop_event, data_queue):
        super().__init__()
        self.serial_connection = serial_connection
        self._stop = stop_event
        self.data_queue = data_queue

    def run(self):
        try:
            while not self._stop.is_set():
                if self.serial_connection.is_open and self.serial_connection.in_waiting > 0:
                    try:
                        raw = self.serial_connection.readline().decode("utf-8").strip()
                    except (serial.SerialException, OSError) as exc:
                        self.error.emit(f"Serial connection error: {exc}")
                        break
                    self.data_queue.put(raw)
                else:
                    self.msleep(50)
        except Exception as exc:  # pragma: no cover - serial read errors
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

    def __init__(self, port, baudrate=9600, deadzone=0, sensitivity=100):
        super().__init__()
        try:
            self.serial_connection = serial.Serial(port, baudrate=baudrate, timeout=1)
            print(f"Connected to joystick on {port} at {baudrate} baud.")
        except serial.SerialException as e:  # pragma: no cover - serial init
            raise RuntimeError(f"Failed to connect to joystick on {port}: {e}")

        self.data_queue = Queue()
        self.roll = 512
        self.pitch = 512
        self.deadzone = deadzone  # percent
        self.sensitivity = sensitivity  # percent

        # Event used to signal the reading thread to stop
        self._stop = threading.Event()

        # Start background QThread to continually read data from the serial port
        self.reading_thread = _SerialReader(self.serial_connection, self._stop, self.data_queue)
        self.reading_thread.error.connect(self.error)
        self.reading_thread.start()

    def set_deadzone(self, percent):
        self.deadzone = max(0, min(100, int(percent)))

    def set_sensitivity(self, percent):
        self.sensitivity = max(1, int(percent))

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
        if self.reading_thread.isRunning():
            self.reading_thread.wait()
        if self.serial_connection.is_open:
            self.serial_connection.close()

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

    def get_raw_values(self):
        """Return the most recent raw pitch and roll values."""
        while not self.data_queue.empty():
            raw_line = self.data_queue.get()
            try:
                self.roll, self.pitch = self._parse_line(raw_line)
                print(f"Flight stick raw X={self.roll} Y={self.pitch}")
            except ValueError:
                # Ignore unrelated lines such as button events
                continue

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

