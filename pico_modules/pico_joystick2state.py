import re
import serial
import threading
import time
from queue import Queue


class JoystickRawHandler:
    """Read raw joystick values over a serial connection.

    The microcontroller is expected to stream values in the format
    "X=<value> Y=<value>" or "<value>,<value>" where each value is in the
    range ``0-1023``. The handler normalises the incoming data to this range
    (inverting the Y axis to match typical HID behaviour) and exposes helper
    methods for retrieving the most recent readings or values mapped to the
    ``0-2000`` range used by the CRSF protocol. Deadzone and sensitivity can
    be adjusted live.
    """

    def __init__(self, port, baudrate=9600, deadzone=0, sensitivity=100):
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

        # Start background thread to continually read data from the serial port
        self.reading_thread = threading.Thread(target=self._read_joystick_data, daemon=True)
        self.reading_thread.start()

    def set_deadzone(self, percent):
        self.deadzone = max(0, min(100, int(percent)))

    def set_sensitivity(self, percent):
        self.sensitivity = max(1, int(percent))

    # ------------------------------------------------------------------
    # Serial helpers
    # ------------------------------------------------------------------
    def _read_joystick_data(self):
        """Continuously read raw lines from the serial connection."""
        while True:
            try:
                if self.serial_connection.is_open and self.serial_connection.in_waiting > 0:
                    raw = self.serial_connection.readline().decode("utf-8").strip()
                    self.data_queue.put(raw)
                else:
                    time.sleep(0.05)
            except serial.SerialException as exc:
                print(f"Serial connection error: {exc}")
                break
            except Exception as exc:  # pragma: no cover - serial read errors
                print(f"Error reading serial data: {exc}")

    def connect_serial(self):
        """Reconnect the serial port if it has been closed."""
        if not self.serial_connection.is_open:
            self.serial_connection.open()

    def close_serial(self):
        if self.serial_connection.is_open:
            self.serial_connection.close()

    # ------------------------------------------------------------------
    # Data processing
    # ------------------------------------------------------------------
    @staticmethod
    def _normalise(value):
        """Constrain ``value`` to the 0-1023 HID range."""
        return max(0, min(1023, int(value)))

    def _parse_line(self, raw_line):
        """Parse a single line of serial data into normalised roll/pitch."""
        if "," in raw_line:
            x, y = map(int, raw_line.split(","))
        else:
            match = re.search(r"X\s*=\s*(\d+)\s*Y\s*=\s*(\d+)", raw_line)
            if not match:
                raise ValueError
            x, y = map(int, match.groups())

        joy_x = self._normalise(x)
        joy_y = 1023 - self._normalise(y)  # Invert Y axis
        return joy_x, joy_y

    def get_raw_values(self):
        """Return the most recent normalised pitch and roll values."""
        while not self.data_queue.empty():
            raw_line = self.data_queue.get()
            try:
                self.roll, self.pitch = self._parse_line(raw_line)
            except ValueError:
                print(f"Malformed data: {raw_line}")

        return self.pitch, self.roll  # pitch first for consistency with callers

    def get_mapped_values(self):
        """Map raw values to the 0-2000 range used by CRSF."""
        pitch, roll = self.get_raw_values()

        def apply_settings(value):
            center = 512
            delta = value - center
            deadzone_range = (self.deadzone / 100) * 512
            if abs(delta) < deadzone_range:
                delta = 0
            delta *= self.sensitivity / 100
            adjusted = center + delta
            return max(0, min(1023, int(adjusted)))

        roll = apply_settings(roll)
        pitch = apply_settings(pitch)

        roll_mapped = int(roll * (2000 / 1023))
        pitch_mapped = int(pitch * (2000 / 1023))
        return roll_mapped, pitch_mapped


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
        joystick.close_serial()
        print("Serial connection closed.")

