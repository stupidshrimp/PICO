import serial
import time
from queue import Queue
import threading
import math


class JoystickRateHandler:
    def __init__(
        self,
        port,
        baudrate=9600,
        update_interval=0.01,
        roll_rate_max=100,
        pitch_rate_max=100,
        roll_sensitivity=1.0,
        pitch_sensitivity=1.0,
        roll_exponent=1.0,
        pitch_exponent=1.0,
        dead_zone=8,  # Dead zone threshold (percentage of the full range, e.g., 10%)
    ):
        """
        Initialize the rate joystick handler for serial communication with user-defined sensitivity, exponential response, and a dead zone.
        """
        try:
            self.serial_connection = serial.Serial(port, baudrate=baudrate, timeout=1)
            print(f"Connected to joystick on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to connect to joystick on {port}: {e}")

        self.update_interval = update_interval
        self.roll_rate_max = roll_rate_max
        self.pitch_rate_max = pitch_rate_max
        self.roll_sensitivity = roll_sensitivity
        self.pitch_sensitivity = pitch_sensitivity
        self.roll_exponent = roll_exponent
        self.pitch_exponent = pitch_exponent
        self.dead_zone = dead_zone

        self.roll = 0.0  # Current roll angle in degrees
        self.pitch = 0.0  # Current pitch angle in degrees
        self.last_time = time.time()

        # Queue to store incoming joystick data
        self.data_queue = Queue()

        # Start a thread to continuously read joystick data
        self.reading_thread = threading.Thread(target=self.read_joystick_data, daemon=True)
        self.reading_thread.start()

    def read_joystick_data(self):
        """
        Continuously read data from the serial port and add it to the queue.
        """
        while True:
            if self.serial_connection.in_waiting > 0:
                try:
                    raw_data = self.serial_connection.readline().decode('utf-8').strip()
                    self.data_queue.put(raw_data)
                except Exception as e:
                    print(f"Error reading serial data: {e}")

    def read_joystick_input(self):
        """
        Get joystick X and Y values from the queue and apply sensitivity, exponential adjustments, and dead zone filtering.
        """
        if not self.data_queue.empty():
            try:
                raw_data = self.data_queue.get()
                x, y = map(int, raw_data.split(","))

                # Dead zone implementation
                dead_zone_threshold = self.dead_zone / 100 * 127  # Dead zone threshold for joystick range (0-127)
                x_centered = x - 127  # Center X around 0
                y_centered = y - 127  # Center Y around 0

                if abs(x_centered) < dead_zone_threshold:
                    roll_rate = 0.0
                else:
                    roll_rate = (
                        math.copysign(
                            abs(x_centered / 127) ** self.roll_exponent,
                            x_centered,
                        )
                        * self.roll_rate_max
                        * self.roll_sensitivity
                    )

                if abs(y_centered) < dead_zone_threshold:
                    pitch_rate = 0.0
                else:
                    pitch_rate = (
                        math.copysign(
                            abs(y_centered / 127) ** self.pitch_exponent,
                            y_centered,
                        )
                        * self.pitch_rate_max
                        * self.pitch_sensitivity
                    )

                return roll_rate, pitch_rate
            except ValueError:
                print(f"Malformed data: {raw_data}")
        return 0.0, 0.0

    def update_angles(self):
        """
        Update roll and pitch angles based on joystick input rates.
        """
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        # Cap delta_time to avoid excessive jumps
        delta_time = min(delta_time, 0.05)  # Max interval of 50ms

        # Get roll and pitch rates
        roll_rate, pitch_rate = self.read_joystick_input()

        # Integrate rates to calculate angles
        self.roll += roll_rate * delta_time
        self.pitch += pitch_rate * delta_time

        # Wrap roll angle to keep it within -180 to 180 degrees
        if self.roll > 180:
            self.roll -= 360
        elif self.roll < -180:
            self.roll += 360

        # Clamp pitch angle to avoid unrealistic values (optional)
        self.pitch = max(-90, min(90, self.pitch))  # Assuming pitch range is -90° to 90°

        # Print updated angles
        # print(
        #     f"Raw Rates: Roll Rate={roll_rate:.2f}, Pitch Rate={pitch_rate:.2f} | "
        #     f"Angles: Roll={self.roll:.2f}°, Pitch={self.pitch:.2f}°"
        # )
        return self.pitch, self.roll   
    
    def update_mapped_angles(self):
        """
        Update roll and pitch angles based on joystick input rates.
        Returns roll and pitch in a range of 0 to 2000.
        """
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        # Cap delta_time to avoid excessive jumps
        delta_time = min(delta_time, 0.05)  # Max interval of 50ms

        # Get roll and pitch rates
        roll_rate, pitch_rate = self.read_joystick_input()

        # Integrate rates to calculate angles
        self.roll += roll_rate * delta_time
        self.pitch += pitch_rate * delta_time

        # Wrap roll angle to keep it within -180 to 180 degrees
        if self.roll > 180:
            self.roll -= 360
        elif self.roll < -180:
            self.roll += 360

        # Clamp pitch angle to avoid unrealistic values (optional)
        self.pitch = max(-90, min(90, self.pitch))  # Assuming pitch range is -90° to 90°

        # Map roll and pitch to the range [0, 2000]
        roll_mapped = int((self.roll + 180) * (2000 / 360))  # Map from [-180, 180] to [0, 2000]
        pitch_mapped = int((self.pitch + 90) * (2000 / 180))  # Map from [-90, 90] to [0, 2000]

        # Print updated angles
        #print(
            #f"Raw Rates: Roll Rate={roll_rate:.2f}, Pitch Rate={pitch_rate:.2f} | "
            #f"Angles: Roll={self.roll:.2f}°, Pitch={self.pitch:.2f}° | "
            #f"Mapped: Roll={roll_mapped}, Pitch={pitch_mapped}"
        #)

        # Return the mapped roll and pitch values
        return roll_mapped, pitch_mapped

    def start_listening(self):
        """
        Start listening to joystick inputs and updating angles.
        """
        try:
            while True:
                self.update_mapped_angles()
                time.sleep(self.update_interval)
        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            self.serial_connection.close()
            print("Serial connection closed.")


# Entry point for testing
if __name__ == "__main__":
    # User-defined parameters
    PORT = "COM14"  # Replace with your COM port
    BAUDRATE = 9600
    UPDATE_INTERVAL = 0.01  # 10 ms update interval
    ROLL_SENSITIVITY = .3  # Example: higher sensitivity for roll
    PITCH_SENSITIVITY = .3  # Example: normal sensitivity for pitch
    ROLL_EXPONENT = 1.5  # Example: exponential curve for roll
    PITCH_EXPONENT = 1.5  # Example: gentler exponential curve for pitch

    joystick_handler = JoystickRateHandler(
        port=PORT,
        baudrate=BAUDRATE,
        update_interval=UPDATE_INTERVAL,
        roll_rate_max=100,  # Max roll rate (degrees/second)
        pitch_rate_max=100,  # Max pitch rate (degrees/second)
        roll_sensitivity=ROLL_SENSITIVITY,
        pitch_sensitivity=PITCH_SENSITIVITY,
        roll_exponent=ROLL_EXPONENT,
        pitch_exponent=PITCH_EXPONENT,
    )

    print("Starting joystick listener. Press Ctrl+C to exit.")
    joystick_handler.start_listening()
