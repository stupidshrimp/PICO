from enum import IntEnum
import serial
from PyQt5.QtCore import QTimer
from serial.tools import list_ports

class CRSFPacketProcessor:
    CRSF_SYNC = 0xC8

    class PacketsTypes(IntEnum):
        RC_CHANNELS_PACKED = 0x16

    def __init__(self, port, baudrate=921600, channels=None):
        """
        Initializes the CRSFPacketProcessor.
        Args:
            port (str): Serial port (e.g., 'COM3').
            baudrate (int): Baudrate for serial communication (default 921600).
            channels (list): Initial channel values (up to 16 channels). Defaults to 1500 for all channels.
        """
        if channels is None:
            channels = [1500] * 16  # Default channel values

        if len(channels) > 16:
            raise ValueError("Maximum of 16 channels supported.")

        # Pad the provided channels to 16 if fewer are given
        self.channels = channels + [1500] * (16 - len(channels))

        self.serial_port = port
        self.baudrate = baudrate
        self.serial = None  # Initialize serial connection as None

        self.connect_serial()

    def connect_serial(self):
        """
        Attempts to connect to the specified serial port.
        If the connection fails, it sets self.serial to None.
        """
        try:
            self.serial = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            print(f"Connected to {self.serial_port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            self.serial = None

    def is_connected(self):
        """
        Checks if the serial connection is active and open.
        Returns:
            bool: True if the serial connection is open, False otherwise.
        """
        return self.serial and self.serial.is_open 
              
    def check_usb_connection(self):
        """
        Checks if the specified USB device is connected by scanning available ports.
        Returns:
            bool: True if the USB device is connected, False otherwise.
        """
        available_ports = [port.device for port in list_ports.comports()]
        return self.serial_port in available_ports
    
  
    
    @staticmethod
    def crc8_dvb_s2(crc, a):
        """
        Calculate CRC8 DVB-S2 checksum for a single byte.
        """
        crc ^= a
        for _ in range(8):
            crc = (crc << 1) ^ 0xD5 if crc & 0x80 else crc << 1
        return crc & 0xFF

    @staticmethod
    def crc8_data(data):
        """
        Calculate the CRC8 checksum for a sequence of data.
        """
        crc = 0
        for a in data:
            crc = CRSFPacketProcessor.crc8_dvb_s2(crc, a)
        return crc

    @staticmethod
    def pack_crsf_to_bytes(channels):
        """
        Pack up to 16 CRSF channel values into bytes.
        """
        result = bytearray()
        dest_shift = 0
        new_val = 0
        for ch in channels:
            new_val |= (ch << dest_shift) & 0xFF
            result.append(new_val)
            src_bits_left = 11 - 8 + dest_shift
            new_val = ch >> (11 - src_bits_left)
            if src_bits_left >= 8:
                result.append(new_val & 0xFF)
                new_val >>= 8
                src_bits_left -= 8
            dest_shift = src_bits_left
        return result

    def create_packet(self):
        """
        Create a CRSF channel packet from the provided channels.
        """
        result = bytearray([self.CRSF_SYNC, 24, self.PacketsTypes.RC_CHANNELS_PACKED])  # Sync, length, type
        result += self.pack_crsf_to_bytes(self.channels)
        result.append(self.crc8_data(result[2:]))  # Append CRC
        return result

    def update_and_send_packet(self, new_channels):
        """
        Update channel values and send the CRSF packet if the connection is valid.
        
        Args:
            new_channels (list): New channel values (up to 16 channels).
        Returns:
            str: Status message indicating success or error.
        """
        if len(new_channels) > 16:
            raise ValueError("Maximum of 16 channels supported.")

        # Update channel values and pad to 16 channels if necessary
        self.channels = new_channels + [1500] * (16 - len(new_channels))

        # Check USB connection
        if not self.check_usb_connection():
            #print("USB device disconnected.")
            return "Error"

        # Check serial connection
        if not self.is_connected():
            print("Serial port not connected. Attempting to reconnect...")
            self.connect_serial()
            if not self.is_connected():
                return "Error"

        # If connected, attempt to transmit packets
        if self.serial:
            try:
                packet = self.create_packet()
                self.serial.write(packet)
                print(f"Packet sent: {packet.hex()} | Channels: {self.channels}")
                return "Good"  # Return "Good" only if transmission is successful
            except Exception as e:
                print(f"Failed to send packet: {e}")
                return f"Error: {e}"

        # If the serial port is not available, return an error
        return "Error"


    def close_serial(self):
        """
        Close the serial port.
        """
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial port closed.")

    def __del__(self):
        """
        Ensure the serial port is closed on object deletion.
        """
        self.close_serial()


# Example usage:
if __name__ == "__main__":
    # Initialize with COM port and default channel values
    processor = CRSFPacketProcessor(port="COM3", channels=[1000, 1500, 2000])

    # Send packets periodically
    timer = QTimer()
    timer.timeout.connect(processor.send_packet)
    timer.start(10)  # Send 100hz

    # Update channels as needed
    processor.update_and_send_packet([1200, 1400, 1800])
