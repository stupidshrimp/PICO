from enum import IntEnum
import struct
import serial
from serial.tools import list_ports

class CRSFPacketProcessor:
    CRSF_SYNC = 0xC8

    class PacketsTypes(IntEnum):
        RC_CHANNELS_PACKED = 0x16

    def __init__(self, port, baudrate=921600, channels=None, telemetry_callback=None):
        """Initialize the CRSFPacketProcessor.

        Parameters
        ----------
        port : str
            Serial port (e.g., ``'COM3'``).
        baudrate : int, optional
            Baudrate for serial communication (default ``921600``).
        channels : list, optional
            Initial channel values (up to 16 channels). Defaults to ``1500`` for
            all channels.
        telemetry_callback : callable, optional
            Function invoked when telemetry packets are decoded. The callback is
            called with ``(packet_type, *values)`` where ``packet_type`` is a
            string identifier such as ``"attitude"`` or ``"gps"``.
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
        self.telemetry_callback = telemetry_callback

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
                # Disable verbose packet transmission debug output to focus on telemetry
                # print(f"Packet sent: {packet.hex()} | Channels: {self.channels}")
                return "Good"  # Return "Good" only if transmission is successful
            except Exception as e:
                print(f"Failed to send packet: {e}")
                return f"Error: {e}"

        # If the serial port is not available, return an error
        return "Error"


    def read_serial_data(self):
        """Read available telemetry data and decode known packet types."""
        if self.serial and self.serial.in_waiting > 0:
            try:
                data = self.serial.read(self.serial.in_waiting)
                if len(data) < 3:
                    return
                packet_type = data[2]
                # Ignore parameter settings packets
                if packet_type == 0x3A:
                    return
                print(f"\n--- Received Telemetry Packet (Type: {packet_type:#04x}) ---")
                if packet_type == 0x14:
                    self.decode_link_statistics(data)
                elif packet_type == 0x02:
                    self.decode_gps(data)
                elif packet_type == 0x08:
                    self.decode_battery(data)
                elif packet_type == 0x1E:
                    self.decode_attitude(data)
                elif packet_type == 0xF0:
                    self.decode_custom(data)
                else:
                    print("Unknown telemetry packet:", data.hex())
            except Exception as e:
                print(f"Error reading serial data: {e}")


    def decode_link_statistics(self, data):
        """Decode link statistics telemetry packet and print the info."""
        # Link statistics packets contain ten bytes of payload following the type
        # field.  The fields are defined in the CRSF protocol specification as:
        #   RSSI1, RSSI2, Link quality, SNR, Active antenna, RF mode,
        #   TX power, Downlink RSSI, Downlink link quality, Downlink SNR
        if len(data) < 13:
            print("Link statistics packet too short.")
            return
        try:
            (
                rssi_a,
                rssi_b,
                link_quality,
                snr,
                active_antenna,
                rf_mode,
                tx_power_enum,
                downlink_rssi,
                downlink_lq,
                downlink_snr,
            ) = struct.unpack("=bbBbBBBbBb", data[3:13])

            # Map TX power enumeration to milliwatts as defined by the protocol.
            tx_power_map = [0, 10, 25, 100, 500, 1000, 2000, 250]
            tx_power = tx_power_map[tx_power_enum] if tx_power_enum < len(tx_power_map) else tx_power_enum

            print("--- Link Statistics ---")
            print(f"RSSI A: {rssi_a} dBm")
            print(f"RSSI B: {rssi_b} dBm")
            print(f"Link Quality: {link_quality}%")
            print(f"SNR: {snr} dB")
            print(f"Active Antenna: {active_antenna}")
            print(f"RF Mode: {rf_mode}")
            print(f"TX Power: {tx_power} mW")
            print(f"Downlink RSSI: {downlink_rssi} dBm")
            print(f"Downlink Link Quality: {downlink_lq}%")
            print(f"Downlink SNR: {downlink_snr} dB")
        except Exception as e:
            print("Error decoding link statistics:", e)


    def decode_gps(self, data):
        """Decode a CRSF GPS telemetry packet.

        The CRSF protocol reports ground speed in km/h. Convert this value
        to miles per hour before invoking the telemetry callback.
        """
        if len(data) < 18:
            print("GPS packet too short.")
            return
        try:
            payload = data[3:18]
            lat_raw, lon_raw, speed, course, alt, sats = struct.unpack(
                ">iiHHHB", payload
            )
            lat = lat_raw / 1000000.0
            lon = lon_raw / 1000000.0
            speed_kmh = speed
            speed_mph = speed_kmh * 0.621371
            course = course / 10.0
            alt = alt / 10.0
            if self.telemetry_callback:
                self.telemetry_callback("gps", lat, lon, speed_mph, course, alt, sats)
            print("--- Decoded GPS Telemetry ---")
            print(f"Latitude: {lat}")
            print(f"Longitude: {lon}")
            print(f"Speed: {speed_mph} mph")
            print(f"Ground Course: {course}")
            print(f"Altitude: {alt}")
            print(f"Satellites: {sats}")
        except Exception as e:
            print("Error decoding GPS data:", e)


    def decode_battery(self, data):
        """Decode battery telemetry packet and print the info."""
        if len(data) < 9:
            print("Battery sensor packet too short.")
            return
        try:
            voltage, current, capacity = struct.unpack("<HHH", data[3:9])
            print("--- Battery Sensor ---")
            print(f"Battery Voltage: {voltage / 1000.0} V")
            print(f"Battery Current: {current / 100.0} A")
            print(f"Capacity Used: {capacity} mAh")
        except Exception as e:
            print("Error decoding battery sensor data:", e)


    def decode_attitude(self, data):
        """Decode an attitude telemetry packet (0x1E) and print roll, pitch and yaw."""
        if len(data) < 9:
            print("Attitude packet too short.")
            return
        try:
            pitch, roll, yaw = struct.unpack(">hhh", data[3:9])
            pitch /= 10
            roll /= 10
            yaw /= 10
            if self.telemetry_callback:
                self.telemetry_callback("attitude", pitch, roll, yaw)
            print("--- Attitude Data (0x1E) ---")
            print(f"Pitch: {pitch}")
            print(f"Roll:  {roll}")
            print(f"Yaw:   {yaw}")
        except Exception as e:
            print("Error decoding attitude data:", e)


    def decode_custom(self, data):
        """Decode a custom telemetry packet (0xF0) with 16 bytes of data."""
        if len(data) < 20:
            print("Custom telemetry packet too short.")
            return
        try:
            payload = data[3:19]
            crc = data[19]
            print("--- Custom Telemetry Packet (0xF0) ---")
            print("Payload (16 bytes):", payload.hex())
            print("CRC:", hex(crc))
        except Exception as e:
            print("Error decoding custom telemetry packet:", e)


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

