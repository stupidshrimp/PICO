from enum import IntEnum
import struct
from PySide6.QtCore import QObject, Signal, QIODevice
from PySide6.QtSerialPort import QSerialPort
from serial.tools import list_ports

class CRSFPacketProcessor(QObject):
    """Process CRSF packets and emit telemetry via Qt signals."""

    CRSF_SYNC = 0xC8

    telemetry_ready = Signal(object)

    class PacketsTypes(IntEnum):
        RC_CHANNELS_PACKED = 0x16

    def __init__(self, port, baudrate=921600, channels=None):
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
        """
        # Ensure the underlying QObject is initialised so that Qt signals
        # remain valid for the lifetime of this processor.  Without this call
        # ``telemetry_ready.emit`` can raise ``RuntimeError: Signal source has
        # been deleted`` when packets are decoded.
        super().__init__()
        if channels is None:
            channels = [1500] * 16  # Default channel values

        if len(channels) > 16:
            raise ValueError("Maximum of 16 channels supported.")

        # Pad the provided channels to 16 if fewer are given
        self.channels = channels + [1500] * (16 - len(channels))

        self.serial_port = port
        self.baudrate = baudrate
        self.serial = None  # QSerialPort instance
        self._rx_buffer = bytearray()

        self.connect_serial()

    def connect_serial(self):
        """
        Attempts to connect to the specified serial port.
        If the connection fails, it sets self.serial to None.
        """
        try:
            self.serial = QSerialPort(self.serial_port)
            self.serial.setBaudRate(self.baudrate)
            self.serial.readyRead.connect(self.read_serial_data)
            if self.serial.open(QIODevice.ReadWrite):
                print(f"Connected to {self.serial_port} at {self.baudrate} baud.")
            else:
                print(f"Failed to open serial port: {self.serial.errorString()}")
                self.serial = None
        except Exception as e:
            print(f"Failed to open serial port: {e}")
            self.serial = None

    def is_connected(self):
        """
        Checks if the serial connection is active and open.
        Returns:
            bool: True if the serial connection is open, False otherwise.
        """
        return self.serial and self.serial.isOpen()
              
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
        """Update channel values and send the CRSF packet if the connection is valid."""
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
                self.serial.write(bytes(packet))
                # Disable verbose packet transmission debug output to focus on telemetry
                # print(f"Packet sent: {packet.hex()} | Channels: {self.channels}")
                return "Good"  # Return "Good" only if transmission is successful
            except Exception as e:
                print(f"Failed to send packet: {e}")
                return f"Error: {e}"

        # If the serial port is not available, return an error
        return "Error"


    def read_serial_data(self):
        """Read all available telemetry data and decode every packet in the buffer."""
        if not (self.serial and self.serial.bytesAvailable() > 0):
            return

        try:
            # Accumulate new bytes in a persistent buffer
            self._rx_buffer.extend(self.serial.readAll())

            # Process all complete frames in the buffer
            while True:
                # Minimum frame is address + length + type + CRC (payload can be 1 byte)
                if len(self._rx_buffer) < 5:
                    break

                # Synchronise to the CRSF sync byte
                if self._rx_buffer[0] != self.CRSF_SYNC:
                    self._rx_buffer.pop(0)
                    continue

                frame_len = self._rx_buffer[1]
                total_len = frame_len + 2  # include sync and length fields
                if len(self._rx_buffer) < total_len:
                    # Wait for more data
                    break

                frame = bytes(self._rx_buffer[:total_len])
                del self._rx_buffer[:total_len]

                packet_type = frame[2]
                if packet_type == 0x3A:
                    continue  # Ignore parameter settings packets

                print(f"\n--- Received Telemetry Packet (Type: {packet_type:#04x}) ---")
                if packet_type == 0x14:
                    self.decode_link_statistics(frame)
                elif packet_type == 0x02:
                    self.decode_gps(frame)
                elif packet_type == 0x08:
                    self.decode_battery(frame)
                elif packet_type == 0x1E:
                    self.decode_attitude(frame)
                elif packet_type == 0xF0:
                    self.decode_custom(frame)
                else:
                    print("Unknown telemetry packet:", frame.hex())
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

            self.telemetry_ready.emit(
                (
                    "link_stats",
                    rssi_a,
                    rssi_b,
                    link_quality,
                    snr,
                    downlink_lq,
                    downlink_snr,
                )
            )
        except Exception as e:
            print("Error decoding link statistics:", e)


    def decode_gps(self, data):
        """Decode a CRSF GPS telemetry packet.

        The telemetry packet provides the aircraft's ground speed and
        altitude.  Earlier versions of this ground station assumed the speed
        was reported in kilometres per hour and applied a conversion factor
        to miles per hour, which resulted in the displayed value being roughly
        half of the actual speed.  The incoming ``speed`` value is already in
        miles per hour, so we simply treat it as such.  The altitude is
        transmitted in decimeters, which we convert to feet before invoking
        the telemetry callback.
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
            # ``speed`` is provided in miles per hour by the telemetry source.
            # Using it directly avoids double-conversion that previously
            # halved the displayed value.
            speed_mph = float(speed)
            course = course / 10.0
            alt_m = alt / 10.0
            alt_ft = alt_m * 3.28084
            self.telemetry_ready.emit(
                ("gps", lat, lon, speed_mph, course, alt_ft, sats)
            )
            print("--- Decoded GPS Telemetry ---")
            print(f"Latitude: {lat}")
            print(f"Longitude: {lon}")
            print(f"Speed: {speed_mph} mph")
            print(f"Ground Course: {course}")
            print(f"Altitude: {alt_ft} ft")
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
            self.telemetry_ready.emit(("attitude", pitch, roll, yaw))
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
        if self.serial and self.serial.isOpen():
            self.serial.close()
            print("Serial port closed.")

    def __del__(self):
        """
        Ensure the serial port is closed on object deletion.
        """
        self.close_serial()

