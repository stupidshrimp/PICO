from enum import IntEnum
import struct
import logging
import time
from PySide6.QtCore import QObject, Signal, QIODevice, QThread, Slot, QMetaObject, Qt

from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo

logger = logging.getLogger(__name__)

class CRSFPacketProcessor(QObject):
    """Process CRSF packets and emit telemetry via Qt signals."""

    CRSF_SYNC = 0xC8
    TELEMETRY_SYNC = 0xEA  # Start byte used for telemetry frames

    telemetry_ready = Signal(object)
    channel_update = Signal(list)
    error = Signal(str)

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
        # Buffer for incoming telemetry bytes.  Telemetry packets can be
        # fragmented or arrive in bursts.  A persistent buffer lets us decode
        # every packet instead of only the first one in each serial read.
        self._rx_buffer = bytearray()

        # Move telemetry processing off the GUI thread.  The processor lives in
        # its own QThread where serial I/O and packet decoding occur so video
        # rendering remains responsive even when telemetry arrives rapidly.
        self._thread = QThread()
        self.moveToThread(self._thread)
        self._thread.started.connect(self.connect_serial)
        self.channel_update.connect(self.update_and_send_packet)
        self._thread.start()

        # Track the last time we queried the OS for available serial ports.
        # Enumerating ports on Windows is relatively expensive and, on some
        # systems, can even trigger access-violation crashes if performed in
        # rapid succession from multiple threads.  Throttling these checks to
        # at most once per second avoids the problematic behaviour while still
        # providing timely detection of disconnects.
        self._last_port_check = 0.0
        self._port_check_interval = 1.0  # seconds

    @Slot()
    def connect_serial(self):
        """Attempt to connect to the specified serial port in the worker thread."""
        try:
            self.serial = QSerialPort(self.serial_port)
            self.serial.setBaudRate(self.baudrate)
            self.serial.setDataBits(QSerialPort.Data8)
            self.serial.setParity(QSerialPort.NoParity)
            self.serial.setStopBits(QSerialPort.OneStop)
            self.serial.setFlowControl(QSerialPort.NoFlowControl)
            self.serial.setReadBufferSize(4096)
            self.serial.readyRead.connect(self.read_serial_data)
            self.serial.errorOccurred.connect(self._handle_serial_error)
            if self.serial.open(QIODevice.ReadWrite):
                print(
                    f"Connected to {self.serial_port} at {self.baudrate} baud."
                )
            else:
                logger.error(
                    "Failed to open serial port: %s", self.serial.errorString()
                )
                self.error.emit(f"Failed to open serial port: {self.serial.errorString()}")
                self.serial = None
        except Exception as e:
            logger.exception("Failed to open serial port")
            self.error.emit(f"Failed to open serial port: {e}")
            self.serial = None

    @Slot(QSerialPort.SerialPortError)
    def _handle_serial_error(self, err):
        """Log serial port errors, ignoring harmless NoError signals."""
        if err == QSerialPort.SerialPortError.NoError:
            return
        logger.error("Serial error: %s (%s)", err, self.serial.errorString())

    @Slot(result=bool)
    def is_connected(self):
        """
        Checks if the serial connection is active and open.
        Returns:
            bool: True if the serial connection is open, False otherwise.
        """
        return self.serial and self.serial.isOpen()

    @Slot(result=bool)
    def check_usb_connection(self):
        """
        Checks if the specified USB device is connected by scanning available ports.
        Returns:
            bool: True if the USB device is connected, False otherwise.
        """
        # To avoid repeated calls into ``QSerialPortInfo.availablePorts`` we only
        # enumerate ports if a minimum interval has elapsed since the previous
        # check.  Using Qt's own enumeration avoids crashes observed with
        # ``serial.tools.list_ports`` on Windows when called from worker
        # threads.
        now = time.monotonic()
        if now - self._last_port_check < self._port_check_interval:
            return True

        self._last_port_check = now
        try:
            available_ports = [p.portName() for p in QSerialPortInfo.availablePorts()]
        except Exception as exc:  # pragma: no cover - platform dependent
            logger.warning("Failed to enumerate serial ports: %s", exc)
            return True

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

    @Slot(list)
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
            logger.warning("Serial port not connected. Attempting to reconnect...")
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
                logger.exception("Failed to send packet")
                self.error.emit(f"Failed to send packet: {e}")
                return f"Error: {e}"

        # If the serial port is not available, return an error
        return "Error"


    @Slot()
    def read_serial_data(self):
        """Read available telemetry data and decode all received packets."""
        if not self.serial or self.serial.bytesAvailable() <= 0:
            return

        try:
            # Read all currently available bytes and append them to the buffer.
            # Keeping the buffering logic centralised allows for easy decoding
            # without producing verbose serial output.
            new_data = bytes(self.serial.readAll())
            if new_data:
                self._rx_buffer.extend(new_data)
                # Prevent unbounded growth if we fall behind
                MAX_BUF = 8192
                if len(self._rx_buffer) > MAX_BUF:
                    logger.warning(
                        "RX buffer overflow (%d). Dropping to resync.",
                        len(self._rx_buffer),
                    )
                    self._rx_buffer = self._rx_buffer[-512:]

            # Process packets while a complete frame is present in the buffer
            while True:
                # Need at least sync, length and type
                if len(self._rx_buffer) < 3:
                    break

                # Discard bytes until a valid device address is found.  CRSF
                # telemetry frames use 0xEA while outbound channel frames use
                # 0xC8.  Accept either so we can decode telemetry regardless of
                # source.
                if self._rx_buffer[0] not in (self.CRSF_SYNC, self.TELEMETRY_SYNC):
                    del self._rx_buffer[0]
                    continue

                length = self._rx_buffer[1]

                # Drop frames with impossible lengths (need at least type+crc = 2)
                # and cap the maximum size to 64 bytes.
                if length < 2 or length > 64:
                    del self._rx_buffer[0]
                    continue

                frame_end = length + 2  # sync + length + payload + crc


                # Wait for the rest of the frame if it's not all here yet
                if len(self._rx_buffer) < frame_end:
                    break

                # Verify the CRC before decoding.  If it doesn't match we
                # discard only the sync byte and try again, which helps us
                # resynchronise with the stream when bytes are dropped.
                if (
                    self.crc8_data(self._rx_buffer[2:frame_end - 1])
                    != self._rx_buffer[frame_end - 1]
                ):
                    # CRC mismatch means the packet is corrupt and cannot be parsed
                    del self._rx_buffer[0]
                    continue


                # Extract complete packet and remove from buffer
                packet = bytes(self._rx_buffer[:frame_end])
                del self._rx_buffer[:frame_end]

                packet_type = packet[2]

                # Ignore parameter setting packets (0x3A)
                if packet_type == 0x3A:
                    continue

                # Telemetry packets are processed without verbose debug logging

                if packet_type == 0x14:
                    self.decode_link_statistics(packet)
                elif packet_type == 0x02:
                    self.decode_gps(packet)
                elif packet_type == 0x08:
                    self.decode_battery(packet)
                elif packet_type == 0x1E:
                    self.decode_attitude(packet)
                elif packet_type == 0xF0:
                    self.decode_custom(packet)
                else:
                    # Unknown packet type encountered
                    pass

        except Exception as e:
            logger.exception("Error reading serial data")
            self.error.emit(f"Error reading serial data: {e}")


    def decode_link_statistics(self, data):
        """Decode link statistics telemetry packet and emit the info."""
        if len(data) < 13:
            logger.warning("Link statistics packet too short")
            return
        if data[1] < 12:
            logger.warning("Link stats length byte unexpected: %d", data[1])
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
        except Exception:
            logger.exception("Failed to parse link statistics packet")


    def decode_gps(self, data):
        """Decode a CRSF GPS telemetry packet."""
        if len(data) < 18:
            logger.warning("GPS packet too short")
            return
        if data[1] < 17:
            logger.warning("GPS length byte unexpected: %d", data[1])
            return
        try:
            payload = data[3:18]
            lat_raw, lon_raw, speed, course, alt, sats = struct.unpack(
                ">iiHHHB", payload
            )
            lat = lat_raw / 1000000.0
            lon = lon_raw / 1000000.0
            speed_mph = float(speed)
            course = course / 10.0
            alt_m = alt / 10.0
            alt_ft = alt_m * 3.28084
            self.telemetry_ready.emit(
                ("gps", lat, lon, speed_mph, course, alt_ft, sats)
            )
        except Exception:
            logger.exception("Failed to parse GPS packet")


    def decode_battery(self, data):
        """Decode battery telemetry packet."""
        if len(data) < 9:
            logger.warning("Battery packet too short")
            return
        if data[1] < 8:
            logger.warning("Battery length byte unexpected: %d", data[1])
            return
        try:
            voltage, current, capacity = struct.unpack("<HHH", data[3:9])
            # Decoded values are currently unused but parsing is retained
            # to validate packet structure.
        except Exception:
            logger.exception("Failed to parse battery packet")


    def decode_attitude(self, data):
        """Decode an attitude telemetry packet (0x1E)."""
        if len(data) < 9:
            logger.warning("Attitude packet too short")
            return
        if data[1] < 8:
            logger.warning("Attitude length byte unexpected: %d", data[1])
            return
        try:
            pitch, roll, yaw = struct.unpack(">hhh", data[3:9])
            pitch /= 10
            roll /= 10
            yaw /= 10
            self.telemetry_ready.emit(("attitude", pitch, roll, yaw))
        except Exception:
            logger.exception("Failed to parse attitude packet")


    def decode_custom(self, data):
        """Decode a custom telemetry packet (0xF0) with 16 bytes of data."""
        if len(data) < 20:
            logger.warning("Custom telemetry packet too short")
            return
        if data[1] < 18:
            logger.warning("Custom telemetry length byte unexpected: %d", data[1])
            return
        try:
            payload = data[3:19]
            crc = data[19]
            # Custom telemetry data is parsed but not emitted.
        except Exception:
            logger.exception("Failed to parse custom telemetry packet")


    @Slot()
    def close_serial(self):
        """Close the serial port in the worker thread."""
        try:
            if self.serial:
                try:
                    self.serial.readyRead.disconnect(self.read_serial_data)
                except Exception:
                    pass
                if self.serial.isOpen():
                    self.serial.close()
        finally:
            self.serial = None

    def __del__(self):  # pragma: no cover - defensive finaliser
        try:
            QMetaObject.invokeMethod(self, "close_serial", Qt.QueuedConnection)
            if hasattr(self, "_thread") and self._thread.isRunning():
                self._thread.quit()
                self._thread.wait()
        except Exception:
            pass

