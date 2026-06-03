from enum import IntEnum
import struct
import logging
import time
import math
import threading
from PySide6.QtCore import (
    QObject,
    Signal,
    QIODevice,
    QThread,
    Slot,
    QMetaObject,
    Qt,
)

from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo

logger = logging.getLogger(__name__)

CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_MAX = 1811
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_COUNT = 16


class _HighResolutionTransmitPacer:
    """Wake at CRSF frame deadlines without busy-polling the Qt event loop."""

    def __init__(self, processor):
        self._processor = processor
        self._stop_event = threading.Event()
        self._reschedule_event = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._send_in_flight = False

    def isActive(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self.isActive():
            self.reschedule()
            return

        self._stop_event.clear()
        self._reschedule_event.clear()
        with self._lock:
            self._send_in_flight = False
        self._thread = threading.Thread(
            target=self._run,
            name="CRSFTransmitPacer",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._reschedule_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None
        self.mark_send_complete()

    def reschedule(self):
        self._reschedule_event.set()

    def mark_send_complete(self):
        with self._lock:
            self._send_in_flight = False

    def _claim_send_slot(self):
        with self._lock:
            if self._send_in_flight:
                return False
            self._send_in_flight = True
            return True

    def _run(self):
        next_deadline = time.perf_counter()

        while not self._stop_event.is_set():
            interval_s = max(0.001, self._processor._tx_interval_ms / 1000.0)
            now = time.perf_counter()
            wait_s = next_deadline - now

            if wait_s > 0:
                if self._reschedule_event.wait(wait_s):
                    self._reschedule_event.clear()
                    next_deadline = time.perf_counter()
                continue

            if not self._processor._tx_enabled:
                return

            if self._claim_send_slot():
                queued = QMetaObject.invokeMethod(
                    self._processor,
                    "send_current_packet",
                    Qt.QueuedConnection,
                )
                if queued is False:
                    self.mark_send_complete()

            next_deadline += interval_s
            now = time.perf_counter()
            if next_deadline <= now:
                missed_intervals = int((now - next_deadline) / interval_s) + 1
                next_deadline += missed_intervals * interval_s


class CRSFPacketProcessor(QObject):
    """Process CRSF packets and emit telemetry via Qt signals."""

    CRSF_SYNC = 0xC8
    TELEMETRY_SYNC = 0xEA  # Start byte used for telemetry frames

    telemetry_ready = Signal(object)
    serial_data = Signal(object)
    channel_update = Signal(list)
    packet_interval_update = Signal(int)
    transmission_enabled_update = Signal(bool)
    transmission_start_update = Signal(list)
    raw_serial_debug_update = Signal(bool)
    error = Signal(str)

    class PacketsTypes(IntEnum):
        RC_CHANNELS_PACKED = 0x16

    def __init__(
        self,
        port,
        baudrate=921600,
        channels=None,
        packet_interval_ms=4,
        transmission_enabled=True,
        raw_serial_debug_enabled=False,
    ):
        """Initialize the CRSFPacketProcessor.

        Parameters
        ----------
        port : str
            Serial port (e.g., ``'COM3'``).
        baudrate : int, optional
            Baudrate for serial communication (default ``921600``).
        channels : list, optional
            Initial CRSF channel values (up to 16 channels). Defaults to
            ``CRSF_CHANNEL_CENTER`` for all channels; CRSF channel units are
            not servo microseconds.
        packet_interval_ms : int, optional
            Worker-thread transmit cadence in milliseconds. A value of ``4``
            targets the 250 Hz ELRS packet rate without relying on the GUI
            event loop.
        transmission_enabled : bool, optional
            Initial state for periodic RC frame transmission. Pass ``False``
            when the UI transmission control is currently stopped so reconnects
            do not restart RC output implicitly.
        raw_serial_debug_enabled : bool, optional
            Initial state for forwarding raw serial byte chunks to the GUI debug
            page.  Keep disabled during normal operation to avoid unnecessary
            cross-thread signal traffic.
        """
        # Ensure the underlying QObject is initialised so that Qt signals
        # remain valid for the lifetime of this processor.  Without this call
        # ``telemetry_ready.emit`` can raise ``RuntimeError: Signal source has
        # been deleted`` when packets are decoded.
        super().__init__()
        if channels is None:
            channels = [CRSF_CHANNEL_CENTER] * CRSF_CHANNEL_COUNT

        if len(channels) > CRSF_CHANNEL_COUNT:
            raise ValueError(f"Maximum of {CRSF_CHANNEL_COUNT} channels supported.")

        # Pad/clamp provided channels in CRSF units before any writes occur.
        self.channels = self._normalise_channels(channels)

        port_info = QSerialPortInfo(port)
        self.serial_port = port_info.systemLocation() or port
        self.baudrate = baudrate
        self.serial = None  # QSerialPort instance
        self._tx_interval_ms = max(1, int(packet_interval_ms or 4))
        self._tx_enabled = bool(transmission_enabled)
        self._raw_serial_debug_enabled = bool(raw_serial_debug_enabled)
        self._tx_pacer = None
        # Buffer for incoming telemetry bytes.  Telemetry packets can be
        # fragmented or arrive in bursts.  A persistent buffer lets us decode
        # every packet instead of only the first one in each serial read.
        self._rx_buffer = bytearray()

        # The transmit pacer can queue the first send immediately after the
        # worker thread opens the serial port, so every field used by
        # ``send_current_packet`` must exist before ``_thread.start()``.
        self._last_port_check = 0.0
        self._port_check_interval = 1.0  # seconds
        self._last_reconnect_attempt = 0.0
        self._reconnect_interval = 1.0  # seconds

        # Move telemetry processing off the GUI thread.  The processor lives in
        # its own QThread where serial I/O and packet decoding occur so video
        # rendering remains responsive even when telemetry arrives rapidly.
        self._thread = QThread()
        self.moveToThread(self._thread)
        self._thread.started.connect(self.connect_serial)
        self.channel_update.connect(self.update_and_send_packet)
        self.packet_interval_update.connect(self.set_packet_interval)
        self.transmission_enabled_update.connect(self.set_transmission_enabled)
        self.transmission_start_update.connect(self.update_channels_and_enable)
        self.raw_serial_debug_update.connect(self.set_raw_serial_debug_enabled)
        self._thread.start()


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
            self.serial.setReadBufferSize(65536)
            self.serial.readyRead.connect(self.read_serial_data)
            self.serial.errorOccurred.connect(self._handle_serial_error)
            if self.serial.open(QIODevice.ReadWrite):
                print(
                    f"Connected to {self.serial_port} at {self.baudrate} baud."
                )
                self._ensure_tx_timer()
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
            available_ports = [p.systemLocation() for p in QSerialPortInfo.availablePorts()]
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

    def _normalise_channels(self, new_channels):
        """Return exactly 16 CRSF channels, truncating or padding as needed."""
        if len(new_channels) > CRSF_CHANNEL_COUNT:
            logger.warning(
                "Received %d channel values; truncating to %d",
                len(new_channels),
                CRSF_CHANNEL_COUNT,
            )
            new_channels = new_channels[:CRSF_CHANNEL_COUNT]

        normalised = []
        for value in new_channels:
            try:
                channel = int(value)
            except (TypeError, ValueError):
                channel = CRSF_CHANNEL_CENTER
            normalised.append(
                max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, channel))
            )

        return normalised + [CRSF_CHANNEL_CENTER] * (
            CRSF_CHANNEL_COUNT - len(normalised)
        )

    def _ensure_tx_timer(self):
        """Create/start the bounded high-resolution transmit pacer.

        Desktop millisecond timers can collapse a requested 4 ms interval to
        the host scheduler quantum (about 15-16 ms on common Windows
        configurations), which matches the observed ~62 Hz control-frame rate.
        Use a small pacing thread that sleeps until each perf-counter deadline
        and queues the actual serial write back onto this object's Qt worker
        thread.  This preserves QSerialPort thread affinity without leaving a
        zero-interval QTimer busy-polling between packets.  The pacer allows
        only one queued send at a time, so slow writes or reconnect attempts
        drop/coalesce ticks instead of building a backlog of stale RC frames.
        """
        if self._tx_pacer is None:
            self._tx_pacer = _HighResolutionTransmitPacer(self)

        if self._tx_enabled and not self._tx_pacer.isActive():
            self._tx_pacer.start()

    @Slot(int)
    def set_packet_interval(self, interval_ms):
        """Update the worker-thread transmit cadence."""
        try:
            self._tx_interval_ms = max(1, int(interval_ms))
        except (TypeError, ValueError):
            self._tx_interval_ms = 4

        if self._tx_pacer and self._tx_pacer.isActive():
            self._tx_pacer.reschedule()

    @Slot(bool)
    def set_transmission_enabled(self, enabled):
        """Enable or disable periodic RC frame transmission."""
        self._tx_enabled = bool(enabled)
        if not self._tx_enabled:
            if self._tx_pacer:
                self._tx_pacer.stop()
            return
        self._ensure_tx_timer()

    @Slot(list)
    def update_and_send_packet(self, new_channels):
        """Update channel values; periodic transmission is handled in this thread."""
        try:
            self.channels = self._normalise_channels(new_channels)
            self._ensure_tx_timer()
            return "Good"
        except Exception as exc:  # Ensure worker thread stays alive
            logger.exception("Exception in update_and_send_packet")
            self.error.emit(f"Failed to update channels: {exc}")
            return f"Error: {exc}"

    @Slot(list)
    def update_channels_and_enable(self, new_channels):
        """Refresh channels before enabling periodic RC frame transmission."""
        try:
            self.channels = self._normalise_channels(new_channels)
            self._tx_enabled = True
            self._ensure_tx_timer()
            return "Good"
        except Exception as exc:  # Ensure worker thread stays alive
            logger.exception("Exception in update_channels_and_enable")
            self.error.emit(f"Failed to start transmission: {exc}")
            return f"Error: {exc}"

    @Slot(bool)
    def set_raw_serial_debug_enabled(self, enabled):
        """Enable or disable forwarding raw serial chunks to the GUI debug tab."""
        self._raw_serial_debug_enabled = bool(enabled)

    @Slot()
    def send_current_packet(self):
        """Write the latest channel packet at the configured ELRS cadence."""
        try:
            if not self._tx_enabled:
                return "Disabled"

            # Check USB connection. This call is internally throttled so the
            # 250 Hz transmit loop does not enumerate ports on every tick.
            if not self.check_usb_connection():
                return "Error"

            if not self.is_connected():
                now = time.monotonic()
                if now - self._last_reconnect_attempt < self._reconnect_interval:
                    return "Error"
                self._last_reconnect_attempt = now
                logger.warning(
                    "Serial port not connected. Attempting to reconnect..."
                )
                self.connect_serial()
                if not self.is_connected():
                    return "Error"

            if self.serial:
                packet = self.create_packet()
                bytes_written = self.serial.write(bytes(packet))
                if bytes_written == -1:
                    raise IOError(self.serial.errorString())
                return "Good"

            return "Error"

        except Exception as exc:
            logger.exception("Failed to send packet")
            self.error.emit(f"Failed to send packet: {exc}")
            return f"Error: {exc}"
        finally:
            tx_pacer = getattr(self, "_tx_pacer", None)
            if tx_pacer:
                tx_pacer.mark_send_complete()


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
                if getattr(self, "_raw_serial_debug_enabled", False):
                    try:
                        self.serial_data.emit(new_data)
                    except Exception:
                        logger.debug("Serial data signal emit failed", exc_info=True)
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

                # ``frame_end`` already accounts for the CRC byte at
                # ``frame_end - 1``.  Everything between the packet type and the
                # CRC forms the payload regardless of the envelope type.
                payload = packet[3 : frame_end - 1]

                if packet_type == 0x14:
                    self._decode_link_statistics_payload(payload)
                    continue

                # Treat any other recognised telemetry envelopes the same way
                # regardless of whether they arrived as standalone frames or as
                # piggybacked payload bytes from a link-stats packet.
                if not self._decode_payload(packet_type, payload):
                    # Unknown packet type encountered
                    pass

        except Exception as e:
            logger.exception("Error reading serial data")
            self.error.emit(f"Error reading serial data: {e}")


    def _decode_link_statistics_payload(self, payload: bytes | memoryview) -> None:
        """Decode link statistics and forward any embedded telemetry."""

        if len(payload) < 10:
            logger.warning("Link statistics payload too short: %d", len(payload))
            return

        stats_view = memoryview(payload)
        stats_bytes = stats_view[:10]

        try:
            (
                rssi_a,
                rssi_b,
                link_quality,
                snr,
                _active_antenna,
                _rf_mode,
                _tx_power_enum,
                downlink_rssi,
                downlink_lq,
                downlink_snr,
            ) = struct.unpack("=bbBbBBBbBb", stats_bytes)
        except Exception:
            logger.exception("Failed to parse link statistics packet")
            return

        remaining = stats_view[10:]
        piggyback_count = 0
        if remaining:
            piggyback_count = self._count_piggyback_packets(remaining)

        self.telemetry_ready.emit(
            (
                "link_stats",
                rssi_a,
                rssi_b,
                link_quality,
                snr,
                downlink_lq,
                downlink_snr,
                piggyback_count,
            )
        )

        # Any remaining bytes are piggybacked telemetry that should be decoded
        # as if they arrived in a dedicated telemetry frame.
        if remaining:
            self._decode_telemetry_stream(remaining.tobytes())

    def _count_piggyback_packets(self, payload: memoryview) -> int:
        """Return the number of recognised telemetry packets in ``payload``."""

        index = 0
        length = len(payload)
        count = 0
        while index < length:
            packet_type = payload[index]
            index += 1
            consumed = self._decode_payload(packet_type, payload[index:], emit=False)
            if not consumed:
                break
            index += consumed
            count += 1
        return count


    def _decode_telemetry_stream(self, payload: bytes) -> None:
        """Decode one or more telemetry packets from ``payload``.

        Telemetry data piggybacked inside link-stat frames is laid out as a
        sequence of ``(packet_type, packet_payload)`` pairs.  Each payload has
        a fixed size for the packet types we understand, allowing the decoder to
        walk the stream and emit the embedded telemetry events.
        """

        index = 0
        payload_len = len(payload)
        while index < payload_len:
            packet_type = payload[index]
            index += 1
            consumed = self._decode_payload(packet_type, payload[index:])
            if not consumed:
                logger.debug(
                    "Piggyback telemetry type 0x%02X could not be parsed", packet_type
                )
                return
            index += consumed


    def _decode_payload(
        self, packet_type: int, payload: bytes | memoryview, *, emit: bool = True
    ) -> int:
        """Decode a single telemetry payload.

        Parameters
        ----------
        packet_type:
            Telemetry packet type identifier (e.g. ``0x1E`` for attitude).
        payload:
            Byte sequence immediately following the packet type.  The decoder
            consumes only the bytes needed for the recognised telemetry format
            and returns how many payload bytes were consumed.  Zero indicates
            the packet type was not recognised.
        """

        view = memoryview(payload)

        if packet_type == 0x02:  # GPS
            needed = 15
            if len(view) < needed:
                logger.warning(
                    "GPS payload too short: expected %d bytes, got %d",
                    needed,
                    len(view),
                )
                return 0

            try:
                lat_raw, lon_raw, spd_raw, crs_raw, alt_raw, sats = struct.unpack(
                    ">iiHHHB", view[:15]
                )
            except Exception:
                logger.exception("Failed to unpack GPS payload")
                return 0

            lat = lat_raw / 1e7
            lon = lon_raw / 1e7
            speed_mph = spd_raw * 0.0621371  # km/h -> mph
            # Altitude is encoded in meters with a +1000 m offset in the CRSF
            # GPS packet. The firmware provides altitude in centimeters and the
            # CRSF implementation converts it to meters before adding the
            # offset (encoded_altitude = (alt_cm / 100) + 1000). Subtract 1000
            # to recover meters and convert to feet for the UI.
            alt_ft = (alt_raw - 1000) * 3.28084
            course = crs_raw / 100.0

            if emit:
                self.telemetry_ready.emit(
                    ("gps", lat, lon, alt_ft, speed_mph, course, sats)
                )
            return needed

        if packet_type == 0x08:  # Battery
            minimum = 6
            if len(view) < minimum:
                logger.warning(
                    "Battery payload too short: expected at least %d bytes, got %d",
                    minimum,
                    len(view),
                )
                return 0

            try:
                voltage_raw, current_raw, capacity = struct.unpack("<HHH", view[:6])
            except Exception:
                logger.exception("Failed to unpack battery payload")
                return 0

            percent = float(view[6]) if len(view) > 6 else None

            voltage = (voltage_raw + 5) / 10.0
            current = current_raw / 10.0

            if percent is not None:
                if emit:
                    self.telemetry_ready.emit(
                        ("battery", voltage, current, capacity, percent)
                    )
                return 7

            if emit:
                self.telemetry_ready.emit(("battery", voltage, current, capacity))
            return 6

        if packet_type == 0x1E:  # Attitude
            needed = 6
            if len(view) < needed:
                logger.warning(
                    "Attitude payload too short: expected %d bytes, got %d",
                    needed,
                    len(view),
                )
                return 0

            try:
                pitch_raw, roll_raw, yaw_raw = struct.unpack(">hhh", view[:6])
            except Exception:
                logger.exception("Failed to unpack attitude payload")
                return 0

            roll = math.degrees(roll_raw / 10000.0)
            pitch = math.degrees(-pitch_raw / 10000.0)
            yaw = math.degrees(yaw_raw / 10000.0)

            if emit:
                self.telemetry_ready.emit(("attitude", pitch, roll, yaw))
            return needed

        if packet_type == 0x3A:  # Handset timing synchronisation
            payload_size = 9
            dest = orig = None
            start = 0

            # Extended frames prepend destination and origin addresses to the
            # payload.  Piggybacked telemetry omits these two bytes, so consume
            # them only when present.
            if len(view) >= payload_size + 2:
                dest = int(view[0])
                orig = int(view[1])
                start = 2

            if len(view) < start + payload_size:
                logger.warning(
                    "Handset payload too short: expected at least %d bytes, got %d",
                    start + payload_size,
                    len(view),
                )
                return 0

            handset_view = view[start : start + payload_size]

            subtype = int(handset_view[0])
            try:
                rate_raw, offset_raw = struct.unpack(">II", handset_view[1:9])
            except Exception:
                logger.exception("Failed to unpack handset payload")
                return 0

            if emit:
                self.telemetry_ready.emit(
                    ("handset_timing", subtype, rate_raw, offset_raw, dest, orig)
                )

            return start + payload_size

        if packet_type == 0xF0:  # Custom telemetry
            needed = 16
            if len(view) < needed:
                logger.warning(
                    "Custom telemetry payload too short: expected %d bytes, got %d",
                    needed,
                    len(view),
                )
                return 0

            # Custom telemetry is currently not emitted, but consuming the
            # payload keeps the parser synchronised with any subsequent bytes.
            return needed

        return 0


    @Slot()
    def close_serial(self):
        """Close the serial port in the worker thread."""
        try:
            if self._tx_pacer:
                self._tx_pacer.stop()
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

