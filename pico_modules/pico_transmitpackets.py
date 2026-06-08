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

            self._processor._tx_pacer_ticks = getattr(self._processor, "_tx_pacer_ticks", 0) + 1
            if self._claim_send_slot():
                queued = QMetaObject.invokeMethod(
                    self._processor,
                    "send_current_packet",
                    Qt.QueuedConnection,
                )
                if queued is False:
                    self.mark_send_complete()
            else:
                self._processor._tx_pacer_coalesced = getattr(self._processor, "_tx_pacer_coalesced", 0) + 1
                self._processor._diag_tx_coalesced = getattr(self._processor, "_diag_tx_coalesced", 0) + 1

            next_deadline += interval_s
            now = time.perf_counter()
            if next_deadline <= now:
                missed_intervals = int((now - next_deadline) / interval_s) + 1
                next_deadline += missed_intervals * interval_s


class CRSFPacketProcessor(QObject):
    """Process CRSF packets and emit telemetry via Qt signals."""

    CRSF_SYNC = 0xC8
    TELEMETRY_SYNC = 0xEA  # Start byte used for telemetry frames

    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
    CRSF_ADDRESS_ELRS_LUA = 0xEF

    CRSF_FRAMETYPE_DEVICE_PING = 0x28
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D


    telemetry_ready = Signal(object)
    serial_data = Signal(object)
    channel_update = Signal(list)
    packet_interval_update = Signal(int)
    transmission_enabled_update = Signal(bool)
    transmission_start_update = Signal(list)
    raw_serial_debug_update = Signal(bool)
    parameter_query_request = Signal()
    diagnostic_enabled_update = Signal(bool)
    transmit_debug_update = Signal(object)
    parameter_query_update = Signal(str)
    link_diagnostics_update = Signal(object)
    safe_shutdown_update = Signal(list, int)
    safe_shutdown_complete = Signal(bool)
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
        self._tx_debug_window_start = time.perf_counter()
        self._tx_debug_last_write_perf = None
        self._tx_pacer_ticks = 0
        self._tx_pacer_coalesced = 0
        self._tx_send_attempts = 0
        self._tx_sent_packets = 0
        self._tx_write_errors = 0
        self._tx_bytes_written = 0
        self._tx_last_interval_ms = None
        self._tx_last_bytes_to_write = 0
        self._parameter_query_active = False
        self._link_diagnostics_enabled = False
        self._diag_window_start = time.perf_counter()
        self._reset_link_diagnostics_window()

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
        self.parameter_query_request.connect(self.query_elrs_parameters)
        self.diagnostic_enabled_update.connect(self.set_link_diagnostics_enabled)
        self.safe_shutdown_update.connect(self.send_safe_shutdown_packets)
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
        self._reset_tx_debug_window()

    @Slot(bool)
    def set_transmission_enabled(self, enabled):
        """Enable or disable periodic RC frame transmission."""
        self._tx_enabled = bool(enabled)
        self._reset_tx_debug_window()
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


    def _reset_link_diagnostics_window(self) -> None:
        self._diag_window_start = time.perf_counter()
        self._diag_rx_bytes = 0
        self._diag_rx_frames = 0
        self._diag_rx_crc_errors = 0
        self._diag_rx_dropped_bytes = 0
        self._diag_rx_invalid_lengths = 0
        self._diag_rx_buffer_overflows = 0
        self._diag_rx_unknown_payloads = 0
        self._diag_rx_max_buffer = 0
        self._diag_frame_counts = {}
        self._diag_tx_attempts = 0
        self._diag_tx_sent = 0
        self._diag_tx_bytes = 0
        self._diag_tx_errors = 0
        self._diag_tx_coalesced = 0

    def _ensure_link_diagnostics_attrs(self) -> None:
        if not hasattr(self, "_link_diagnostics_enabled"):
            self._link_diagnostics_enabled = False
        if not hasattr(self, "_diag_rx_bytes"):
            self._reset_link_diagnostics_window()

    @Slot(bool)
    def set_link_diagnostics_enabled(self, enabled: bool) -> None:
        self._ensure_link_diagnostics_attrs()
        self._link_diagnostics_enabled = bool(enabled)
        self._reset_link_diagnostics_window()
        signal = getattr(self, "link_diagnostics_update", None)
        if signal is not None:
            signal.emit({"enabled": self._link_diagnostics_enabled, "event": "state"})

    def _maybe_emit_link_diagnostics(self, force: bool = False) -> None:
        self._ensure_link_diagnostics_attrs()
        if not self._link_diagnostics_enabled:
            return
        now = time.perf_counter()
        elapsed = now - self._diag_window_start
        if not force and elapsed < 1.0:
            return
        if elapsed <= 0:
            elapsed = 1e-6
        counts = dict(self._diag_frame_counts)
        stats = {
            "enabled": True,
            "window_s": elapsed,
            "rx_bytes_per_s": self._diag_rx_bytes / elapsed,
            "rx_frame_hz": self._diag_rx_frames / elapsed,
            "rx_attitude_hz": counts.get(0x1E, 0) / elapsed,
            "rx_gps_hz": counts.get(0x02, 0) / elapsed,
            "rx_link_stats_hz": counts.get(0x14, 0) / elapsed,
            "rx_device_info_hz": counts.get(self.CRSF_FRAMETYPE_DEVICE_INFO, 0) / elapsed,
            "rx_param_entry_hz": counts.get(self.CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 0) / elapsed,
            "rx_crc_error_hz": self._diag_rx_crc_errors / elapsed,
            "rx_dropped_bytes_per_s": self._diag_rx_dropped_bytes / elapsed,
            "rx_invalid_length_hz": self._diag_rx_invalid_lengths / elapsed,
            "rx_unknown_payload_hz": self._diag_rx_unknown_payloads / elapsed,
            "rx_buffer_overflows": self._diag_rx_buffer_overflows,
            "rx_max_buffer": self._diag_rx_max_buffer,
            "tx_attempt_hz": self._diag_tx_attempts / elapsed,
            "tx_serial_write_hz": self._diag_tx_sent / elapsed,
            "tx_bytes_per_s": self._diag_tx_bytes / elapsed,
            "tx_error_hz": self._diag_tx_errors / elapsed,
            "tx_coalesced_hz": self._diag_tx_coalesced / elapsed,
            "bytes_to_write": self._tx_last_bytes_to_write,
            "tx_enabled": self._tx_enabled,
            "connected": bool(self.is_connected()),
        }
        signal = getattr(self, "link_diagnostics_update", None)
        if signal is not None:
            signal.emit(stats)
        self._reset_link_diagnostics_window()

    @Slot(bool)
    def set_raw_serial_debug_enabled(self, enabled):
        """Enable or disable forwarding raw serial chunks to the GUI debug tab."""
        self._raw_serial_debug_enabled = bool(enabled)

    def _ensure_tx_debug_attrs(self):
        if not hasattr(self, "_tx_debug_window_start"):
            self._tx_debug_window_start = time.perf_counter()
        if not hasattr(self, "_tx_debug_last_write_perf"):
            self._tx_debug_last_write_perf = None
        for attr in (
            "_tx_pacer_ticks",
            "_tx_pacer_coalesced",
            "_tx_send_attempts",
            "_tx_sent_packets",
            "_tx_write_errors",
            "_tx_bytes_written",
            "_tx_last_bytes_to_write",
        ):
            if not hasattr(self, attr):
                setattr(self, attr, 0)
        if not hasattr(self, "_tx_last_interval_ms"):
            self._tx_last_interval_ms = None

    def _reset_tx_debug_window(self):
        self._ensure_tx_debug_attrs()
        self._tx_debug_window_start = time.perf_counter()
        self._tx_pacer_ticks = 0
        self._tx_pacer_coalesced = 0
        self._tx_send_attempts = 0
        self._tx_sent_packets = 0
        self._tx_write_errors = 0
        self._tx_bytes_written = 0
        self._tx_last_interval_ms = None
        self._tx_last_bytes_to_write = 0

    def _maybe_emit_tx_debug(self):
        self._ensure_tx_debug_attrs()
        now = time.perf_counter()
        elapsed = now - self._tx_debug_window_start
        if elapsed < 1.0:
            return

        stats = {
            "target_hz": 1000.0 / max(1, self._tx_interval_ms),
            "window_s": elapsed,
            "pacer_hz": self._tx_pacer_ticks / elapsed if elapsed > 0 else 0.0,
            "send_attempt_hz": self._tx_send_attempts / elapsed if elapsed > 0 else 0.0,
            "serial_write_hz": self._tx_sent_packets / elapsed if elapsed > 0 else 0.0,
            "bytes_per_s": self._tx_bytes_written / elapsed if elapsed > 0 else 0.0,
            "coalesced_ticks": self._tx_pacer_coalesced,
            "write_errors": self._tx_write_errors,
            "last_interval_ms": self._tx_last_interval_ms,
            "bytes_to_write": self._tx_last_bytes_to_write,
            "enabled": self._tx_enabled,
            "connected": bool(self.is_connected()),
        }
        signal = getattr(self, "transmit_debug_update", None)
        if signal is not None:
            signal.emit(stats)
        self._reset_tx_debug_window()


    def _serial_is_open(self, serial) -> bool:
        is_open = getattr(serial, "isOpen", None)
        if is_open is None:
            return True
        return bool(is_open())

    def _serial_error_string(self) -> str:
        serial = getattr(self, "serial", None)
        error_string = getattr(serial, "errorString", None)
        if error_string is None:
            return "unknown serial error"
        return str(error_string())

    def _record_tx_write_error(self) -> None:
        self._tx_write_errors += 1
        self._diag_tx_errors += 1

    def _wait_for_serial_write_drain(self, serial, timeout_ms: int = 20) -> bool:
        """Wait until Qt reports that the current serial write buffer is empty."""

        if serial is None or not self._serial_is_open(serial):
            return False

        deadline = time.monotonic() + max(1, int(timeout_ms)) / 1000.0
        bytes_to_write = getattr(serial, "bytesToWrite", None)
        wait_for_bytes_written = getattr(serial, "waitForBytesWritten", None)

        while True:
            pending = None
            if bytes_to_write is not None:
                try:
                    pending = int(bytes_to_write())
                    self._tx_last_bytes_to_write = pending
                except Exception:
                    pending = None

            if pending == 0:
                return True

            if wait_for_bytes_written is None:
                return pending is None

            remaining_ms = int((deadline - time.monotonic()) * 1000)
            if remaining_ms <= 0:
                return pending == 0

            if not bool(wait_for_bytes_written(max(1, remaining_ms))):
                if bytes_to_write is None:
                    return False
                try:
                    pending = int(bytes_to_write())
                    self._tx_last_bytes_to_write = pending
                except Exception:
                    return False
                return pending == 0

    @Slot(list, int)
    def send_safe_shutdown_packets(self, safe_channels, frame_count=3):
        """Send throttle-cut/disarmed frames before periodic TX is disabled."""

        success = False
        try:
            self.channels = self._normalise_channels(safe_channels)
            count = max(1, int(frame_count or 1))
            success = True
            for _ in range(count):
                result = self.send_current_packet()
                frame_success = result == "Good"
                serial = getattr(self, "serial", None)
                if frame_success:
                    frame_success = self._wait_for_serial_write_drain(serial, 20)
                    if not frame_success:
                        self._record_tx_write_error()
                        message = "Safe shutdown CRSF frame did not flush before timeout"
                        logger.error(message)
                        self.error.emit(message)
                success = success and frame_success
                QThread.msleep(max(1, int(getattr(self, "_tx_interval_ms", 4))))
        except Exception as exc:
            logger.exception("Failed to send safe shutdown packets")
            self.error.emit(f"Failed to send safe shutdown packets: {exc}")
            success = False
        finally:
            self.safe_shutdown_complete.emit(success)

    @Slot()
    def send_current_packet(self):
        """Write the latest channel packet at the configured ELRS cadence."""
        try:
            self._ensure_tx_debug_attrs()
            self._ensure_link_diagnostics_attrs()
            self._tx_send_attempts += 1
            self._diag_tx_attempts += 1
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
                expected_bytes = len(packet)
                bytes_written = int(self.serial.write(bytes(packet)))
                if bytes_written != expected_bytes:
                    self._record_tx_write_error()
                    if bytes_written == -1:
                        raise IOError(self._serial_error_string())
                    raise IOError(
                        f"Short CRSF serial write: wrote {bytes_written} of "
                        f"{expected_bytes} bytes"
                    )
                now_perf = time.perf_counter()
                if self._tx_debug_last_write_perf is not None:
                    self._tx_last_interval_ms = (now_perf - self._tx_debug_last_write_perf) * 1000.0
                self._tx_debug_last_write_perf = now_perf
                self._tx_sent_packets += 1
                self._tx_bytes_written += bytes_written
                self._diag_tx_sent += 1
                self._diag_tx_bytes += bytes_written
                try:
                    self._tx_last_bytes_to_write = int(self.serial.bytesToWrite())
                except Exception:
                    self._tx_last_bytes_to_write = 0
                self._maybe_emit_tx_debug()
                self._maybe_emit_link_diagnostics()
                return "Good"

            return "Error"

        except Exception as exc:
            logger.exception("Failed to send packet")
            self.error.emit(f"Failed to send packet: {exc}")
            return f"Error: {exc}"
        finally:
            try:
                self._maybe_emit_tx_debug()
            except Exception:
                logger.debug("TX debug emit failed", exc_info=True)
            tx_pacer = getattr(self, "_tx_pacer", None)
            if tx_pacer:
                tx_pacer.mark_send_complete()


    @Slot()
    def read_serial_data(self):
        """Read available telemetry data and decode all received packets."""
        if getattr(self, "_parameter_query_active", False):
            return
        if not self.serial or self.serial.bytesAvailable() <= 0:
            return

        try:
            self._ensure_link_diagnostics_attrs()
            # Read all currently available bytes and append them to the buffer.
            # Keeping the buffering logic centralised allows for easy decoding
            # without producing verbose serial output.
            new_data = bytes(self.serial.readAll())
            if new_data:
                self._diag_rx_bytes += len(new_data)
                if getattr(self, "_raw_serial_debug_enabled", False):
                    try:
                        self.serial_data.emit(new_data)
                    except Exception:
                        logger.debug("Serial data signal emit failed", exc_info=True)
                self._rx_buffer.extend(new_data)
                # Prevent unbounded growth if we fall behind
                MAX_BUF = 8192
                if len(self._rx_buffer) > MAX_BUF:
                    self._diag_rx_buffer_overflows += 1
                    logger.warning(
                        "RX buffer overflow (%d). Dropping to resync.",
                        len(self._rx_buffer),
                    )
                    self._rx_buffer = self._rx_buffer[-512:]
                self._diag_rx_max_buffer = max(self._diag_rx_max_buffer, len(self._rx_buffer))

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
                    self._diag_rx_dropped_bytes += 1
                    del self._rx_buffer[0]
                    continue

                length = self._rx_buffer[1]

                # Drop frames with impossible lengths (need at least type+crc = 2)
                # and cap the maximum size to 64 bytes.
                if length < 2 or length > 64:
                    self._diag_rx_invalid_lengths += 1
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
                    self._diag_rx_crc_errors += 1
                    del self._rx_buffer[0]
                    continue


                # Extract complete packet and remove from buffer
                packet = bytes(self._rx_buffer[:frame_end])
                del self._rx_buffer[:frame_end]

                packet_type = packet[2]
                self._diag_rx_frames += 1
                self._diag_frame_counts[packet_type] = self._diag_frame_counts.get(packet_type, 0) + 1

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
                    self._diag_rx_unknown_payloads += 1

            self._maybe_emit_link_diagnostics()

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



    def _emit_parameter_query_line(self, message: str) -> None:
        signal = getattr(self, "parameter_query_update", None)
        if signal is not None:
            signal.emit(message)

    def _create_extended_packet(self, frame_type: int, dest: int, origin: int, payload: bytes = b"") -> bytes:
        length = len(payload) + 4  # type + destination + origin + crc
        packet = bytearray([dest, length, frame_type, dest, origin])
        packet.extend(payload)
        packet.append(self.crc8_data(packet[2:]))
        return bytes(packet)

    def _try_extract_parameter_entry(self, packet: dict[str, object]) -> dict[str, object] | None:
        data = packet.get("data", b"")
        if not isinstance(data, bytes) or len(data) < 4:
            return None

        field_id = data[0]
        chunks_remaining = data[1]
        return self._decode_parameter_entry(field_id, chunks_remaining, data[2:])

    def _decode_parameter_entry(
        self, field_id: int, chunks_remaining: int, payload: bytes
    ) -> dict[str, object] | None:
        if len(payload) < 2:
            return None

        parent = payload[0]
        param_type = payload[1] & 0x3F
        rest = payload[2:]
        nul = rest.find(b"\x00")
        if nul < 0:
            return None

        name = rest[:nul].decode("ascii", errors="replace")
        value = None
        selected = None
        options: list[str] = []
        units = ""
        detail_offset = nul + 1

        if param_type == 9:  # CRSF_TEXT_SELECTION
            option_blob = rest[detail_offset:]
            opt_nul = option_blob.find(b"\x00")
            if opt_nul >= 0:
                options_text = option_blob[:opt_nul].decode("ascii", errors="replace")
                options = options_text.split(";") if options_text else []
                value_offset = detail_offset + opt_nul + 1
                if len(rest) >= value_offset + 4:
                    value = int(rest[value_offset])
                    if 0 <= value < len(options):
                        selected = options[value]
                    units_blob = rest[value_offset + 4:]
                    units_nul = units_blob.find(b"\x00")
                    if units_nul >= 0:
                        units = units_blob[:units_nul].decode("ascii", errors="replace")
                    else:
                        units = units_blob.decode("ascii", errors="replace")

        return {
            "field_id": field_id,
            "chunks_remaining": chunks_remaining,
            "parent": parent,
            "type": param_type,
            "name": name,
            "value": value,
            "selected": selected,
            "options": options,
            "units": units,
        }

    def _parse_parameter_query_frames(self, buffer: bytearray) -> list[dict[str, object]]:
        frames: list[dict[str, object]] = []
        while True:
            if len(buffer) < 3:
                break
            if buffer[0] not in (
                self.CRSF_SYNC,
                self.TELEMETRY_SYNC,
                self.CRSF_ADDRESS_RADIO_TRANSMITTER,
                self.CRSF_ADDRESS_CRSF_TRANSMITTER,
            ):
                del buffer[0]
                continue

            length = buffer[1]
            if length < 2 or length > 64:
                del buffer[0]
                continue

            frame_end = length + 2
            if len(buffer) < frame_end:
                break

            if self.crc8_data(buffer[2:frame_end - 1]) != buffer[frame_end - 1]:
                del buffer[0]
                continue

            packet = bytes(buffer[:frame_end])
            del buffer[:frame_end]
            frame_type = packet[2]
            payload = packet[3:frame_end - 1]
            frame: dict[str, object] = {
                "address": packet[0],
                "type": frame_type,
                "payload": payload,
                "raw": packet,
            }
            if frame_type >= self.CRSF_FRAMETYPE_DEVICE_PING and len(payload) >= 2:
                frame["dest"] = payload[0]
                frame["origin"] = payload[1]
                frame["data"] = payload[2:]
            else:
                frame["data"] = payload
            frames.append(frame)
        return frames

    def _write_query_packet(self, packet: bytes) -> bool:
        if not self.serial:
            return False
        written = self.serial.write(packet)
        if written == -1:
            self._emit_parameter_query_line(f"CRSF query write failed: {self.serial.errorString()}")
            return False
        self.serial.waitForBytesWritten(100)
        return True

    def _collect_query_frames(self, timeout_ms: int = 250) -> list[dict[str, object]]:
        if not self.serial:
            return []
        deadline = time.monotonic() + timeout_ms / 1000.0
        buffer = bytearray()
        frames: list[dict[str, object]] = []
        while time.monotonic() < deadline:
            if self.serial.bytesAvailable() <= 0:
                remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
                self.serial.waitForReadyRead(min(50, remaining_ms))
            if self.serial.bytesAvailable() > 0:
                buffer.extend(bytes(self.serial.readAll()))
                frames.extend(self._parse_parameter_query_frames(buffer))
        return frames

    def _read_parameter_chunk(self, field_id: int, chunk_index: int) -> dict[str, object] | None:
        packet = self._create_extended_packet(
            self.CRSF_FRAMETYPE_PARAMETER_READ,
            self.CRSF_ADDRESS_CRSF_TRANSMITTER,
            self.CRSF_ADDRESS_ELRS_LUA,
            bytes([field_id, chunk_index]),
        )
        if not self._write_query_packet(packet):
            return None

        for frame in self._collect_query_frames(180):
            if frame.get("type") != self.CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
                continue
            data = frame.get("data", b"")
            if not isinstance(data, bytes) or len(data) < 2:
                continue
            if data[0] != field_id:
                continue
            return {
                "field_id": int(data[0]),
                "chunks_remaining": int(data[1]),
                "chunk_payload": data[2:],
            }
        return None

    def _read_single_parameter(self, field_id: int) -> dict[str, object] | None:
        first_chunk = self._read_parameter_chunk(field_id, 0)
        if not first_chunk:
            return None

        chunks = [bytes(first_chunk["chunk_payload"])]
        chunks_remaining = int(first_chunk["chunks_remaining"])
        chunk_index = 1
        while chunks_remaining > 0:
            chunk = self._read_parameter_chunk(field_id, chunk_index)
            if not chunk:
                self._emit_parameter_query_line(
                    f"Parameter {field_id} chunk {chunk_index} missing; unable to assemble entry."
                )
                return None
            chunks.append(bytes(chunk["chunk_payload"]))
            chunks_remaining = int(chunk["chunks_remaining"])
            chunk_index += 1

        assembled_payload = b"".join(chunks)
        return self._decode_parameter_entry(field_id, 0, assembled_payload)

    @Slot()
    def query_elrs_parameters(self):
        """Query the TX module's CRSF parameter table while RC transmission is stopped."""
        if self._tx_enabled:
            self._emit_parameter_query_line("ELRS parameter query blocked: stop packet transmission first.")
            return
        if not self.is_connected():
            self._emit_parameter_query_line("ELRS parameter query failed: CRSF serial port is not connected.")
            return

        constants = (
            ("DEVICE_PING", self.CRSF_FRAMETYPE_DEVICE_PING),
            ("DEVICE_INFO", self.CRSF_FRAMETYPE_DEVICE_INFO),
            ("PARAMETER_SETTINGS_ENTRY", self.CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY),
            ("PARAMETER_READ", self.CRSF_FRAMETYPE_PARAMETER_READ),
            ("PARAMETER_WRITE", self.CRSF_FRAMETYPE_PARAMETER_WRITE),
        )
        self._emit_parameter_query_line("CRSF parameter query frame types:")
        for name, value in constants:
            self._emit_parameter_query_line(f"  {name:<28} 0x{value:02X}")

        self._parameter_query_active = True
        old_raw_debug = self._raw_serial_debug_enabled
        self._raw_serial_debug_enabled = False
        try:
            try:
                self.serial.readyRead.disconnect(self.read_serial_data)
            except Exception:
                pass
            self._rx_buffer.clear()
            if self.serial.bytesAvailable() > 0:
                self.serial.readAll()

            self._emit_parameter_query_line("Sending DEVICE_PING to ELRS TX module...")
            ping = self._create_extended_packet(
                self.CRSF_FRAMETYPE_DEVICE_PING,
                self.CRSF_ADDRESS_CRSF_TRANSMITTER,
                self.CRSF_ADDRESS_ELRS_LUA,
            )
            self._write_query_packet(ping)
            device_name = None
            field_count = None
            for frame in self._collect_query_frames(500):
                if frame.get("type") != self.CRSF_FRAMETYPE_DEVICE_INFO:
                    continue
                data = frame.get("data", b"")
                if not isinstance(data, bytes):
                    continue
                nul = data.find(b"\x00")
                if nul >= 0:
                    device_name = data[:nul].decode("ascii", errors="replace")
                    info = data[nul + 1:]
                    if len(info) >= 14:
                        field_count = int(info[12])
                    self._emit_parameter_query_line(
                        f"DEVICE_INFO: name={device_name or '--'} field_count={field_count if field_count is not None else '--'}"
                    )
                    break

            max_field = field_count if field_count is not None else 64
            telem_entry = None
            self._emit_parameter_query_line(f"Reading up to {max_field} parameter entries...")
            for field_id in range(1, max_field + 1):
                entry = self._read_single_parameter(field_id)
                if not entry:
                    continue
                if entry.get("name") == "Telem Ratio":
                    telem_entry = entry
                    break

            if telem_entry is None:
                self._emit_parameter_query_line("Telem Ratio parameter was not found in the TX module parameter table.")
                return

            value = telem_entry.get("value")
            selected = telem_entry.get("selected") or "--"
            options = telem_entry.get("options") or []
            self._emit_parameter_query_line(
                f"Telem Ratio: value={value if value is not None else '--'} selected={selected}"
            )
            if options:
                self._emit_parameter_query_line(f"Telem Ratio options: {';'.join(str(option) for option in options)}")
            if value == 8 and selected == "1:2":
                self._emit_parameter_query_line("Confirmed: TX module reports telemetry ratio 1:2.")
            else:
                self._emit_parameter_query_line("Warning: TX module did not report Telem Ratio as value 8 / 1:2.")
        except Exception as exc:
            logger.exception("ELRS parameter query failed")
            self._emit_parameter_query_line(f"ELRS parameter query failed: {exc}")
        finally:
            self._raw_serial_debug_enabled = old_raw_debug
            try:
                self.serial.readyRead.connect(self.read_serial_data)
            except Exception:
                pass
            self._parameter_query_active = False

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

