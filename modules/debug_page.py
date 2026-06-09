from __future__ import annotations

import math
from collections import deque
from datetime import datetime, timedelta
from typing import Iterable, Mapping, Sequence

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCursor, QFont
from PySide6.QtWidgets import (
    QCheckBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QPlainTextEdit,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)


class DebugPage:
    """Create the Debug tab and display live telemetry data selections."""

    _PACKET_LABELS = {
        "attitude": "Attitude packets",
        "gps": "GPS packets",
        "battery": "Battery packets",
        "link_stats": "Link statistics packets",
        "control": "Control channel updates (GS timer)",
        "control_tx": "Control serial writes (GS worker)",
    }

    def __init__(self, main_window) -> None:
        self._main_window = main_window
        self._ui = main_window.ui
        self._packet_checkboxes: dict[str, QCheckBox] = {}
        self._monitoring = False
        self._packet_timestamps: deque[datetime] = deque()
        self._frequency_window = timedelta(seconds=5)
        self._frequency_timer = QTimer()
        self._frequency_timer.setInterval(1000)
        self._frequency_timer.timeout.connect(self._refresh_frequency_label)

        # Control packets are emitted at a high frequency which can overwhelm
        # the debug console and starve the event loop.  Buffer the most recent
        # values and only flush them to the UI at a throttled interval so
        # telemetry transmission remains responsive even while monitoring.
        self._control_log_interval = timedelta(milliseconds=100)
        self._control_last_log = datetime.min
        self._control_buffer_count = 0
        self._control_buffer_values: tuple[float | int, ...] | None = None

        self._create_navigation_button()
        self._build_page()

    # ------------------------------------------------------------------
    # UI construction helpers
    # ------------------------------------------------------------------
    def _create_navigation_button(self) -> None:
        font: QFont = self._ui.btn_home.font()
        button = QPushButton(self._ui.topMenu)
        button.setObjectName("btn_debug")
        size_policy: QSizePolicy = self._ui.btn_home.sizePolicy()
        button.setSizePolicy(size_policy)
        button.setMinimumSize(self._ui.btn_home.minimumSize())
        button.setFont(font)
        button.setCursor(QCursor(Qt.PointingHandCursor))
        button.setLayoutDirection(Qt.LeftToRight)
        button.setStyleSheet(
            "background-image: url(:/icons/images/icons/cil-terminal.png);"
        )
        button.setText("Debug")
        self._ui.verticalLayout_8.addWidget(button)
        self._ui.btn_debug = button

    def _build_page(self) -> None:
        page = QWidget()
        page.setObjectName("debug_page")
        layout = QVBoxLayout(page)
        layout.setSpacing(12)

        description = QLabel(
            "Select the data streams you want to observe and press Start Monitoring to begin."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        checkbox_grid = QGridLayout()
        checkbox_grid.setSpacing(6)
        row = 0
        for key, label in self._PACKET_LABELS.items():
            checkbox = QCheckBox(label)
            checkbox_grid.addWidget(checkbox, row, 0)
            self._packet_checkboxes[key] = checkbox
            row += 1
        layout.addLayout(checkbox_grid)

        self.joystick_checkbox = QCheckBox("Joystick data")
        layout.addWidget(self.joystick_checkbox)

        self.serial_all_checkbox = QCheckBox("Serial all")
        layout.addWidget(self.serial_all_checkbox)

        self.telemetry_all_checkbox = QCheckBox("Telemetry all")
        layout.addWidget(self.telemetry_all_checkbox)

        button_row = QHBoxLayout()
        button_row.addStretch()
        self.parameter_query_button = QPushButton("Query ELRS Parameters")
        self.parameter_query_button.setMinimumSize(220, 48)
        self.parameter_query_button.clicked.connect(self._on_parameter_query_clicked)
        button_row.addWidget(self.parameter_query_button)
        self.link_diagnostics_button = QPushButton("Start Link Diagnostics")
        self.link_diagnostics_button.setMinimumSize(220, 48)
        self.link_diagnostics_button.clicked.connect(self._on_link_diagnostics_clicked)
        button_row.addWidget(self.link_diagnostics_button)
        self.monitor_button = QPushButton("Start Monitoring")
        self.monitor_button.setMinimumSize(200, 48)
        self.monitor_button.clicked.connect(self._on_monitor_clicked)
        button_row.addWidget(self.monitor_button)
        button_row.addStretch()
        layout.addLayout(button_row)

        self.frequency_label = QLabel("Packet frequency: --")
        self.frequency_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(self.frequency_label)

        self.output_edit = QPlainTextEdit()
        self.output_edit.setReadOnly(True)
        self.output_edit.setObjectName("debugOutput")
        layout.addWidget(self.output_edit, 1)

        spacer = QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
        layout.addItem(spacer)

        self._ui.debug_page = page
        self._ui.stackedWidget.addWidget(page)

    # ------------------------------------------------------------------
    # Callbacks wired to UI controls
    # ------------------------------------------------------------------
    def _on_monitor_clicked(self) -> None:
        if self._monitoring:
            self.monitor_button.setEnabled(False)
            self._main_window.stop_debug_monitoring()
            return

        packet_selection = {
            name for name, checkbox in self._packet_checkboxes.items() if checkbox.isChecked()
        }
        include_joystick = self.joystick_checkbox.isChecked()
        serial_all = self.serial_all_checkbox.isChecked()
        telemetry_all = self.telemetry_all_checkbox.isChecked()
        if not (packet_selection or include_joystick or serial_all or telemetry_all):
            self.append_message("Select at least one data source before monitoring.")
            return

        self.output_edit.clear()
        self.monitor_button.setEnabled(False)
        self._main_window.start_debug_monitoring(
            packet_selection,
            include_joystick,
            serial_all,
            telemetry_all,
        )

    def _on_parameter_query_clicked(self) -> None:
        self._main_window.query_elrs_parameters_from_debug_page()

    def _on_link_diagnostics_clicked(self) -> None:
        self._main_window.toggle_link_diagnostics_from_debug_page()

    # ------------------------------------------------------------------
    # Methods invoked from the main window
    # ------------------------------------------------------------------
    def begin_monitoring(
        self,
        packets: Iterable[str],
        include_joystick: bool,
        serial_all: bool,
        telemetry_all: bool,
    ) -> None:
        self._monitoring = True
        self.monitor_button.setText("Stop Monitoring")
        self.monitor_button.setEnabled(True)
        self._packet_timestamps.clear()
        self._frequency_timer.start()
        self._update_frequency_label(datetime.now())
        self._control_last_log = datetime.min
        self._control_buffer_count = 0
        self._control_buffer_values = None
        if packets or include_joystick or serial_all or telemetry_all:
            labels = [self._PACKET_LABELS.get(name, name) for name in packets]
            if include_joystick:
                labels.append("Joystick data")
            if serial_all:
                labels.append("Serial all")
            if telemetry_all:
                labels.append("Telemetry all")
            enabled = ", ".join(sorted(labels, key=str.lower))
            self.append_message(f"Monitoring started for: {enabled}.")
        else:
            self.append_message("Monitoring started.")

    def end_monitoring(self) -> None:
        if not self._monitoring:
            return
        self._flush_control_buffer()
        self._monitoring = False
        self.monitor_button.setText("Start Monitoring")
        self.monitor_button.setEnabled(True)
        self._frequency_timer.stop()
        self._packet_timestamps.clear()
        self.frequency_label.setText("Packet frequency: --")
        self.append_message("Monitoring stopped.")

    def set_parameter_query_enabled(self, enabled: bool, reason: str = "") -> None:
        if hasattr(self, "parameter_query_button"):
            self.parameter_query_button.setEnabled(enabled)
            self.parameter_query_button.setToolTip(reason)


    def set_link_diagnostics_enabled(self, enabled: bool, active: bool, reason: str = "") -> None:
        if hasattr(self, "link_diagnostics_button"):
            self.link_diagnostics_button.setEnabled(enabled)
            self.link_diagnostics_button.setText(
                "Stop Link Diagnostics" if active else "Start Link Diagnostics"
            )
            self.link_diagnostics_button.setToolTip(reason)

    def log_link_diagnostics(self, stats: Mapping[str, object]) -> None:
        if stats.get("event") == "state":
            state = "started" if stats.get("enabled") else "stopped"
            self.append_message(f"Link diagnostics {state}.")
            return

        timestamp = datetime.now().strftime("%H:%M:%S")
        detail = (
            f"rx_bytes={float(stats.get('rx_bytes_per_s', 0.0)):.0f}/s "
            f"rx_frame={float(stats.get('rx_frame_hz', 0.0)):.1f}Hz "
            f"att={float(stats.get('rx_attitude_hz', 0.0)):.1f}Hz "
            f"gps={float(stats.get('rx_gps_hz', 0.0)):.1f}Hz "
            f"link={float(stats.get('rx_link_stats_hz', 0.0)):.1f}Hz "
            f"crc_err={float(stats.get('rx_crc_error_hz', 0.0)):.1f}Hz "
            f"drop={float(stats.get('rx_dropped_bytes_per_s', 0.0)):.0f}/s "
            f"invalid_len={float(stats.get('rx_invalid_length_hz', 0.0)):.1f}Hz "
            f"unknown={float(stats.get('rx_unknown_payload_hz', 0.0)):.1f}Hz "
            f"buf_max={int(stats.get('rx_max_buffer', 0))} "
            f"buf_ovf={int(stats.get('rx_buffer_overflows', 0))} "
            f"tx_attempt={float(stats.get('tx_attempt_hz', 0.0)):.1f}Hz "
            f"tx_write={float(stats.get('tx_serial_write_hz', 0.0)):.1f}Hz "
            f"tx_bytes={float(stats.get('tx_bytes_per_s', 0.0)):.0f}/s "
            f"tx_coalesced={float(stats.get('tx_coalesced_hz', 0.0)):.1f}Hz "
            f"queued={int(stats.get('bytes_to_write', 0))} "
            f"tx_enabled={bool(stats.get('tx_enabled', False))} "
            f"connected={bool(stats.get('connected', False))}"
        )
        self.output_edit.appendPlainText(f"[{timestamp}] link_diag: {detail}")

    def append_message(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.output_edit.appendPlainText(f"[{timestamp}] {message}")

    def log_packet(self, packet_type: str, values: Mapping[str, object] | Sequence[float | int]) -> None:
        if not self._monitoring:
            return
        now = datetime.now()
        self._packet_timestamps.append(now)
        self._update_frequency_label(now)
        if packet_type == "control":
            self._buffer_control_packet(now, values)
            return
        timestamp = now.strftime("%H:%M:%S")
        if packet_type == "control_tx" and isinstance(values, Mapping):
            last_interval = values.get("last_interval_ms")
            last_interval_text = "--" if last_interval is None else f"{float(last_interval):.2f}"
            detail = (
                f"target={float(values.get('target_hz', 0.0)):.1f}Hz "
                f"pacer={float(values.get('pacer_hz', 0.0)):.1f}Hz "
                f"attempt={float(values.get('send_attempt_hz', 0.0)):.1f}Hz "
                f"serial_write={float(values.get('serial_write_hz', 0.0)):.1f}Hz "
                f"bytes={float(values.get('bytes_per_s', 0.0)):.0f}/s "
                f"queued_bytes={int(values.get('bytes_to_write', 0))} "
                f"coalesced={int(values.get('coalesced_ticks', 0))} "
                f"errors={int(values.get('write_errors', 0))} "
                f"last_dt={last_interval_text}ms "
                f"connected={bool(values.get('connected', False))}"
            )
        elif packet_type == "attitude" and len(values) >= 3:
            pitch, roll, yaw = values[:3]
            detail = f"pitch={pitch:.2f}\u00b0 roll={roll:.2f}\u00b0 yaw={yaw:.2f}\u00b0"
        elif packet_type == "gps" and len(values) >= 6:
            lat, lon, alt_ft, speed_mph, course, sats = values[:6]
            detail = (
                f"lat={lat:.6f} lon={lon:.6f} alt={alt_ft:.0f} ft "
                f"speed={speed_mph:.1f} mph course={course:.1f}\u00b0 sats={int(sats)}"
            )
        elif packet_type == "battery" and len(values) >= 3:
            voltage, current, capacity = values[:3]
            percent = values[3] if len(values) >= 4 else None
            detail = (
                f"voltage={voltage:.1f} V current={current:.1f} A capacity={int(capacity)} mAh"
            )
            if percent is not None:
                detail += f" percent={percent:.0f}%"
        elif packet_type == "link_stats" and len(values) >= 6:
            rssi_a, rssi_b, link_quality, snr, downlink_lq, downlink_snr = values[:6]
            detail = (
                f"RSSI_A={rssi_a} RSSI_B={rssi_b} LQ={link_quality}% "
                f"SNR={snr} dB Downlink_LQ={downlink_lq}% Downlink_SNR={downlink_snr} dB"
            )
        elif packet_type == "joystick" and len(values) >= 2:
            pitch, roll = values[:2]
            detail = f"pitch={pitch:.1f} roll={roll:.1f}"
        else:
            detail = " ".join(str(value) for value in values)
        self.output_edit.appendPlainText(f"[{timestamp}] {packet_type}: {detail}")

    # ------------------------------------------------------------------
    # Control packet throttling helpers
    # ------------------------------------------------------------------
    def _buffer_control_packet(self, now: datetime, values: Sequence[float | int]) -> None:
        """Store the latest control channels and flush at a throttled rate."""

        self._control_buffer_values = tuple(values)
        self._control_buffer_count += 1
        if now - self._control_last_log >= self._control_log_interval:
            self._flush_control_buffer(now)

    def _flush_control_buffer(self, now: datetime | None = None) -> None:
        """Emit a summarised control packet entry if buffered."""

        if self._control_buffer_count == 0 or self._control_buffer_values is None:
            return

        if now is None:
            now = datetime.now()

        timestamp = now.strftime("%H:%M:%S")
        detail = " ".join(
            f"ch{index + 1}={int(value)}"
            for index, value in enumerate(self._control_buffer_values)
        )
        if self._control_buffer_count > 1:
            detail += f" (aggregated {self._control_buffer_count} packets)"

        self.output_edit.appendPlainText(f"[{timestamp}] control: {detail}")
        self._control_last_log = now
        self._control_buffer_count = 0
        self._control_buffer_values = None

    def log_serial_data(self, data: bytes) -> None:
        if not self._monitoring or not data:
            return

        now = datetime.now()
        self._packet_timestamps.append(now)
        self._update_frequency_label(now)
        timestamp = now.strftime("%H:%M:%S")
        hex_repr = data.hex(" ")
        ascii_repr = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
        self.output_edit.appendPlainText(
            f"[{timestamp}] serial all: hex={hex_repr} ascii={ascii_repr}"
        )

    def monitoring_active(self) -> bool:
        return self._monitoring

    # ------------------------------------------------------------------
    # Packet frequency calculations
    # ------------------------------------------------------------------
    def _trim_packet_timestamps(self, reference_time: datetime) -> None:
        cutoff = reference_time - self._frequency_window
        while self._packet_timestamps and self._packet_timestamps[0] < cutoff:
            self._packet_timestamps.popleft()

    def _update_frequency_label(self, reference_time: datetime | None = None) -> None:
        if reference_time is None:
            reference_time = datetime.now()
        self._trim_packet_timestamps(reference_time)
        if not self._monitoring:
            self.frequency_label.setText("Packet frequency: --")
            return

        packet_count = len(self._packet_timestamps)
        if packet_count <= 1:
            if packet_count == 0:
                self.frequency_label.setText("Packet frequency: 0.0 packets/sec")
            else:
                self.frequency_label.setText("Packet frequency: <1 packet/sec")
            return

        elapsed = (self._packet_timestamps[-1] - self._packet_timestamps[0]).total_seconds()
        if elapsed <= 0:
            frequency = float(packet_count)
        else:
            frequency = (packet_count - 1) / elapsed
        self.frequency_label.setText(f"Packet frequency: {frequency:.1f} packets/sec")

    def _refresh_frequency_label(self) -> None:
        self._update_frequency_label()
