from __future__ import annotations

import math
from collections import deque
from datetime import datetime, timedelta
from typing import Iterable, Sequence

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
        "control": "Control packet transmission",
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
        self._monitoring = False
        self.monitor_button.setText("Start Monitoring")
        self.monitor_button.setEnabled(True)
        self._frequency_timer.stop()
        self._packet_timestamps.clear()
        self.frequency_label.setText("Packet frequency: --")
        self.append_message("Monitoring stopped.")

    def append_message(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.output_edit.appendPlainText(f"[{timestamp}] {message}")

    def log_packet(self, packet_type: str, values: Sequence[float | int]) -> None:
        if not self._monitoring:
            return
        now = datetime.now()
        self._packet_timestamps.append(now)
        self._update_frequency_label(now)
        timestamp = now.strftime("%H:%M:%S")
        if packet_type == "attitude" and len(values) >= 3:
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
            piggyback_info = ""
            if len(values) >= 7:
                piggyback_count_value = values[6]
                try:
                    piggyback_count = int(piggyback_count_value)
                except (TypeError, ValueError):
                    try:
                        as_float = float(piggyback_count_value)
                    except (TypeError, ValueError):
                        piggyback_info = f" piggyback_packets={piggyback_count_value}"
                    else:
                        if math.isnan(as_float) or as_float <= 0:
                            piggyback_info = " piggyback_packets=0"
                        else:
                            piggyback_info = f" piggyback_packets={as_float:g}"
                else:
                    piggyback_info = (
                        f" piggyback_packets={piggyback_count}" if piggyback_count else ""
                    )
                    if piggyback_info == "":
                        piggyback_info = " piggyback_packets=0"
            if piggyback_info == "":
                piggyback_info = " piggyback_packets=0"
            detail = (
                f"RSSI_A={rssi_a} RSSI_B={rssi_b} LQ={link_quality}% "
                f"SNR={snr} dB Downlink_LQ={downlink_lq}% Downlink_SNR={downlink_snr} dB"
                f"{piggyback_info}"
            )
        elif packet_type == "joystick" and len(values) >= 2:
            pitch, roll = values[:2]
            detail = f"pitch={pitch:.1f} roll={roll:.1f}"
        elif packet_type == "control":
            detail = " ".join(
                f"ch{index + 1}={int(value)}" for index, value in enumerate(values)
            )
        else:
            detail = " ".join(str(value) for value in values)
        self.output_edit.appendPlainText(f"[{timestamp}] {packet_type}: {detail}")

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
