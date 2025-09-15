"""Telemetry data page construction and data management."""

from collections import deque

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCursor
from PySide6.QtWidgets import (
    QLabel,
    QPushButton,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
)
import pyqtgraph as pg


class DataPage:
    """Encapsulate the telemetry data page and its supporting data."""

    _MAX_POINTS = 200

    def __init__(self, main_window) -> None:
        self._main_window = main_window
        self._ui = main_window.ui

        self._create_navigation_button()
        self._build_page()
        self._init_data_storage()
        self._init_timers()

    # ------------------------------------------------------------------
    # UI construction helpers
    # ------------------------------------------------------------------
    def _create_navigation_button(self) -> None:
        font = self._ui.btn_home.font()
        button = QPushButton(self._ui.topMenu)
        button.setObjectName("btn_data")
        size_policy = self._ui.btn_home.sizePolicy()
        button.setSizePolicy(size_policy)
        button.setMinimumSize(self._ui.btn_home.minimumSize())
        button.setFont(font)
        button.setCursor(QCursor(Qt.PointingHandCursor))
        button.setLayoutDirection(Qt.LeftToRight)
        button.setStyleSheet(
            "background-image: url(:/icons/images/icons/cil-chart-line.png);"
        )
        button.setText("Telemetry Data")
        self._ui.verticalLayout_8.addWidget(button)
        self._ui.btn_data = button

    def _build_page(self) -> None:
        page = QWidget()
        self._ui.data_page = page
        self._ui.stackedWidget.addWidget(page)

        layout = QVBoxLayout(page)

        title_label = QLabel("Telemetry Data")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title_label)

        attitude_label = QLabel("Attitude")
        attitude_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(attitude_label)

        attitude_layout = QHBoxLayout()
        layout.addLayout(attitude_layout, 1)
        plot_height = 150
        self.roll_plot = pg.PlotWidget()
        self.roll_plot.setTitle("Roll")
        self.roll_plot.setMinimumHeight(plot_height)
        attitude_layout.addWidget(self.roll_plot)
        self.pitch_plot = pg.PlotWidget()
        self.pitch_plot.setTitle("Pitch")
        self.pitch_plot.setMinimumHeight(plot_height)
        attitude_layout.addWidget(self.pitch_plot)
        self.yaw_plot = pg.PlotWidget()
        self.yaw_plot.setTitle("Yaw")
        self.yaw_plot.setMinimumHeight(plot_height)
        attitude_layout.addWidget(self.yaw_plot)

        flight_label = QLabel("Flight Telemetry")
        flight_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(flight_label)

        flight_layout = QHBoxLayout()
        layout.addLayout(flight_layout, 1)
        self.airspeed_plot = pg.PlotWidget()
        self.airspeed_plot.setTitle("Air speed")
        self.airspeed_plot.setMinimumHeight(plot_height)
        flight_layout.addWidget(self.airspeed_plot)
        self.altitude_plot = pg.PlotWidget()
        self.altitude_plot.setTitle("Altitude")
        self.altitude_plot.setMinimumHeight(plot_height)
        flight_layout.addWidget(self.altitude_plot)

        signal_label = QLabel("Signal Health")
        signal_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(signal_label)

        signal_layout = QGridLayout()
        layout.addLayout(signal_layout, 1)
        self.rssi_a_plot = pg.PlotWidget()
        self.rssi_a_plot.setTitle("RSSI A")
        self.rssi_b_plot = pg.PlotWidget()
        self.rssi_b_plot.setTitle("RSSI B")
        self.link_quality_plot = pg.PlotWidget()
        self.link_quality_plot.setTitle("Link Quality")
        self.downlink_quality_plot = pg.PlotWidget()
        self.downlink_quality_plot.setTitle("Downlink Quality")
        self.snr_plot = pg.PlotWidget()
        self.snr_plot.setTitle("SNR")
        self.downlink_snr_plot = pg.PlotWidget()
        self.downlink_snr_plot.setTitle("Downlink SNR")
        signal_layout.addWidget(self.rssi_a_plot, 0, 0)
        signal_layout.addWidget(self.rssi_b_plot, 0, 1)
        signal_layout.addWidget(self.link_quality_plot, 0, 2)
        signal_layout.addWidget(self.downlink_quality_plot, 1, 0)
        signal_layout.addWidget(self.snr_plot, 1, 1)
        signal_layout.addWidget(self.downlink_snr_plot, 1, 2)

        self.packet_rate_label = QLabel("Packets Received Rate: 0 Hz")
        layout.addWidget(self.packet_rate_label)

    def _init_data_storage(self) -> None:
        max_points = self._MAX_POINTS
        self.roll_data = deque([0] * max_points, maxlen=max_points)
        self.pitch_data = deque([0] * max_points, maxlen=max_points)
        self.yaw_data = deque([0] * max_points, maxlen=max_points)
        self.airspeed_data = deque([0] * max_points, maxlen=max_points)
        self.altitude_data = deque([0] * max_points, maxlen=max_points)
        self.rssi_a_data = deque([0] * max_points, maxlen=max_points)
        self.rssi_b_data = deque([0] * max_points, maxlen=max_points)
        self.link_quality_data = deque([0] * max_points, maxlen=max_points)
        self.downlink_quality_data = deque([0] * max_points, maxlen=max_points)
        self.snr_data = deque([0] * max_points, maxlen=max_points)
        self.downlink_snr_data = deque([0] * max_points, maxlen=max_points)

        self.roll_curve = self.roll_plot.plot()
        self.pitch_curve = self.pitch_plot.plot()
        self.yaw_curve = self.yaw_plot.plot()
        self.airspeed_curve = self.airspeed_plot.plot()
        self.altitude_curve = self.altitude_plot.plot()
        self.rssi_a_curve = self.rssi_a_plot.plot()
        self.rssi_b_curve = self.rssi_b_plot.plot()
        self.link_quality_curve = self.link_quality_plot.plot()
        self.downlink_quality_curve = self.downlink_quality_plot.plot()
        self.snr_curve = self.snr_plot.plot()
        self.downlink_snr_curve = self.downlink_snr_plot.plot()

    def _init_timers(self) -> None:
        self.graph_timer = QTimer(self._main_window)
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.start(100)

        self.packet_rate = 0
        self.packet_count = 0
        self.packet_rate_timer = QTimer(self._main_window)
        self.packet_rate_timer.timeout.connect(self.update_packet_rate)
        self.packet_rate_timer.start(1000)

    # ------------------------------------------------------------------
    # Data update methods
    # ------------------------------------------------------------------
    def record_packet(self) -> None:
        self.packet_count += 1

    def add_attitude(self, pitch: float, roll: float, yaw: float) -> None:
        self.pitch_data.append(pitch)
        self.roll_data.append(roll)
        self.yaw_data.append(yaw)

    def add_flight_metrics(self, altitude: float, airspeed: float) -> None:
        self.altitude_data.append(altitude)
        self.airspeed_data.append(airspeed)

    def add_link_stats(
        self,
        rssi_a: float,
        rssi_b: float,
        link_quality: float,
        downlink_quality: float,
        snr: float,
        downlink_snr: float,
    ) -> None:
        self.rssi_a_data.append(rssi_a)
        self.rssi_b_data.append(rssi_b)
        self.link_quality_data.append(link_quality)
        self.downlink_quality_data.append(downlink_quality)
        self.snr_data.append(snr)
        self.downlink_snr_data.append(downlink_snr)

    # ------------------------------------------------------------------
    # Timer callbacks
    # ------------------------------------------------------------------
    def update_graphs(self) -> None:
        self.roll_curve.setData(self.roll_data)
        self.pitch_curve.setData(self.pitch_data)
        self.yaw_curve.setData(self.yaw_data)
        self.airspeed_curve.setData(self.airspeed_data)
        self.altitude_curve.setData(self.altitude_data)
        self.rssi_a_curve.setData(self.rssi_a_data)
        self.rssi_b_curve.setData(self.rssi_b_data)
        self.link_quality_curve.setData(self.link_quality_data)
        self.downlink_quality_curve.setData(self.downlink_quality_data)
        self.snr_curve.setData(self.snr_data)
        self.downlink_snr_curve.setData(self.downlink_snr_data)

    def update_packet_rate(self) -> None:
        self.packet_rate_label.setText(
            f"Packets Received Rate: {self.packet_rate} Hz"
        )
        self.packet_rate = self.packet_count
        self.packet_count = 0


__all__ = ["DataPage"]
