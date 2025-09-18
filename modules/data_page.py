"""Telemetry data page construction and data management."""

import numpy as np

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


class _DataSeries:
    """Container that tracks a scrolling data buffer and its plot curve."""

    __slots__ = ("curve", "data", "count", "_write_index")

    def __init__(self, curve: pg.PlotDataItem, max_points: int) -> None:
        self.curve = curve
        self.data = np.zeros(max_points, dtype=np.float32)
        self.count = 0
        self._write_index = 0

    def append(self, value: float) -> None:
        data = self.data
        data[self._write_index] = value
        self._write_index = (self._write_index + 1) % data.size
        if self.count < data.size:
            self.count += 1

    def render(self, x_values: np.ndarray) -> None:
        if self.count == 0:
            self.curve.clear()
            return

        data = self.data
        if self.count < data.size:
            view = slice(0, self.count)
            self.curve.setData(x_values[-self.count :], data[view])
            return

        if self._write_index == 0:
            ordered = data
        else:
            ordered = np.concatenate((data[self._write_index :], data[: self._write_index]))
        self.curve.setData(x_values, ordered)


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

    def _init_data_storage(self) -> None:
        max_points = self._MAX_POINTS
        self._x_values = np.linspace(-(max_points - 1), 0, max_points, dtype=np.float32)

        self.roll_series = _DataSeries(self.roll_plot.plot(), max_points)
        self.pitch_series = _DataSeries(self.pitch_plot.plot(), max_points)
        self.yaw_series = _DataSeries(self.yaw_plot.plot(), max_points)
        self.airspeed_series = _DataSeries(self.airspeed_plot.plot(), max_points)
        self.altitude_series = _DataSeries(self.altitude_plot.plot(), max_points)
        self.rssi_a_series = _DataSeries(self.rssi_a_plot.plot(), max_points)
        self.rssi_b_series = _DataSeries(self.rssi_b_plot.plot(), max_points)
        self.link_quality_series = _DataSeries(
            self.link_quality_plot.plot(), max_points
        )
        self.downlink_quality_series = _DataSeries(
            self.downlink_quality_plot.plot(), max_points
        )
        self.snr_series = _DataSeries(self.snr_plot.plot(), max_points)
        self.downlink_snr_series = _DataSeries(
            self.downlink_snr_plot.plot(), max_points
        )

        self._series = (
            self.roll_series,
            self.pitch_series,
            self.yaw_series,
            self.airspeed_series,
            self.altitude_series,
            self.rssi_a_series,
            self.rssi_b_series,
            self.link_quality_series,
            self.downlink_quality_series,
            self.snr_series,
            self.downlink_snr_series,
        )

        self._graph_dirty = True

    def _init_timers(self) -> None:
        self.graph_timer = QTimer(self._main_window)
        # Use the precise timer type so the refresh cadence isn't throttled to
        # the coarse timer's ~50 ms granularity.  This keeps the plots on the
        # data tab rendering at the intended frame rate.
        self.graph_timer.setTimerType(Qt.PreciseTimer)
        self.graph_timer.timeout.connect(self.update_graphs)
        self._graph_interval_ms = 33
        self._graphs_active = False
        self._ui.stackedWidget.currentChanged.connect(self._on_page_changed)
        self._on_page_changed(self._ui.stackedWidget.currentIndex())

    # ------------------------------------------------------------------
    # Data update methods
    # ------------------------------------------------------------------
    def add_attitude(self, pitch: float, roll: float, yaw: float) -> None:
        self.pitch_series.append(pitch)
        self.roll_series.append(roll)
        self.yaw_series.append(yaw)
        self._graph_dirty = True

    def add_flight_metrics(self, altitude: float, airspeed: float) -> None:
        self.altitude_series.append(altitude)
        self.airspeed_series.append(airspeed)
        self._graph_dirty = True

    def add_link_stats(
        self,
        rssi_a: float,
        rssi_b: float,
        link_quality: float,
        downlink_quality: float,
        snr: float,
        downlink_snr: float,
    ) -> None:
        self.rssi_a_series.append(rssi_a)
        self.rssi_b_series.append(rssi_b)
        self.link_quality_series.append(link_quality)
        self.downlink_quality_series.append(downlink_quality)
        self.snr_series.append(snr)
        self.downlink_snr_series.append(downlink_snr)
        self._graph_dirty = True

    # ------------------------------------------------------------------
    # Timer callbacks
    # ------------------------------------------------------------------
    def update_graphs(self) -> None:
        if not self._graph_dirty or not self._graphs_active:
            return

        for series in self._series:
            series.render(self._x_values)

        self._graph_dirty = False

    def _on_page_changed(self, index: int) -> None:
        is_current = self._ui.stackedWidget.widget(index) is self._ui.data_page
        if is_current == self._graphs_active:
            return

        self._graphs_active = is_current
        if is_current:
            self.graph_timer.start(self._graph_interval_ms)
            if self._graph_dirty:
                self.update_graphs()
        else:
            self.graph_timer.stop()

__all__ = ["DataPage"]
