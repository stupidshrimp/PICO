"""Sorties page for reviewing recorded telemetry logs."""

from __future__ import annotations

import csv
import math
import os
from datetime import datetime
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QCursor
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)


class SortiesPage:
    """Encapsulates the sortie review tab and its plotting logic."""

    _STICK_SCALE_DEG = 90.0

    def __init__(self, main_window) -> None:
        self._main_window = main_window
        self._ui = main_window.ui
        self._sortie_dir = main_window.sortie_directory

        self._create_navigation_button()
        self._build_page()
        self.refresh_sortie_list()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _create_navigation_button(self) -> None:
        """Add the Sorties button to the left navigation menu."""

        button = QPushButton(self._ui.topMenu)
        button.setObjectName("btn_sorties")
        button.setSizePolicy(self._ui.btn_home.sizePolicy())
        button.setMinimumSize(self._ui.btn_home.minimumSize())
        button.setFont(self._ui.btn_home.font())
        button.setCursor(QCursor(Qt.PointingHandCursor))
        button.setStyleSheet(
            "background-image: url(:/icons/images/icons/cil-history.png);"
        )
        button.setText("Sorties")
        self._ui.verticalLayout_8.addWidget(button)
        self._ui.btn_sorties = button

    def _build_page(self) -> None:
        """Create the sorties review page and wire up controls."""

        page = QWidget()
        page.setObjectName("sorties_page")
        layout = QVBoxLayout(page)
        layout.setSpacing(12)

        title = QLabel("Sortie Analysis")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        controls = QHBoxLayout()
        controls.setSpacing(8)
        controls.addWidget(QLabel("Select sortie:"))

        self.sortie_combo = QComboBox()
        self.sortie_combo.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.sortie_combo.currentIndexChanged.connect(self._load_selected_sortie)
        controls.addWidget(self.sortie_combo, 1)

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.setCursor(QCursor(Qt.PointingHandCursor))
        self.refresh_button.clicked.connect(self.refresh_sortie_list)
        controls.addWidget(self.refresh_button)

        controls.addItem(QSpacerItem(20, 20, QSizePolicy.Expanding, QSizePolicy.Minimum))
        layout.addLayout(controls)

        self.status_label = QLabel("Select a sortie to view its telemetry plots.")
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(self.status_label)

        self.piggyback_label = QLabel("")
        self.piggyback_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.piggyback_label.hide()
        layout.addWidget(self.piggyback_label)

        # Plot widgets stacked vertically for clarity
        self.roll_plot, self.roll_actual_curve, self.roll_stick_curve = self._create_dual_plot(
            "Roll",
            "Roll (deg)",
            "Actual roll",
            "Flight stick command",
        )
        layout.addWidget(self.roll_plot, 1)

        self.pitch_plot, self.pitch_actual_curve, self.pitch_stick_curve = self._create_dual_plot(
            "Pitch",
            "Pitch (deg)",
            "Actual pitch",
            "Flight stick command",
        )
        layout.addWidget(self.pitch_plot, 1)

        self.yaw_plot, self.yaw_actual_curve, self.yaw_stick_curve = self._create_dual_plot(
            "Yaw",
            "Yaw (deg)",
            "Actual yaw",
            "Flight stick command",
        )
        layout.addWidget(self.yaw_plot, 1)

        self.speed_plot, self.airspeed_curve, self.throttle_curve = self._create_dual_plot(
            "Airspeed & Throttle",
            "Airspeed (mph)",
            "Air speed",
            "Throttle command (scaled)",
        )
        layout.addWidget(self.speed_plot, 1)

        self.altitude_plot, self.altitude_curve, _ = self._create_dual_plot(
            "Altitude",
            "Altitude (ft)",
            "Altitude",
            None,
        )
        layout.addWidget(self.altitude_plot, 1)

        self.signal_plot = self._create_signal_plot()
        layout.addWidget(self.signal_plot, 1)

        layout.addStretch(1)

        self._ui.sorties_page = page
        self._ui.stackedWidget.addWidget(page)

    def _create_dual_plot(
        self,
        title: str,
        left_label: str,
        primary_name: str,
        secondary_name: Optional[str],
    ):
        plot = pg.PlotWidget()
        plot.setBackground("k")
        plot.showGrid(x=True, y=True, alpha=0.2)
        plot.setTitle(title)
        plot.setMinimumHeight(180)
        plot.setLabel("left", left_label)
        plot.setLabel("bottom", "Time", units="s")
        legend = None
        if secondary_name:
            legend = plot.addLegend(offset=(10, -10))
        primary_curve = plot.plot(
            pen=pg.mkPen(color="#4adede", width=2),
            name=primary_name,
        )
        secondary_curve = None
        if secondary_name:
            secondary_curve = plot.plot(
                pen=pg.mkPen(color="#f6c90e", width=2, style=Qt.PenStyle.DashLine),
                name=secondary_name,
            )
        return plot, primary_curve, secondary_curve

    def _create_signal_plot(self) -> pg.PlotWidget:
        plot = pg.PlotWidget()
        plot.setBackground("k")
        plot.showGrid(x=True, y=True, alpha=0.2)
        plot.setTitle("Signal Health")
        plot.setMinimumHeight(180)
        plot.setLabel("left", "Signal metrics")
        plot.setLabel("bottom", "Time", units="s")
        legend = plot.addLegend(offset=(10, -10))
        self.snr_curve = plot.plot(
            pen=pg.mkPen(color="#70ff70", width=2),
            name="SNR (dB)",
        )
        self.link_quality_curve = plot.plot(
            pen=pg.mkPen(color="#4adede", width=2, style=Qt.PenStyle.DotLine),
            name="Link quality (%)",
        )
        self.rssi_curve = plot.plot(
            pen=pg.mkPen(color="#f6c90e", width=2),
            name="RSSI avg (dBm)",
        )
        return plot

    # ------------------------------------------------------------------
    # Sortie listing helpers
    # ------------------------------------------------------------------
    def refresh_sortie_list(self) -> None:
        """Reload the sortie directory into the combo box."""

        current_path = self.sortie_combo.currentData()
        files: list[str] = []
        if os.path.isdir(self._sortie_dir):
            for name in os.listdir(self._sortie_dir):
                if name.lower().endswith(".csv"):
                    files.append(name)
        files.sort(reverse=True)

        self.sortie_combo.blockSignals(True)
        self.sortie_combo.clear()
        for name in files:
            path = os.path.join(self._sortie_dir, name)
            self.sortie_combo.addItem(name, path)
        self.sortie_combo.blockSignals(False)

        if not files:
            self.status_label.setText("No sortie logs found. Record a flight to get started.")
            self._clear_plots()
            self.piggyback_label.hide()
            return

        if current_path:
            for index in range(self.sortie_combo.count()):
                if self.sortie_combo.itemData(index) == current_path:
                    self.sortie_combo.setCurrentIndex(index)
                    self._load_selected_sortie()
                    return

        self.sortie_combo.setCurrentIndex(0)
        self._load_selected_sortie()

    def _load_selected_sortie(self) -> None:
        path = self.sortie_combo.currentData()
        if not path:
            self._clear_plots()
            self.status_label.setText("Select a sortie to view its telemetry plots.")
            self.piggyback_label.hide()
            return

        dataset = self._parse_sortie_file(path)
        if dataset is None:
            self._clear_plots()
            basename = os.path.basename(path)
            self.status_label.setText(f"Failed to load {basename}. File may be empty or corrupted.")
            self.piggyback_label.hide()
            return

        self._update_plots(dataset)
        basename = os.path.basename(path)
        duration = dataset["time"][-1] if dataset["time"].size else 0.0
        samples = dataset["time"].size
        piggyback_indicator = dataset.get("link_piggyback_indicator")
        piggyback_events = 0
        if piggyback_indicator is not None and piggyback_indicator.size:
            piggyback_events = int(np.nansum(piggyback_indicator))
        self.status_label.setText(
            f"{basename} — {samples} samples, duration {duration:.1f} s"
        )
        if piggyback_events:
            self.piggyback_label.setText(
                f"{piggyback_events} link-stat packets carried piggyback telemetry."
            )
        else:
            self.piggyback_label.setText(
                "No piggyback telemetry was recorded in link statistics packets."
            )
        self.piggyback_label.show()

    # ------------------------------------------------------------------
    # Data parsing and plotting
    # ------------------------------------------------------------------
    def _parse_sortie_file(self, path: str) -> Optional[dict[str, np.ndarray]]:
        try:
            with open(path, newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                if reader.fieldnames is None:
                    return None
                timestamps: list[datetime] = []
                series: dict[str, list[float]] = {
                    "roll": [],
                    "pitch": [],
                    "yaw": [],
                    "stick_roll": [],
                    "stick_pitch": [],
                    "stick_yaw": [],
                    "stick_throttle": [],
                    "airspeed_mph": [],
                    "altitude_ft": [],
                    "snr": [],
                    "link_quality": [],
                    "rssi_a": [],
                    "rssi_b": [],
                    "link_piggyback_count": [],
                }
                piggyback_indicator: list[float] = []
                for row in reader:
                    timestamp = row.get("timestamp")
                    if not timestamp:
                        continue
                    try:
                        dt = datetime.fromisoformat(timestamp)
                    except ValueError:
                        try:
                            dt = datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S.%f")
                        except ValueError:
                            continue
                    timestamps.append(dt)
                    for key in series.keys():
                        series[key].append(self._parse_float(row.get(key)))
                    piggyback_value = series["link_piggyback_count"][-1]
                    packet_type = (row.get("packet_type") or "").strip().lower()
                    if (
                        packet_type == "link_stats"
                        and math.isfinite(piggyback_value)
                        and piggyback_value > 0.0
                    ):
                        piggyback_indicator.append(1.0)
                    else:
                        piggyback_indicator.append(0.0)
        except OSError:
            return None

        if not timestamps:
            return None

        start = timestamps[0]
        times = np.array([(ts - start).total_seconds() for ts in timestamps], dtype=float)
        dataset: dict[str, np.ndarray] = {"time": times}
        for key, values in series.items():
            dataset[key] = self._to_array(values)
        dataset["link_piggyback_indicator"] = self._to_array(piggyback_indicator)
        return dataset

    @staticmethod
    def _parse_float(value: Optional[str]) -> float:
        if value is None:
            return float("nan")
        value = value.strip()
        if not value or value.lower() == "none":
            return float("nan")
        try:
            return float(value)
        except ValueError:
            return float("nan")

    @staticmethod
    def _to_array(values: list[float]) -> np.ndarray:
        if not values:
            return np.array([], dtype=float)
        return np.array(values, dtype=float)

    def _update_plots(self, dataset: dict[str, np.ndarray]) -> None:
        time_values = dataset["time"]

        roll = dataset["roll"]
        stick_roll = self._scale_stick_angles(dataset["stick_roll"])
        self._set_curve_data(self.roll_actual_curve, time_values, roll)
        self._set_curve_data(self.roll_stick_curve, time_values, stick_roll)

        pitch = dataset["pitch"]
        stick_pitch = self._scale_stick_angles(dataset["stick_pitch"])
        self._set_curve_data(self.pitch_actual_curve, time_values, pitch)
        self._set_curve_data(self.pitch_stick_curve, time_values, stick_pitch)

        yaw = dataset["yaw"]
        stick_yaw = self._scale_stick_angles(dataset["stick_yaw"])
        self._set_curve_data(self.yaw_actual_curve, time_values, yaw)
        self._set_curve_data(self.yaw_stick_curve, time_values, stick_yaw)

        airspeed = dataset["airspeed_mph"]
        throttle = self._prepare_throttle(dataset["stick_throttle"])
        throttle_scaled = throttle * self._determine_scale(airspeed, throttle)
        self._set_curve_data(self.airspeed_curve, time_values, airspeed)
        self._set_curve_data(self.throttle_curve, time_values, throttle_scaled)

        altitude = dataset["altitude_ft"]
        self._set_curve_data(self.altitude_curve, time_values, altitude)

        snr = dataset["snr"]
        link_quality = dataset["link_quality"]
        rssi_avg = self._average_rssi(dataset["rssi_a"], dataset["rssi_b"])
        self._set_curve_data(self.snr_curve, time_values, snr)
        self._set_curve_data(self.link_quality_curve, time_values, link_quality)
        self._set_curve_data(self.rssi_curve, time_values, rssi_avg)

    def _set_curve_data(self, curve: Optional[pg.PlotDataItem], x: np.ndarray, y: np.ndarray) -> None:
        if curve is None:
            return
        if x.size == 0 or y.size == 0:
            curve.setData([], [])
            return
        if not np.any(np.isfinite(y)):
            curve.setData([], [])
            return
        curve.setData(x, y)

    def _clear_plots(self) -> None:
        for curve in (
            self.roll_actual_curve,
            self.roll_stick_curve,
            self.pitch_actual_curve,
            self.pitch_stick_curve,
            self.yaw_actual_curve,
            self.yaw_stick_curve,
            self.airspeed_curve,
            self.throttle_curve,
            self.altitude_curve,
            self.snr_curve,
            self.link_quality_curve,
            self.rssi_curve,
        ):
            if curve is not None:
                curve.setData([], [])
        self.piggyback_label.hide()

    def _scale_stick_angles(self, values: np.ndarray) -> np.ndarray:
        if values.size == 0:
            return values
        scaled = values.copy()
        max_abs = self._nanmax_abs(scaled)
        if not np.isnan(max_abs) and max_abs <= 1.5:
            scaled *= self._STICK_SCALE_DEG
        return scaled

    def _prepare_throttle(self, values: np.ndarray) -> np.ndarray:
        if values.size == 0:
            return values
        throttle = values.copy()
        max_abs = self._nanmax_abs(throttle)
        if not np.isnan(max_abs) and max_abs <= 1.5:
            throttle *= 100.0
        return throttle

    def _determine_scale(self, reference: np.ndarray, series: np.ndarray) -> float:
        ref_max = self._nanmax_abs(reference)
        series_max = self._nanmax_abs(series)
        if np.isnan(ref_max) or np.isnan(series_max) or series_max == 0:
            return 1.0
        return ref_max / series_max

    def _average_rssi(self, rssi_a: np.ndarray, rssi_b: np.ndarray) -> np.ndarray:
        if rssi_a.size == 0 or rssi_b.size == 0:
            return np.array([], dtype=float)
        sum_values = np.zeros_like(rssi_a, dtype=float)
        counts = np.zeros_like(rssi_a, dtype=float)
        for values in (rssi_a, rssi_b):
            mask = np.isfinite(values)
            sum_values[mask] += values[mask]
            counts[mask] += 1
        with np.errstate(divide="ignore", invalid="ignore"):
            avg = sum_values / counts
        avg[counts == 0] = np.nan
        return avg

    @staticmethod
    def _nanmax_abs(values: np.ndarray) -> float:
        if values.size == 0:
            return float("nan")
        finite = np.isfinite(values)
        if not np.any(finite):
            return float("nan")
        return float(np.max(np.abs(values[finite])))
