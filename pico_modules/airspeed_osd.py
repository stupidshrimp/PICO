"""Airspeed tape on-screen display widget.

This widget renders a vertical airspeed tape similar to those seen on
primary flight displays. It accepts airspeed values (in miles per hour)
and draws a scrolling scale with tick marks and a central readout showing
the current airspeed. The telemetry source is expected to supply the
airspeed value in mph; here we only provide the visual representation.
"""

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPen, QFont, QColor, QPolygon
from PySide6.QtCore import Qt, QPoint


class AirspeedOSD(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._airspeed = 0.0
        self.setMinimumWidth(80)

    def setAirspeed(self, airspeed: float) -> None:
        """Update the displayed airspeed.

        Parameters
        ----------
        airspeed: float
            Airspeed value in miles per hour.
        """
        self._airspeed = airspeed
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # -------------------- Display constants -------------------- #
        SCALE = 2.0              # Pixels per airspeed unit (mph)
        TICK_INTERVAL = 5        # Minor tick every 5 mph
        MAJOR_INTERVAL = 25      # Major tick every 25 mph
        MAJOR_LEN = 20           # Length of major tick in pixels
        MINOR_LEN = 10           # Length of minor tick in pixels
        BOX_HEIGHT = 40          # Height of centre readout box

        center_y = self.height() / 2
        half_height_units = (self.height() / SCALE) / 2

        start_spd = int(self._airspeed - half_height_units) - (
            int(self._airspeed - half_height_units) % TICK_INTERVAL
        ) - TICK_INTERVAL
        end_spd = int(self._airspeed + half_height_units) + TICK_INTERVAL

        pen = QPen(Qt.green, 2)
        painter.setPen(pen)
        painter.setFont(QFont("Arial", 10))

        # Draw tick marks and labels
        for spd in range(start_spd, end_spd + TICK_INTERVAL, TICK_INTERVAL):
            y = center_y + (self._airspeed - spd) * SCALE
            if spd % MAJOR_INTERVAL == 0:
                tick_len = MAJOR_LEN
                painter.drawLine(0, y, tick_len, y)
                painter.drawText(tick_len + 5, y + 4, f"{spd}")
            else:
                tick_len = MINOR_LEN
                painter.drawLine(0, y, tick_len, y)

        # Draw centre readout box
        box_top = center_y - BOX_HEIGHT / 2
        painter.fillRect(0, box_top, self.width(), BOX_HEIGHT, QColor(0, 0, 0, 180))
        painter.drawRect(0, box_top, self.width() - 1, BOX_HEIGHT - 1)
        painter.drawText(self.width() - 60, center_y + 8, f"{int(self._airspeed)} mph")

        # Draw pointer triangle to right of readout box
        painter.setBrush(Qt.green)
        pointer = QPolygon([
            QPoint(self.width(), center_y),
            QPoint(self.width() - 15, center_y - 10),
            QPoint(self.width() - 15, center_y + 10),
        ])
        painter.drawPolygon(pointer)
        painter.end()
