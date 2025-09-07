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
        self._initialized = False
        self._smoothing = 0.2  # Weight for new samples
        self.setMinimumWidth(80)
        # Allow the widget to blend with anything behind it
        self.setAttribute(Qt.WA_TranslucentBackground)

    def setAirspeed(self, airspeed: float) -> None:
        """Update the displayed airspeed.

        Parameters
        ----------
        airspeed: float
            Airspeed value in miles per hour.
        """
        if not self._initialized:
            self._airspeed = airspeed
            self._initialized = True
        else:
            self._airspeed = (
                self._airspeed * (1 - self._smoothing) + airspeed * self._smoothing
            )
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # -------------------- Display constants -------------------- #
        SCALE = 8                # Pixels per airspeed unit (mph)
        TICK_INTERVAL = 1        # Minor tick every 1 mph
        MAJOR_INTERVAL = 5       # Major tick every 5 mph
        MAJOR_LEN = 20           # Length of major tick in pixels
        MINOR_LEN = 10           # Length of minor tick in pixels
        BOX_HEIGHT = 40          # Height of centre readout box

        center_y = self.height() / 2
        half_height_units = (self.height() / SCALE) / 2

        start_spd = int(self._airspeed - half_height_units) - (
            int(self._airspeed - half_height_units) % TICK_INTERVAL
        ) - TICK_INTERVAL
        end_spd = int(self._airspeed + half_height_units) + TICK_INTERVAL

        painter.setFont(QFont("Arial", 10))

        FADE_HEIGHT = 30  # Pixels from top/bottom edge to start fading

        # Draw tick marks and labels with alpha fade near the edges
        for spd in range(start_spd, end_spd + TICK_INTERVAL, TICK_INTERVAL):
            y = center_y + (self._airspeed - spd) * SCALE

            distance_to_edge = min(y, self.height() - y)
            if distance_to_edge < FADE_HEIGHT:
                alpha = distance_to_edge / FADE_HEIGHT
            else:
                alpha = 1.0
            color = QColor(0, 255, 0)
            color.setAlphaF(alpha)
            pen = QPen(color, 2)
            painter.setPen(pen)

            if spd % MAJOR_INTERVAL == 0:
                painter.drawLine(0, y, MAJOR_LEN, y)
                painter.drawText(MAJOR_LEN + 5, y + 4, f"{spd}")
            else:
                painter.drawLine(0, y, MINOR_LEN, y)

        # Draw centre readout box
        painter.setPen(QPen(Qt.green, 2))
        box_top = center_y - BOX_HEIGHT / 2
        painter.fillRect(0, box_top, self.width(), BOX_HEIGHT, QColor(0, 0, 0, 180))
        painter.drawRect(0, box_top, self.width() - 1, BOX_HEIGHT - 1)
        painter.drawText(self.width() - 60, center_y + 8, f"{int(self._airspeed)} mph")

        # Draw pointer triangle to right of readout box
        painter.setBrush(Qt.green)
        painter.setPen(QPen(Qt.green, 2))
        pointer = QPolygon([
            QPoint(self.width(), center_y),
            QPoint(self.width() - 15, center_y - 10),
            QPoint(self.width() - 15, center_y + 10),
        ])
        painter.drawPolygon(pointer)

        painter.end()
