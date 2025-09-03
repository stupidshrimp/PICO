"""Altitude tape on-screen display widget.

This widget renders a vertical altitude tape similar to those seen on
primary flight displays. It accepts altitude values (e.g., in feet or
meters) and draws a scrolling scale with tick marks and a central readout
showing the current altitude.  The telemetry source is expected to supply
the altitude value; here we only provide the visual representation.
"""

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPen, QFont, QColor, QPolygon
from PySide6.QtCore import Qt, QPoint


class AltitudeOSD(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._altitude = 0.0
        self.setMinimumWidth(80)

    def setAltitude(self, altitude: float) -> None:
        """Update the displayed altitude.

        Parameters
        ----------
        altitude: float
            Altitude value in user-defined units (e.g., feet or meters).
        """
        self._altitude = altitude
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # -------------------- Display constants -------------------- #
        SCALE = 0.5               # Pixels per altitude unit
        TICK_INTERVAL = 20        # Minor tick every 20 units
        MAJOR_INTERVAL = 100      # Major tick every 100 units
        MAJOR_LEN = 20            # Length of major tick in pixels
        MINOR_LEN = 10            # Length of minor tick in pixels
        BOX_HEIGHT = 40           # Height of centre readout box

        center_y = self.height() / 2
        half_height_units = (self.height() / SCALE) / 2

        start_alt = int(self._altitude - half_height_units) - (
            int(self._altitude - half_height_units) % TICK_INTERVAL
        ) - TICK_INTERVAL
        end_alt = int(self._altitude + half_height_units) + TICK_INTERVAL

        pen = QPen(Qt.green, 2)
        painter.setPen(pen)
        painter.setFont(QFont("Arial", 10))

        # Draw tick marks and labels
        for alt in range(start_alt, end_alt + TICK_INTERVAL, TICK_INTERVAL):
            y = center_y + (self._altitude - alt) * SCALE
            if alt % MAJOR_INTERVAL == 0:
                tick_len = MAJOR_LEN
                painter.drawLine(self.width() - tick_len, y, self.width(), y)
                painter.drawText(self.width() - tick_len - 35, y + 4, f"{alt}")
            else:
                tick_len = MINOR_LEN
                painter.drawLine(self.width() - tick_len, y, self.width(), y)

        # Draw centre readout box
        box_top = center_y - BOX_HEIGHT / 2
        painter.fillRect(0, box_top, self.width(), BOX_HEIGHT, QColor(0, 0, 0, 180))
        painter.drawRect(0, box_top, self.width() - 1, BOX_HEIGHT - 1)
        painter.drawText(5, center_y + 8, f"{int(self._altitude)}")

        # Draw pointer triangle to left of readout box
        painter.setBrush(Qt.green)
        pointer = QPolygon([
            QPoint(0, center_y),
            QPoint(15, center_y - 10),
            QPoint(15, center_y + 10),
        ])
        painter.drawPolygon(pointer)
        painter.end()
