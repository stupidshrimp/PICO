"""Altitude tape on-screen display widget.

This widget renders a vertical altitude tape similar to those seen on
primary flight displays. It accepts altitude values in feet and draws a

scrolling scale with tick marks every 10 ft and a central readout showing
the current altitude. The telemetry source is expected to supply the
altitude value; here we only provide the visual representation.
"""

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPen, QFont, QColor, QPolygon
from PySide6.QtCore import Qt, QPoint


class AltitudeOSD(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._altitude = 0.0
        self._initialized = False
        self._smoothing = 0.2  # Weight for new samples
        self.setMinimumWidth(80)
        # Allow the widget to blend with anything behind it
        self.setAttribute(Qt.WA_TranslucentBackground)

    def setAltitude(self, altitude: float) -> None:
        """Update the displayed altitude.

        Parameters
        ----------
        altitude: float
            Altitude value in feet.
        """
        if not self._initialized:
            self._altitude = altitude
            self._initialized = True
        else:
            self._altitude = (
                self._altitude * (1 - self._smoothing) + altitude * self._smoothing
            )
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # -------------------- Display constants -------------------- #
        SCALE = 0.5               # Pixels per altitude foot
        TICK_INTERVAL = 10        # Minor tick every 10 ft
        TICKS_PER_LABEL = 10      # Label every 10 ticks (100 ft)

        MAJOR_LEN = 20            # Length of major tick in pixels
        MINOR_LEN = 10            # Length of minor tick in pixels
        BOX_HEIGHT = 40           # Height of centre readout box

        center_y = self.height() / 2
        half_height_units = (self.height() / SCALE) / 2

        start_alt = int(self._altitude - half_height_units) - (
            int(self._altitude - half_height_units) % TICK_INTERVAL
        ) - TICK_INTERVAL
        end_alt = int(self._altitude + half_height_units) + TICK_INTERVAL

        painter.setFont(QFont("Arial", 10))

        FADE_HEIGHT = 30  # Pixels from top/bottom edge to start fading

        # Draw tick marks and labels with alpha fade near the edges
        for alt in range(start_alt, end_alt + TICK_INTERVAL, TICK_INTERVAL):
            y = center_y + (self._altitude - alt) * SCALE

            distance_to_edge = min(y, self.height() - y)
            if distance_to_edge < FADE_HEIGHT:
                alpha = distance_to_edge / FADE_HEIGHT
            else:
                alpha = 1.0
            color = QColor(0, 255, 0)
            color.setAlphaF(alpha)
            pen = QPen(color, 2)
            painter.setPen(pen)

            if (alt // TICK_INTERVAL) % TICKS_PER_LABEL == 0:
                tick_len = MAJOR_LEN
                painter.drawLine(self.width() - tick_len, y, self.width(), y)
                painter.drawText(
                    self.width() - tick_len - 35, y + 4, f"{alt // TICK_INTERVAL}"
                )
            else:
                tick_len = MINOR_LEN
                painter.drawLine(self.width() - tick_len, y, self.width(), y)

        # Draw centre readout box
        painter.setPen(QPen(Qt.green, 2))
        box_top = center_y - BOX_HEIGHT / 2
        painter.fillRect(0, box_top, self.width(), BOX_HEIGHT, QColor(0, 0, 0, 180))
        painter.drawRect(0, box_top, self.width() - 1, BOX_HEIGHT - 1)
        painter.drawText(5, center_y + 8, f"{int(self._altitude)}")

        # Draw pointer triangle to left of readout box
        painter.setBrush(Qt.green)
        painter.setPen(QPen(Qt.green, 2))
        pointer = QPolygon([
            QPoint(0, center_y),
            QPoint(15, center_y - 10),
            QPoint(15, center_y + 10),
        ])
        painter.drawPolygon(pointer)

        painter.end()
