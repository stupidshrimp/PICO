"""Compass (yaw) on-screen display widget.

This widget renders a horizontal heading tape similar to those found on
modern aircraft primary flight displays.  It accepts yaw values in
degrees and draws a scrolling compass scale with tick marks and labels.
A vertical line at the centre of the widget indicates the current heading.
"""

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPen, QFont, QColor
from PySide6.QtCore import Qt


class CompassOSD(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._yaw = 0.0
        self.setMinimumHeight(50)

    def setYaw(self, yaw_deg: float) -> None:
        """Update the displayed yaw in degrees."""
        # Normalize yaw to 0-360 range
        self._yaw = yaw_deg % 360.0
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # Opaque background to avoid cutting with the horizon widget behind
        painter.fillRect(self.rect(), QColor(0, 0, 0, 180))

        # -------------------- Display constants -------------------- #
        SCALE = 4                 # Pixels per degree
        TICK_INTERVAL = 5         # Minor tick every 5 deg
        MAJOR_INTERVAL = 30       # Major tick every 30 deg
        MAJOR_LEN = 15            # Major tick length in pixels
        MINOR_LEN = 7             # Minor tick length in pixels
        FADE_ZONE = 40            # Pixels from edge where ticks fade

        center_x = self.width() / 2
        height = self.height()

        half_width_deg = self.width() / (2 * SCALE)
        start_deg = int(self._yaw - half_width_deg) - TICK_INTERVAL
        end_deg = int(self._yaw + half_width_deg) + TICK_INTERVAL

        painter.setFont(QFont("Arial", 10))

        # Draw tick marks and labels
        for deg in range(start_deg, end_deg + 1, TICK_INTERVAL):
            x = center_x + (deg - self._yaw) * SCALE

            distance_to_edge = min(x, self.width() - x)
            if distance_to_edge <= 0:
                alpha = 0.0
            elif distance_to_edge < FADE_ZONE:
                alpha = distance_to_edge / FADE_ZONE
            else:
                alpha = 1.0
            color = QColor(0, 255, 0)
            color.setAlphaF(alpha)
            painter.setPen(QPen(color, 2))

            if deg % MAJOR_INTERVAL == 0:
                heading = deg % 360
                if heading == 0:
                    tick_len = int(MAJOR_LEN * 1.5)
                else:
                    tick_len = MAJOR_LEN
                painter.drawLine(x, height, x, height - tick_len)
                if heading == 0:
                    label = "N"
                elif heading == 90:
                    label = "E"
                elif heading == 180:
                    label = "S"
                elif heading == 270:
                    label = "W"
                else:
                    label = f"{heading:03d}"
                painter.drawText(x - 10, height - tick_len - 2, label)
            else:
                painter.drawLine(x, height, x, height - MINOR_LEN)

        # Centre indicator showing current heading
        # Draw a simple vertical line rather than a triangular arrow
        painter.setPen(QPen(Qt.green, 2))
        painter.drawLine(center_x, 0, center_x, height)

        painter.end()
