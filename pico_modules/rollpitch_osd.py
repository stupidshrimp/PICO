"""Simple roll/pitch on‑screen display widget.

This module draws an artificial horizon (pitch ladder) that reacts to roll
and pitch values.  The previous version only rendered a handful of fixed
"V" shaped rungs and ignored the roll component, which made the horizon look
finite and static.  The new implementation rotates the pitch ladder with the
incoming roll value and renders enough rungs to fill the widget so it appears
infinite.  Rungs alternate between long and short segments creating the
classic staggered pattern used on real attitude indicators.
"""

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPen, QFont
from PySide6.QtCore import Qt

class RollPitchOSD(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pitch = 0.0
        self._roll = 0.0
        self.setMinimumSize(300, 300)

    def setRollPitch(self, roll, pitch):
        """Accept raw joystick readings and normalise for display."""
        # Incoming values are expected in the 0-1023 HID range. Convert them to
        # degrees so the pitch ladder renders sensibly. Roll is mapped to
        # ±180°, pitch to ±90°.
        self._roll = (roll - 512) * (180.0 / 512)
        self._pitch = (pitch - 512) * (90.0 / 512)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # -------------------- Tuning Constants -------------------- #
        SCALE       = 4.0   # Pixels per pitch degree
        MAJOR_LEN   = 80    # Half length for long rungs
        MINOR_LEN   = 40    # Half length for short rungs
        GAP_SIZE    = 40    # Gap in the centre

        center_x = self.width() / 2
        center_y = self.height() / 2

        # Draw the pitch ladder rotated by roll around the widget centre
        painter.save()
        painter.translate(center_x, center_y)
        painter.rotate(-self._roll)

        pen = QPen(Qt.green, 2)
        painter.setPen(pen)
        painter.setFont(QFont("Arial", 10))

        # Determine which pitch rungs are visible.  This lets the ladder
        # appear infinite because we always draw enough lines to cover the
        # widget regardless of the current pitch value.
        half_height_deg = (self.height() / 2) / SCALE + 5
        start_pitch = int((self._pitch - half_height_deg) / 5) * 5
        end_pitch   = int((self._pitch + half_height_deg) / 5) * 5

        for index, pitch_deg in enumerate(range(start_pitch, end_pitch + 5, 5)):
            y = (self._pitch - pitch_deg) * SCALE
            half_len = MAJOR_LEN if index % 2 == 0 else MINOR_LEN

            # Left and right segments with a gap in the middle
            painter.drawLine(-half_len, y, -GAP_SIZE / 2, y)
            painter.drawLine(GAP_SIZE / 2, y, half_len, y)

            if pitch_deg % 10 == 0:
                painter.drawText(half_len + 5, y + 3, f"{pitch_deg}")

        painter.restore()

        # Reference cross that stays fixed regardless of roll
        pen = QPen(Qt.gray, 2)
        painter.setPen(pen)
        painter.drawLine(center_x, 0, center_x, self.height())
        painter.drawLine(0, center_y, self.width(), center_y)

        painter.end()
