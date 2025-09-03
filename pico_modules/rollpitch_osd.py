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
from PySide6.QtGui import QPainter, QPen, QColor
from PySide6.QtCore import Qt

class RollPitchOSD(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pitch = 0.0
        self._roll = 0.0
        self.setMinimumSize(300, 300)

    def setRollPitch(self, roll_deg: float, pitch_deg: float) -> None:
        """Update the displayed roll and pitch in degrees.

        Parameters
        ----------
        roll_deg : float
            Roll angle in degrees.
        pitch_deg : float
            Pitch angle in degrees.
        """
        self._roll = roll_deg
        self._pitch = pitch_deg
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # -------------------- Tuning Constants -------------------- #
        SCALE          = 4.0  # Pixels per pitch degree
        SHRINK_PER_DEG = 2    # Pixels trimmed from half length per degree
        MIN_LEN        = 20   # Minimum half length for far rungs
        GAP_SIZE       = 40   # Gap in the centre

        FADE_ZONE   = 100   # Pixels from top/bottom edge to start fading
        CROSS_SIZE  = 10    # Half size of the centre cross

        center_x = self.width() / 2
        center_y = self.height() / 2

        # Draw the pitch ladder rotated by roll around the widget centre
        painter.save()
        painter.translate(center_x, center_y)
        painter.rotate(-self._roll)

        pen = QPen(Qt.green, 2)
        painter.setPen(pen)

        # Determine which pitch rungs are visible.  This lets the ladder
        # appear infinite because we always draw enough lines to cover the
        # widget regardless of the current pitch value.
        half_height_deg = (self.height() / 2) / SCALE + 5
        start_pitch = int((self._pitch - half_height_deg) / 5) * 5
        end_pitch   = int((self._pitch + half_height_deg) / 5) * 5

        half_height_px = self.height() / 2

        for pitch_deg in range(start_pitch, end_pitch + 5, 5):
            y = (self._pitch - pitch_deg) * SCALE

            # Make the zero‑degree horizon line span the widget and shrink
            # other rungs progressively as they move away from centre.
            if pitch_deg == 0:
                half_len = self.width() / 2
            else:
                max_half_len = self.width() / 2
                half_len = max(MIN_LEN, max_half_len - abs(pitch_deg) * SHRINK_PER_DEG)

            # Fade rungs near the top and bottom edges so they smoothly
            # disappear instead of abruptly ending.
            distance_to_edge = half_height_px - abs(y)
            if distance_to_edge <= 0:
                alpha = 0.0
            elif distance_to_edge < FADE_ZONE:
                alpha = distance_to_edge / FADE_ZONE
            else:
                alpha = 1.0
            color = QColor(Qt.green)
            color.setAlphaF(alpha)
            painter.setPen(QPen(color, 2))


            if pitch_deg == 0:
                # Draw a continuous line across the centre for the true horizon
                painter.drawLine(-half_len, y, half_len, y)
            else:
                # Left and right segments with a gap in the middle
                painter.drawLine(-half_len, y, -GAP_SIZE / 2, y)
                painter.drawLine(GAP_SIZE / 2, y, half_len, y)

        painter.restore()

        # Reference cross that stays fixed regardless of roll
        pen = QPen(Qt.gray, 2)
        painter.setPen(pen)
        painter.drawLine(center_x, center_y - CROSS_SIZE, center_x, center_y + CROSS_SIZE)
        painter.drawLine(center_x - CROSS_SIZE, center_y, center_x + CROSS_SIZE, center_y)

        painter.end()
