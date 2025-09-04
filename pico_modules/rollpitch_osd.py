"""Simple roll/pitch on‑screen display widget.

This module draws an artificial horizon (pitch ladder) that reacts to roll
and pitch values.  The previous version only rendered a handful of fixed
"V" shaped rungs and ignored the roll component, which made the horizon look
finite and static.  The new implementation rotates the pitch ladder with the
incoming roll value and renders enough rungs to fill the widget so it appears
infinite.  Rungs now cycle through small, medium, small and large segments
creating a repeating pattern used on real attitude indicators.
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
        SCALE         = 4.0  # Pixels per pitch degree
        GAP_SIZE      = 40   # Gap in the centre
        FADE_ZONE     = 100  # Pixels from top/bottom edge to start fading
        CROSS_SIZE    = 10   # Half size of the centre cross

        PITCH_STEP     = 2.5 # Degrees between rungs

        ZERO_FRACTION   = 0.4  # 0° line length as fraction of full width
        LARGE_FRACTION  = 0.2  # Large rung length as fraction of full width
        MEDIUM_FRACTION = 0.15 # Medium rung length as fraction of full width
        SMALL_FRACTION  = 0.1  # Small rung length as fraction of full width

        center_x = self.width() / 2
        center_y = self.height() / 2

        # Draw the pitch ladder rotated by roll around the widget centre
        painter.save()
        painter.translate(center_x, center_y)
        painter.rotate(-self._roll)

        # Determine which pitch rungs are visible.  This lets the ladder
        # appear infinite because we always draw enough lines to cover the
        # widget regardless of the current pitch value.
        half_height_deg = (self.height() / 2) / SCALE + PITCH_STEP

        start_pitch = int((self._pitch - half_height_deg) * 2)
        start_pitch = start_pitch - (start_pitch % 5) - 5
        end_pitch   = int((self._pitch + half_height_deg) * 2) + 5

        half_height_px = self.height() / 2
        half_width = self.width() / 2
        zero_half_len = half_width * ZERO_FRACTION
        large_half_len = half_width * LARGE_FRACTION
        medium_half_len = half_width * MEDIUM_FRACTION
        small_half_len = half_width * SMALL_FRACTION

        green = QColor(0, 255, 0)
        orange = QColor(255, 165, 0)
        red = QColor(255, 0, 0)

        def blend(c1: QColor, c2: QColor, t: float) -> QColor:
            r = c1.red() + (c2.red() - c1.red()) * t
            g = c1.green() + (c2.green() - c1.green()) * t
            b = c1.blue() + (c2.blue() - c1.blue()) * t
            return QColor(int(r), int(g), int(b))

        for pitch_x2 in range(start_pitch, end_pitch + 5, 5):
            pitch_deg = pitch_x2 / 2.0
            y = (self._pitch - pitch_deg) * SCALE

            # Render a longer zero-degree horizon line and apply a repeating
            # small, medium, small, large pattern to the other rungs.
            rung_index = int(abs(pitch_x2) / 5)
            if pitch_deg == 0:
                half_len = zero_half_len
            else:
                pattern = [small_half_len, medium_half_len, small_half_len, large_half_len]
                half_len = pattern[(rung_index - 1) % 4]

            # Fade rungs near the top and bottom edges so they smoothly
            # disappear instead of abruptly ending.
            distance_to_edge = half_height_px - abs(y)
            if distance_to_edge <= 0:
                alpha = 0.0
            elif distance_to_edge < FADE_ZONE:
                alpha = distance_to_edge / FADE_ZONE
            else:
                alpha = 1.0
            if rung_index <= 7:
                t = rung_index / 7.0
                color = blend(green, orange, t)
            elif rung_index <= 14:
                t = (rung_index - 7) / 7.0
                color = blend(orange, red, t)
            else:
                color = red

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
