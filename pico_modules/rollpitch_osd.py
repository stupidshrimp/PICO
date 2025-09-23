"""Simple roll/pitch on‑screen display widget.

This module draws an artificial horizon (pitch ladder) that reacts to roll
and pitch values.  The previous version only rendered a handful of fixed
"V" shaped rungs and ignored the roll component, which made the horizon look
finite and static.  The new implementation rotates the pitch ladder with the
incoming roll value and renders enough rungs to fill the widget so it appears
infinite.  Rungs now cycle through small, medium, small and large segments
creating a repeating pattern used on real attitude indicators.
"""

import math

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QPen, QColor
from PySide6.QtCore import Qt

class RollPitchOSD(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pitch = 0.0
        self._roll = 0.0
        self._initialized = False
        self._smoothing = 0.2  # Weight for new samples
        self._desired_roll = None
        self._desired_pitch = None
        self._fd_roll_range = 45.0
        self._fd_pitch_range = 25.0
        self.setMinimumSize(300, 300)

    def set_smoothing(self, weight: float) -> None:
        """Adjust the smoothing factor applied to new roll/pitch samples.

        ``weight`` represents the proportion of a new sample to mix into the
        running average. Values greater than 1.0 are treated as percentages and
        scaled accordingly. The final weight is clamped to the range
        ``[0.01, 1.0]`` to prevent the indicator from freezing entirely.
        """

        try:
            value = float(weight)
        except (TypeError, ValueError):
            return

        if value > 1.0:
            value /= 100.0

        self._smoothing = max(0.01, min(1.0, value))

    def setRollPitch(self, roll_deg: float, pitch_deg: float) -> None:
        """Update the displayed roll and pitch in degrees.

        Parameters
        ----------
        roll_deg : float
            Roll angle in degrees.
        pitch_deg : float
            Pitch angle in degrees.
        """
        if (
            roll_deg is None
            or pitch_deg is None
            or not math.isfinite(roll_deg)
            or not math.isfinite(pitch_deg)
        ):
            return
        if not self._initialized:
            self._roll = roll_deg
            self._pitch = pitch_deg
            self._initialized = True
        else:
            self._roll = self._roll * (1 - self._smoothing) + roll_deg * self._smoothing
            self._pitch = self._pitch * (1 - self._smoothing) + pitch_deg * self._smoothing
        self.update()

    def setDesiredAttitude(self, roll_deg, pitch_deg) -> None:
        """Display a flight director cue indicating the desired attitude."""

        if roll_deg is None or pitch_deg is None:
            if self._desired_roll is not None or self._desired_pitch is not None:
                self._desired_roll = None
                self._desired_pitch = None
                self.update()
            return

        try:
            new_roll = float(roll_deg)
            new_pitch = float(pitch_deg)
        except (TypeError, ValueError):
            return

        if self._desired_roll != new_roll or self._desired_pitch != new_pitch:
            self._desired_roll = new_roll
            self._desired_pitch = new_pitch
            self.update()

    def setFlightDirectorRanges(self, roll_range: float, pitch_range: float) -> None:
        """Configure the range in degrees used for the flight director cue."""

        try:
            roll_range = float(roll_range)
            pitch_range = float(pitch_range)
        except (TypeError, ValueError):
            return

        roll_range = max(1.0, abs(roll_range))
        pitch_range = max(1.0, abs(pitch_range))

        if (
            self._fd_roll_range != roll_range
            or self._fd_pitch_range != pitch_range
        ):
            self._fd_roll_range = roll_range
            self._fd_pitch_range = pitch_range
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

        yellow = blend(green, orange, 0.5)

        COLOR_SCALE = 1.5
        GREEN_LIMIT = 7 * COLOR_SCALE
        YELLOW_LIMIT = 14 * COLOR_SCALE
        ORANGE_LIMIT = 21 * COLOR_SCALE

        for pitch_x2 in range(start_pitch, end_pitch + 5, 5):
            pitch_deg = pitch_x2 / 2.0
            # Invert pitch direction so a positive input is drawn upward
            y = (pitch_deg - self._pitch) * SCALE

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
            if rung_index < GREEN_LIMIT:
                color = green
            elif rung_index <= YELLOW_LIMIT:
                t = (rung_index - GREEN_LIMIT) / (YELLOW_LIMIT - GREEN_LIMIT)
                color = blend(yellow, orange, t)
            elif rung_index <= ORANGE_LIMIT:
                t = (rung_index - YELLOW_LIMIT) / (ORANGE_LIMIT - YELLOW_LIMIT)
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

        self._draw_flight_director(painter, center_x, center_y, SCALE)

        painter.end()

    def _draw_flight_director(self, painter: QPainter, center_x: float, center_y: float, scale: float) -> None:
        """Render magenta crossbars that indicate the desired attitude."""

        if self._desired_roll is None or self._desired_pitch is None:
            return

        roll_error = self._desired_roll - self._roll
        pitch_error = self._desired_pitch - self._pitch

        half_width = self.width() / 2
        half_height = self.height() / 2
        margin = 20

        if half_width <= margin or half_height <= margin:
            return

        if self._fd_roll_range > 0:
            roll_scale_px = (half_width - margin) / self._fd_roll_range
        else:
            roll_scale_px = 0.0
        x_offset = roll_error * roll_scale_px
        x_offset = max(-half_width + margin, min(half_width - margin, x_offset))

        y_offset = -pitch_error * scale
        y_offset = max(-half_height + margin, min(half_height - margin, y_offset))

        bar_length = min(self.width(), self.height()) * 0.25
        bar_length = max(30.0, bar_length)
        bar_half = bar_length / 2

        cue_pen = QPen(QColor(255, 0, 255), 4)
        cue_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(cue_pen)

        painter.drawLine(center_x - bar_half, center_y + y_offset, center_x + bar_half, center_y + y_offset)
        painter.drawLine(center_x + x_offset, center_y - bar_half, center_x + x_offset, center_y + bar_half)

        size = 6
        painter.fillRect(
            center_x + x_offset - size / 2,
            center_y + y_offset - size / 2,
            size,
            size,
            QColor(255, 0, 255),
        )
