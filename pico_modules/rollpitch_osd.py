"""Simple roll/pitch on-screen display widget.

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
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtCore import QPointF, QRectF, Qt


class RollPitchOSD(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pitch = 0.0
        self._roll = 0.0
        self._desired_pitch = None
        self._desired_roll = None
        self._desired_visible = False
        self._initialized = False
        self._smoothing = 0.2  # Weight for new samples
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

    def setDesiredRollPitch(
        self, roll_deg: float | None, pitch_deg: float | None, visible: bool = True
    ) -> None:
        """Update the FBW desired-attitude overlay.

        The desired cue is intentionally unsmoothed so it represents the latest
        command sent by the ground station rather than the delayed/smoothed
        telemetry attitude.
        """

        if (
            not visible
            or roll_deg is None
            or pitch_deg is None
            or not math.isfinite(roll_deg)
            or not math.isfinite(pitch_deg)
        ):
            self._desired_visible = False
            self._desired_roll = None
            self._desired_pitch = None
            self.update()
            return

        self._desired_roll = float(roll_deg)
        self._desired_pitch = float(pitch_deg)
        self._desired_visible = True
        self.update()

    def _draw_desired_attitude_overlay(
        self, painter: QPainter, center_x: float, center_y: float, scale: float
    ) -> None:
        """Draw a modern FBW command cue over the actual attitude ladder."""

        if (
            not self._desired_visible
            or self._desired_roll is None
            or self._desired_pitch is None
        ):
            return

        cyan = QColor(0, 229, 255)
        cyan_soft = QColor(0, 229, 255, 82)
        cyan_glow = QColor(0, 229, 255, 42)
        ink = QColor(4, 16, 22, 205)
        white = QColor(235, 252, 255)

        desired_y = self._desired_pitch * scale
        half_width = self.width() / 2
        cue_half_len = max(52.0, half_width * 0.32)
        center_gap = 30.0
        tick = 9.0

        painter.save()
        painter.translate(center_x, center_y)
        painter.rotate(self._desired_roll)

        # Soft glow beneath the target line keeps the command cue readable on
        # bright video while preserving the existing ladder color language.
        painter.setPen(
            QPen(cyan_glow, 9, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
        )
        painter.drawLine(-cue_half_len, desired_y, -center_gap, desired_y)
        painter.drawLine(center_gap, desired_y, cue_half_len, desired_y)

        painter.setPen(
            QPen(cyan_soft, 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
        )
        painter.drawLine(-cue_half_len, desired_y, -center_gap, desired_y)
        painter.drawLine(center_gap, desired_y, cue_half_len, desired_y)

        painter.setPen(QPen(cyan, 2, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(-cue_half_len, desired_y, -center_gap, desired_y)
        painter.drawLine(center_gap, desired_y, cue_half_len, desired_y)
        painter.drawLine(
            -cue_half_len, desired_y - tick, -cue_half_len, desired_y + tick
        )
        painter.drawLine(
            cue_half_len, desired_y - tick, cue_half_len, desired_y + tick
        )

        # A small diamond at the commanded center makes the desired state easy
        # to pick out even when the desired and actual horizons overlap.
        diamond = QPolygonF(
            [
                QPointF(0.0, desired_y - 7.0),
                QPointF(8.0, desired_y),
                QPointF(0.0, desired_y + 7.0),
                QPointF(-8.0, desired_y),
            ]
        )
        painter.setBrush(QBrush(ink))
        painter.setPen(QPen(cyan, 2))
        painter.drawPolygon(diamond)
        painter.restore()

        # Sleek status chip with command and current tracking error.
        roll_error = self._desired_roll - self._roll
        pitch_error = self._desired_pitch - self._pitch
        chip_text = (
            f"FBW CMD  R {self._desired_roll:+04.1f}°  P {self._desired_pitch:+04.1f}°"
            f"   ERR {roll_error:+04.1f}/{pitch_error:+04.1f}°"
        )

        painter.save()
        font = QFont("Inter", 8)
        font.setBold(True)
        painter.setFont(font)
        metrics = painter.fontMetrics()
        chip_width = metrics.horizontalAdvance(chip_text) + 22
        chip_height = 26
        chip_x = max(10, int(center_x - chip_width / 2))
        chip_y = max(10, self.height() - chip_height - 12)
        chip_rect = QRectF(chip_x, chip_y, chip_width, chip_height)

        painter.setPen(QPen(QColor(0, 229, 255, 115), 1))
        painter.setBrush(QBrush(QColor(3, 18, 25, 178)))
        painter.drawRoundedRect(chip_rect, 10, 10)

        painter.setPen(
            QPen(cyan, 2, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
        )
        painter.drawLine(
            chip_x + 9,
            chip_y + chip_height / 2,
            chip_x + 21,
            chip_y + chip_height / 2,
        )
        painter.setPen(white)
        painter.drawText(
            QRectF(chip_x + 27, chip_y, chip_width - 34, chip_height),
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
            chip_text,
        )
        painter.restore()

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
        painter.rotate(self._roll)

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
            # Invert pitch direction so a positive input is drawn downward
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

        self._draw_desired_attitude_overlay(painter, center_x, center_y, SCALE)

        # Reference cross that stays fixed regardless of roll
        pen = QPen(Qt.gray, 2)
        painter.setPen(pen)
        painter.drawLine(center_x, center_y - CROSS_SIZE, center_x, center_y + CROSS_SIZE)
        painter.drawLine(center_x - CROSS_SIZE, center_y, center_x + CROSS_SIZE, center_y)

        painter.end()
