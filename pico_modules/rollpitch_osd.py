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
        SCALE        = 4.0    # Pixels per pitch degree
        LINE_LENGTH  = 80     # Half the rung length (left or right)
        GAP_SIZE     = 40     # Gap in the center
        ANGLE_OFFSET = 10     # Vertical offset for each side of the rung

        # Center of the widget is our reference
        center_x = self.width() / 2
        center_y = self.height() / 2

        # Optional center reference lines (vertical + horizontal)
        pen = QPen(Qt.gray, 2)
        painter.setPen(pen)
        # Vertical center line
        painter.drawLine(center_x, 0, center_x, self.height())
        # Horizontal center line
        painter.drawLine(0, center_y, self.width(), center_y)

        # Use green pen for the pitch ladder lines
        pen = QPen(Qt.green, 2)
        painter.setPen(pen)

        # We'll draw lines for pitch values from +25 down to -25 in steps of 5.
        # In aviation, +pitch is nose up (lines appear below center if you prefer).
        pitch_values = range(25, -30, -5)  # 25, 20, 15, 10, 5, 0, -5, -10, -15, -20, -25

        # Set a font for numeric labels (optional)
        painter.setFont(QFont("Arial", 10))

        for pitch_deg in pitch_values:
            # Vertical offset based on difference between the rung's pitch_deg and current pitch
            # If you want lines to move the opposite direction, swap (pitch_deg - self._pitch)
            dy = (self._pitch - pitch_deg) * SCALE
            base_y = center_y + dy

            # We'll angle each rung so it forms a shallow "V" shape:
            #    left side slopes one way, right side slopes the other
            # Example logic (feel free to invert or swap offsets):
            left_y1  = base_y + ANGLE_OFFSET
            left_y2  = base_y - ANGLE_OFFSET
            right_y1 = base_y - ANGLE_OFFSET
            right_y2 = base_y + ANGLE_OFFSET

            # X coordinates for left and right segments
            left_x1  = center_x - LINE_LENGTH
            left_x2  = center_x - GAP_SIZE / 2
            right_x1 = center_x + GAP_SIZE / 2
            right_x2 = center_x + LINE_LENGTH

            # Draw left angled segment
            painter.drawLine(left_x1, left_y1, left_x2, left_y2)
            # Draw right angled segment
            painter.drawLine(right_x1, right_y1, right_x2, right_y2)

            # Draw numeric label near the left segment
            label_str = f"{pitch_deg}"
            # Adjust the X or Y offset to place the text where you like
            painter.drawText(left_x1 - 25, left_y1, label_str)

        painter.end()
