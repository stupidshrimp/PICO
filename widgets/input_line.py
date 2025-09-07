from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPen
from PySide6.QtWidgets import QWidget


class InputLine(QWidget):
    """Simple line-based indicator for joystick inputs.

    The widget draws a track line oriented vertically or horizontally and a
    small perpendicular marker that moves according to the provided value.
    ``setValue`` accepts values in the range ``-1.0`` to ``1.0``.
    """

    def __init__(self, orientation=Qt.Vertical, parent=None):
        super().__init__(parent)
        self.orientation = orientation
        self._value = 0.0

    def setValue(self, value: float) -> None:
        self._value = max(-1.0, min(1.0, float(value)))
        self.update()

    def paintEvent(self, event):  # noqa: N802 - Qt override naming
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        pen = QPen(Qt.white, 2)
        painter.setPen(pen)
        w = self.width()
        h = self.height()

        if self.orientation == Qt.Vertical:
            x = w // 2
            painter.drawLine(x, 0, x, h)
            y = int((1 - (self._value + 1) / 2) * h)
            painter.drawLine(0, y, w, y)
        else:
            y = h // 2
            painter.drawLine(0, y, w, y)
            x = int((self._value + 1) / 2 * w)
            painter.drawLine(x, 0, x, h)
