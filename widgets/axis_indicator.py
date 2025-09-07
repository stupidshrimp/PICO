from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPen
from PySide6.QtWidgets import QWidget


class AxisIndicator(QWidget):
    """Simple widget to visualise a control input axis.

    The widget draws a track line and a perpendicular indicator that moves
    according to the supplied value.  Values are expected in the range -1 to 1.
    """

    def __init__(self, orientation=Qt.Horizontal, parent=None):
        super().__init__(parent)
        self.orientation = orientation
        self._value = 0.0

    def setValue(self, value: float) -> None:
        """Set the current axis value and update the display."""
        self._value = max(-1.0, min(1.0, value))
        self.update()

    def paintEvent(self, event):  # noqa: D401 - Qt override
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        pen = QPen(Qt.green, 2)
        painter.setPen(pen)
        w = self.width()
        h = self.height()
        if self.orientation == Qt.Horizontal:
            y = h // 2
            painter.drawLine(0, y, w, y)
            x = int((self._value + 1) / 2 * w)
            painter.drawLine(x, 0, x, h)
        else:
            x = w // 2
            painter.drawLine(x, 0, x, h)
            y = int((1 - (self._value + 1) / 2) * h)
            painter.drawLine(0, y, w, y)
