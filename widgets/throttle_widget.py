from PySide6.QtCore import Qt, QRect
from PySide6.QtGui import QPainter, QPen, QBrush, QColor
from PySide6.QtWidgets import QWidget


class ThrottleWidget(QWidget):
    """Stylized widget that displays throttle percentage as a vertical bar."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._percent = 0.0

    def setValue(self, percent: float) -> None:
        """Set the throttle level as a percentage from 0 to 100."""
        self._percent = max(0.0, min(100.0, float(percent)))
        self.update()

    def paintEvent(self, event):  # noqa: N802 - Qt override naming
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()

        # Draw outer border
        pen = QPen(Qt.white, 2)
        painter.setPen(pen)
        painter.drawRect(0, 0, w - 1, h - 1)

        # Fill according to throttle percentage
        fill_height = int(h * (self._percent / 100.0))
        fill_rect = QRect(1, h - fill_height, w - 2, fill_height)
        painter.fillRect(fill_rect, QBrush(QColor(0, 200, 0)))
