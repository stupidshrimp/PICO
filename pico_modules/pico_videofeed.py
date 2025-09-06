from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import (
    Qt,
    QTimer,
    QPropertyAnimation,
    QEasingCurve,
    QAbstractAnimation,
    QThread,
    Signal,
    Slot,
    QObject,
)
from PySide6.QtWidgets import QLabel, QGraphicsOpacityEffect
import cv2
import numpy as np
from typing import Optional

# Reduce OpenCV's log verbosity so failing device indices do not spam stderr.
try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except Exception:
    # ``cv2.utils.logging`` may be unavailable on some builds; ignore in that case.
    pass


class FrameWorker(QObject):
    """Worker thread that captures and processes frames."""

    frame_ready = Signal(QImage)
    error = Signal(str)

    def __init__(self, video_feed):
        super().__init__()
        self.video_feed = video_feed

    @Slot()
    def process_frame(self):
        cap = self.video_feed.cap
        if cap and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                frame = self.video_feed.deinterlace(frame)

                h, w, _ = frame.shape
                margin_x = int(w * 0.02)
                margin_y = int(h * 0.02)
                frame = frame[margin_y:h - margin_y, margin_x:w - margin_x]

                frame = cv2.resize(frame, (1280, 960), interpolation=cv2.INTER_LINEAR)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.frame_ready.emit(qt_image)
            else:
                self.error.emit("Camera Error or Disconnected")
        else:
            self.error.emit("No Camera Detected")


class VideoFeed:
    @staticmethod
    def detect_device_index(preferred_index: Optional[int] = None, max_devices: int = 5) -> int:
        """Return a suitable capture device index.

        If ``preferred_index`` is supplied and available it is used. Otherwise,
        the first available index other than ``0`` (commonly the laptop's
        embedded webcam) is returned. As a final fallback, ``0`` is used.
        """

        available = []
        for idx in range(max_devices):
            try:
                cap = cv2.VideoCapture(idx)
            except cv2.error:
                # Some backends raise an exception for out-of-range indices.
                # Skip any index that cannot be opened instead of crashing.
                continue
            if cap.isOpened():
                available.append(idx)
                cap.release()

        if preferred_index is not None and preferred_index in available:
            return preferred_index

        for idx in available:
            if idx != 0:
                return idx

        return available[0] if available else 0

    def __init__(self, VideoLabel: QLabel, device_index: Optional[int] = None):
        """Initialize the video feed.

        Parameters
        ----------
        VideoLabel: QLabel
            Widget where the video feed will be displayed.
        device_index: int, optional
            Index of the capture device to open. If ``None`` the
            :meth:`detect_device_index` helper is used which prefers external
            devices over the laptop's integrated camera.
        """

        self.label = VideoLabel
        self.device_index = (
            device_index if device_index is not None else self.detect_device_index()
        )
        self.cap = None  # Camera capture object
        self.timer = QTimer()
        self.text_animation = None  # Placeholder for the text animation

        # Worker thread for frame processing
        self.worker_thread = QThread()
        self.worker = FrameWorker(self)
        self.worker.moveToThread(self.worker_thread)
        self.timer.timeout.connect(self.worker.process_frame)
        self.worker.frame_ready.connect(self.update_frame)
        self.worker.error.connect(self._handle_worker_error)
        self.worker_thread.start()

        # Timer for periodically checking camera availability
        self.camera_check_timer = QTimer()
        self.camera_check_timer.timeout.connect(self.check_camera)

    def start(self):
        """Begin checking for the camera and start the feed when available."""
        if not self.camera_check_timer.isActive():
            self.camera_check_timer.start(1000)  # Check every 1 second
        self.check_camera()

    def check_camera(self):
        """Check if the selected camera is available and start the video feed."""
        if self.cap is None or not self.cap.isOpened():
            try:
                self.cap = cv2.VideoCapture(self.device_index)
            except cv2.error:
                # Backend failed to open the device index; treat as unavailable.
                self.cap = None
                self.show_fading_text("No Camera Detected")
                return
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
                self.label.clear()  # Clear any error message
                self.remove_opacity_effect()  # Remove opacity effect
                self.timer.start(30)  # Update frame every 30 ms (~30 FPS)
            else:
                # Show fading error message when no camera is detected
                self.show_fading_text("No Camera Detected")

    def stop(self):
        """Stop the video feed and camera checks."""
        self.timer.stop()
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.cap = None  # Reset the capture object
        self.camera_check_timer.stop()

    @Slot(QImage)
    def update_frame(self, image: QImage):
        """Updates the video feed on the QLabel with a processed frame."""
        if image is not None:
            self.remove_opacity_effect()  # Ensure no fading effect on video feed
            self.label.setPixmap(QPixmap.fromImage(image))

    @Slot(str)
    def _handle_worker_error(self, message: str):
        """Handle errors emitted from the worker thread."""
        self.show_fading_text(message)
        if message == "Camera Error or Disconnected":
            self.stop()

    def deinterlace(self, frame):
        even = frame[0::2]
        odd = frame[1::2]
        blended = ((even.astype("float32") + odd.astype("float32")) / 2).astype("uint8")
        deinterlaced = np.empty_like(frame)
        deinterlaced[0::2] = blended
        deinterlaced[1::2] = blended
        return deinterlaced

    def show_fading_text(self, message):
        """
        Display fading text on the QLabel.
        :param message: Text to display.
        """
        # Set the message on the QLabel
        self.label.setText(message)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("color: red; font-size: 16px;")

        # Apply an opacity effect to the QLabel
        if not hasattr(self.label, "_opacity_effect"):
            opacity_effect = QGraphicsOpacityEffect(self.label)
            self.label.setGraphicsEffect(opacity_effect)
            self.label._opacity_effect = opacity_effect  # Store reference for later

        # Stop the previous animation if it exists
        if self.text_animation is not None and self.text_animation.state() == QAbstractAnimation.Running:
            self.text_animation.stop()

        # Create a new animation for the opacity effect
        self.text_animation = QPropertyAnimation(self.label._opacity_effect, b"opacity")
        self.text_animation.setDuration(2000)  # Total duration: 2 seconds
        self.text_animation.setKeyValueAt(0, 0)     # Fully transparent
        self.text_animation.setKeyValueAt(0.2, 1)  # Fade-in
        self.text_animation.setKeyValueAt(1, 1)    # Fully visible
        self.text_animation.setEasingCurve(QEasingCurve.InOutQuad)
        self.text_animation.setLoopCount(-1)  # Loop indefinitely

        # Start the animation
        self.text_animation.start()

    def remove_opacity_effect(self):
        """Remove the opacity effect from the QLabel."""
        if hasattr(self.label, "_opacity_effect"):
            self.label.setGraphicsEffect(None)  # Remove the effect
            del self.label._opacity_effect  # Delete the reference to free resources
