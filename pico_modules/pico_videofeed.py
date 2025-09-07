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
    QMetaObject,
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
    """Worker thread that owns the capture device and produces frames."""

    frame_ready = Signal(QImage)
    error = Signal(str)

    def __init__(self, device_index: int, video_feed):
        super().__init__()
        self.device_index = device_index
        self.video_feed = video_feed
        self.cap = None
        self._timer = None

    @Slot()
    def ensure_camera(self):
        """Open the capture device if it is not already active."""
        if self.cap and self.cap.isOpened():
            return

        try:
            self.cap = cv2.VideoCapture(self.device_index)
        except cv2.error:
            self.cap = None

        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
            self.start_timer()
        else:
            self.cap = None
            self.error.emit("Not connected")

    def start_timer(self):
        if self._timer is None:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self.process_frame)
        if not self._timer.isActive():
            self._timer.start(30)

    @Slot()
    def start(self):
        """Ensure the camera is open and begin processing."""
        self.ensure_camera()

    @Slot()
    def stop(self):
        """Stop processing frames and release the capture device."""
        if self._timer and self._timer.isActive():
            self._timer.stop()
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.cap = None

    @Slot()
    def process_frame(self):
        try:
            if not self.cap or not self.cap.isOpened():
                self.error.emit("Not connected")
                return

            ret, frame = self.cap.read()
            if not ret:
                self.error.emit("Camera Error or Disconnected")
                return

            frame = self.video_feed.deinterlace(frame)

            h, w, _ = frame.shape

            # Scale the frame to fit a 1280x960 canvas while preserving the
            # original aspect ratio. This avoids the previous 2% crop that
            # unintentionally zoomed the image.
            target_w, target_h = 1280, 960
            scale = min(target_w / w, target_h / h)
            new_w, new_h = int(w * scale), int(h * scale)

            frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

            # Center the resized frame on a black canvas so the output size
            # remains constant without distorting the aspect ratio.
            canvas = np.zeros((target_h, target_w, 3), dtype=frame.dtype)
            x_off = (target_w - new_w) // 2
            y_off = (target_h - new_h) // 2
            canvas[y_off : y_off + new_h, x_off : x_off + new_w] = frame
            frame = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
            self.frame_ready.emit(qt_image)
        except Exception as exc:
            self.error.emit(str(exc))


class VideoFeed:
    @staticmethod
    def detect_device_index(preferred_index: Optional[int] = None) -> Optional[int]:
        """Return ``1`` if that video device is available, otherwise ``None``.

        The ground station expects the VTX to appear as camera device index 1.
        No fallback to other indices is performed.
        """

        index = 1 if preferred_index is None else preferred_index
        try:
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                cap.release()
                return index
            cap.release()
        except cv2.error:
            pass
        return None

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
        self.text_animation = None  # Placeholder for the text animation

        # Worker thread for frame processing
        self.worker_thread = QThread()
        self.worker = FrameWorker(self.device_index, self)
        self.worker.moveToThread(self.worker_thread)
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
        """Request the worker thread to ensure the camera is active."""
        QMetaObject.invokeMethod(self.worker, "start", Qt.QueuedConnection)

    def stop(self):
        """Stop the video feed and camera checks."""
        QMetaObject.invokeMethod(self.worker, "stop", Qt.QueuedConnection)
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
