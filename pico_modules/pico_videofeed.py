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

        if self.device_index in (None, ""):
            self.error.emit("Not connected")
            return

        try:
            self.cap = cv2.VideoCapture(self.device_index)
        except cv2.error:
            self.cap = None

        if self.cap and self.cap.isOpened():
            # Avoid forcing a fixed resolution which could crop or zoom the
            # incoming video.  Using the device's default resolution preserves
            # the original field of view.
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
                if self._timer and self._timer.isActive():
                    self._timer.stop()
                return

            ret, frame = self.cap.read()
            if not ret:
                self.error.emit("Camera Error or Disconnected")
                return

            frame = self.video_feed.deinterlace(frame)

            # Convert the frame without imposing a fixed output size so the
            # full field of view from the capture device is preserved.  Any
            # necessary scaling to fit the display widget is handled in
            # ``update_frame``.
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
            self.frame_ready.emit(qt_image)
        except Exception as exc:
            self.error.emit(str(exc))


class VideoFeed(QObject):
    @staticmethod
    def detect_device_index(preferred_index: int = 1, max_index: int = 5) -> Optional[int]:
        """Return the first available video capture device index.

        Parameters
        ----------
        preferred_index: int, optional
            Index probed first. Defaults to ``1`` to prefer the external VTX
            capture device. Index ``0``, which usually maps to a laptop's
            internal webcam, is ignored.
        max_index: int, optional
            Upper bound (exclusive) for probing additional indices.

        Returns
        -------
        Optional[int]
            Index of the first usable capture device, or ``None`` if no device
            could be opened.
        """

        # ``0`` is typically the laptop's integrated webcam.  Skip it so that
        # automatic detection will only consider external capture devices.  If a
        # caller explicitly wants to use the internal webcam they can pass
        # ``device_index=0`` when constructing :class:`VideoFeed`.
        indices = [preferred_index] + [i for i in range(1, max_index) if i != preferred_index]
        for index in indices:
            try:
                cap = cv2.VideoCapture(index)
                if cap.isOpened():
                    cap.release()
                    return index
                cap.release()
            except cv2.error:
                continue
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
            devices and skips the laptop's integrated webcam.
        """

        super().__init__()
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

    def shutdown(self):
        """Completely stop the worker thread and release resources."""
        self.stop()
        if self.worker_thread.isRunning():
            self.worker_thread.quit()
            self.worker_thread.wait()

    @Slot(QImage)
    def update_frame(self, image: QImage):
        """Updates the video feed on the QLabel with a processed frame."""
        if image is not None:
            self.remove_opacity_effect()  # Ensure no fading effect on video feed
            scaled = image.scaled(
                self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            # Scale the frame to the QLabel's dimensions to avoid cropping that
            # makes the video appear zoomed in.
            self.label.setPixmap(QPixmap.fromImage(scaled))

    @Slot(str)
    def _handle_worker_error(self, message: str):
        """Handle errors emitted from the worker thread."""
        self.show_fading_text(message)
        if message == "Camera Error or Disconnected":
            self.stop()

    def deinterlace(self, frame):
        even = frame[0::2]
        odd = frame[1::2]
        blended = ((even.astype("float32") + odd.astype("float32")) / 2).astype(
            "uint8"
        )
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
