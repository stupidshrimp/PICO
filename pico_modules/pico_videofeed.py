from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, QAbstractAnimation
from PySide6.QtWidgets import QLabel, QGraphicsOpacityEffect
import cv2
import numpy as np


class VideoFeed:
    def __init__(self, VideoLabel: QLabel, device_index: int = 1):
        """Initialize the video feed.

        Parameters
        ----------
        VideoLabel: QLabel
            Widget where the video feed will be displayed.
        device_index: int, optional
            Index of the capture device to open. Defaults to 1 so that the
            laptop's integrated camera (typically index 0) is ignored.
        """

        self.label = VideoLabel
        self.device_index = device_index
        self.cap = None  # Camera capture object
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.text_animation = None  # Placeholder for the text animation

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
            self.cap = cv2.VideoCapture(self.device_index)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
                self.label.clear()  # Clear any error message
                self.remove_opacity_effect()  # Remove opacity effect
                self.timer.start(10)  # Update frame every 10 ms (~100 FPS)
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

    def update_frame(self):
        """Updates the video feed on the QLabel."""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.remove_opacity_effect()  # Ensure no fading effect on video feed

                frame = self.deinterlace(frame)

                h, w, _ = frame.shape
                margin_x = int(w * 0.02)
                margin_y = int(h * 0.02)
                frame = frame[margin_y:h - margin_y, margin_x:w - margin_x]

                frame = cv2.resize(frame, (1280, 960), interpolation=cv2.INTER_LINEAR)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

                self.label.setPixmap(QPixmap.fromImage(qt_image))
            else:
                # If no frame is read, stop the timer and show an error message
                self.show_fading_text("Camera Error or Disconnected")
                self.stop()
        else:
            self.show_fading_text("No Camera Detected")

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
