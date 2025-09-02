from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, QAbstractAnimation
from PySide6.QtWidgets import QLabel, QGraphicsOpacityEffect
import cv2

class VideoFeed:
    def __init__(self, VideoLabel):
        """
        Initializes the video feed class.
        :param VideoLabel: QLabel object where the video feed will be displayed.
        """
        self.label = VideoLabel
        self.cap = None  # Camera capture object
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.text_animation = None  # Placeholder for the text animation

        # Timer for periodically checking camera availability
        self.camera_check_timer = QTimer()
        self.camera_check_timer.timeout.connect(self.check_camera)
        self.camera_check_timer.start(1000)  # Check every 1 second

        # Initially check for camera availability
        self.check_camera()

    def check_camera(self):
        """Check if a camera is available and start video feed if not already running."""
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0)  # Attempt to open the default camera
            if self.cap.isOpened():
                self.label.clear()  # Clear any error message
                self.remove_opacity_effect()  # Remove opacity effect
                self.start()
            else:
                # Show fading error message when no camera is detected
                self.show_fading_text("No Camera Detected")

    def start(self):
        """Start the video feed."""
        if self.cap and self.cap.isOpened():
            self.timer.start(10)  # Update frame every 10 ms (~100 FPS)

    def stop(self):
        """Stop the video feed."""
        self.timer.stop()
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.cap = None  # Reset the capture object

    def update_frame(self):
        """Updates the video feed on the QLabel."""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.remove_opacity_effect()  # Ensure no fading effect on video feed

                # Convert the frame to RGB format for Qt
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

                # Scale the QImage to fit the QLabel dimensions
                scaled_image = qt_image.scaled(
                    self.label.width(),
                    self.label.height(),
                    Qt.KeepAspectRatio
                )

                # Display the scaled image in the QLabel
                self.label.setPixmap(QPixmap.fromImage(scaled_image))
            else:
                # If no frame is read, stop the timer and show an error message
                self.show_fading_text("Camera Error or Disconnected")
                self.stop()
        else:
            self.show_fading_text("No Camera Detected")

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
