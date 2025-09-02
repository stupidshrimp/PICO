from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import QVariantAnimation, Qt
from PyQt5.QtGui import QColor


class LabelManager:
    def __init__(self, labels):
        """
        Initialize the LabelManager with a dictionary of labels.

        Args:
            labels (dict): A dictionary where keys are label names (e.g., "pitch")
                           and values are QLabel references.
        """
        self.labels = labels  # Store QLabel references by name
        self.active_animations = {}  # Track active animations by label name

    def update_label(self, label_name, value):
        """
        Update a specific label with the given value and check for errors.

        Args:
            label_name (str): The name of the label to update.
            value (str or float): The value to update the label with.
        """
        if label_name not in self.labels:
            print(f"Label '{label_name}' not found.")
            return

        label = self.labels[label_name]

        # Set the label's text
        if isinstance(value, (int, float)):  # Check if the value is a number
            label.setText(f"{label_name.capitalize()}: {value:.1f}")
        else:  # Handle non-numeric values (e.g., error messages)
            label.setText(f"{label_name.capitalize()}: {value}")

        # Check for error and apply animation if needed
        if isinstance(value, str) and "Error" in value.lower():
            self.apply_error_animation(label_name, label)

    def update_labels(self, updates):
        """
        Update multiple labels at once and check for errors.

        Args:
            updates (dict): A dictionary of label names and their corresponding values.
        """
        for label_name, value in updates.items():
            self.update_label(label_name, value)

    def apply_error_animation(self, label_name, label):
        """
        Apply a fading red error animation to a QLabel.

        Args:
            label_name (str): The name of the label to apply the animation to.
            label (QLabel): The QLabel to apply the animation to.
        """
        # Prevent re-triggering if an animation is already active for this label
        if label_name in self.active_animations and self.active_animations[label_name]:
            return

        # Mark the animation as active
        self.active_animations[label_name] = True

        # Define the red color for the error highlight
        start_color = QColor(255, 0, 0)  # Red
        end_color = QColor(255, 255, 255)  # White (default background)

        # Create the animation
        animation = QVariantAnimation(
            startValue=start_color,
            endValue=end_color,
            duration=1000,  # Animation duration in milliseconds
        )

        # Update the label's background color during the animation
        def update_color(value):
            color_style = f"background-color: {value.name()};"
            label.setStyleSheet(color_style)

        animation.valueChanged.connect(update_color)

        # Reset the label's style to default after the animation ends
        def reset_style():
            label.setStyleSheet("")
            self.active_animations[label_name] = False  # Mark the animation as finished

        animation.finished.connect(reset_style)

        # Start the animation
        animation.start()

        # Keep a reference to the animation to prevent garbage collection
        self.active_animations[label_name] = animation
