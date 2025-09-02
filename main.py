import sys
import os
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QComboBox,
    QLabel,
    QHBoxLayout,
    QWidget,
    QVBoxLayout,
)
from PySide6.QtCore import Qt, QTimer

from serial.tools import list_ports

from modules import *
from widgets import *

from pico_modules.pico_videofeed import VideoFeed
from pico_modules.pico_joystick2state import JoystickRateHandler
from pico_modules.pico_transmitpackets import CRSFPacketProcessor

from pico_modules.labelsmanager import LabelManager

# Import the custom OSD module
from pico_modules.rollpitch_osd import RollPitchOSD

from config import load_config


def validate_port(name: str, port: str) -> bool:
    """Validate that a serial port exists on the system."""
    available = [p.device for p in list_ports.comports()]
    if port not in available:
        print(
            f"Warning: {name} port '{port}' not found. Available ports: {', '.join(available) or 'None'}"
        )
        return False
    return True

widgets = None

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # Ensure the main window always fits its contents by removing the
        # fixed minimum size added in the generated UI file and adjusting the
        # window to the layout's size hint. This allows the GUI to resize based
        # on the widgets it contains and prevents it from being smaller than the
        # required size.
        self.setMinimumSize(0, 0)
        self.adjustSize()
        # Default to a wider and shorter window
        self.resize(1600, 600)
        self.setMinimumSize(self.size())

        global widgets
        widgets = self.ui

        # Update application branding
        self.ui.titleLeftApp.setText("PICO Program")
        self.ui.titleLeftDescription.setText(
            "a Modern UAS control platform"
        )

        # Remove unwanted side tabs
        self.ui.btn_save.hide()
        self.ui.btn_exit.hide()

        # Rename side tab buttons
        self.ui.btn_new.setText("Command")
        self.ui.btn_widgets.setText("Configuration")

        # Use frameless window and translucent background
        # self.setWindowFlags(Qt.FramelessWindowHint)
        # self.setAttribute(Qt.WA_TranslucentBackground)

        # Initialize the video feed
        self.video_feed = VideoFeed(self.ui.VideoLabel)
        # Start the video feed
        self.video_feed.start()        

        config = load_config()

        # Configuration sections
        self.joystick_cfg = config.get("joystick", {})
        self.crsf_cfg = config.get("crsf", {})

        # Joystick parameters
        self.UPDATE_INTERVAL = 0.01  # 10 ms update interval
        self.ROLL_SENSITIVITY = 0.3  # Higher sensitivity for roll
        self.PITCH_SENSITIVITY = 0.3  # Normal sensitivity for pitch
        self.ROLL_EXPONENT = 1.5  # Exponential curve for roll
        self.PITCH_EXPONENT = 1.5  # Gentler exponential curve for pitch

        self.joystick = None
        if validate_port("joystick", self.joystick_cfg.get("port")):
            try:
                self.joystick = JoystickRateHandler(
                    port=self.joystick_cfg.get("port"),
                    baudrate=self.joystick_cfg.get("baudrate"),
                    update_interval=self.UPDATE_INTERVAL,
                    roll_rate_max=100,  # Max roll rate (degrees/second)
                    pitch_rate_max=100,  # Max pitch rate (degrees/second)
                    roll_sensitivity=self.ROLL_SENSITIVITY,
                    pitch_sensitivity=self.PITCH_SENSITIVITY,
                    roll_exponent=self.ROLL_EXPONENT,
                    pitch_exponent=self.PITCH_EXPONENT,
                )
            except Exception as e:
                print(f"Failed to initialize joystick: {e}")
        else:
            print("Joystick disabled due to unavailable port.")

        # Initialize CRSFPacketProcessor
        self.crsf_processor = None
        if validate_port("CRSF", self.crsf_cfg.get("port")):
            try:
                self.crsf_processor = CRSFPacketProcessor(
                    port=self.crsf_cfg.get("port"),
                    baudrate=self.crsf_cfg.get("baudrate"),
                )
            except Exception as e:
                print(f"Failed to initialize CRSF processor: {e}")
        else:
            print("CRSF disabled due to unavailable port.")

        # Setup configuration page for COM port selections
        self.setup_configuration_page()

        # Create a dictionary of QLabel references for LabelManager
        labels = {
            "pitch": self.ui.PitchLabel1,
            "roll": self.ui.RollLabel1,
            "yaw": self.ui.YawLabel1,
            "Transmit Status": self.ui.transmitstatus1,
        }

        # Initialize the LabelManager with the labels
        self.label_manager = LabelManager(labels)

        # Timer for joystick label updates (10ms interval, 100Hz)
        self.label_update_timer = QTimer(self)
        self.label_update_timer.timeout.connect(self.update_labels)
        self.label_update_timer.start(10)

        # Timer for transmitting data (10ms interval, 100Hz)
        self.transmit_timer = QTimer(self)
        self.transmit_timer.timeout.connect(self.transmit_data)
        self.transmit_timer.start(10)

        # --------------------------------------------------------------------
        # OSD Overlay Setup - Create and initialize the RollPitchOSD widget
        # --------------------------------------------------------------------
        # Here we assume that in your .ui file there is a placeholder widget named "rollpitchosd"
        # We create our custom RollPitchOSD instance with that widget as its parent.
        self.rollpitch_osd = RollPitchOSD(self.ui.rollpitchosd)
        self.rollpitch_osd.resize(self.ui.rollpitchosd.size())
        self.rollpitch_osd.show()

        # For simulation, we set up our simulated pitch values.
        self.simulated_pitch = 0.0    # Start at 0 degrees
        self.pitch_direction = 1      # 1 means increasing pitch

        # For this demo, we'll keep roll constant (for example, 0)
        self.simulated_roll = 0.0

        # Timer to animate the OSD overlay (update every 100ms)
        self.osd_timer = QTimer(self)
        self.osd_timer.timeout.connect(self.update_osd_animation)
        self.osd_timer.start(100)

        # TOGGLE MENU
        widgets.toggleButton.clicked.connect(lambda: UIFunctions.toggleMenu(self, True))

        # SET UI DEFINITIONS
        UIFunctions.uiDefinitions(self)

        # LEFT MENUS
        widgets.btn_home.clicked.connect(self.buttonClick)
        widgets.btn_widgets.clicked.connect(self.buttonClick)
        widgets.btn_new.clicked.connect(self.buttonClick)

        # EXTRA RIGHT BOX
        widgets.settingsTopBtn.clicked.connect(lambda: UIFunctions.toggleRightBox(self, True))

        # REMOVE SETTINGS TAB AND ICON
        widgets.toggleLeftBox.hide()
        widgets.bottomMenu.hide()
        widgets.extraLeftBox.hide()

        # TOP BUTTONS (Close, Minimize, Maximize)
        widgets.closeAppBtn.clicked.connect(self.close)
        widgets.minimizeAppBtn.clicked.connect(self.showMinimized)
        widgets.maximizeRestoreAppBtn.clicked.connect(lambda: UIFunctions.maximize_restore(self))

        # SET HOME PAGE AND SELECT MENU
        widgets.stackedWidget.setCurrentWidget(widgets.home)
        widgets.btn_home.setStyleSheet(UIFunctions.selectMenu(widgets.btn_home.styleSheet()))

    def update_osd_animation(self):
        """
        Update the OSD overlay's pitch value for demonstration.
        The simulated pitch value will cycle from 0 up to 50, then down to -50, and repeat.
        """
        # Increment the pitch
        self.simulated_pitch += self.pitch_direction * 2  # Change by 2 degrees per update

        # Reverse direction if we reach the limits
        if self.simulated_pitch >= 50:
            self.pitch_direction = -1
        elif self.simulated_pitch <= -50:
            self.pitch_direction = 1

        # Update the RollPitchOSD widget with the new pitch (and constant roll)
        self.rollpitch_osd.setRollPitch(self.simulated_roll, self.simulated_pitch)


    def update_labels(self):
        """
        Fetch joystick data and update labels with raw angles.
        """
        if not self.joystick:
            self.label_manager.update_labels({
                "pitch": "N/A",
                "roll": "N/A",
                "yaw": "N/A",
            })
            return
        try:
            # Fetch raw joystick angles for updating the labels
            raw_pitch, raw_roll = self.joystick.update_angles()

            # Update labels with raw joystick data
            self.label_manager.update_labels({
                "pitch": raw_pitch,
                "roll": raw_roll,
                "yaw": 0,  # Replace with actual yaw data if available
            })
        except Exception as e:
            self.label_manager.update_labels({
                "pitch": "Error",
                "roll": "Error",
                "yaw": "Error",
            })

    def transmit_data(self):
        """
        Transmit CRSF packets using mapped joystick angles and update transmit status.
        """
        if not self.joystick or not self.crsf_processor:
            self.label_manager.update_labels({
                "Transmit Status": "Hardware Unavailable",
            })
            return

        try:
            # Fetch mapped joystick angles for transmission
            mapped_pitch, mapped_roll = self.joystick.update_mapped_angles()

            # Prepare CRSF channels for transmission
            channels = [1500] * 16
            channels[0] = int(mapped_roll)
            channels[1] = int(mapped_pitch)

            # Send CRSF packet and get status
            status = self.crsf_processor.update_and_send_packet(channels)

            # Update Transmit Status label based on the status
            self.label_manager.update_labels({
                "Transmit Status": status,
            })

            # If the status indicates an error, apply a fading red animation
            if "error" in status.lower():
                self.label_manager.apply_error_animation("Transmit Status", self.ui.transmitstatus1)

        except Exception as e:
            print(f"Error during transmission: {e}")
            self.label_manager.update_labels({
                "Transmit Status": "Error",
            })
            self.label_manager.apply_error_animation("Transmit Status", self.ui.transmitstatus1)

    def update_labels_and_transmit(self):
        """
        Fetch joystick data, update labels using raw angles, and transmit CRSF packets using mapped angles.
        """
        if not self.joystick or not self.crsf_processor:
            self.label_manager.update_labels({
                "pitch": "N/A",
                "roll": "N/A",
                "yaw": "N/A",
                "Transmit Status": "Hardware Unavailable",
            })
            return

        try:
            # Fetch raw joystick angles for updating the labels
            raw_pitch, raw_roll = self.joystick.update_angles()

            # Update labels with raw joystick data
            self.label_manager.update_labels({
                "pitch": raw_pitch,
                "roll": raw_roll,
                "yaw": 0,  # Replace with actual yaw data if available
            })

            # Fetch mapped angles for CRSF transmission
            mapped_pitch, mapped_roll = self.joystick.update_mapped_angles()

            # Prepare CRSF channels for transmission
            channels = [1500] * 16  # Default values for all channels
            channels[0] = int(mapped_roll)  # Map roll to channel 1
            channels[1] = int(mapped_pitch)  # Map pitch to channel 2

            # Send CRSF packet
            self.crsf_processor.update_and_send_packet(channels)

            # Update Transmit Status label
            self.label_manager.update_labels({
                "Transmit Status": "Good",
            })

        except Exception as e:
            print(f"Error during transmission: {e}")
            # Update labels with error status
            self.label_manager.update_labels({
                "pitch": "Error",
                "roll": "Error",
                "yaw": "Error",
                "Transmit Status": "Error",
            })

    def setup_configuration_page(self):
        """Create configuration page for selecting COM ports."""
        self.ui.configuration_page = QWidget()
        widgets.configuration_page = self.ui.configuration_page
        widgets.stackedWidget.addWidget(self.ui.configuration_page)

        layout = QVBoxLayout(self.ui.configuration_page)
        ports = [p.device for p in list_ports.comports()]

        def add_port_selector(title):
            container = QWidget()
            row = QHBoxLayout(container)
            row.addWidget(QLabel(title))
            combo = QComboBox()
            combo.addItems(ports)
            row.addWidget(combo)
            layout.addWidget(container)
            return combo

        self.control_port_combo = add_port_selector("Control System")
        self.video_port_combo = add_port_selector("Video Transmitter")
        self.elrs_port_combo = add_port_selector("ELRS Transmitter")

        # Set default selections
        self.control_port_combo.setCurrentText(self.joystick_cfg.get("port", ""))
        self.elrs_port_combo.setCurrentText(self.crsf_cfg.get("port", ""))

        # Connect signals
        self.control_port_combo.currentTextChanged.connect(
            self.on_control_port_selected
        )
        self.video_port_combo.currentTextChanged.connect(
            self.on_video_port_selected
        )
        self.elrs_port_combo.currentTextChanged.connect(
            self.on_elrs_port_selected
        )

    def on_control_port_selected(self, port: str):
        """Handle selection of control system port."""
        self.joystick_cfg["port"] = port
        if self.joystick:
            try:
                self.joystick.serial_connection.close()
            except Exception:
                pass
            self.joystick = None
        if validate_port("joystick", port):
            try:
                self.joystick = JoystickRateHandler(
                    port=port,
                    baudrate=self.joystick_cfg.get("baudrate"),
                    update_interval=self.UPDATE_INTERVAL,
                    roll_rate_max=100,
                    pitch_rate_max=100,
                    roll_sensitivity=self.ROLL_SENSITIVITY,
                    pitch_sensitivity=self.PITCH_SENSITIVITY,
                    roll_exponent=self.ROLL_EXPONENT,
                    pitch_exponent=self.PITCH_EXPONENT,
                )
            except Exception as e:
                print(f"Failed to initialize joystick: {e}")

    def on_video_port_selected(self, port: str):
        """Handle selection of video transmitter port."""
        self.video_port = port
        print(f"Video transmitter port set to {port}")

    def on_elrs_port_selected(self, port: str):
        """Handle selection of ELRS transmitter port."""
        self.crsf_cfg["port"] = port
        if self.crsf_processor:
            try:
                self.crsf_processor.close_serial()
            except Exception:
                pass
            self.crsf_processor = None
        if validate_port("CRSF", port):
            try:
                self.crsf_processor = CRSFPacketProcessor(
                    port=port, baudrate=self.crsf_cfg.get("baudrate")
                )
            except Exception as e:
                print(f"Failed to initialize CRSF processor: {e}")

    def buttonClick(self):
        # GET BUTTON CLICKED
        btn = self.sender()
        btnName = btn.objectName()

        # SHOW HOME PAGE
        if btnName == "btn_home":
            widgets.stackedWidget.setCurrentWidget(widgets.home)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

        # SHOW WIDGETS PAGE
        if btnName == "btn_widgets":
            widgets.stackedWidget.setCurrentWidget(widgets.configuration_page)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

        # SHOW NEW PAGE
        if btnName == "btn_new":
            widgets.stackedWidget.setCurrentWidget(widgets.new_page)  # SET PAGE
            UIFunctions.resetStyle(self, btnName)  # RESET ANOTHERS BUTTONS SELECTED
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))  # SELECT MENU
            # Resize window to fit the contents of the command page
            self.setMinimumSize(0, 0)
            self.adjustSize()
            self.setMinimumSize(self.size())

    def resizeEvent(self, event):
        UIFunctions.resize_grips(self)
        self.rollpitch_osd.resize(self.ui.rollpitchosd.size())

    def mousePressEvent(self, event):
        """Capture the position of the mouse press."""
        if event.buttons() == Qt.LeftButton:
            self.dragPos = event.globalPosition().toPoint()

    def closeEvent(self, event):
        """
        Releases resources when the window is closed.
        """
        self.video_feed.stop()
        self.crsf_processor.close_serial()  # Ensure serial port is closed
        super().closeEvent(event)
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
