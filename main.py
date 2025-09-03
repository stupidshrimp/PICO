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
    QLineEdit,
    QSlider,
    QFrame,
)
from PySide6.QtCore import Qt, QTimer

from serial.tools import list_ports

from modules import *
from widgets import *

from pico_modules.pico_videofeed import VideoFeed
from pico_modules.pico_joystick2state import JoystickRawHandler
from pico_modules.pico_transmitpackets import CRSFPacketProcessor

from pico_modules.labelsmanager import LabelManager

# Import the custom OSD module
from pico_modules.rollpitch_osd import RollPitchOSD
from pico_modules.altitude_osd import AltitudeOSD
from pico_modules.airspeed_osd import AirspeedOSD

from config import load_config, save_config


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
        # Size the window using the command page and keep it fixed. This
        # ensures the GUI is always large enough for its contents and does not
        # change size when switching between pages.
        self.ui.stackedWidget.setCurrentWidget(self.ui.new_page)
        self.adjustSize()
        self.setFixedSize(self.size())
        self.ui.stackedWidget.setCurrentWidget(self.ui.home)

        global widgets
        widgets = self.ui

        # Timers used for blinking "Not Connected" indicators
        self.status_timers = {}

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

        self.config = load_config()

        # Configuration sections
        self.joystick_cfg = self.config.setdefault("joystick", {})
        self.crsf_cfg = self.config.setdefault("crsf", {})
        self.vtx_cfg = self.config.setdefault("vtx", {})

        # Initialize the video feed using the configured device index
        self.video_port = self.vtx_cfg.get("port", "")
        video_index = self.vtx_cfg.get("device_index", 1)
        self.video_feed = VideoFeed(self.ui.VideoLabel, device_index=video_index)
        if validate_port("VTX", self.video_port):
            self.video_feed.start()
        else:
            print("VTX video disabled due to unavailable port.")
        self.joystick = None
        if validate_port("joystick", self.joystick_cfg.get("port")):
            try:
                self.joystick = JoystickRawHandler(
                    port=self.joystick_cfg.get("port"),
                    baudrate=self.joystick_cfg.get("baudrate"),
                    deadzone=self.joystick_cfg.get("deadzone", 0),
                    sensitivity=self.joystick_cfg.get("sensitivity", 100),
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
                    telemetry_callback=self.handle_telemetry,
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

        # Variables updated from telemetry packets
        self.telemetry_pitch = None
        self.telemetry_roll = None
        self.telemetry_yaw = None
        self.current_altitude = None
        self.current_airspeed = None

        # Timer for updating labels/OSD (10ms interval, 100Hz)
        self.label_update_timer = QTimer(self)
        self.label_update_timer.timeout.connect(self.update_labels)
        self.label_update_timer.start(10)

        # Timer for transmitting data (default from config)
        self.transmit_timer = QTimer(self)
        self.transmit_timer.timeout.connect(self.transmit_data)
        self.transmit_timer.start(self.crsf_cfg.get("packet_interval", 10))

        # Timer for reading incoming telemetry packets
        if self.crsf_processor:
            self.telemetry_timer = QTimer(self)
            self.telemetry_timer.timeout.connect(self.crsf_processor.read_serial_data)
            self.telemetry_timer.start(1)


        # --------------------------------------------------------------------
        # OSD Overlay Setup - Create and initialize the RollPitchOSD widget
        # --------------------------------------------------------------------
        # Here we assume that in your .ui file there is a placeholder widget named "rollpitchosd"
        # We create our custom RollPitchOSD instance with that widget as its parent.
        self.rollpitch_osd = RollPitchOSD(self.ui.rollpitchosd)
        self.rollpitch_osd.resize(self.ui.rollpitchosd.size())
        self.rollpitch_osd.show()

        # Altitude OSD placeholder - receives telemetry altitude values
        self.altitude_osd = AltitudeOSD(self.ui.altitudeosd)
        self.altitude_osd.resize(self.ui.altitudeosd.size())
        self.altitude_osd.setAltitude(0.0)
        self.altitude_osd.show()

        # Airspeed OSD placeholder - receives telemetry airspeed values
        self.airspeed_osd = AirspeedOSD(self.ui.airspeedosd)
        self.airspeed_osd.resize(self.ui.airspeedosd.size())
        self.airspeed_osd.setAirspeed(0.0)
        self.airspeed_osd.show()

        # Connect OSD visibility checkboxes to show/hide the overlays
        self.ui.chk_altitude.toggled.connect(self.altitude_osd.setVisible)
        self.ui.chk_airspeed.toggled.connect(self.airspeed_osd.setVisible)
        self.ui.chk_attitude.toggled.connect(self.rollpitch_osd.setVisible)

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


    def update_labels(self) -> None:
        """Update GUI labels and OSD widgets using the latest telemetry."""
        if self.telemetry_pitch is None:
            self.rollpitch_osd.setRollPitch(0.0, 0.0)
            self.altitude_osd.setAltitude(self.current_altitude or 0.0)
            self.airspeed_osd.setAirspeed(self.current_airspeed or 0.0)
            self.label_manager.update_labels(
                {
                    "pitch": "N/A",
                    "roll": "N/A",
                    "yaw": "N/A",
                }
            )
            return

        self.label_manager.update_labels(
            {
                "pitch": f"{self.telemetry_pitch:.1f}",
                "roll": f"{self.telemetry_roll:.1f}",
                "yaw": f"{self.telemetry_yaw:.1f}",
            }
        )
        self.rollpitch_osd.setRollPitch(self.telemetry_roll, self.telemetry_pitch)
        if self.current_altitude is not None:
            self.altitude_osd.setAltitude(self.current_altitude)
        if self.current_airspeed is not None:
            self.airspeed_osd.setAirspeed(self.current_airspeed)

    def handle_telemetry(self, packet_type, *values) -> None:
        """Receive decoded telemetry from ``CRSFPacketProcessor`` and cache it."""
        if packet_type == "attitude":
            pitch, roll, yaw = values
            self.telemetry_pitch = pitch
            self.telemetry_roll = roll
            self.telemetry_yaw = yaw
        elif packet_type == "gps":
            _lat, _lon, speed, _course, alt, _sats = values
            self.current_airspeed = speed
            self.current_altitude = alt

    def transmit_data(self):
        """
        Transmit CRSF packets using mapped joystick values and update transmit status.
        """
        if not self.joystick or not self.crsf_processor:
            self.label_manager.update_labels({
                "Transmit Status": "Hardware Unavailable",
            })
            return

        try:
            # Fetch mapped joystick values for transmission
            mapped_roll, mapped_pitch = self.joystick.get_mapped_values()

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

    def setup_configuration_page(self):
        """Create configuration page for selecting settings."""
        self.ui.configuration_page = QWidget()
        widgets.configuration_page = self.ui.configuration_page
        widgets.stackedWidget.addWidget(self.ui.configuration_page)

        layout = QVBoxLayout(self.ui.configuration_page)
        ports = [p.device for p in list_ports.comports()]

        def add_section(title):
            section = QWidget()
            vbox = QVBoxLayout(section)
            header = QHBoxLayout()
            label = QLabel(title)
            label.setStyleSheet("font-weight: bold;")
            status = QLabel()
            status.setStyleSheet("color: red;")
            header.addWidget(label)
            header.addWidget(status)
            header.addStretch()
            vbox.addLayout(header)
            layout.addWidget(section)
            return vbox, status

        def add_separator():
            line = QFrame()
            line.setFrameShape(QFrame.HLine)
            line.setFrameShadow(QFrame.Sunken)
            line.setStyleSheet("background-color: rgb(44, 49, 58);")
            layout.addWidget(line)

        # Radiofrequency settings
        rf_layout, self.rf_status = add_section("Radiofrequency Settings")
        rf_layout.addWidget(
            QLabel(f"Baud rate: {self.crsf_cfg.get('baudrate', 'N/A')}")
        )
        rf_port_row = QHBoxLayout()
        rf_port_row.addWidget(QLabel("Port"))
        self.elrs_port_combo = QComboBox()
        self.elrs_port_combo.addItems(ports)
        rf_port_row.addWidget(self.elrs_port_combo)
        rf_layout.addLayout(rf_port_row)

        rate_row = QHBoxLayout()
        rate_row.addWidget(QLabel("Packet Interval (ms)"))
        self.packet_interval_edit = QLineEdit()
        self.packet_interval_edit.setText(
            str(self.crsf_cfg.get("packet_interval", 10))
        )
        self.packet_interval_edit.setFixedWidth(80)
        rate_row.addWidget(self.packet_interval_edit)
        rf_layout.addLayout(rate_row)
        add_separator()

        # Control system settings
        control_layout, self.control_status = add_section("Control System Settings")
        control_layout.addWidget(
            QLabel(f"Baud rate: {self.joystick_cfg.get('baudrate', 'N/A')}")
        )
        control_port_row = QHBoxLayout()
        control_port_row.addWidget(QLabel("Port"))
        self.control_port_combo = QComboBox()
        self.control_port_combo.addItems(ports)
        control_port_row.addWidget(self.control_port_combo)
        control_layout.addLayout(control_port_row)

        dz_row = QHBoxLayout()
        dz_row.addWidget(QLabel("Deadzone (%)"))
        self.deadzone_slider = QSlider(Qt.Horizontal)
        self.deadzone_slider.setRange(0, 100)
        self.deadzone_slider.setValue(self.joystick_cfg.get("deadzone", 0))
        dz_row.addWidget(self.deadzone_slider)
        self.deadzone_value_label = QLabel(str(self.deadzone_slider.value()))
        dz_row.addWidget(self.deadzone_value_label)
        control_layout.addLayout(dz_row)

        sens_row = QHBoxLayout()
        sens_row.addWidget(QLabel("Sensitivity (%)"))
        self.sensitivity_slider = QSlider(Qt.Horizontal)
        self.sensitivity_slider.setRange(1, 200)
        self.sensitivity_slider.setValue(self.joystick_cfg.get("sensitivity", 100))
        sens_row.addWidget(self.sensitivity_slider)
        self.sensitivity_value_label = QLabel(str(self.sensitivity_slider.value()))
        sens_row.addWidget(self.sensitivity_value_label)
        control_layout.addLayout(sens_row)
        add_separator()

        # VTX settings
        vtx_layout, self.vtx_status = add_section("VTX System Settings")
        vtx_port_row = QHBoxLayout()
        vtx_port_row.addWidget(QLabel("Port"))
        self.video_port_combo = QComboBox()
        self.video_port_combo.addItems(ports)
        vtx_port_row.addWidget(self.video_port_combo)
        vtx_layout.addLayout(vtx_port_row)

        # Set default selections
        self.control_port_combo.setCurrentText(self.joystick_cfg.get("port", ""))
        self.elrs_port_combo.setCurrentText(self.crsf_cfg.get("port", ""))
        self.video_port_combo.setCurrentText(self.vtx_cfg.get("port", ""))

        # Connect signals
        self.control_port_combo.currentTextChanged.connect(self.on_control_port_selected)
        self.video_port_combo.currentTextChanged.connect(self.on_video_port_selected)
        self.elrs_port_combo.currentTextChanged.connect(self.on_elrs_port_selected)
        self.packet_interval_edit.editingFinished.connect(self.on_packet_interval_changed)
        self.deadzone_slider.valueChanged.connect(self.on_deadzone_changed)
        self.sensitivity_slider.valueChanged.connect(self.on_sensitivity_changed)

        # Initial connection status
        self.update_connection_status(self.control_status, self.joystick is not None)
        self.update_connection_status(self.rf_status, self.crsf_processor is not None)
        self.update_connection_status(
            self.vtx_status, validate_port("VTX", self.video_port_combo.currentText())
        )

    def on_control_port_selected(self, port: str):
        """Handle selection of control system port."""
        self.joystick_cfg["port"] = port
        if self.joystick:
            try:
                self.joystick.close_serial()
            except Exception:
                pass
            self.joystick = None
        if validate_port("joystick", port):
            try:
                self.joystick = JoystickRawHandler(
                    port=port,
                    baudrate=self.joystick_cfg.get("baudrate"),
                    deadzone=self.joystick_cfg.get("deadzone", 0),
                    sensitivity=self.joystick_cfg.get("sensitivity", 100),
                )
            except Exception as e:
                print(f"Failed to initialize joystick: {e}")
        self.update_connection_status(self.control_status, self.joystick is not None)
        save_config(self.config)

    def on_video_port_selected(self, port: str):
        """Handle selection of video transmitter port."""
        self.video_port = port
        self.vtx_cfg["port"] = port
        valid = validate_port("VTX", port)
        self.update_connection_status(self.vtx_status, valid)
        if valid:
            self.video_feed.start()
        else:
            self.video_feed.stop()
        save_config(self.config)

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
                    port=port,
                    baudrate=self.crsf_cfg.get("baudrate"),
                    telemetry_callback=self.handle_telemetry,
                )
            except Exception as e:
                print(f"Failed to initialize CRSF processor: {e}")
        self.update_connection_status(self.rf_status, self.crsf_processor is not None)
        save_config(self.config)

    def on_packet_interval_changed(self):
        try:
            interval = int(self.packet_interval_edit.text())
        except ValueError:
            interval = self.crsf_cfg.get("packet_interval", 10)
            self.packet_interval_edit.setText(str(interval))
        self.crsf_cfg["packet_interval"] = interval
        self.transmit_timer.start(interval)
        save_config(self.config)

    def on_deadzone_changed(self, value: int):
        self.joystick_cfg["deadzone"] = value
        self.deadzone_value_label.setText(str(value))
        if self.joystick:
            self.joystick.set_deadzone(value)
        save_config(self.config)

    def on_sensitivity_changed(self, value: int):
        self.joystick_cfg["sensitivity"] = value
        self.sensitivity_value_label.setText(str(value))
        if self.joystick:
            self.joystick.set_sensitivity(value)
        save_config(self.config)

    def start_blinking(self, label: QLabel):
        timer = QTimer(self)
        timer.setInterval(500)
        timer.timeout.connect(lambda: label.setVisible(not label.isVisible()))
        timer.start()
        self.status_timers[label] = timer

    def stop_blinking(self, label: QLabel):
        timer = self.status_timers.pop(label, None)
        if timer:
            timer.stop()
        label.setVisible(True)

    def update_connection_status(self, label: QLabel, connected: bool):
        if connected:
            label.setText("")
            self.stop_blinking(label)
        else:
            label.setText("Not Connected")
            label.setStyleSheet("color: red;")
            if label not in self.status_timers:
                self.start_blinking(label)

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
        if self.crsf_processor:
            self.crsf_processor.close_serial()  # Ensure serial port is closed
        if self.joystick:
            self.joystick.close_serial()
        super().closeEvent(event)
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
