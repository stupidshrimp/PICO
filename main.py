import os
import json
import time
import csv
import logging
from collections import deque
from datetime import datetime
from typing import Optional
import re

# When running on Windows, the combination of Qt's hardware accelerated scene
# graph and Chromium's GPU pipeline inside ``QWebEngineView`` would eventually
# exhaust the Direct3D device.  The GUI would continually allocate new staging
# textures until the driver reset, leading to "device loss" errors and the
# application's memory usage climbing until it crashed.  Force both Qt and the
# embedded Chromium instance to fall back to software rendering so resource
# allocation stays bounded.
os.environ.setdefault("QT_OPENGL", "software")
os.environ.setdefault("QTWEBENGINE_DISABLE_GPU", "1")

logging.basicConfig(
    filename="debug.log",
    level=logging.ERROR,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

import faulthandler, sys, atexit

# Ensure that fatal errors and unhandled exceptions are written to the log
_logfile = open("debug.log", "a")
faulthandler.enable(_logfile)

def log_uncaught_exceptions(exctype, value, tb):
    """Log any uncaught exceptions to debug.log."""
    logging.critical("Uncaught exception", exc_info=(exctype, value, tb))

sys.excepthook = log_uncaught_exceptions
atexit.register(_logfile.close)


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
    QMessageBox,
    QSizeGrip,
    QGraphicsDropShadowEffect,
    QPushButton,
    QGridLayout,
    QSizePolicy,
    QLayout,
    QProgressBar,
    QCheckBox,
    QStackedLayout,
    QSpinBox,
    QDoubleSpinBox,
    QScrollArea,
)
from PySide6.QtCore import (
    Qt,
    QTimer,
    QMetaObject,
    Slot,
    QUrl,
    QEvent,
    QPropertyAnimation,
    QEasingCurve,
    QParallelAnimationGroup,
)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtGui import QIcon, QShortcut, QKeySequence, QPixmap
from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
import shiboken6

from serial.tools import list_ports


class _MainWindowPlaceholder:
    pass


MainWindow = _MainWindowPlaceholder
from modules.app_settings import Settings
from modules.ui_functions import UIFunctions
from ui_mainwindow import Ui_MainWindow
from widgets.input_line import InputLine
from widgets.throttle_widget import ThrottleWidget

from pico_modules.pico_videofeed import VideoFeed
from pico_modules.pico_joystick2state import JoystickRawHandler
from pico_modules.pico_transmitpackets import CRSFPacketProcessor

# Import the custom OSD module
from pico_modules.rollpitch_osd import RollPitchOSD
from pico_modules.altitude_osd import AltitudeOSD
from pico_modules.airspeed_osd import AirspeedOSD
from pico_modules.compass_osd import CompassOSD

from config import load_config, save_config

from modules.data_page import DataPage
from modules.debug_page import DebugPage
from modules.sorties_page import SortiesPage


def validate_port(name: str, port: str) -> bool:
    """Validate that a serial port exists on the system."""
    available = [p.device for p in list_ports.comports()]

    # Treat empty or explicit "Not connected" selections as no connection.
    if not port or port.lower() == "not connected":
        print(f"{name} port: Not connected")
        return False

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
        self._configure_metric_labels()

        self.battery_percent_bar = None
        self.autopilot_time_label = None
        self.autopilot_longitude_label = None
        self.autopilot_latitude_label = None
        # Size the window using the command page and keep it fixed. This
        # ensures the GUI is always large enough for its contents and does not
        # change size when switching between pages.
        self.ui.stackedWidget.setCurrentWidget(self.ui.new_page)
        self.adjustSize()
        self.setFixedSize(self.size())
        self.ui.stackedWidget.setCurrentWidget(self.ui.home)

        # Control mode setup
        self.control_mode = "Manual"  # Default mode
        self.control_mode_channel = 4  # Channel 5 (0-based index)
        self.update_control_mode_label()
        # Shortcut to toggle control mode
        self.mode_shortcut = QShortcut(QKeySequence("Ctrl+M"), self)
        self.mode_shortcut.activated.connect(self.toggle_control_mode)

        # Sortie recording state and controls
        self._sortie_fields = [
            "pitch",
            "roll",
            "yaw",
            "stick_pitch",
            "stick_roll",
            "stick_yaw",
            "stick_throttle",
            "latitude",
            "longitude",
            "altitude_ft",
            "airspeed_mph",
            "ground_course",
            "satellites",
            "rssi_a",
            "rssi_b",
            "link_quality",
            "snr",
            "downlink_quality",
            "downlink_snr",
            "link_piggyback_count",
            "battery_voltage",
            "battery_current",
            "battery_capacity",
            "battery_percent",
        ]
        self._sortie_headers = ["timestamp", "packet_type", *self._sortie_fields]
        self.sortie_directory = os.path.join(os.path.dirname(__file__), "sortie data")
        self.sortie_recording = False
        self.sortie_file = None
        self.sortie_writer = None
        self.sortie_filename = None
        self._sortie_ready_state = False
        self._sortie_stale_timeout = 2.0
        self.last_telemetry_time = None
        self._rate_window_seconds = 1.0
        self._packet_times = {
            "attitude": deque(),
            "gps": deque(),
            "total": deque(),
        }

        self._settings_title_style = (
            "color: white; font-weight: bold; text-decoration: underline;"
        )
        for label_name in (
            "osdLabel",
            "attitudeSmoothingLabel",
            "telemetryWarningLabel",
        ):
            label = getattr(self.ui, label_name, None)
            if label is not None:
                label.setStyleSheet(self._settings_title_style)

        # Load configuration before building settings-dependent UI
        self.config = load_config()
        self.map_cfg = self.config.setdefault("map", {})
        self._initialize_map_configuration()
        self.aircraft_cfg = self.config.setdefault("aircraft", {})
        self.aircraft_cfg.setdefault("battery_cells", "3s")
        self._battery_full_voltage = 0.0
        self._update_battery_full_voltage()

        # Keep a placeholder for the GPS map area without loading any assets
        self.map_view = self.ui.mapframe
        self._setup_gps_map()

        self._setup_command_sidebar()
        self._setup_sortie_section()
        self.sortie_shortcut = QShortcut(QKeySequence("Ctrl+R"), self)
        self.sortie_shortcut.activated.connect(self.toggle_sortie_recording)

        global widgets
        widgets = self.ui

        # Timers used for blinking "Not Connected" indicators
        self.status_timers = {}

        # Update application branding
        self.ui.titleLeftApp.setText("PICO Program")
        self.ui.titleLeftDescription.setText(
            "a Modern UAS control platform"
        )

        logo_pixmap = QPixmap()
        logo_path = None
        for candidate in (
            os.path.join(os.path.dirname(__file__), "images", "logo.png"),
            os.path.join(os.path.dirname(__file__), "images", "images", "logo.png"),
            ":/images/images/logo.png",
        ):
            test_pixmap = QPixmap(candidate)
            if not test_pixmap.isNull():
                logo_pixmap = test_pixmap
                logo_path = candidate
                break

        if logo_path is not None:

            app_icon = QIcon(logo_path)
            self.setWindowIcon(app_icon)
            app = QApplication.instance()
            if app is not None:
                app.setWindowIcon(app_icon)

            scaled_logo = logo_pixmap.scaled(
                48,
                48,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )

            self._branding_logo = QLabel(self.ui.topLogoInfo)
            self._branding_logo.setObjectName("titleLogo")
            self._branding_logo.setGeometry(10, 4, 48, 42)
            self._branding_logo.setPixmap(scaled_logo)
            self._branding_logo.setAlignment(Qt.AlignmentFlag.AlignCenter)


        # Remove unwanted side tabs
        self.ui.btn_save.hide()
        self.ui.btn_exit.hide()

        # Rename side tab buttons
        self.ui.btn_new.setText("Command")
        self.ui.btn_widgets.setText("Configuration")

        # Add Data tab and associated graphs
        self.data_page = DataPage(self)

        # Add Sorties tab for reviewing recorded telemetry logs
        self.sorties_page = SortiesPage(self)

        # Add Debug tab for monitoring raw telemetry and joystick data
        self.debug_page = DebugPage(self)

        self._debug_packets: set[str] = set()
        self._debug_monitoring = False
        self._debug_include_joystick = False
        # Disable telemetry packet debug logging by default; it can be toggled
        # explicitly if needed for troubleshooting.
        self._telemetry_debug_logging = False

        # Use frameless window and translucent background
        # self.setWindowFlags(Qt.FramelessWindowHint)
        # self.setAttribute(Qt.WA_TranslucentBackground)

        # Configuration sections
        self.joystick_cfg = self.config.setdefault("joystick", {})
        self.joystick_cfg.setdefault("yaw_sensitivity", 100)
        self.crsf_cfg = self.config.setdefault("crsf", {})
        self.vtx_cfg = self.config.setdefault("vtx", {})
        self.warning_cfg = self.config.setdefault("warnings", {})
        self.warning_cfg.setdefault("stall_alarm_enabled", True)
        self.warning_cfg.setdefault("altitude_alarm_enabled", True)
        self.warning_cfg.setdefault("bank_angle_alarm_enabled", True)

        self.osd_cfg = self.config.setdefault("osd", {})
        smoothing_percent = int(self.osd_cfg.get("attitude_smoothing", 20))
        self.osd_cfg["attitude_smoothing"] = smoothing_percent
        if hasattr(self.ui, "attitudeSmoothingSlider"):
            self.ui.attitudeSmoothingSlider.setMinimum(1)
            self.ui.attitudeSmoothingSlider.setMaximum(100)
            self.ui.attitudeSmoothingSlider.setValue(smoothing_percent)
            self.ui.attitudeSmoothingSlider.valueChanged.connect(
                self.on_attitude_smoothing_changed
            )
        if hasattr(self.ui, "attitudeSmoothingValue"):
            self.ui.attitudeSmoothingValue.setText(f"{smoothing_percent}%")

        if hasattr(self.ui, "telemetryWarningLabel"):
            self.ui.telemetryWarningLabel.setContentsMargins(0, 12, 0, 0)
        if hasattr(self.ui, "chk_alarm_airspeed"):
            self.ui.chk_alarm_airspeed.setChecked(
                self.warning_cfg.get("stall_alarm_enabled", True)
            )
            self.ui.chk_alarm_airspeed.toggled.connect(self.on_stall_alarm_toggled)
        if hasattr(self.ui, "chk_alarm_altitude"):
            self.ui.chk_alarm_altitude.setChecked(
                self.warning_cfg.get("altitude_alarm_enabled", True)
            )
            self.ui.chk_alarm_altitude.toggled.connect(
                self.on_altitude_alarm_toggled
            )
        if hasattr(self.ui, "chk_alarm_bank"):
            self.ui.chk_alarm_bank.setChecked(
                self.warning_cfg.get("bank_angle_alarm_enabled", True)
            )
            self.ui.chk_alarm_bank.toggled.connect(self.on_bank_alarm_toggled)

        # Track last worker error to prevent dialog spam
        self._last_error_message = None
        self._last_error_time = 0

        # Warning system state
        self.stall_alarm_playing = False
        self.altitude_alarm_playing = False
        self.roll_alarm_playing = False
        self.stall_alarm_start_time = None
        self.altitude_alarm_start_time = None
        self.roll_alarm_start_time = None
        self.sound_players = {}
        self.last_attitude_packet_time = None
        self.attitude_first_received_time = None
        self.attitude_connected = False
        self.last_link_stats_packet_time = None
        self.link_stats_first_received_time = None
        self.link_stats_connected = False
        self.attitude_monitor_timer = QTimer(self)
        self.attitude_monitor_timer.timeout.connect(
            self.check_attitude_connection
        )
        self.attitude_monitor_timer.start(200)

        # Now that configuration dictionaries are available, finish initializing
        # the configuration page and any hardware interfaces that depend on
        # them.
        self._make_configuration_scrollable()

        # Initialize the video feed.  If a specific capture device index is
        # provided in the configuration it takes precedence.  Otherwise,
        # automatically scan for an available device. ``detect_device_index``
        # prefers the external VTX at index 1 but falls back to other indices
        # if needed.
        video_index = self.vtx_cfg.get("device_index")
        if video_index is None:
            video_index = VideoFeed.detect_device_index()
            if video_index is not None:
                print(
                    f"Using auto-detected video capture device index: {video_index}"
                )
            else:
                print(
                    "No video capture device found; feed will remain disconnected."
                )
        else:
            print(f"Using configured video capture device index: {video_index}")

        self.video_feed = VideoFeed(self.ui.VideoLabel, device_index=video_index)
        self.video_feed.worker.error.connect(self.handle_worker_error)

        # Start the video feed immediately. The previous implementation
        # attempted to validate a serial "port" before starting the
        # feed, which prevented connection to USB video receivers that do
        # not expose a serial interface. By starting unconditionally, any
        # available capture device at ``device_index`` will be used.
        self.video_feed.start()

    def _make_configuration_scrollable(self) -> None:
        """Wrap the configuration controls in a scroll area.

        The configuration page contains many toggles and sliders that can
        extend beyond the fixed application size.  Embedding the container
        inside a ``QScrollArea`` ensures every control remains accessible
        regardless of the window height.
        """

        layout = getattr(self.ui, "verticalLayout_13", None)
        top_menus = getattr(self.ui, "topMenus", None)
        content_settings = getattr(self.ui, "contentSettings", None)

        if not layout or top_menus is None or content_settings is None:
            return

        scroll_area = QScrollArea(content_settings)
        scroll_area.setObjectName("configurationScrollArea")
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        scroll_area.setContentsMargins(0, 0, 0, 0)

        layout.removeWidget(top_menus)
        top_menus.setParent(scroll_area)
        top_menus.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        scroll_area.setWidget(top_menus)
        layout.addWidget(scroll_area)

        self._configuration_scroll_area = scroll_area
        self.joystick = None
        if validate_port("joystick", self.joystick_cfg.get("port")):
            try:
                self.joystick = JoystickRawHandler(
                    port=self.joystick_cfg.get("port"),
                    baudrate=self.joystick_cfg.get("baudrate"),
                    deadzone=self.joystick_cfg.get("deadzone", 0),
                    sensitivity=self.joystick_cfg.get("sensitivity", 100),
                    smoothing=self.joystick_cfg.get("smoothing", 0),
                )
                self.joystick.error.connect(self.handle_worker_error)
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
                self.crsf_processor.telemetry_ready.connect(
                    self.handle_telemetry_wrapper
                )
                self.crsf_processor.error.connect(self.handle_worker_error)
            except Exception as e:
                print(f"Failed to initialize CRSF processor: {e}")
        else:
            print("CRSF disabled due to unavailable port.")

        # Timer for transmitting data (default from config)
        self.transmit_timer = QTimer(self)
        self.transmit_timer.timeout.connect(self.transmit_data)
        transmit_interval = self.crsf_cfg.get("packet_interval", 15)
        self.transmit_timer.start(transmit_interval)

        # Track transmission state and countdown handling for the configuration
        # page's terminate/start button.
        self.transmission_active = True
        self._transmission_hold_timer = QTimer(self)
        self._transmission_hold_timer.setInterval(1000)
        self._transmission_hold_timer.timeout.connect(
            self._on_transmission_hold_tick
        )
        self._transmission_hold_in_progress = False
        self._transmission_hold_remaining = 0
        self._transmission_pressed_while_inactive = False

        # Setup configuration page for COM port selections
        self.setup_configuration_page()

        # Create joystick input indicators
        self.pitch_indicator = InputLine(Qt.Vertical, self.ui.pitchInput)
        self.pitch_indicator.resize(self.ui.pitchInput.size())
        self.pitch_indicator.show()
        self.roll_indicator = InputLine(Qt.Horizontal, self.ui.rollInput)
        self.roll_indicator.resize(self.ui.rollInput.size())
        self.roll_indicator.show()
        self.yaw_indicator = InputLine(Qt.Horizontal, self.ui.yawInput)
        self.yaw_indicator.resize(self.ui.yawInput.size())
        self.yaw_indicator.show()
        self.yaw_value = 0.0
        self.yaw_target_value = 0.0
        self._yaw_keys_pressed: set[int] = set()
        self.yaw_sensitivity = int(self.joystick_cfg.get("yaw_sensitivity", 100))
        self._yaw_step_base = 0.05
        self.yaw_indicator.setValue(self.yaw_value)
        self.yaw_update_timer = QTimer(self)
        self.yaw_update_timer.timeout.connect(self.update_yaw)
        self.yaw_update_timer.start(50)
        self.throttle_percent = 0
        self.target_throttle_percent = 0
        self.throttle_indicator = ThrottleWidget(self.ui.throttleInput)
        self.throttle_indicator.resize(self.ui.throttleInput.size())
        self.throttle_indicator.show()
        self._stick_angle_scale = 90.0
        self._last_stick_pitch_norm: Optional[float] = None
        self._last_stick_roll_norm: Optional[float] = None
        self._stick_last_update = 0.0

        # Timer used to ramp the throttle toward its target value
        self.throttle_ramp_timer = QTimer(self)
        self.throttle_ramp_timer.timeout.connect(self.update_throttle)
        # Update roughly 20 times a second
        self.throttle_ramp_timer.start(50)

        # Global shortcut to immediately cut the throttle
        self.throttle_cut_shortcut = QShortcut(QKeySequence(Qt.Key_Space), self)
        self.throttle_cut_shortcut.activated.connect(self.cut_throttle)

        # Variables updated from telemetry packets
        self.telemetry_pitch = None
        self.telemetry_roll = None
        self.telemetry_yaw = None
        self.gps_lat = None
        self.gps_lon = None
        self._latest_gps_fix = None
        self._latest_gps_fix_seq = 0
        self._last_pushed_gps_fix_seq = 0
        self._gps_first_fix_sent = False
        self._gps_has_lock: Optional[bool] = None
        self.current_altitude = None
        self.current_airspeed = None
        self.telemetry_state = {field: None for field in self._sortie_fields}
        self._update_battery_full_voltage()

        # Timer used to refresh labels/OSD widgets at a fixed rate. Telemetry
        # packets only update the cached values above; the GUI is refreshed by
        # this timer regardless of packet arrival rate.
        self.label_update_timer = QTimer(self)
        self.label_update_timer.timeout.connect(self.update_labels)

        self.label_update_timer.start(0)

        self._gps_map_timer = QTimer(self)
        self._gps_map_timer.setInterval(1000)
        self._gps_map_timer.timeout.connect(self._push_gps_to_map)
        self._gps_map_timer.start()

        # --------------------------------------------------------------------
        # OSD Overlay Setup - Create and initialize the RollPitchOSD widget
        # --------------------------------------------------------------------
        # Here we assume that in your .ui file there is a placeholder widget named "rollpitchosd"
        # We create our custom RollPitchOSD instance with that widget as its parent.
        self.rollpitch_osd = RollPitchOSD(self.ui.rollpitchosd)
        self.rollpitch_osd.set_smoothing(
            self.osd_cfg.get("attitude_smoothing", 20) / 100.0
        )
        self.rollpitch_osd.resize(self.ui.rollpitchosd.size())
        self.rollpitch_osd.show()

        # Compass OSD placeholder - receives telemetry yaw values
        self.compass_osd = CompassOSD(self.ui.yawosd)
        self.compass_osd.resize(self.ui.yawosd.size())
        self.compass_osd.setYaw(0.0)
        self.compass_osd.show()
        self.compass_osd.raise_()

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
        # ``chk_compass`` was added in a later revision; guard lookup for
        # compatibility while defaulting to a visible compass when absent.
        if hasattr(self.ui, "chk_compass"):
            self.compass_osd.setVisible(self.ui.chk_compass.isChecked())
            self.ui.chk_compass.toggled.connect(self.compass_osd.setVisible)

        # TOGGLE MENU
        widgets.toggleButton.clicked.connect(lambda: UIFunctions.toggleMenu(self, True))

        # SET UI DEFINITIONS
        UIFunctions.uiDefinitions(self)

        # LEFT MENUS
        widgets.btn_home.clicked.connect(self.buttonClick)
        widgets.btn_widgets.clicked.connect(self.buttonClick)
        widgets.btn_new.clicked.connect(self.buttonClick)
        widgets.btn_data.clicked.connect(self.buttonClick)
        widgets.btn_sorties.clicked.connect(self.buttonClick)
        widgets.btn_debug.clicked.connect(self.buttonClick)

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
        widgets.fullScreenAppBtn.clicked.connect(lambda: UIFunctions.full_screen(self))

        # SET HOME PAGE AND SELECT MENU
        widgets.stackedWidget.setCurrentWidget(widgets.home)
        widgets.btn_home.setStyleSheet(UIFunctions.selectMenu(widgets.btn_home.styleSheet()))


    def _configure_metric_labels(self) -> None:
        """Constrain telemetry labels so they wrap within their sections."""

        signal_labels = [
            getattr(self.ui, "rssiALabel", None),
            getattr(self.ui, "rssiBLabel", None),
            getattr(self.ui, "linkQualityLabel", None),
            getattr(self.ui, "snrLabel", None),
            getattr(self.ui, "downlinkQualityLabel", None),
            getattr(self.ui, "downlinkSnrLabel", None),
        ]
        for label in signal_labels:
            if label is None:
                continue
            label.setWordWrap(True)
            label.setMinimumWidth(0)
            label.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred
            )

        grid_layout = getattr(self.ui, "signalMetricsGrid", None)
        if grid_layout is not None:
            grid_layout.setColumnStretch(0, 1)
            grid_layout.setColumnStretch(1, 1)

        stats_labels = [
            getattr(self.ui, "attitudeRateLabel", None),
            getattr(self.ui, "totalRateLabel", None),
        ]
        for label in stats_labels:
            if label is None:
                continue
            label.setWordWrap(True)
            label.setMinimumWidth(0)
            label.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred
            )

        stats_layout = getattr(self.ui, "telemetryStatsRowLayout", None)
        if stats_layout is not None:
            for index in range(stats_layout.count()):
                stats_layout.setStretch(index, 1)


    def _clear_layout(self, layout: Optional[QLayout]) -> None:
        """Remove all items from a layout without deleting the widgets."""

        if layout is None:
            return

        while layout.count():
            item = layout.takeAt(0)
            child_widget = item.widget()
            child_layout = item.layout()

            if child_widget is not None:
                child_widget.setParent(None)

            if child_layout is not None:
                self._clear_layout(child_layout)


    def _setup_command_sidebar(self) -> None:
        """Lay out the command tab's right column with stacked sections."""

        frame = self.ui.frame_4
        column_layout = frame.layout()
        if column_layout is None:
            column_layout = QVBoxLayout(frame)
        command_spacer = getattr(self.ui, "commandVideoSpacer", None)

        self._clear_layout(column_layout)

        column_layout.setContentsMargins(0, 12, 0, 12)
        column_layout.setSpacing(12)
        column_layout.setAlignment(Qt.AlignTop)

        panel_style = "\n".join(
            (
                "background-color: rgba(26, 30, 36, 200);",
                "border: 1px solid rgb(62, 68, 82);",
                "border-radius: 10px;",
            )
        )
        self._sidebar_panel_style = panel_style

        signal_container = QFrame(frame)
        signal_container.setObjectName("signalHealthContainer")
        signal_container.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        )
        signal_container.setStyleSheet(panel_style)

        signal_layout = QVBoxLayout(signal_container)
        signal_layout.setContentsMargins(12, 12, 12, 12)
        signal_layout.setSpacing(10)

        signal_title = self.ui.signalHealthTitle
        signal_title.setParent(signal_container)
        signal_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        signal_title.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        )
        signal_layout.addWidget(signal_title)

        metrics_grid = QGridLayout()
        metrics_grid.setContentsMargins(0, 0, 0, 0)
        metrics_grid.setHorizontalSpacing(20)
        metrics_grid.setVerticalSpacing(6)
        metrics_grid.setColumnStretch(0, 1)
        metrics_grid.setColumnStretch(1, 1)
        signal_layout.addLayout(metrics_grid)
        self.ui.signalMetricsGrid = metrics_grid

        for row, (left_widget, right_widget) in enumerate(
            (
                (self.ui.rssiALabel, self.ui.rssiBLabel),
                (self.ui.linkQualityLabel, self.ui.snrLabel),
                (self.ui.downlinkQualityLabel, self.ui.downlinkSnrLabel),
            ),
        ):
            left_widget.setParent(signal_container)
            right_widget.setParent(signal_container)
            left_widget.setWordWrap(True)
            right_widget.setWordWrap(True)
            left_widget.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            right_widget.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            left_widget.setSizePolicy(
                QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            )
            right_widget.setSizePolicy(
                QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            )
            metrics_grid.addWidget(left_widget, row, 0)
            metrics_grid.addWidget(right_widget, row, 1)

        column_layout.addWidget(signal_container)

        battery_container = QFrame(frame)
        battery_container.setObjectName("batteryHealthContainer")
        battery_container.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        )
        battery_container.setStyleSheet(panel_style)

        battery_layout = QVBoxLayout(battery_container)
        battery_layout.setContentsMargins(12, 12, 12, 12)
        battery_layout.setSpacing(10)

        battery_title = QLabel("Battery", battery_container)
        battery_title.setObjectName("batteryHealthTitle")
        battery_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        battery_title.setFont(signal_title.font())
        battery_title.setStyleSheet("color: white;")
        battery_layout.addWidget(battery_title)

        self.battery_percent_bar = QProgressBar(battery_container)
        self.battery_percent_bar.setRange(0, 100)
        self.battery_percent_bar.setValue(0)
        self.battery_percent_bar.setFormat("--%")
        self.battery_percent_bar.setAlignment(Qt.AlignCenter)
        self.battery_percent_bar.setTextVisible(True)
        self.battery_percent_bar.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        )
        battery_layout.addWidget(self.battery_percent_bar)

        column_layout.addWidget(battery_container)

        autopilot_container = QFrame(frame)
        autopilot_container.setObjectName("autopilotContainer")
        autopilot_container.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        )
        autopilot_container.setStyleSheet(panel_style)

        autopilot_layout = QVBoxLayout(autopilot_container)
        autopilot_layout.setContentsMargins(12, 12, 12, 12)
        autopilot_layout.setSpacing(10)

        autopilot_title = QLabel("Autopilot", autopilot_container)
        autopilot_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        autopilot_title.setFont(signal_title.font())
        autopilot_title.setStyleSheet("color: white;")
        autopilot_layout.addWidget(autopilot_title)

        self.autopilot_time_label = QLabel("Time to target: --", autopilot_container)
        self.autopilot_longitude_label = QLabel("Longitude: --", autopilot_container)
        self.autopilot_latitude_label = QLabel("Latitude: --", autopilot_container)

        for label in (
            self.autopilot_time_label,
            self.autopilot_longitude_label,
            self.autopilot_latitude_label,
        ):
            label.setWordWrap(True)
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            label.setSizePolicy(
                QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            )
            autopilot_layout.addWidget(label)

        column_layout.addWidget(autopilot_container)

    def _setup_sortie_section(self) -> None:
        """Create the Sortie controls within the settings sidebar."""

        settings_container = getattr(self.ui, "topMenus", None)
        settings_layout = getattr(self.ui, "verticalLayout_14", None)
        if settings_container is None or settings_layout is None:
            return

        sortie_container = QWidget(settings_container)
        sortie_container.setObjectName("sortieSettingsContainer")

        sortie_layout = QVBoxLayout(sortie_container)
        sortie_layout.setContentsMargins(0, 12, 0, 0)
        sortie_layout.setSpacing(8)

        sorties_title = QLabel("Sortie", sortie_container)
        sorties_title.setObjectName("sortiesTitle")
        sorties_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        sorties_title.setStyleSheet(self._settings_title_style)
        sortie_layout.addWidget(sorties_title)

        self.ui.sortieRecordButton = QPushButton(
            "Awaiting Telemetry", sortie_container
        )
        self.ui.sortieRecordButton.setObjectName("sortieRecordButton")
        self.ui.sortieRecordButton.setCursor(Qt.PointingHandCursor)
        self.ui.sortieRecordButton.setFixedHeight(36)
        self.ui.sortieRecordButton.setMinimumWidth(160)
        sortie_layout.addWidget(self.ui.sortieRecordButton, 0, Qt.AlignLeft)

        settings_layout.addWidget(sortie_container, 0, Qt.AlignTop)

        self._sortie_idle_style = (
            "QPushButton {"
            "background-color: #2e7d32;"
            "color: white;"
            "border: 1px solid #1b5e20;"
            "border-radius: 6px;"
            "padding: 8px 18px;"
            "}"
            "QPushButton:hover {background-color: #388e3c;}"
            "QPushButton:pressed {background-color: #1b5e20;}"
            "QPushButton:disabled {background-color: #555555; color: #999999; border-color: #444444;}"
        )
        self._sortie_active_style = (
            "QPushButton {"
            "background-color: #c62828;"
            "color: white;"
            "border: 1px solid #8e0000;"
            "border-radius: 6px;"
            "padding: 8px 18px;"
            "}"
            "QPushButton:hover {background-color: #d32f2f;}"
            "QPushButton:pressed {background-color: #b71c1c;}"
        )

        self.ui.sortieRecordButton.setEnabled(False)

        self.ui.sortieRecordButton.clicked.connect(self.toggle_sortie_recording)
        self._update_sortie_ui_state()

    def _update_sortie_ui_state(self) -> None:
        """Refresh the sortie button appearance and availability."""

        if self.sortie_recording:
            self.ui.sortieRecordButton.setEnabled(True)
            self.ui.sortieRecordButton.setText("Stop Recording")
            self.ui.sortieRecordButton.setStyleSheet(self._sortie_active_style)
            self.ui.sortieRecordButton.setToolTip("Stop recording telemetry (Ctrl+R)")
            return

        ready = self._sortie_ready_state
        self.ui.sortieRecordButton.setStyleSheet(self._sortie_idle_style)
        if ready:
            self.ui.sortieRecordButton.setEnabled(True)
            self.ui.sortieRecordButton.setText("Start Recording")
            self.ui.sortieRecordButton.setToolTip("Start recording telemetry (Ctrl+R)")
        else:
            self.ui.sortieRecordButton.setEnabled(False)
            self.ui.sortieRecordButton.setText("Awaiting Telemetry")
            self.ui.sortieRecordButton.setToolTip(
                "Telemetry data required to start recording"
            )

    def _sortie_can_record(self) -> bool:
        """Return ``True`` when telemetry data has been received recently."""

        if self.last_telemetry_time is None:
            return False
        return (time.monotonic() - self.last_telemetry_time) <= self._sortie_stale_timeout

    def _update_sortie_button_availability(self, force: bool = False) -> None:
        """Enable or disable the sortie button according to telemetry status."""

        if self.sortie_recording and not force:
            return

        can_record = self._sortie_can_record()
        if force or can_record != self._sortie_ready_state:
            self._sortie_ready_state = can_record
            self._update_sortie_ui_state()

    def _show_telemetry_required_message(self) -> None:
        """Inform the user that telemetry data is required before recording."""

        QMessageBox.warning(
            self,
            "Telemetry Required",
            "Telemetry data is not currently being received.\n"
            "Recording can only start when live telemetry is available.",
        )

    def toggle_sortie_recording(self) -> None:
        """Toggle telemetry sortie recording on or off."""

        if self.sortie_recording:
            self.stop_sortie_recording()
            return

        if not self._sortie_can_record():
            self._show_telemetry_required_message()
            return

        self.start_sortie_recording()

    def start_sortie_recording(self) -> None:
        """Begin recording telemetry to a CSV sortie file."""

        if self.sortie_recording:
            return

        try:
            os.makedirs(self.sortie_directory, exist_ok=True)
        except OSError as exc:
            logging.error(
                "Failed to create sortie directory %s: %s", self.sortie_directory, exc
            )
            QMessageBox.critical(
                self,
                "Recording Error",
                f"Could not create the sortie data folder:\n{exc}",
            )
            return
        date_str = datetime.now().strftime("%m-%d-%Y")
        pattern = re.compile(rf"{re.escape(date_str)}-sortie_(\\d+)\\.csv$")

        next_index = 1
        try:
            existing = os.listdir(self.sortie_directory)
        except OSError as exc:
            logging.error("Failed to list sortie directory: %s", exc)
            QMessageBox.critical(
                self,
                "Recording Error",
                f"Could not access the sortie data folder:\n{exc}",
            )
            return

        for name in existing:
            match = pattern.match(name)
            if match:
                next_index = max(next_index, int(match.group(1)) + 1)

        filename = f"{date_str}-sortie_{next_index}.csv"
        filepath = os.path.join(self.sortie_directory, filename)

        try:
            self.sortie_file = open(filepath, "w", newline="", encoding="utf-8")
        except OSError as exc:
            logging.error("Failed to create sortie log at %s: %s", filepath, exc)
            QMessageBox.critical(
                self,
                "Recording Error",
                f"Could not create sortie log file:\n{exc}",
            )
            self.sortie_file = None
            return

        self.sortie_writer = csv.writer(self.sortie_file)
        self.sortie_writer.writerow(self._sortie_headers)
        self.sortie_file.flush()

        self.sortie_recording = True
        self.sortie_filename = filename
        self._sortie_ready_state = True
        self._update_sortie_ui_state()

    def stop_sortie_recording(self) -> None:
        """Stop telemetry sortie recording and close the file handle."""

        if not self.sortie_recording:
            return

        if self.sortie_file:
            try:
                self.sortie_file.flush()
            except OSError:
                pass
            self.sortie_file.close()

        self.sortie_file = None
        self.sortie_writer = None
        self.sortie_recording = False
        self.sortie_filename = None
        self._update_sortie_button_availability(force=True)

    def _record_telemetry_sample(self, packet_type: str) -> None:
        """Write the current telemetry snapshot to the sortie log."""

        if not self.sortie_recording or not self.sortie_writer:
            return

        # Ensure the recorded stick state reflects the most recent inputs.
        if time.monotonic() - self._stick_last_update > 0.2:
            self._capture_stick_state()

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        row = [timestamp, packet_type]
        for field in self._sortie_fields:
            value = self.telemetry_state.get(field)
            row.append("" if value is None else value)

        try:
            self.sortie_writer.writerow(row)
            if self.sortie_file:
                self.sortie_file.flush()
        except Exception as exc:  # noqa: BLE001 - broad to stop logging on failure
            logging.error("Failed to write telemetry sortie row: %s", exc)
            self.stop_sortie_recording()
            QMessageBox.critical(
                self,
                "Recording Error",
                "Telemetry recording stopped due to a write error.\n"
                f"Details: {exc}",
            )

    def _capture_stick_state(
        self,
    ) -> tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        """Update cached joystick values for UI, logging, and sortie records."""

        joy_pitch = joy_roll = None
        norm_pitch = norm_roll = None

        if self.joystick:
            try:
                joy_pitch, joy_roll = self.joystick.get_raw_values()
            except Exception:
                joy_pitch = joy_roll = None

        if joy_pitch is not None and joy_roll is not None:
            norm_pitch = (joy_pitch - 512) / 512
            norm_roll = (joy_roll - 512) / 512
            norm_pitch = max(-1.0, min(1.0, norm_pitch))
            norm_roll = max(-1.0, min(1.0, norm_roll))
            self._last_stick_pitch_norm = norm_pitch
            self._last_stick_roll_norm = norm_roll
        else:
            norm_pitch = self._last_stick_pitch_norm
            norm_roll = self._last_stick_roll_norm

        if norm_pitch is not None:
            self.telemetry_state["stick_pitch"] = norm_pitch * self._stick_angle_scale
        else:
            self.telemetry_state["stick_pitch"] = None

        if norm_roll is not None:
            self.telemetry_state["stick_roll"] = norm_roll * self._stick_angle_scale
        else:
            self.telemetry_state["stick_roll"] = None

        self.telemetry_state["stick_yaw"] = float(self.yaw_value) * self._stick_angle_scale
        self.telemetry_state["stick_throttle"] = float(self.throttle_percent)
        self._stick_last_update = time.monotonic()

        return joy_pitch, joy_roll, norm_pitch, norm_roll

    @Slot(str)
    def handle_worker_error(self, message: str):
        """Handle errors emitted from worker threads."""
        logging.error("Worker error: %s", message)

        if "Serial connection error" in message and self.joystick:
            try:
                self.joystick.close()
            except Exception:  # noqa: BLE001 - best effort cleanup
                pass
            self.joystick = None
            self.update_connection_status(self.control_status, False)
            self.play_sound("flightcontrolsystemsoffline")
        # Suppress repetitive pop-ups for camera availability checks; the
        # VideoFeed class already overlays these messages on the feed. Showing
        # a QMessageBox for every check forces the user to constantly dismiss
        # the dialog, which is disruptive when a camera is disconnected.
        if message in {"Not connected", "Camera Error or Disconnected"}:
            return

        now = time.time()
        if (
            message != self._last_error_message
            or now - self._last_error_time > 2
        ):
            self._last_error_message = message
            self._last_error_time = now
            QMessageBox.critical(self, "Worker Error", message)

    @Slot()
    def update_labels(self) -> None:
        """Update GUI labels using joystick inputs and refresh OSD widgets."""

        self._update_sortie_button_availability()
        self._update_rate_labels()
        # Check for any telemetry-based warnings
        self.check_warnings()

        # ------------------------------------------------------------------
        # Joystick values update the label texts
        # ------------------------------------------------------------------
        joy_pitch, joy_roll, norm_pitch, norm_roll = self._capture_stick_state()
        if (
            self._debug_monitoring
            and self._debug_include_joystick
            and joy_pitch is not None
            and joy_roll is not None
        ):
            self.debug_page.log_packet("joystick", (joy_pitch, joy_roll))

        if norm_pitch is None or norm_roll is None:
            self.pitch_indicator.setValue(0)
            self.roll_indicator.setValue(0)
        else:
            self.pitch_indicator.setValue(norm_pitch)
            self.roll_indicator.setValue(norm_roll)
        self.yaw_indicator.setValue(self.yaw_value)
        self.throttle_indicator.setValue(self.throttle_percent)

        # ------------------------------------------------------------------
        # Telemetry still drives the OSD widgets
        # ------------------------------------------------------------------
        if self.telemetry_pitch is None:
            self.rollpitch_osd.setRollPitch(0.0, 0.0)
            self.altitude_osd.setAltitude(self.current_altitude or 0.0)
            self.airspeed_osd.setAirspeed(self.current_airspeed or 0.0)
            self.compass_osd.setYaw(self.telemetry_yaw or 0.0)
            return

        self.rollpitch_osd.setRollPitch(self.telemetry_roll, self.telemetry_pitch)
        if self.current_altitude is not None:
            self.altitude_osd.setAltitude(self.current_altitude)
        if self.current_airspeed is not None:
            self.airspeed_osd.setAirspeed(self.current_airspeed)
        if self.telemetry_yaw is not None:
            self.compass_osd.setYaw(self.telemetry_yaw)

    def cut_throttle(self) -> None:
        """Immediately drop the throttle to zero."""
        self.target_throttle_percent = 0
        self.throttle_percent = 0
        self.throttle_indicator.setValue(self.throttle_percent)

    def keyPressEvent(self, event):  # noqa: N802 - Qt override naming
        mapping = {
            Qt.Key_A: 25,
            Qt.Key_S: 50,
            Qt.Key_D: 75,
            Qt.Key_F: 100,
        }
        if event.key() in mapping:
            self.target_throttle_percent = mapping[event.key()]
            event.accept()
        elif event.key() in (Qt.Key_Q, Qt.Key_E):
            self._handle_yaw_key(event.key(), True)
            event.accept()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):  # noqa: N802 - Qt override naming
        if event.key() in (Qt.Key_Q, Qt.Key_E):
            if event.isAutoRepeat():
                return
            self._handle_yaw_key(event.key(), False)
            event.accept()
            return
        super().keyReleaseEvent(event)

    def update_throttle(self):
        """Gradually move the throttle toward its target value."""
        if self.throttle_percent == self.target_throttle_percent:
            return
        # Use a larger step so the throttle reaches the target value more quickly
        step = 20
        if self.throttle_percent < self.target_throttle_percent:
            self.throttle_percent = min(
                self.throttle_percent + step, self.target_throttle_percent
            )
        else:
            self.throttle_percent = max(
                self.throttle_percent - step, self.target_throttle_percent
            )
        self.throttle_indicator.setValue(self.throttle_percent)

    def update_yaw(self) -> None:
        """Gradually move the yaw indicator toward its target value."""

        step_scale = max(1, int(self.yaw_sensitivity)) / 100.0
        step = self._yaw_step_base * step_scale
        diff = self.yaw_target_value - self.yaw_value
        if abs(diff) <= step:
            if self.yaw_value != self.yaw_target_value:
                self.yaw_value = self.yaw_target_value
                self.yaw_indicator.setValue(self.yaw_value)
            return

        self.yaw_value += step if diff > 0 else -step
        self.yaw_value = max(-1.0, min(1.0, self.yaw_value))
        self.yaw_indicator.setValue(self.yaw_value)

    def _handle_yaw_key(self, key: int, pressed: bool) -> None:
        """Update yaw target tracking based on keyboard input."""

        if pressed:
            self._yaw_keys_pressed.add(key)
        else:
            self._yaw_keys_pressed.discard(key)

        if Qt.Key_Q in self._yaw_keys_pressed and Qt.Key_E in self._yaw_keys_pressed:
            target = 0.0
        elif Qt.Key_Q in self._yaw_keys_pressed:
            target = -1.0
        elif Qt.Key_E in self._yaw_keys_pressed:
            target = 1.0
        else:
            target = 0.0

        self.yaw_target_value = target

    def classify_rssi(self, rssi):
        if rssi >= -60:
            return "Excellent", "green"
        elif rssi >= -75:
            return "Good", "green"
        elif rssi >= -85:
            return "Fair", "yellow"
        elif rssi >= -95:
            return "Weak", "orange"
        else:
            return "Critical", "red"

    def classify_snr(self, snr):
        if snr >= 15:
            return "Excellent", "green"
        elif snr >= 10:
            return "Good", "green"
        elif snr >= 5:
            return "Fair", "yellow"
        elif snr >= 0:
            return "Weak", "orange"
        else:
            return "Critical", "red"

    def classify_quality(self, quality):
        if quality >= 80:
            return "Excellent", "green"
        elif quality >= 60:
            return "Good", "green"
        elif quality >= 40:
            return "Fair", "yellow"
        elif quality >= 20:
            return "Weak", "orange"
        else:
            return "Critical", "red"

    def set_label(self, label, name, value, color=None):
        label.setText(f"{name}: {value}")
        if color:
            label.setStyleSheet(f"color: {color}")
        else:
            label.setStyleSheet("")

    def _update_battery_health_display(
        self,
        voltage: Optional[float],
        _current: Optional[float],
        _capacity: Optional[int],
        _percent: Optional[float],
    ) -> None:
        if self.battery_percent_bar is None:
            return

        if not voltage or self._battery_full_voltage <= 0:
            self.telemetry_state["battery_percent"] = None
            self.battery_percent_bar.setValue(0)
            self.battery_percent_bar.setFormat("--%")
            return

        ratio = max(0.0, min(float(voltage) / self._battery_full_voltage, 1.0))
        percent = ratio * 100.0
        self.telemetry_state["battery_percent"] = percent
        self.battery_percent_bar.setRange(0, 100)
        self.battery_percent_bar.setValue(int(round(percent)))
        self.battery_percent_bar.setFormat(f"{percent:.0f}%")

    def _update_battery_full_voltage(self) -> None:
        voltage_map = {
            "2s": 8.4,
            "3s": 12.6,
            "4s": 16.8,
            "5s": 21.0,
        }
        selection = str(self.aircraft_cfg.get("battery_cells", "3s")).lower()
        self._battery_full_voltage = voltage_map.get(selection, 12.6)

        if hasattr(self, "telemetry_state") and isinstance(self.telemetry_state, dict):
            voltage = self.telemetry_state.get("battery_voltage")
            current = self.telemetry_state.get("battery_current")
            capacity = self.telemetry_state.get("battery_capacity")
            percent = self.telemetry_state.get("battery_percent")
            self._update_battery_health_display(voltage, current, capacity, percent)

    def _prune_packet_times(self, queue, current_time: float) -> None:
        cutoff = current_time - self._rate_window_seconds
        while queue and queue[0] < cutoff:
            queue.popleft()

    def _add_packet_timestamp(self, key: str, timestamp: float) -> None:
        queue = self._packet_times[key]
        queue.append(timestamp)
        self._prune_packet_times(queue, timestamp)

    def _update_packet_rates(self, packet_type: str, timestamp: float) -> None:
        self._add_packet_timestamp("total", timestamp)
        if packet_type == "attitude":
            self._add_packet_timestamp("attitude", timestamp)
        elif packet_type == "gps":
            self._add_packet_timestamp("gps", timestamp)

    def _update_rate_labels(self) -> None:
        if not hasattr(self.ui, "attitudeRateLabel"):
            return

        now = time.monotonic()
        for queue in self._packet_times.values():
            self._prune_packet_times(queue, now)

        label_map = (
            ("attitude", getattr(self.ui, "attitudeRateLabel", None), "Attitude rate"),
            ("gps", getattr(self.ui, "gpsRateLabel", None), "GPS rate"),
            ("total", getattr(self.ui, "totalRateLabel", None), "Total rate"),
        )
        for key, label, prefix in label_map:
            if label is None:
                continue
            queue = self._packet_times[key]
            if queue:
                rate = len(queue) / self._rate_window_seconds
                label.setText(f"{prefix}: {rate:.1f} Hz")
            else:
                label.setText(f"{prefix}: -- Hz")

    def play_sound(self, name: str):
        """Play a warning sound identified by ``name``.

        ``name`` may be provided without an extension (``elrsconnected``) or
        with a full filename (``elrsinitiated.mp3``). Files are loaded from the
        ``audio`` directory. Player instances are cached so repeated alerts
        reuse the same player.
        """

        if os.path.splitext(name)[1]:
            file_path = os.path.join("audio", name)
            cache_key = name
        else:
            file_path = os.path.join("audio", f"{name}.mp3")
            cache_key = name

        if not os.path.exists(file_path):
            logging.warning("Sound file not found: %s", file_path)
            return

        # Reuse an existing player if it hasn't been deleted; otherwise create
        # a fresh QMediaPlayer/QAudioOutput pair.
        player_output = self.sound_players.get(cache_key)
        if player_output:
            player, output = player_output
            if not shiboken6.isValid(player) or not shiboken6.isValid(output):
                player, output = QMediaPlayer(), QAudioOutput()
        else:
            player, output = QMediaPlayer(), QAudioOutput()

        player.setAudioOutput(output)
        player.setSource(QUrl.fromLocalFile(file_path))
        output.setVolume(1.0)

        # Ensure any previous connections are removed before connecting our
        # cleanup handler.
        media_status_signal = player.mediaStatusChanged
        if hasattr(media_status_signal, "isConnected"):
            try:
                is_connected = media_status_signal.isConnected()
            except Exception:
                is_connected = True
        else:
            is_connected = True

        if is_connected:
            try:
                media_status_signal.disconnect()
            except Exception:
                pass

        def handle_status(status, *, cache_key=cache_key, player=player, output=output):
            if status == QMediaPlayer.MediaStatus.EndOfMedia:
                try:
                    player.mediaStatusChanged.disconnect(handle_status)
                except Exception:
                    pass

                # Allow the audio backend a brief moment to drain any buffered
                # samples before tearing down the output objects. Destroying the
                # output immediately can clip the tail end of the playback on
                # some platforms.
                self.sound_players.pop(cache_key, None)

                def cleanup():
                    player.deleteLater()
                    output.deleteLater()

                QTimer.singleShot(200, cleanup)

        player.mediaStatusChanged.connect(handle_status)
        player.play()
        self.sound_players[cache_key] = (player, output)

    def play_sound_sequence(self, names, finished_callback=None):
        """Play a sequence of warning sounds in order.

        ``finished_callback`` is called when the sequence has completed.
        """
        if not names:
            if finished_callback:
                finished_callback()
            return

        name = names[0]
        if os.path.splitext(name)[1]:
            file_path = os.path.join("audio", name)
            cache_key = name
        else:
            file_path = os.path.join("audio", f"{name}.mp3")
            cache_key = name

        if not os.path.exists(file_path):
            logging.warning("Sound file not found: %s", file_path)
            if finished_callback:
                finished_callback()
            return

        player_output = self.sound_players.get(cache_key)
        if player_output:
            player, output = player_output
            if not shiboken6.isValid(player) or not shiboken6.isValid(output):
                player, output = QMediaPlayer(), QAudioOutput()
        else:
            player, output = QMediaPlayer(), QAudioOutput()

        player.setAudioOutput(output)
        player.setSource(QUrl.fromLocalFile(file_path))
        output.setVolume(1.0)

        try:
            player.mediaStatusChanged.disconnect()
        except Exception:
            pass

        def handle_status(status, *, cache_key=cache_key, player=player, output=output):
            if status == QMediaPlayer.MediaStatus.EndOfMedia:
                try:
                    player.mediaStatusChanged.disconnect(handle_status)
                except Exception:
                    pass

                self.sound_players.pop(cache_key, None)

                def cleanup():
                    player.deleteLater()
                    output.deleteLater()
                    if len(names) > 1:
                        self.play_sound_sequence(names[1:], finished_callback)
                    elif finished_callback:
                        finished_callback()

                QTimer.singleShot(200, cleanup)

        player.mediaStatusChanged.connect(handle_status)
        player.play()
        self.sound_players[cache_key] = (player, output)

    def _handle_connection_sound(self, source: str, connected: bool) -> None:
        """Play the connection or disconnection sound for ``source``."""

        sound_map = {
            ("attitude", True): "telemetryonline",
            ("attitude", False): "telemetryoffline",
            ("link_stats", True): "connectedalarm",
            ("link_stats", False): "disconnectedalarm",
        }

        self.play_sound_sequence([sound_map[(source, connected)]])

    def check_attitude_connection(self):
        """Monitor attitude packet reception and play connection sounds."""
        now = time.monotonic()
        if self.attitude_connected:
            if (
                self.last_attitude_packet_time is None
                or now - self.last_attitude_packet_time > 1.0
            ):
                self.attitude_connected = False
                self._handle_connection_sound("attitude", False)
                self.attitude_first_received_time = None
        else:
            if (
                self.last_attitude_packet_time is None
                or now - self.last_attitude_packet_time > 1.0
            ):
                self.attitude_first_received_time = None

        if self.link_stats_connected:
            if (
                self.last_link_stats_packet_time is None
                or now - self.last_link_stats_packet_time > 1.0
            ):
                self.link_stats_connected = False
                self._handle_connection_sound("link_stats", False)
                self.link_stats_first_received_time = None
        else:
            if (
                self.last_link_stats_packet_time is None
                or now - self.last_link_stats_packet_time > 1.0
            ):
                self.link_stats_first_received_time = None

    def check_warnings(self):
        """Evaluate telemetry values against configured thresholds and play alarms."""
        if (
            self.current_airspeed is None
            or self.current_altitude is None
            or self.telemetry_roll is None
        ):
            return

        now = time.monotonic()

        # Airspeed warning: low airspeed at high altitude
        stall_enabled = self.warning_cfg.get("stall_alarm_enabled", True)
        if (
            stall_enabled
            and self.current_airspeed < self.warning_cfg.get("stall_airspeed", 0)
            and self.current_altitude > self.warning_cfg.get("stall_altitude", 0)
        ):
            if self.stall_alarm_start_time is None:
                self.stall_alarm_start_time = now
            elif now - self.stall_alarm_start_time > 1.0 and not self.stall_alarm_playing:
                self.stall_alarm_playing = True
                self.play_sound_sequence(
                    ["whoopalarm", "airspeedlowarning", "airspeedlowarning"],
                    finished_callback=lambda: setattr(self, "stall_alarm_playing", False),
                )
        else:
            self.stall_alarm_start_time = None
            self.stall_alarm_playing = False

        # Altitude warning: low altitude at high airspeed
        altitude_enabled = self.warning_cfg.get("altitude_alarm_enabled", True)
        if (
            altitude_enabled
            and self.current_altitude
            < self.warning_cfg.get("altitude_alarm_altitude", 0)
            and self.current_airspeed
            > self.warning_cfg.get("altitude_alarm_airspeed", 0)
        ):
            if self.altitude_alarm_start_time is None:
                self.altitude_alarm_start_time = now
            elif (
                now - self.altitude_alarm_start_time > 1.0
                and not self.altitude_alarm_playing
            ):
                self.altitude_alarm_playing = True
                self.play_sound_sequence(
                    ["beepalarm", "altitudewarning", "pullupwarning"],
                    finished_callback=lambda: setattr(
                        self, "altitude_alarm_playing", False
                    ),
                )
        else:
            self.altitude_alarm_start_time = None
            self.altitude_alarm_playing = False

        # Roll angle warning
        bank_enabled = self.warning_cfg.get("bank_angle_alarm_enabled", True)
        if (
            bank_enabled
            and abs(self.telemetry_roll) > self.warning_cfg.get("roll_angle", 0)
        ):
            if self.roll_alarm_start_time is None:
                self.roll_alarm_start_time = now
            elif now - self.roll_alarm_start_time > 1.0 and not self.roll_alarm_playing:
                self.roll_alarm_playing = True
                self.play_sound_sequence(
                    ["downupalarm", "bankanglewarning"],
                    finished_callback=lambda: setattr(self, "roll_alarm_playing", False),
                )
        else:
            self.roll_alarm_start_time = None
            self.roll_alarm_playing = False

    @Slot(object)
    def handle_telemetry_wrapper(self, data) -> None:
        """Unpack CRSF telemetry and forward it to ``handle_telemetry``."""
        packet_type, *values = data
        self.handle_telemetry(packet_type, *values)

    def handle_telemetry(self, packet_type, *values) -> None:
        """Receive decoded telemetry from ``CRSFPacketProcessor`` and cache it."""
        if self._telemetry_debug_logging and packet_type != "link_stats":
            logging.debug("Telemetry %s: %s", packet_type, values)
        self.last_telemetry_time = time.monotonic()
        self._update_sortie_button_availability()
        now = self.last_telemetry_time
        if packet_type == "attitude":
            self.last_attitude_packet_time = now
            if not self.attitude_connected:
                if self.attitude_first_received_time is None:
                    self.attitude_first_received_time = now
                elif now - self.attitude_first_received_time >= 1.0:
                    self.attitude_connected = True
                    self.attitude_first_received_time = None
                    self._handle_connection_sound("attitude", True)
            else:
                self.attitude_first_received_time = None

            pitch, roll, yaw = values
            self.telemetry_pitch = pitch
            self.telemetry_roll = roll
            self.telemetry_yaw = yaw
            self.telemetry_state["pitch"] = pitch
            self.telemetry_state["roll"] = roll
            self.telemetry_state["yaw"] = yaw
            self.data_page.add_attitude(pitch, roll, yaw)
        elif packet_type == "gps":
            # Order: latitude, longitude, altitude (ft), speed (mph),
            # ground course, satellites
            lat, lon, alt, speed, course, sats = values
            try:
                lat_value = float(lat)
                lon_value = float(lon)
            except (TypeError, ValueError):
                lat_value = None
                lon_value = None
            self.gps_lat = lat_value if lat_value is not None else lat
            self.gps_lon = lon_value if lon_value is not None else lon
            self.current_altitude = alt
            self.current_airspeed = speed
            self.telemetry_state["latitude"] = lat_value if lat_value is not None else lat
            self.telemetry_state["longitude"] = lon_value if lon_value is not None else lon
            self.telemetry_state["altitude_ft"] = alt
            self.telemetry_state["airspeed_mph"] = speed
            self.telemetry_state["ground_course"] = course
            self.telemetry_state["satellites"] = sats
            self.data_page.add_flight_metrics(alt, speed)
            if (
                lat_value is not None
                and lon_value is not None
                and not (lat_value == 0.0 and lon_value == 0.0)
            ):
                self._latest_gps_fix = (lat_value, lon_value)
                self._latest_gps_fix_seq += 1
                self._set_gps_lock_state(True)
            else:
                self._set_gps_lock_state(False)
        elif packet_type == "battery":
            if len(values) >= 3:
                voltage, current, capacity = values[:3]
                percent = values[3] if len(values) >= 4 else None
                self.telemetry_state["battery_voltage"] = voltage
                self.telemetry_state["battery_current"] = current
                self.telemetry_state["battery_capacity"] = capacity
                self.telemetry_state["battery_percent"] = percent
                self._update_battery_health_display(voltage, current, capacity, percent)
        elif packet_type == "link_stats":
            self.last_link_stats_packet_time = now
            if not self.link_stats_connected:
                if self.link_stats_first_received_time is None:
                    self.link_stats_first_received_time = now
                elif now - self.link_stats_first_received_time >= 1.0:
                    self.link_stats_connected = True
                    self.link_stats_first_received_time = None
                    self._handle_connection_sound("link_stats", True)
            else:
                self.link_stats_first_received_time = None

            piggyback_count = 0
            if len(values) >= 7:
                (
                    rssi_a,
                    rssi_b,
                    link_quality,
                    snr,
                    downlink_lq,
                    downlink_snr,
                    piggyback_count,
                ) = values[:7]
            else:
                (
                    rssi_a,
                    rssi_b,
                    link_quality,
                    snr,
                    downlink_lq,
                    downlink_snr,
                ) = values[:6]
            self.telemetry_state["rssi_a"] = rssi_a
            self.telemetry_state["rssi_b"] = rssi_b
            self.telemetry_state["link_quality"] = link_quality
            self.telemetry_state["snr"] = snr
            self.telemetry_state["downlink_quality"] = downlink_lq
            self.telemetry_state["downlink_snr"] = downlink_snr
            self.telemetry_state["link_piggyback_count"] = piggyback_count
            self.data_page.add_link_stats(
                rssi_a,
                rssi_b,
                link_quality,
                downlink_lq,
                snr,
                downlink_snr,
                piggyback_count,
            )
            _, color = self.classify_quality(link_quality)
            self.set_label(
                self.ui.linkQualityLabel,
                "Link quality",
                f"{link_quality}%",
                color,
            )
            _, color = self.classify_quality(downlink_lq)
            self.set_label(
                self.ui.downlinkQualityLabel,
                "Downlink quality",
                f"{downlink_lq}%",
                color,
            )
            cat, color = self.classify_rssi(rssi_a)
            self.set_label(self.ui.rssiALabel, "RSSI A", cat, color)
            cat, color = self.classify_rssi(rssi_b)
            self.set_label(self.ui.rssiBLabel, "RSSI B", cat, color)
            cat, color = self.classify_snr(snr)
            self.set_label(self.ui.snrLabel, "SNR", cat, color)
            cat, color = self.classify_snr(downlink_snr)
            self.set_label(self.ui.downlinkSnrLabel, "Downlink SNR", cat, color)

        if self._debug_monitoring and packet_type in self._debug_packets:
            self.debug_page.log_packet(packet_type, values)

        self._update_packet_rates(packet_type, now)
        self._update_rate_labels()
        self._record_telemetry_sample(packet_type)

    @staticmethod
    def _map_axis_to_crsf(value: float) -> int:
        """Map a normalized control value (``-1`` to ``1``) to CRSF range."""

        value = max(-1.0, min(1.0, float(value)))
        out_min, out_max = 172, 1811
        return int(round((value + 1.0) * 0.5 * (out_max - out_min) + out_min))

    def transmit_data(self):
        """
        Transmit CRSF packets using mapped joystick values.
        """
        if not self.crsf_processor:
            return

        channels = [1500] * 16
        if self.joystick:
            try:
                mapped_roll, mapped_pitch = self.joystick.get_mapped_values()
                channels[0] = int(mapped_roll)
                channels[1] = int(mapped_pitch)
            except Exception as e:
                print(f"Error during transmission: {e}")
        # Map throttle percentage to CRSF channel range (172-1811)
        throttle_min = 172
        throttle_max = 1811
        throttle_span = throttle_max - throttle_min
        clamped_percent = max(0.0, min(100.0, self.throttle_percent))
        throttle_value = int((clamped_percent / 100) * throttle_span + throttle_min)
        channels[2] = throttle_value
        # Map yaw input to channel 4 (index 3)
        channels[3] = self._map_axis_to_crsf(self.yaw_value)

        # Control mode channel: send low for Manual, high for Fly-By-Wire
        mode_value = 1700 if self.control_mode == "Fly-By-Wire" else 400
        channels[self.control_mode_channel] = mode_value
        try:
            self.crsf_processor.channel_update.emit(channels)
        except Exception as e:
            print(f"Error during transmission: {e}")
        else:
            if self._debug_monitoring and "control" in self._debug_packets:
                self.debug_page.log_packet("control", channels)

    def update_control_mode_label(self):
        """Update the control mode indicator text and color."""
        if hasattr(self.ui, "controlModeLabel"):
            color = "rgb(0, 255, 0)" if self.control_mode == "Manual" else "rgb(255, 165, 0)"
            self.ui.controlModeLabel.setText(self.control_mode)
            self.ui.controlModeLabel.setStyleSheet(f"color: {color};")

    def toggle_control_mode(self):
        """Toggle between Manual and Fly-By-Wire modes."""
        self.control_mode = "Fly-By-Wire" if self.control_mode == "Manual" else "Manual"
        self.update_control_mode_label()
        sound_name = "fbw" if self.control_mode == "Fly-By-Wire" else "manual"
        self.play_sound(sound_name)

    def setup_configuration_page(self):
        """Create configuration page for selecting settings."""
        self.ui.configuration_page = QWidget()
        widgets.configuration_page = self.ui.configuration_page
        widgets.stackedWidget.addWidget(self.ui.configuration_page)

        container_layout = QVBoxLayout(self.ui.configuration_page)
        container_layout.setContentsMargins(0, 0, 0, 0)

        scroll_area = QScrollArea(self.ui.configuration_page)
        scroll_area.setObjectName("configurationSettingsScrollArea")
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        scroll_area.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded
        )
        container_layout.addWidget(scroll_area)

        scroll_contents = QWidget()
        layout = QVBoxLayout(scroll_contents)
        layout.setContentsMargins(0, 0, 0, 0)

        scroll_area.setWidget(scroll_contents)

        self.reinitialize_ports_button = QPushButton("Re-initialize serial ports")
        self.reinitialize_ports_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.reinitialize_ports_button.setMinimumHeight(44)
        button_policy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.reinitialize_ports_button.setSizePolicy(button_policy)
        self.reinitialize_ports_button.setStyleSheet(
            "QPushButton {"
            "background-color: rgb(46, 125, 50);"
            "color: white;"
            "font-weight: bold;"
            "border-radius: 8px;"
            "padding: 8px 16px;"
            "}"
            "QPushButton:hover {background-color: rgb(56, 142, 60);}"
            "QPushButton:pressed {background-color: rgb(35, 97, 39);}" 
            "QPushButton:disabled {background-color: rgb(80, 80, 80); color: rgb(180, 180, 180);}" 
        )
        layout.addWidget(self.reinitialize_ports_button)
        layout.addSpacing(12)

        self._transmission_button_styles = {
            "active": (
                "rgb(176, 0, 32)",
                "rgb(200, 0, 40)",
                "rgb(140, 0, 28)",
            ),
            "hold": (
                "rgb(255, 140, 0)",
                "rgb(255, 160, 16)",
                "rgb(204, 112, 0)",
            ),
            "inactive": (
                "rgb(46, 125, 50)",
                "rgb(56, 142, 60)",
                "rgb(35, 97, 39)",
            ),
        }

        self.transmission_control_button = QPushButton()
        self.transmission_control_button.setCursor(
            Qt.CursorShape.PointingHandCursor
        )
        self.transmission_control_button.setMinimumHeight(44)
        self.transmission_control_button.setSizePolicy(button_policy)
        self._apply_transmission_button_style("active")
        self.transmission_control_button.setText("Terminate transmission")
        layout.addWidget(self.transmission_control_button)
        layout.addSpacing(12)

        self.transmission_control_button.pressed.connect(
            self._on_transmission_button_pressed
        )
        self.transmission_control_button.released.connect(
            self._on_transmission_button_released
        )

        ports = ["Not connected"] + [p.device for p in list_ports.comports()]

        def add_section(title, *, show_status=True):
            section = QWidget()
            vbox = QVBoxLayout(section)
            header = QHBoxLayout()
            label = QLabel(title)
            label.setStyleSheet("font-weight: bold;")
            header.addWidget(label)
            if show_status:
                status = QLabel()
                status.setStyleSheet("color: red;")
                header.addWidget(status)
            else:
                status = None
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
        rf_layout.addWidget(
            QLabel(
                "ELRS firmware set to 250 Hz packet rate with telemetry reporting rate of 75 Hz (13 ms)."
            )
        )
        self.pico_rate_label = QLabel()
        rf_layout.addWidget(self.pico_rate_label)
        self.update_pico_rate_label()
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
            str(self.crsf_cfg.get("packet_interval", 3))
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

        yaw_sens_row = QHBoxLayout()
        yaw_sens_row.addWidget(QLabel("Yaw sensitivity (%)"))
        self.yaw_sensitivity_slider = QSlider(Qt.Horizontal)
        self.yaw_sensitivity_slider.setRange(1, 200)
        self.yaw_sensitivity_slider.setValue(
            self.joystick_cfg.get("yaw_sensitivity", 100)
        )
        yaw_sens_row.addWidget(self.yaw_sensitivity_slider)
        self.yaw_sensitivity_value_label = QLabel(
            str(self.yaw_sensitivity_slider.value())
        )
        yaw_sens_row.addWidget(self.yaw_sensitivity_value_label)
        control_layout.addLayout(yaw_sens_row)

        smooth_row = QHBoxLayout()
        smooth_row.addWidget(QLabel("Smoothing (%)"))
        self.smoothing_slider = QSlider(Qt.Horizontal)
        self.smoothing_slider.setRange(0, 100)
        self.smoothing_slider.setValue(self.joystick_cfg.get("smoothing", 0))
        smooth_row.addWidget(self.smoothing_slider)
        self.smoothing_value_label = QLabel(str(self.smoothing_slider.value()))
        smooth_row.addWidget(self.smoothing_value_label)
        control_layout.addLayout(smooth_row)
        add_separator()

        aircraft_layout, _ = add_section(
            "Aircraft configuration", show_status=False
        )
        battery_row = QHBoxLayout()
        battery_row.addWidget(QLabel("Battery pack"))
        self.battery_type_combo = QComboBox()
        self.battery_type_combo.addItems(["2s", "3s", "4s", "5s"])
        battery_row.addWidget(self.battery_type_combo)
        battery_row.addStretch()
        aircraft_layout.addLayout(battery_row)

        add_separator()

        map_layout, _ = add_section("GPS Map Settings", show_status=False)
        self.map_enabled_checkbox = QCheckBox("Enable offline map rendering")
        self.map_enabled_checkbox.setChecked(self.map_cfg.get("enabled", True))
        map_layout.addWidget(self.map_enabled_checkbox)

        follow_row = QHBoxLayout()
        self.map_follow_checkbox = QCheckBox("Follow GPS position")
        self.map_follow_checkbox.setChecked(self._gps_follow_enabled)
        follow_row.addWidget(self.map_follow_checkbox)
        follow_row.addStretch()
        map_layout.addLayout(follow_row)

        zoom_row = QHBoxLayout()
        zoom_row.addWidget(QLabel("Default zoom"))
        self.map_zoom_spin = QSpinBox()
        self.map_zoom_spin.setRange(self._map_min_zoom, self._map_max_zoom)
        self.map_zoom_spin.setValue(int(self.map_cfg.get("zoom", self._map_initial_zoom)))
        zoom_row.addWidget(self.map_zoom_spin)
        zoom_row.addStretch()
        map_layout.addLayout(zoom_row)

        center_row = QHBoxLayout()
        center_row.addWidget(QLabel("Default center"))
        self.map_lat_spin = QDoubleSpinBox()
        self.map_lat_spin.setRange(-90.0, 90.0)
        self.map_lat_spin.setDecimals(6)
        self.map_lat_spin.setValue(float(self.map_cfg.get("center", [0.0, 0.0])[0]))
        center_row.addWidget(self.map_lat_spin)
        self.map_lon_spin = QDoubleSpinBox()
        self.map_lon_spin.setRange(-180.0, 180.0)
        self.map_lon_spin.setDecimals(6)
        self.map_lon_spin.setValue(float(self.map_cfg.get("center", [0.0, 0.0])[1]))
        center_row.addWidget(self.map_lon_spin)
        center_row.addStretch()
        map_layout.addLayout(center_row)

        if not (self._map_tiles_available and self._map_html_available):
            self.map_enabled_checkbox.setEnabled(False)
            self.map_enabled_checkbox.setToolTip(
                "Offline map assets are not available. Provide tiles and index.html to enable rendering."
            )
            for widget in (
                self.map_follow_checkbox,
                self.map_zoom_spin,
                self.map_lat_spin,
                self.map_lon_spin,
            ):
                widget.setToolTip(
                    "GPS defaults can be edited now and will apply once map assets are installed."
                )

        self.map_enabled_checkbox.toggled.connect(self.on_map_enabled_toggled)
        self.map_follow_checkbox.toggled.connect(self.on_map_follow_toggled)
        self.map_zoom_spin.valueChanged.connect(self.on_map_zoom_changed)
        self.map_lat_spin.valueChanged.connect(self.on_map_center_changed)
        self.map_lon_spin.valueChanged.connect(self.on_map_center_changed)

        add_separator()

        # VTX settings (video receiver is treated as a camera device, so no
        # serial port configuration is required)
        vtx_layout, self.vtx_status = add_section(
            "VTX System Settings", show_status=False
        )
        add_separator()

        # Warning system settings
        warn_layout, _ = add_section("Warning System")

        warn_layout.addWidget(QLabel("Air Speed Alarm"))
        stall_speed_row = QHBoxLayout()
        stall_speed_row.addWidget(QLabel("Airspeed <"))
        self.stall_speed_slider = QSlider(Qt.Horizontal)
        self.stall_speed_slider.setRange(0, 200)
        self.stall_speed_slider.setValue(
            self.warning_cfg.get("stall_airspeed", 0)
        )
        stall_speed_row.addWidget(self.stall_speed_slider)
        self.stall_speed_value = QLabel(str(self.stall_speed_slider.value()))
        stall_speed_row.addWidget(self.stall_speed_value)
        warn_layout.addLayout(stall_speed_row)

        stall_alt_row = QHBoxLayout()
        stall_alt_row.addWidget(QLabel("Altitude >"))
        self.stall_alt_slider = QSlider(Qt.Horizontal)
        self.stall_alt_slider.setRange(0, 1000)
        self.stall_alt_slider.setValue(
            self.warning_cfg.get("stall_altitude", 0)
        )
        stall_alt_row.addWidget(self.stall_alt_slider)
        self.stall_alt_value = QLabel(str(self.stall_alt_slider.value()))
        stall_alt_row.addWidget(self.stall_alt_value)
        warn_layout.addLayout(stall_alt_row)

        warn_layout.addWidget(QLabel("Altitude Alarm"))
        alt_alarm_alt_row = QHBoxLayout()
        alt_alarm_alt_row.addWidget(QLabel("Altitude <"))
        self.alt_alarm_alt_slider = QSlider(Qt.Horizontal)
        self.alt_alarm_alt_slider.setRange(0, 1000)
        self.alt_alarm_alt_slider.setValue(
            self.warning_cfg.get("altitude_alarm_altitude", 0)
        )
        alt_alarm_alt_row.addWidget(self.alt_alarm_alt_slider)
        self.alt_alarm_alt_value = QLabel(str(self.alt_alarm_alt_slider.value()))
        alt_alarm_alt_row.addWidget(self.alt_alarm_alt_value)
        warn_layout.addLayout(alt_alarm_alt_row)

        alt_alarm_speed_row = QHBoxLayout()
        alt_alarm_speed_row.addWidget(QLabel("Airspeed >"))
        self.alt_alarm_speed_slider = QSlider(Qt.Horizontal)
        self.alt_alarm_speed_slider.setRange(0, 200)
        self.alt_alarm_speed_slider.setValue(
            self.warning_cfg.get("altitude_alarm_airspeed", 0)
        )
        alt_alarm_speed_row.addWidget(self.alt_alarm_speed_slider)
        self.alt_alarm_speed_value = QLabel(str(self.alt_alarm_speed_slider.value()))
        alt_alarm_speed_row.addWidget(self.alt_alarm_speed_value)
        warn_layout.addLayout(alt_alarm_speed_row)

        roll_row = QHBoxLayout()
        roll_row.addWidget(QLabel("Roll |>|"))
        self.roll_angle_slider = QSlider(Qt.Horizontal)
        self.roll_angle_slider.setRange(0, 180)
        self.roll_angle_slider.setValue(
            self.warning_cfg.get("roll_angle", 0)
        )
        roll_row.addWidget(self.roll_angle_slider)
        self.roll_angle_value = QLabel(str(self.roll_angle_slider.value()))
        roll_row.addWidget(self.roll_angle_value)
        warn_layout.addLayout(roll_row)

        # Set default selections
        self.control_port_combo.setCurrentText(
            self.joystick_cfg.get("port", "Not connected")
        )
        self.elrs_port_combo.setCurrentText(
            self.crsf_cfg.get("port", "Not connected")
        )
        self.battery_type_combo.setCurrentText(
            self.aircraft_cfg.get("battery_cells", "3s")
        )
        # Connect signals
        self.control_port_combo.currentTextChanged.connect(self.on_control_port_selected)
        self.elrs_port_combo.currentTextChanged.connect(self.on_elrs_port_selected)
        self.packet_interval_edit.editingFinished.connect(self.on_packet_interval_changed)
        self.deadzone_slider.valueChanged.connect(self.on_deadzone_changed)
        self.sensitivity_slider.valueChanged.connect(self.on_sensitivity_changed)
        self.yaw_sensitivity_slider.valueChanged.connect(
            self.on_yaw_sensitivity_changed
        )
        self.smoothing_slider.valueChanged.connect(self.on_smoothing_changed)
        self.stall_speed_slider.valueChanged.connect(self.on_stall_speed_changed)
        self.stall_alt_slider.valueChanged.connect(self.on_stall_alt_changed)
        self.alt_alarm_alt_slider.valueChanged.connect(self.on_alt_alarm_alt_changed)
        self.alt_alarm_speed_slider.valueChanged.connect(self.on_alt_alarm_speed_changed)
        self.roll_angle_slider.valueChanged.connect(self.on_roll_angle_changed)
        self.battery_type_combo.currentTextChanged.connect(
            self.on_battery_type_changed
        )

        # Initial connection status
        self.update_connection_status(self.control_status, self.joystick is not None)
        self.update_connection_status(self.rf_status, self.crsf_processor is not None)
        # Video connection status is derived from the video feed itself
        layout.addStretch()

        self.update_connection_status(self.vtx_status, False)
        # Ensure the port lists reflect currently connected devices
        self.update_port_lists()

        self.reinitialize_ports_button.clicked.connect(self.reinitialize_serial_ports)

    def _apply_transmission_button_style(self, state: str) -> None:
        """Apply the configured stylesheet variant for the transmission button."""

        background, hover, pressed = self._transmission_button_styles[state]
        stylesheet = (
            "QPushButton {"
            f"background-color: {background};"
            "color: white;"
            "font-weight: bold;"
            "border-radius: 8px;"
            "padding: 8px 16px;"
            "}"
            f"QPushButton:hover {{background-color: {hover};}}"
            f"QPushButton:pressed {{background-color: {pressed};}}"
            "QPushButton:disabled {background-color: rgb(80, 80, 80); color: rgb(180, 180, 180);}"
        )
        self.transmission_control_button.setStyleSheet(stylesheet)

    def _on_transmission_button_pressed(self) -> None:
        """Handle press events for the terminate/start transmission button."""

        self._transmission_pressed_while_inactive = not self.transmission_active

        if not self.transmission_active:
            return

        self._transmission_hold_in_progress = True
        self._transmission_hold_remaining = 3
        self._apply_transmission_button_style("hold")
        self._update_transmission_hold_display()
        self._transmission_hold_timer.start()

    def _on_transmission_button_released(self) -> None:
        """Handle release events for the terminate/start transmission button."""

        if self._transmission_hold_in_progress:
            self._cancel_transmission_hold()
            return

        if self._transmission_pressed_while_inactive and not self.transmission_active:
            self._start_transmission()

    def _on_transmission_hold_tick(self) -> None:
        """Update the hold-to-terminate countdown while the button is pressed."""

        if not self.transmission_control_button.isDown():
            self._cancel_transmission_hold()
            return

        self._transmission_hold_remaining -= 1
        if self._transmission_hold_remaining > 0:
            self._update_transmission_hold_display()
            return

        self._transmission_hold_timer.stop()
        self._transmission_hold_in_progress = False
        self._terminate_transmission()

    def _update_transmission_hold_display(self) -> None:
        """Refresh the countdown text while the terminate button is held."""

        self.transmission_control_button.setText(
            f"Hold for {self._transmission_hold_remaining} seconds to terminate transmission"
        )

    def _cancel_transmission_hold(self) -> None:
        """Cancel an in-progress hold-to-terminate action."""

        self._transmission_hold_timer.stop()
        self._transmission_hold_in_progress = False
        if self.transmission_active:
            self._apply_transmission_button_style("active")
            self.transmission_control_button.setText("Terminate transmission")

    def _terminate_transmission(self) -> None:
        """Stop packet transmission and update button state."""

        if not self.transmission_active:
            return

        self.transmit_timer.stop()
        self.transmission_active = False
        self._transmission_pressed_while_inactive = False
        self._transmission_hold_in_progress = False
        self._transmission_hold_timer.stop()
        self._apply_transmission_button_style("inactive")
        self.transmission_control_button.setText("Start transmitting packets")
        self.play_sound("elrsterminated.mp3")

    def _start_transmission(self) -> None:
        """Resume packet transmission and reset button state."""

        if self.transmission_active:
            return

        interval = self.crsf_cfg.get("packet_interval", 3)
        self.transmit_timer.start(interval)
        self.transmission_active = True
        self._transmission_pressed_while_inactive = False
        self._apply_transmission_button_style("active")
        self.transmission_control_button.setText("Terminate transmission")
        self.play_sound("elrsinitiated.mp3")

    def update_port_lists(self):
        """Refresh available serial ports and update the dropdowns."""
        ports = ["Not connected"] + [p.device for p in list_ports.comports()]

        def refresh(combo, handler):
            current = combo.currentText()
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(ports)
            combo.blockSignals(False)
            if current in ports:
                combo.setCurrentText(current)
            else:
                combo.setCurrentText("Not connected")
                handler("Not connected")

        refresh(self.control_port_combo, self.on_control_port_selected)
        refresh(self.elrs_port_combo, self.on_elrs_port_selected)
        # Video receiver uses a fixed device index; no port list to refresh

    def reinitialize_serial_ports(self):
        """Rescan serial ports and restart active connections."""

        self.update_port_lists()

        for combo, handler in (
            (self.control_port_combo, self.on_control_port_selected),
            (self.elrs_port_combo, self.on_elrs_port_selected),
        ):
            handler(combo.currentText())

    def on_control_port_selected(self, port: str):
        """Handle selection of control system port."""
        self.joystick_cfg["port"] = port
        if self.joystick:
            try:
                self.joystick.close()
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
                    smoothing=self.joystick_cfg.get("smoothing", 0),
                )
            except Exception as e:
                print(f"Failed to initialize joystick: {e}")
        self.update_connection_status(self.control_status, self.joystick is not None)
        save_config(self.config)

    def on_map_enabled_toggled(self, checked: bool):
        self.map_cfg["enabled"] = bool(checked)
        self._update_map_enabled_state()
        save_config(self.config)

    def on_map_follow_toggled(self, checked: bool):
        self.map_cfg["follow"] = bool(checked)
        self._gps_follow_enabled = bool(checked)
        self._sync_follow_state_to_map()
        save_config(self.config)

    def on_map_zoom_changed(self, value: int):
        try:
            zoom = int(value)
        except (TypeError, ValueError):
            zoom = self.map_cfg.get("zoom", self._map_initial_zoom)
        zoom = max(self._map_min_zoom, min(self._map_max_zoom, zoom))
        self.map_cfg["zoom"] = zoom
        if not self._gps_first_fix_sent:
            self._apply_initial_map_view()
        save_config(self.config)

    def on_map_center_changed(self):
        lat = float(self.map_lat_spin.value())
        lon = float(self.map_lon_spin.value())
        self.map_cfg["center"] = [lat, lon]
        if not self._gps_first_fix_sent:
            self._apply_initial_map_view()
        save_config(self.config)

    def on_elrs_port_selected(self, port: str):
        """Handle selection of ELRS transmitter port."""
        self.crsf_cfg["port"] = port
        if self.crsf_processor:
            try:
                thread = self.crsf_processor._thread
                QMetaObject.invokeMethod(
                    self.crsf_processor, "close_serial", Qt.BlockingQueuedConnection
                )
                thread.quit()
                thread.wait()
            except Exception:
                pass
            self.crsf_processor = None
        if validate_port("CRSF", port):
            try:
                self.crsf_processor = CRSFPacketProcessor(
                    port=port,
                    baudrate=self.crsf_cfg.get("baudrate"),
                )
                self.crsf_processor.telemetry_ready.connect(
                    self.handle_telemetry_wrapper
                )
            except Exception as e:
                print(f"Failed to initialize CRSF processor: {e}")
        self.update_connection_status(self.rf_status, self.crsf_processor is not None)
        save_config(self.config)

    def on_packet_interval_changed(self):
        try:
            interval = int(self.packet_interval_edit.text())
        except ValueError:
            interval = self.crsf_cfg.get("packet_interval", 3)
            self.packet_interval_edit.setText(str(interval))
        self.crsf_cfg["packet_interval"] = interval
        if self.transmission_active:
            self.transmit_timer.start(interval)
        self.update_pico_rate_label()
        save_config(self.config)

    def update_pico_rate_label(self):
        interval = self.crsf_cfg.get("packet_interval", 3)
        freq = 0
        if interval:
            freq = 1000 / interval
        self.pico_rate_label.setText(f"PICO writing packets at {freq:.0f} Hz.")

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

    def on_yaw_sensitivity_changed(self, value: int):
        self.joystick_cfg["yaw_sensitivity"] = value
        self.yaw_sensitivity_value_label.setText(str(value))
        self.yaw_sensitivity = int(value)
        save_config(self.config)

    def on_smoothing_changed(self, value: int):
        self.joystick_cfg["smoothing"] = value
        self.smoothing_value_label.setText(str(value))
        if self.joystick:
            self.joystick.set_smoothing(value)
        save_config(self.config)

    def on_attitude_smoothing_changed(self, value: int):
        try:
            percent = int(value)
        except (TypeError, ValueError):
            percent = self.osd_cfg.get("attitude_smoothing", 20)
        percent = max(1, min(100, percent))
        self.osd_cfg["attitude_smoothing"] = percent
        if hasattr(self.ui, "attitudeSmoothingValue"):
            self.ui.attitudeSmoothingValue.setText(f"{percent}%")
        if hasattr(self, "rollpitch_osd"):
            self.rollpitch_osd.set_smoothing(percent / 100.0)
        save_config(self.config)

    def on_stall_speed_changed(self, value: int):
        self.warning_cfg["stall_airspeed"] = value
        self.stall_speed_value.setText(str(value))
        save_config(self.config)

    def on_stall_alt_changed(self, value: int):
        self.warning_cfg["stall_altitude"] = value
        self.stall_alt_value.setText(str(value))
        save_config(self.config)

    def on_alt_alarm_alt_changed(self, value: int):
        self.warning_cfg["altitude_alarm_altitude"] = value
        self.alt_alarm_alt_value.setText(str(value))
        save_config(self.config)

    def on_alt_alarm_speed_changed(self, value: int):
        self.warning_cfg["altitude_alarm_airspeed"] = value
        self.alt_alarm_speed_value.setText(str(value))
        save_config(self.config)

    def on_roll_angle_changed(self, value: int):
        self.warning_cfg["roll_angle"] = value
        self.roll_angle_value.setText(str(value))
        save_config(self.config)

    def on_battery_type_changed(self, selection: str):
        self.aircraft_cfg["battery_cells"] = selection
        self._update_battery_full_voltage()
        save_config(self.config)

    def on_stall_alarm_toggled(self, checked: bool):
        self.warning_cfg["stall_alarm_enabled"] = checked
        if not checked:
            self.stall_alarm_start_time = None
            self.stall_alarm_playing = False
        save_config(self.config)

    def on_altitude_alarm_toggled(self, checked: bool):
        self.warning_cfg["altitude_alarm_enabled"] = checked
        if not checked:
            self.altitude_alarm_start_time = None
            self.altitude_alarm_playing = False
        save_config(self.config)

    def on_bank_alarm_toggled(self, checked: bool):
        self.warning_cfg["bank_angle_alarm_enabled"] = checked
        if not checked:
            self.roll_alarm_start_time = None
            self.roll_alarm_playing = False
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

    def update_connection_status(self, label: Optional[QLabel], connected: bool):
        if label is None:
            return
        if connected:
            label.setText("")
            self.stop_blinking(label)
        else:
            label.setText("Not Connected")
            label.setStyleSheet("color: red;")
            if label not in self.status_timers:
                self.start_blinking(label)

    @staticmethod
    def _is_valid_coordinate(value) -> bool:
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            return False
        try:
            lat = float(value[0])
            lon = float(value[1])
        except (TypeError, ValueError):
            return False
        return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0

    def _initialize_map_configuration(self):
        self.map_cfg.setdefault("enabled", True)
        self.map_cfg.setdefault("follow", True)
        self.map_cfg.setdefault("center", [42.0467957, -87.816288])
        self.map_cfg.setdefault("zoom", 13)

        self._app_root = os.path.dirname(os.path.abspath(__file__))
        self._map_directory = os.path.join(self._app_root, "map")
        self._map_tiles_directory = os.path.join(self._map_directory, "chicago_xyz")
        (
            self._map_tiles_available,
            self._map_tiles_status_message,
        ) = self._verify_map_tiles_directory(self._map_tiles_directory)

        zoom_levels = (
            self._detect_map_zoom_levels(self._map_tiles_directory)
            if self._map_tiles_available
            else []
        )
        if zoom_levels:
            self._map_min_zoom = min(zoom_levels)
            self._map_max_zoom = max(zoom_levels)
        else:
            self._map_min_zoom = 0
            self._map_max_zoom = 13

        center = self.map_cfg.get("center", [0.0, 0.0])
        if not self._is_valid_coordinate(center):
            center = [0.0, 0.0]

        zoom = self.map_cfg.get("zoom", self._map_max_zoom)
        try:
            zoom = int(zoom)
        except (TypeError, ValueError):
            zoom = self._map_max_zoom
        zoom = max(self._map_min_zoom, min(self._map_max_zoom, zoom))
        self.map_cfg["zoom"] = zoom

        self._map_initial_center = [float(center[0]), float(center[1])]
        self._map_initial_zoom = zoom
        self._gps_follow_enabled = bool(self.map_cfg.get("follow", True))

        self._map_html_path = os.path.join(self._map_directory, "index.html")
        self._map_html_available = os.path.isfile(self._map_html_path)
        if not self._map_tiles_available and not self._map_tiles_status_message:
            tiles_path = os.path.abspath(self._map_tiles_directory)
            self._map_tiles_status_message = f"Offline map tiles not found at\n{tiles_path}"
        if not self._map_html_available:
            html_path = os.path.abspath(self._map_html_path)
            self._map_tiles_status_message = (
                f"Offline map page not found at\n{html_path}"
                if not self._map_tiles_status_message
                else self._map_tiles_status_message
            )

        self._gps_map_widget = None
        self._gps_map_placeholder_label = None
        self._gps_map_container_layout = None
        self._gps_map_ready = False
        self._map_html_url = (
            QUrl.fromLocalFile(self._map_html_path)
            if self._map_html_available
            else None
        )

    def _verify_map_tiles_directory(self, directory: str) -> tuple[bool, str]:
        abs_directory = os.path.abspath(directory)
        if not os.path.isdir(directory):
            return False, f"Offline map tiles not found at\n{abs_directory}"

        zoom_levels = self._detect_map_zoom_levels(directory)
        if not zoom_levels:
            return (
                False,
                "No zoom level folders were found in the offline map tiles directory.",
            )

        tile_extensions = (".png", ".jpg", ".jpeg")
        extensions_display = ", ".join(ext.lstrip(".").upper() for ext in tile_extensions)
        sample_tile: Optional[str] = None
        for zoom in sorted(zoom_levels):
            zoom_dir = os.path.join(directory, str(zoom))
            try:
                x_entries = [entry for entry in os.listdir(zoom_dir) if entry.isdigit()]
            except OSError as exc:
                return (
                    False,
                    f"Unable to read zoom level folder {zoom_dir}:\n{exc}",
                )
            for x_entry in x_entries:
                x_dir = os.path.join(zoom_dir, x_entry)
                try:
                    tiles = [
                        entry
                        for entry in os.listdir(x_dir)
                        if entry.lower().endswith(tile_extensions)
                    ]
                except OSError as exc:
                    return (
                        False,
                        f"Unable to read tile column folder {x_dir}:\n{exc}",
                    )
                if tiles:
                    sample_tile = os.path.join(x_dir, tiles[0])
                    break
            if sample_tile:
                break

        if not sample_tile:
            return (
                False,
                f"No {extensions_display} tiles were found in the offline map tiles directory.",
            )

        try:
            if os.path.getsize(sample_tile) <= 0:
                return False, f"Sample tile is empty:\n{sample_tile}"
        except OSError as exc:
            return (
                False,
                f"Unable to inspect sample tile {sample_tile}:\n{exc}",
            )

        return True, ""

    def _detect_map_zoom_levels(self, directory: str) -> list[int]:
        zoom_levels: list[int] = []
        try:
            for entry in os.listdir(directory):
                if entry.isdigit():
                    zoom_levels.append(int(entry))
        except (FileNotFoundError, NotADirectoryError, PermissionError):
            return []
        return sorted(zoom_levels)

    def _setup_gps_map(self):
        container = self.ui.mapframe
        existing_layout = container.layout()
        if existing_layout is not None:
            QWidget().setLayout(existing_layout)  # type: ignore[arg-type]

        layout = QStackedLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setStackingMode(QStackedLayout.StackOne)
        container.setLayout(layout)
        self._gps_map_container_layout = layout

        placeholder = QLabel("GPS map disabled in settings.", container)
        placeholder.setWordWrap(True)
        placeholder.setAlignment(Qt.AlignCenter)
        placeholder.setStyleSheet("color: rgb(200, 200, 200);")
        layout.addWidget(placeholder)
        self._gps_map_placeholder_label = placeholder

        if not self._map_tiles_available:
            message = self._map_tiles_status_message
            if not message:
                tiles_path = os.path.abspath(self._map_tiles_directory)
                message = f"Offline map tiles not found at\n{tiles_path}"
            placeholder.setText(message)
            self._gps_map_widget = None
            self._update_map_enabled_state()
            return

        if not self._map_html_available or self._map_html_url is None:
            placeholder.setText(self._map_tiles_status_message or "Map page not available.")
            self._gps_map_widget = None
            self._update_map_enabled_state()
            return

        map_widget = QWebEngineView(container)
        map_widget.setContextMenuPolicy(Qt.NoContextMenu)
        layout.addWidget(map_widget)
        map_widget.loadFinished.connect(self._on_gps_map_load_finished)
        map_widget.load(self._map_html_url)
        self._gps_map_widget = map_widget

        self._update_map_enabled_state()

    def _on_gps_map_load_finished(self, ok: bool) -> None:
        if not ok:
            self._gps_map_ready = False
            if self._gps_map_placeholder_label is not None:
                self._gps_map_placeholder_label.setText(
                    "Failed to load GPS map. Check that index.html and Leaflet assets are present."
                )
            self._update_map_enabled_state()
            return

        self._gps_map_ready = True
        if self._gps_map_widget is not None:
            self._gps_map_widget.page().runJavaScript(
                f"window.setMaxZoom({int(self._map_max_zoom)});"
            )
        self._apply_initial_map_view()
        self._sync_gps_lock_state_to_map()
        self._sync_follow_state_to_map()
        if self._latest_gps_fix is not None:
            lat, lon = self._latest_gps_fix
            self._invoke_update_gps(float(lat), float(lon))

    def _apply_initial_map_view(self):
        if not self.map_cfg.get("enabled", True):
            return
        if not self._gps_map_ready or self._gps_map_widget is None:
            return
        center = self.map_cfg.get("center", self._map_initial_center)
        zoom = float(self.map_cfg.get("zoom", self._map_initial_zoom))
        lat = float(center[0]) if len(center) >= 1 else self._map_initial_center[0]
        lon = float(center[1]) if len(center) >= 2 else self._map_initial_center[1]
        script = f"window.setInitialView({lat:.8f}, {lon:.8f}, {zoom});"
        self._gps_map_widget.page().runJavaScript(script)

    def _sync_follow_state_to_map(self) -> None:
        if not self._gps_map_ready:
            return
        if self._latest_gps_fix is None:
            return
        self._invoke_update_gps(float(self._latest_gps_fix[0]), float(self._latest_gps_fix[1]))

    def _invoke_update_gps(
        self, lat: float, lon: float, *, force_follow: Optional[bool] = None
    ) -> None:
        if self._gps_map_widget is None or not self._gps_map_ready:
            return
        if force_follow is None:
            follow_enabled = self._gps_follow_enabled
        else:
            follow_enabled = force_follow
        follow = "true" if follow_enabled else "false"
        script = f"window.updateGPS({lat:.8f}, {lon:.8f}, {follow});"
        self._gps_map_widget.page().runJavaScript(script)

    def _sync_gps_lock_state_to_map(self) -> None:
        if (
            self._gps_has_lock is None
            or self._gps_map_widget is None
            or not self._gps_map_ready
        ):
            return
        lock_state = "true" if self._gps_has_lock else "false"
        self._gps_map_widget.page().runJavaScript(f"window.setGpsLock({lock_state});")

    def _set_gps_lock_state(self, has_lock: bool) -> None:
        if self._gps_has_lock == has_lock:
            return
        self._gps_has_lock = has_lock
        if not has_lock:
            self._latest_gps_fix = None
            default_lat, default_lon = self._map_initial_center
            self._invoke_update_gps(
                float(default_lat), float(default_lon), force_follow=True
            )
        self._sync_gps_lock_state_to_map()

    def _push_gps_to_map(self):
        if not self.map_cfg.get("enabled", True):
            return
        if not self._gps_map_ready or self._gps_map_widget is None:
            return
        if self._latest_gps_fix is None:
            return
        if self._latest_gps_fix_seq == self._last_pushed_gps_fix_seq:
            return

        lat, lon = self._latest_gps_fix
        self._invoke_update_gps(float(lat), float(lon))
        self._last_pushed_gps_fix_seq = self._latest_gps_fix_seq
        if not self._gps_first_fix_sent:
            self._gps_first_fix_sent = True

    def _update_map_enabled_state(self):
        if self._gps_map_container_layout is None or self._gps_map_placeholder_label is None:
            return
        if (
            self._gps_map_widget is None
            or not self._map_tiles_available
            or not self._map_html_available
        ):
            self._gps_map_container_layout.setCurrentWidget(self._gps_map_placeholder_label)
            return
        if self.map_cfg.get("enabled", True):
            self._gps_map_container_layout.setCurrentWidget(self._gps_map_widget)
        else:
            self._gps_map_placeholder_label.setText("GPS map disabled in settings.")
            self._gps_map_container_layout.setCurrentWidget(self._gps_map_placeholder_label)

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
            self.update_port_lists()
            widgets.stackedWidget.setCurrentWidget(widgets.configuration_page)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

        # SHOW NEW PAGE
        if btnName == "btn_new":
            widgets.stackedWidget.setCurrentWidget(widgets.new_page)  # SET PAGE
            UIFunctions.resetStyle(self, btnName)  # RESET ANOTHERS BUTTONS SELECTED
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))  # SELECT MENU

        # SHOW DATA PAGE
        if btnName == "btn_data":
            widgets.stackedWidget.setCurrentWidget(widgets.data_page)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

        if btnName == "btn_sorties":
            widgets.stackedWidget.setCurrentWidget(widgets.sorties_page)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

        if btnName == "btn_debug":
            widgets.stackedWidget.setCurrentWidget(widgets.debug_page)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

    def start_debug_monitoring(self, packets: set[str], include_joystick: bool) -> None:
        """Begin forwarding selected telemetry streams to the Debug tab."""

        self._debug_packets = set(packets)
        self._debug_include_joystick = include_joystick
        self._debug_monitoring = True
        self.debug_page.begin_monitoring(self._debug_packets, include_joystick)
        if self._debug_packets and not self.crsf_processor:
            self.debug_page.append_message(
                "Telemetry transmitter not connected; waiting for packets."
            )
        if include_joystick and not self.joystick:
            self.debug_page.append_message(
                "Joystick not connected; waiting for data."
            )

    def stop_debug_monitoring(self) -> None:
        """Stop forwarding telemetry to the Debug tab."""

        if not self._debug_monitoring:
            return
        self._debug_monitoring = False
        self._debug_packets.clear()
        self._debug_include_joystick = False
        self.debug_page.end_monitoring()

    def resizeEvent(self, event):
        UIFunctions.resize_grips(self)
        self.rollpitch_osd.resize(self.ui.rollpitchosd.size())

    def mousePressEvent(self, event):
        """Capture the position of the mouse press."""
        if event.buttons() == Qt.LeftButton:
            self.dragPos = event.globalPosition().toPoint()

    def cleanup(self):
        """Clean up peripheral resources if they exist."""
        self.stop_sortie_recording()
        self.stop_debug_monitoring()
        if self.joystick:
            self.joystick.close()
            self.joystick = None
    def closeEvent(self, event):
        """
        Releases resources when the window is closed.
        """
        self.video_feed.shutdown()
        if self.crsf_processor:
            thread = self.crsf_processor._thread
            QMetaObject.invokeMethod(
                self.crsf_processor, "close_serial", Qt.BlockingQueuedConnection
            )
            thread.quit()
            thread.wait()
        self.cleanup()
        super().closeEvent(event)


if __name__ == "__main__":
    QApplication.setAttribute(Qt.AA_UseSoftwareOpenGL, True)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.aboutToQuit.connect(window.cleanup)
    try:
        exit_code = app.exec()
    finally:
        window.cleanup()
    sys.exit(exit_code)

