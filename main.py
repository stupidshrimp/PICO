import os
import json
import time
import csv
import logging
import math
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
    QStyle,
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
    QElapsedTimer,
    QEventLoop,
)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtGui import QIcon, QShortcut, QKeySequence, QPixmap, QPalette, QColor
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
from pico_modules.pico_transmitpackets import (
    CRSF_CHANNEL_CENTER,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MIN,
    CRSFPacketProcessor,
)

# Import the custom OSD module
from pico_modules.rollpitch_osd import RollPitchOSD
from pico_modules.altitude_osd import AltitudeOSD
from pico_modules.airspeed_osd import AirspeedOSD
from pico_modules.compass_osd import CompassOSD

from config import (
    ALLOWED_ATTITUDE_PACKET_RATES_HZ,
    packet_interval_ms_from_rate,
    packet_rate_hz_from_interval,
    load_config,
    save_config,
)

from modules.data_page import DataPage
from modules.debug_page import DebugPage
from modules.sorties_page import SortiesPage
from modules.documentation_page import DocumentationPage
from modules.preflight_page import PreFlightChecklistPage


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
    # Keep these in lockstep with flight_controller/Main.ino. The GS scales
    # Fly-By-Wire stick commands against these redundant FC safety limits, while
    # the configurable ``fbw`` settings determine the operator-commanded limits.
    FBW_FC_MAX_ROLL_ANGLE_DEG = 80.0
    FBW_FC_MAX_PITCH_ANGLE_DEG = 80.0
    DEFAULT_FBW_MAX_ROLL_ANGLE_DEG = 45.0
    DEFAULT_FBW_MAX_PITCH_ANGLE_DEG = 30.0
    # Keep in lockstep with AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH in
    # flight_controller/Main.ino; CH3 auto-throttle setpoints are scaled by this
    # fixed range on both the GS and FC.
    AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH = 100.0
    JOYSTICK_THROTTLE_MODE_BUTTON = 1
    JOYSTICK_CONTROL_MODE_BUTTON = 13
    JOYSTICK_YAW_LEFT_BUTTON = 14
    JOYSTICK_YAW_RIGHT_BUTTON = 15
    JOYSTICK_ELEVATOR_TRIM_DOWN_BUTTON = 8
    JOYSTICK_ELEVATOR_TRIM_UP_BUTTON = 7
    JOYSTICK_AILERON_TRIM_LEFT_BUTTON = 6
    JOYSTICK_AILERON_TRIM_RIGHT_BUTTON = 0
    TRIM_STEP_NORMALIZED = 0.02
    TRIM_MAX_LEVEL = 25
    SINK_RATE_STALE_TIMEOUT_S = 0.5
    SINK_RATE_WINDOW_S = 1.5
    SINK_RATE_MIN_WINDOW_S = 0.75
    SINK_RATE_MIN_SAMPLE_INTERVAL_S = 0.01


    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self._configure_metric_labels()

        self.battery_percent_bar = None
        self.autopilot_time_label = None
        self.autopilot_longitude_label = None
        self.autopilot_latitude_label = None
        self.gps_fix_status_label = None
        self.gps_fix_status_dot = None
        # Size the window using the command page and keep it fixed. This
        # ensures the GUI is always large enough for its contents and does not
        # change size when switching between pages.
        self.ui.stackedWidget.setCurrentWidget(self.ui.new_page)
        self.adjustSize()
        self.setFixedSize(self.size())
        self.ui.stackedWidget.setCurrentWidget(self.ui.home)

        # Control mode setup
        self.control_mode = "Manual"  # Default mode
        self.elrs_arm_channel = 4  # Channel 5/AUX1 (0-based index)
        self.control_mode_channel = 5  # Channel 6/AUX2 (0-based index)
        self.desired_fbw_roll = None
        self.desired_fbw_pitch = None
        self._latest_control_channels = [CRSF_CHANNEL_CENTER] * 16
        self._safe_shutdown_frame_count = 3
        self._safe_shutdown_timeout_ms = 250
        self.update_control_mode_label()
        # Shortcut to toggle control mode
        self.mode_shortcut = QShortcut(QKeySequence("Ctrl+M"), self)
        self.mode_shortcut.activated.connect(self.toggle_control_mode)

        # Throttle mode setup. Manual mode sends CH3 as throttle percent; auto
        # throttle sends CH3 as a desired airspeed setpoint for the FC-side PID.
        self.throttle_mode = "Manual"
        self.throttle_target_airspeed_mph = 20.0
        self.throttle_mode_channel = 6  # Channel 7/AUX3 (0-based index), CH5 is reserved.
        self.auto_throttle_speed_channel_max_mph = self.AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH
        self._setup_throttle_mode_indicator()
        self.update_throttle_mode_label()
        self.throttle_mode_shortcut = QShortcut(QKeySequence("Ctrl+B"), self)
        self.throttle_mode_shortcut.activated.connect(self.toggle_throttle_mode)

        # Trim state. Positive elevator trim commands nose-up trim; positive
        # aileron trim commands left-bank trim, matching the FC roll convention.
        self.elevator_trim_level = 0
        self.aileron_trim_level = 0
        self._setup_trim_indicator()
        self.update_trim_labels()

        # Airborne detection state. The detector uses fresh GPS telemetry,
        # pitot airspeed, and barometric altitude above a grounded baseline to
        # debounce transitions between grounded and airborne.
        self.airborne_state = "grounded"
        self.airborne_baseline_altitude_ft = None
        self._airborne_has_ground_baseline = False
        self._airborne_takeoff_start_time = None
        self._airborne_landing_start_time = None
        self._last_airborne_indicator_state = None
        self._gps_has_lock: Optional[bool] = None
        self._last_gps_fix_indicator_state = None

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
        self._sortie_flush_interval = 0.5
        self._last_sortie_flush_time = 0.0
        self.last_telemetry_time = None
        self._rate_window_seconds = 1.0
        self._packet_times = {
            "attitude": deque(),
            "gps": deque(),
            "total": deque(),
        }
        self._last_telemetry_packet_times = {}
        self._last_attitude_packet_timestamp = None
        self._last_control_update_debug_timestamp = None

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
        self._setup_airborne_indicator()
        self._setup_gps_fix_indicator()
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

        # Pre-flight checklist is temporarily hidden from the sidebar.
        self.preflight_page = None

        # Add Data tab and associated graphs
        self.data_page = DataPage(self)

        # Add Sorties tab for reviewing recorded telemetry logs
        self.sorties_page = SortiesPage(self)

        # Add Debug tab for monitoring raw telemetry and joystick data
        self.debug_page = DebugPage(self)

        # Add Documentation tab for detailed operational guides
        self.documentation_page = DocumentationPage(self)

        self._debug_packets: set[str] = set()
        self._debug_monitoring = False
        self._last_control_update_debug_timestamp = None
        self._debug_include_joystick = False
        self._debug_serial_all = False
        self._debug_telemetry_all = False
        self._link_diagnostics_active = False
        self._update_link_diagnostics_button_state()
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
        self.fbw_cfg = self.config.setdefault("fbw", {})
        self.fbw_max_roll_angle_deg = self._validated_fbw_limit(
            self.fbw_cfg.get(
                "max_roll_angle_deg", self.DEFAULT_FBW_MAX_ROLL_ANGLE_DEG
            ),
            self.FBW_FC_MAX_ROLL_ANGLE_DEG,
            self.DEFAULT_FBW_MAX_ROLL_ANGLE_DEG,
        )
        self.fbw_max_pitch_angle_deg = self._validated_fbw_limit(
            self.fbw_cfg.get(
                "max_pitch_angle_deg", self.DEFAULT_FBW_MAX_PITCH_ANGLE_DEG
            ),
            self.FBW_FC_MAX_PITCH_ANGLE_DEG,
            self.DEFAULT_FBW_MAX_PITCH_ANGLE_DEG,
        )
        self.fbw_cfg["max_roll_angle_deg"] = self.fbw_max_roll_angle_deg
        self.fbw_cfg["max_pitch_angle_deg"] = self.fbw_max_pitch_angle_deg
        self.throttle_cfg = self.config.setdefault("throttle", {})
        self.throttle_cfg.setdefault("target_airspeed_mph", 20.0)
        self.auto_throttle_speed_channel_max_mph = self.AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH
        self.throttle_target_airspeed_mph = self._clamp_auto_throttle_speed(
            self.throttle_cfg.get("target_airspeed_mph", 20.0)
        )
        self.throttle_cfg["target_airspeed_mph"] = self.throttle_target_airspeed_mph
        # Do not make the CH3 speed scale configurable unless the FC-side
        # AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH is synchronized too.
        self.throttle_cfg.pop("speed_channel_max_mph", None)
        self.update_throttle_mode_label()
        self.vtx_cfg = self.config.setdefault("vtx", {})
        self.warning_cfg = self.config.setdefault("warnings", {})
        self.warning_cfg.setdefault("warning_alarms_enabled", True)
        self.warning_cfg.setdefault("stall_alarm_enabled", True)
        self.warning_cfg.setdefault("altitude_alarm_enabled", True)
        self.warning_cfg.setdefault("bank_angle_alarm_enabled", True)
        self.warning_cfg.setdefault("sink_rate_alarm_enabled", True)
        self.warning_cfg.setdefault("sink_rate_threshold_fps", 10.0)
        self.airborne_cfg = self.config.setdefault("airborne", {})
        self.airborne_cfg.setdefault("takeoff_airspeed_multiplier", 1.2)
        self.airborne_cfg.setdefault("takeoff_altitude_ft", 15.0)
        self.airborne_cfg.setdefault("landed_airspeed_mph", 7.0)
        self.airborne_cfg.setdefault("landed_altitude_ft", 5.0)
        self.airborne_cfg.setdefault("takeoff_hold_s", 2.0)
        self.airborne_cfg.setdefault("landing_hold_s", 5.0)
        self.airborne_cfg.setdefault("gps_fresh_timeout_s", 2.0)

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
        if hasattr(self.ui, "chk_alarm_master"):
            self.ui.chk_alarm_master.setChecked(
                self.warning_cfg.get("warning_alarms_enabled", True)
            )
            self.ui.chk_alarm_master.toggled.connect(self.on_warning_alarms_toggled)
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
        if hasattr(self.ui, "chk_alarm_sink_rate"):
            self.ui.chk_alarm_sink_rate.setChecked(
                self.warning_cfg.get("sink_rate_alarm_enabled", True)
            )
            self.ui.chk_alarm_sink_rate.toggled.connect(
                self.on_sink_rate_alarm_toggled
            )

        # Track last worker error to prevent dialog spam
        self._last_error_message = None
        self._last_error_time = 0

        # Warning system state
        self.stall_alarm_playing = False
        self.altitude_alarm_playing = False
        self.roll_alarm_playing = False
        self.sink_rate_alarm_playing = False
        self.stall_alarm_start_time = None
        self.altitude_alarm_start_time = None
        self.roll_alarm_start_time = None
        self.sink_rate_alarm_start_time = None
        self.sound_players = {}
        self._muted_sounds = {}
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
        # Seed safe control values before the CRSF worker is created.  The
        # worker may begin transmitting as soon as the port opens, so its
        # initial channel cache must already reflect the UI's safe mapping
        # instead of constructor defaults.
        self.yaw_value = 0.0
        self.yaw_target_value = 0.0
        self.throttle_percent = 0
        self.target_throttle_percent = 0

        self.joystick = self._create_joystick_handler(self.joystick_cfg.get("port"))

        # Initialize CRSFPacketProcessor in a safe, idle state.  Opening a valid
        # transmitter port must not start RC output until the operator explicitly
        # presses the Start transmitting packets button.
        self.crsf_processor = self._create_crsf_processor(
            self.crsf_cfg.get("port"), transmission_enabled=False
        )

        # Timer for transmitting data (default from config).  A connected CRSF
        # port is not the same thing as active transmission; the operator must
        # explicitly start RC output.
        self.transmit_timer = QTimer(self)
        self.transmit_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.transmit_timer.timeout.connect(self.transmit_data)
        self.transmission_active = False

        # Track transmission state and countdown handling for the configuration
        # page's terminate/start button.
        self._update_parameter_query_button_state()
        self._update_link_diagnostics_button_state()
        self._transmission_hold_timer = QTimer(self)
        self._transmission_hold_timer.setInterval(50)
        self._transmission_hold_timer.timeout.connect(
            self._on_transmission_hold_tick
        )
        self._transmission_hold_in_progress = False
        self._transmission_hold_required_ms = 3000
        self._transmission_hold_elapsed_timer = QElapsedTimer()
        self._transmission_pressed_while_inactive = False

        # Setup configuration page for COM port selections
        self.setup_configuration_page()
        self._start_serial_port_monitor()

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
        self._yaw_keys_pressed: set[int] = set()
        self._joystick_yaw_buttons_pressed: set[int] = set()
        self.yaw_sensitivity = int(self.joystick_cfg.get("yaw_sensitivity", 100))
        self._yaw_step_base = 0.05
        self.yaw_indicator.setValue(self.yaw_value)
        self.yaw_update_timer = QTimer(self)
        self.yaw_update_timer.timeout.connect(self.update_yaw)
        self.yaw_update_timer.start(50)
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
        self.current_altitude = None
        self.current_airspeed = None
        self.current_sink_rate_fps = None
        self._sink_rate_samples = deque()
        self._last_sink_rate_time = None
        self.last_airspeed_packet_time = None
        self.telemetry_state = {field: None for field in self._sortie_fields}
        self._update_battery_full_voltage()

        # Timer used to refresh labels/OSD widgets at a fixed rate. Telemetry
        # packets only update the cached values above; the GUI is refreshed by
        # this timer regardless of packet arrival rate.
        self.label_update_timer = QTimer(self)
        self.label_update_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.label_update_timer.timeout.connect(self.update_labels)

        # Bound UI/OSD refresh work to about 30 Hz instead of running whenever
        # the event loop is idle, which keeps video, map, audio, and serial
        # processing responsive under load.
        self.label_update_timer.start(33)

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
        if hasattr(widgets, "btn_preflight"):
            widgets.btn_preflight.hide()
        widgets.btn_data.clicked.connect(self.buttonClick)
        widgets.btn_sorties.clicked.connect(self.buttonClick)
        widgets.btn_debug.clicked.connect(self.buttonClick)
        widgets.btn_documentation.clicked.connect(self.buttonClick)

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

        def style_section_header(label: QLabel) -> None:
            """Make command sidebar section titles bold without boxed label borders."""

            title_font = label.font()
            title_font.setBold(True)
            title_font.setUnderline(False)
            label.setFont(title_font)
            label.setStyleSheet(
                "color: white; background: transparent; border: none;"
            )

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
        style_section_header(signal_title)
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
        style_section_header(battery_title)
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

        flight_status_container = QFrame(frame)
        flight_status_container.setObjectName("flightStatusContainer")
        flight_status_container.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        )
        flight_status_container.setStyleSheet(panel_style)

        flight_status_layout = QVBoxLayout(flight_status_container)
        flight_status_layout.setContentsMargins(12, 12, 12, 12)
        flight_status_layout.setSpacing(10)

        flight_status_row = QHBoxLayout()
        flight_status_row.setContentsMargins(0, 0, 0, 0)
        flight_status_row.setSpacing(12)

        flight_status_text_layout = QVBoxLayout()
        flight_status_text_layout.setContentsMargins(0, 0, 0, 0)
        flight_status_text_layout.setSpacing(2)

        flight_status_title = QLabel("Flight Status", flight_status_container)
        flight_status_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        flight_status_title.setFont(signal_title.font())
        style_section_header(flight_status_title)
        flight_status_text_layout.addWidget(flight_status_title)

        self.airborne_status_label = QLabel("GROUNDED", flight_status_container)
        self.airborne_status_label.setObjectName("airborneStatusLabel")
        self.airborne_status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.airborne_status_label.setMinimumHeight(28)
        flight_status_text_layout.addWidget(self.airborne_status_label)

        flight_status_row.addLayout(flight_status_text_layout, 1)

        self.airborne_status_dot = QLabel(flight_status_container)
        self.airborne_status_dot.setObjectName("airborneStatusDot")
        self.airborne_status_dot.setFixedSize(14, 14)
        flight_status_row.addWidget(
            self.airborne_status_dot, 0, Qt.AlignRight | Qt.AlignVCenter
        )
        flight_status_layout.addLayout(flight_status_row)

        gps_fix_row = QHBoxLayout()
        gps_fix_row.setContentsMargins(0, 0, 0, 0)
        gps_fix_row.setSpacing(12)

        gps_fix_text_layout = QVBoxLayout()
        gps_fix_text_layout.setContentsMargins(0, 0, 0, 0)
        gps_fix_text_layout.setSpacing(2)

        gps_fix_title = QLabel("GPS Fix", flight_status_container)
        gps_fix_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        gps_fix_title.setFont(signal_title.font())
        style_section_header(gps_fix_title)
        gps_fix_text_layout.addWidget(gps_fix_title)

        self.gps_fix_status_label = QLabel("NO FIX", flight_status_container)
        self.gps_fix_status_label.setObjectName("gpsFixStatusLabel")
        self.gps_fix_status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.gps_fix_status_label.setMinimumHeight(28)
        gps_fix_text_layout.addWidget(self.gps_fix_status_label)

        gps_fix_row.addLayout(gps_fix_text_layout, 1)

        self.gps_fix_status_dot = QLabel(flight_status_container)
        self.gps_fix_status_dot.setObjectName("gpsFixStatusDot")
        self.gps_fix_status_dot.setFixedSize(14, 14)
        gps_fix_row.addWidget(
            self.gps_fix_status_dot, 0, Qt.AlignRight | Qt.AlignVCenter
        )
        flight_status_layout.addLayout(gps_fix_row)

        column_layout.addWidget(flight_status_container)

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
        style_section_header(autopilot_title)
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

        show_autopilot_section = False
        autopilot_container.setVisible(show_autopilot_section)
        if show_autopilot_section:
            column_layout.addWidget(autopilot_container)

    def _setup_airborne_indicator(self) -> None:
        """Initialise the command-page grounded/airborne indicator."""

        self._last_airborne_indicator_state = None
        self._update_airborne_indicator()

    def _update_airborne_indicator(self) -> None:
        """Render a minimal flight-state pill in the command sidebar."""

        label = getattr(self, "airborne_status_label", None)
        dot = getattr(self, "airborne_status_dot", None)
        if label is None or dot is None:
            return

        state = "airborne" if self.airborne_state == "airborne" else "grounded"
        if state == self._last_airborne_indicator_state:
            return

        if state == "airborne":
            text = "AIRBORNE"
            accent = "#00e5ff"
            background = "rgba(0, 229, 255, 32)"
            border = "rgba(0, 229, 255, 155)"
            dot_shadow = "rgba(0, 229, 255, 95)"
        else:
            text = "GROUNDED"
            accent = "#9aa4b2"
            background = "rgba(154, 164, 178, 24)"
            border = "rgba(154, 164, 178, 105)"
            dot_shadow = "rgba(154, 164, 178, 70)"

        label.setText(text)
        label.setStyleSheet(
            "font-size: 13px;"
            "font-weight: 700;"
            "letter-spacing: 2px;"
            f"color: {accent};"
            f"background-color: {background};"
            f"border: 1px solid {border};"
            "border-radius: 14px;"
            "padding: 5px 12px;"
        )
        dot.setStyleSheet(
            f"background-color: {accent};"
            f"border: 3px solid {dot_shadow};"
            "border-radius: 7px;"
        )
        self._last_airborne_indicator_state = state

    def _setup_gps_fix_indicator(self) -> None:
        """Initialise the command-page GPS fix indicator."""

        self._last_gps_fix_indicator_state = None
        self._update_gps_fix_indicator()

    def _update_gps_fix_indicator(self) -> None:
        """Render the current GPS fix state in the command sidebar."""

        label = getattr(self, "gps_fix_status_label", None)
        dot = getattr(self, "gps_fix_status_dot", None)
        if label is None or dot is None:
            return

        state = "fix" if bool(self._gps_has_lock) else "no_fix"
        if state == self._last_gps_fix_indicator_state:
            return

        if state == "fix":
            text = "FIX VALID"
            accent = "#21d07a"
            background = "rgba(33, 208, 122, 32)"
            border = "rgba(33, 208, 122, 155)"
            dot_shadow = "rgba(33, 208, 122, 95)"
        else:
            text = "NO FIX"
            accent = "#ff5252"
            background = "rgba(255, 82, 82, 28)"
            border = "rgba(255, 82, 82, 135)"
            dot_shadow = "rgba(255, 82, 82, 75)"

        label.setText(text)
        label.setStyleSheet(
            "font-size: 13px;"
            "font-weight: 700;"
            "letter-spacing: 2px;"
            f"color: {accent};"
            f"background-color: {background};"
            f"border: 1px solid {border};"
            "border-radius: 14px;"
            "padding: 5px 12px;"
        )
        dot.setStyleSheet(
            f"background-color: {accent};"
            f"border: 3px solid {dot_shadow};"
            "border-radius: 7px;"
        )
        self._last_gps_fix_indicator_state = state

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
        self._last_sortie_flush_time = time.monotonic()

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
                now = time.monotonic()
                if now - self._last_sortie_flush_time >= self._sortie_flush_interval:
                    self.sortie_file.flush()
                    self._last_sortie_flush_time = now
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
        self.telemetry_state["stick_throttle"] = float(self._throttle_indicator_value())
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
        now = time.monotonic()
        self._update_airborne_state(now)
        # Check for any telemetry-based warnings
        self.check_warnings(now)

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

        self._handle_joystick_button_events()
        self._update_desired_fbw_attitude_from_stick(joy_pitch, joy_roll)

        if norm_pitch is None or norm_roll is None:
            self.pitch_indicator.setValue(0)
            self.roll_indicator.setValue(0)
        else:
            self.pitch_indicator.setValue(norm_pitch)
            self.roll_indicator.setValue(norm_roll)
        self.yaw_indicator.setValue(self.yaw_value)
        self._update_throttle_indicator()

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
        """Immediately drop the throttle to zero and return to manual throttle."""
        self.throttle_mode = "Manual"
        self.target_throttle_percent = 0
        self.throttle_percent = 0
        self.update_throttle_mode_label()
        self._update_throttle_indicator()

    def keyPressEvent(self, event):  # noqa: N802 - Qt override naming
        manual_mapping = {
            Qt.Key_A: 25,
            Qt.Key_S: 50,
            Qt.Key_D: 75,
            Qt.Key_F: 100,
        }
        if event.key() in manual_mapping:
            if self.throttle_mode == "Manual":
                self.target_throttle_percent = manual_mapping[event.key()]
            # In Auto Throttle mode CH3 is driven only by the configured target
            # airspeed on the configuration page, so throttle hotkeys do not
            # mutate the active setpoint.
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
        """Update manual throttle ramping and refresh the throttle indicator."""
        if self.throttle_mode == "Auto Throttle":
            self._update_throttle_indicator()
            return

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
        self._update_throttle_indicator()

    def _clamp_auto_throttle_speed(self, speed_mph: float) -> float:
        """Clamp an auto-throttle speed setpoint to the CH3 converter range."""

        try:
            speed = float(speed_mph)
        except (TypeError, ValueError):
            speed = 20.0
        if not math.isfinite(speed):
            speed = 20.0
        return max(0.0, min(self.auto_throttle_speed_channel_max_mph, speed))

    def set_auto_throttle_target_speed(self, speed_mph: float) -> None:
        """Persist the configured target speed sent on CH3 in Auto Throttle."""

        self.throttle_target_airspeed_mph = self._clamp_auto_throttle_speed(speed_mph)
        self.throttle_cfg["target_airspeed_mph"] = self.throttle_target_airspeed_mph
        if hasattr(self, "auto_throttle_target_spin"):
            self.auto_throttle_target_spin.blockSignals(True)
            self.auto_throttle_target_spin.setValue(self.throttle_target_airspeed_mph)
            self.auto_throttle_target_spin.blockSignals(False)
        self.update_throttle_mode_label()
        self._update_throttle_indicator()
        save_config(self.config)

    def _throttle_indicator_value(self) -> float:
        """Return the value shown by the throttle bar in the current mode."""

        if self.throttle_mode == "Auto Throttle":
            return self.throttle_target_airspeed_mph
        return float(getattr(self, "throttle_percent", 0))

    def _update_throttle_indicator(self) -> None:
        """Show manual throttle percent or auto-throttle target speed on the bar."""

        if not hasattr(self, "throttle_indicator"):
            return
        value = self._throttle_indicator_value()
        self.throttle_indicator.setValue(value)
        if self.throttle_mode == "Auto Throttle":
            self.throttle_indicator.setToolTip(
                f"Auto throttle target: {self.throttle_target_airspeed_mph:.0f} mph"
            )
        else:
            self.throttle_indicator.setToolTip(
                f"Manual throttle command: {self.throttle_percent:.0f}%"
            )

    def _safe_float(self, value, default: Optional[float] = None) -> Optional[float]:
        """Return ``value`` as a finite float, or ``default`` when invalid."""

        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return default
        return parsed if math.isfinite(parsed) else default

    def _airborne_config_value(self, key: str, default: float) -> float:
        """Read a positive numeric airborne-detector setting."""

        value = self._safe_float(self.airborne_cfg.get(key), default)
        if value is None:
            return float(default)
        return max(0.0, value)

    def _takeoff_airspeed_threshold_mph(self) -> float:
        """Derive takeoff speed from the stall warning within the desired band."""

        stall_speed = self._safe_float(self.warning_cfg.get("stall_airspeed"), 10.0)
        multiplier = self._airborne_config_value("takeoff_airspeed_multiplier", 1.2)
        derived = max(0.0, (stall_speed or 0.0) * multiplier)
        return max(12.0, min(18.0, derived))

    def _airspeed_value_fresh(self, now: float, timeout: float) -> bool:
        if self.last_airspeed_packet_time is None:
            return False
        return (now - self.last_airspeed_packet_time) <= max(0.0, timeout)

    def _reset_sink_rate_estimate(self) -> None:
        """Clear cached sink-rate state so stale samples cannot trigger alarms."""

        self.current_sink_rate_fps = None
        self._sink_rate_samples.clear()
        self._last_sink_rate_time = None

    def _update_sink_rate_from_altitude(
        self, altitude_ft: Optional[float], timestamp: float
    ) -> None:
        """Update the descent-rate estimate from successive altitude samples."""

        if altitude_ft is None:
            self._reset_sink_rate_estimate()
            return

        sample_time = timestamp
        if (
            self._last_sink_rate_time is not None
            and timestamp - self._last_sink_rate_time
            < self.SINK_RATE_MIN_SAMPLE_INTERVAL_S
            and self._sink_rate_samples
        ):
            # Serial reads can drain several GPS frames in one GUI callback.
            # Treat those near-simultaneous arrivals as one sample so burst
            # delivery does not collapse the time window and inflate sink rate.
            sample_time = self._last_sink_rate_time
            self._sink_rate_samples[-1] = (sample_time, altitude_ft)
        else:
            self._sink_rate_samples.append((sample_time, altitude_ft))

        cutoff = sample_time - self.SINK_RATE_WINDOW_S
        while self._sink_rate_samples and self._sink_rate_samples[0][0] < cutoff:
            self._sink_rate_samples.popleft()

        self._last_sink_rate_time = sample_time
        if len(self._sink_rate_samples) < 2:
            self.current_sink_rate_fps = None
            return

        window_duration = self._sink_rate_samples[-1][0] - self._sink_rate_samples[0][0]
        if window_duration < self.SINK_RATE_MIN_WINDOW_S:
            self.current_sink_rate_fps = None
            return

        mean_time = sum(sample_time for sample_time, _ in self._sink_rate_samples) / len(
            self._sink_rate_samples
        )
        mean_altitude = sum(altitude for _, altitude in self._sink_rate_samples) / len(
            self._sink_rate_samples
        )
        time_variance = sum(
            (sample_time - mean_time) ** 2 for sample_time, _ in self._sink_rate_samples
        )
        if time_variance <= 0:
            self.current_sink_rate_fps = None
            return

        altitude_time_covariance = sum(
            (sample_time - mean_time) * (altitude - mean_altitude)
            for sample_time, altitude in self._sink_rate_samples
        )
        vertical_speed_fps = altitude_time_covariance / time_variance
        self.current_sink_rate_fps = max(0.0, -vertical_speed_fps)

    def _sink_rate_value_fresh(self, now: float) -> bool:
        """Return whether the sink-rate estimate is based on recent GPS data."""

        if self._last_sink_rate_time is None:
            return False

        if now - self._last_sink_rate_time <= self.SINK_RATE_STALE_TIMEOUT_S:
            return True

        self._reset_sink_rate_estimate()
        return False

    def _current_altitude_agl_ft(self) -> Optional[float]:
        altitude = self._safe_float(self.current_altitude)
        if altitude is None or self.airborne_baseline_altitude_ft is None:
            return None
        return altitude - self.airborne_baseline_altitude_ft

    def _reset_airborne_baseline(
        self, altitude_ft: float, *, ground_baseline: bool = True
    ) -> None:
        self.airborne_baseline_altitude_ft = altitude_ft
        self._airborne_has_ground_baseline = ground_baseline
        self._airborne_takeoff_start_time = None
        self._airborne_landing_start_time = None

    def _update_airborne_state(self, now: Optional[float] = None) -> None:
        """Debounce telemetry into either grounded or airborne state."""

        if now is None:
            now = time.monotonic()

        gps_timeout = self._airborne_config_value("gps_fresh_timeout_s", 2.0)
        if (
            not self._is_packet_fresh("gps", gps_timeout)
            or not self._airspeed_value_fresh(now, gps_timeout)
        ):
            self._airborne_takeoff_start_time = None
            self._airborne_landing_start_time = None
            return

        airspeed = self._safe_float(self.current_airspeed)
        altitude = self._safe_float(self.current_altitude)
        if airspeed is None or altitude is None:
            self._airborne_takeoff_start_time = None
            self._airborne_landing_start_time = None
            return

        landed_airspeed = self._airborne_config_value("landed_airspeed_mph", 7.0)
        if self.airborne_baseline_altitude_ft is None:
            self.airborne_baseline_altitude_ft = altitude
            self._airborne_has_ground_baseline = airspeed < landed_airspeed
        elif (
            self.airborne_state == "grounded"
            and not self._airborne_has_ground_baseline
            and airspeed < landed_airspeed
        ):
            # If telemetry came online during a fast/high-airspeed condition,
            # wait until the aircraft is slow before trusting the current
            # altitude as a real ground baseline.
            self._reset_airborne_baseline(altitude)

        altitude_agl = self._current_altitude_agl_ft()
        if altitude_agl is None:
            self._reset_airborne_baseline(
                altitude, ground_baseline=airspeed < landed_airspeed
            )
            altitude_agl = 0.0

        if self.airborne_state == "grounded":
            takeoff_airspeed = self._takeoff_airspeed_threshold_mph()
            takeoff_condition = airspeed > takeoff_airspeed and (
                altitude_agl > self._airborne_config_value("takeoff_altitude_ft", 15.0)
                or not self._airborne_has_ground_baseline
            )
            if takeoff_condition:
                if self._airborne_takeoff_start_time is None:
                    self._airborne_takeoff_start_time = now
                elif now - self._airborne_takeoff_start_time >= self._airborne_config_value(
                    "takeoff_hold_s", 2.0
                ):
                    self.airborne_state = "airborne"
                    self._airborne_takeoff_start_time = None
                    self._airborne_landing_start_time = None
                    self._update_airborne_indicator()
            else:
                self._airborne_takeoff_start_time = None
            return

        landed_condition = (
            airspeed < self._airborne_config_value("landed_airspeed_mph", 7.0)
            and altitude_agl < self._airborne_config_value("landed_altitude_ft", 5.0)
        )
        if landed_condition:
            if self._airborne_landing_start_time is None:
                self._airborne_landing_start_time = now
            elif now - self._airborne_landing_start_time >= self._airborne_config_value(
                "landing_hold_s", 5.0
            ):
                self.airborne_state = "grounded"
                self._reset_airborne_baseline(altitude)
                self._update_airborne_indicator()
        else:
            self._airborne_landing_start_time = None

    def _is_airborne(self) -> bool:
        return self.airborne_state == "airborne"

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
        self._refresh_yaw_target()

    def _handle_joystick_yaw_button(self, button: int, pressed: bool) -> None:
        """Update yaw target tracking based on joystick button input."""

        if pressed:
            self._joystick_yaw_buttons_pressed.add(button)
        else:
            self._joystick_yaw_buttons_pressed.discard(button)
        self._refresh_yaw_target()

    def _refresh_yaw_target(self) -> None:
        """Combine keyboard and joystick yaw inputs into one yaw target."""

        left_active = (
            Qt.Key_Q in self._yaw_keys_pressed
            or self.JOYSTICK_YAW_LEFT_BUTTON in self._joystick_yaw_buttons_pressed
        )
        right_active = (
            Qt.Key_E in self._yaw_keys_pressed
            or self.JOYSTICK_YAW_RIGHT_BUTTON in self._joystick_yaw_buttons_pressed
        )


        if left_active and right_active:
            target = 0.0
        elif left_active:
            target = -1.0
        elif right_active:
            target = 1.0
        else:
            target = 0.0

        self.yaw_target_value = target

    def _handle_joystick_button_events(self) -> None:
        """Apply joystick button edges to mode toggles and yaw controls."""

        joystick = getattr(self, "joystick", None)
        if joystick is None or not hasattr(joystick, "consume_button_events"):
            if self._joystick_yaw_buttons_pressed:
                self._joystick_yaw_buttons_pressed.clear()
                self._refresh_yaw_target()
            return

        for button, pressed in joystick.consume_button_events():
            if button == self.JOYSTICK_CONTROL_MODE_BUTTON and pressed:
                self.toggle_control_mode()
            elif button == self.JOYSTICK_THROTTLE_MODE_BUTTON and pressed:
                self.toggle_throttle_mode()
            elif button == self.JOYSTICK_YAW_LEFT_BUTTON:
                self._handle_joystick_yaw_button(button, pressed)
            elif button == self.JOYSTICK_YAW_RIGHT_BUTTON:
                self._handle_joystick_yaw_button(button, pressed)
            elif pressed and button == self.JOYSTICK_ELEVATOR_TRIM_DOWN_BUTTON:
                self._set_trim_level("elevator", -1, "elevatortrimup.mp3")
            elif pressed and button == self.JOYSTICK_ELEVATOR_TRIM_UP_BUTTON:
                self._set_trim_level("elevator", 1, "elevatortrimdown.mp3")
            elif pressed and button == self.JOYSTICK_AILERON_TRIM_LEFT_BUTTON:
                self._set_trim_level("aileron", 1, "ailerontrimright.mp3")
            elif pressed and button == self.JOYSTICK_AILERON_TRIM_RIGHT_BUTTON:
                self._set_trim_level("aileron", -1, "ailerontrimleft.mp3")

    def _setup_trim_indicator(self) -> None:
        """Create trim status labels beside the throttle mode indicator."""

        layout = getattr(self.ui, "controlInputsLayout", None)
        parent = getattr(self.ui, "controlSectionFrame", self)
        if layout is None:
            return

        self.trimModeLayout = QVBoxLayout()
        self.trimModeLayout.setSpacing(4)
        self.trimModeLayout.setObjectName("trimModeLayout")
        self.trimModeLayout.setContentsMargins(0, 8, 0, 0)

        self.trimModeTitle = QLabel(parent)
        self.trimModeTitle.setObjectName("trimModeTitle")
        title_font = self.trimModeTitle.font()
        title_font.setBold(True)
        title_font.setUnderline(True)
        self.trimModeTitle.setFont(title_font)
        self.trimModeTitle.setText("Trim")
        self.trimModeTitle.setAlignment(
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter
        )

        self.aileronTrimLabel = QLabel(parent)
        self.aileronTrimLabel.setObjectName("aileronTrimLabel")
        self.aileronTrimLabel.setAlignment(
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter
        )

        self.elevatorTrimLabel = QLabel(parent)
        self.elevatorTrimLabel.setObjectName("elevatorTrimLabel")
        self.elevatorTrimLabel.setAlignment(
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter
        )

        self.trimModeLayout.addWidget(self.trimModeTitle)
        self.trimModeLayout.addWidget(self.aileronTrimLabel)
        self.trimModeLayout.addWidget(self.elevatorTrimLabel)

        spacer_index = None
        for index in range(layout.count()):
            item = layout.itemAt(index)
            if item is not None and item.spacerItem() is not None:
                spacer_index = index
                break

        if spacer_index is None:
            layout.addLayout(self.trimModeLayout)
        else:
            layout.insertLayout(spacer_index, self.trimModeLayout)

    @staticmethod
    def _trim_label_text(
        axis: str, level: int, positive_label: str, negative_label: str
    ) -> str:
        """Format a signed trim level for the command-page trim indicator."""

        if level > 0:
            return f"{axis}: +{level} {positive_label}"
        if level < 0:
            return f"{axis}: {level} {negative_label}"
        return f"{axis}: 0 Neutral"

    def update_trim_labels(self) -> None:
        """Update the aileron and elevator trim level labels."""

        if hasattr(self, "aileronTrimLabel"):
            self.aileronTrimLabel.setText(
                self._trim_label_text("Ail", self.aileron_trim_level, "Left", "Right")
            )
            self.aileronTrimLabel.setStyleSheet(
                "color: rgb(0, 255, 0);"
                if self.aileron_trim_level == 0
                else "color: rgb(255, 165, 0);"
            )
        if hasattr(self, "elevatorTrimLabel"):
            self.elevatorTrimLabel.setText(
                self._trim_label_text("Elev", self.elevator_trim_level, "Up", "Down")
            )
            self.elevatorTrimLabel.setStyleSheet(
                "color: rgb(0, 255, 0);"
                if self.elevator_trim_level == 0
                else "color: rgb(255, 165, 0);"
            )

    def _set_trim_level(self, axis: str, delta: int, sound_name: str) -> None:
        """Apply one joystick trim step, refresh the UI, and play its cue."""

        if axis == "elevator":
            self.elevator_trim_level = max(
                -self.TRIM_MAX_LEVEL,
                min(self.TRIM_MAX_LEVEL, self.elevator_trim_level + delta),
            )
        elif axis == "aileron":
            self.aileron_trim_level = max(
                -self.TRIM_MAX_LEVEL,
                min(self.TRIM_MAX_LEVEL, self.aileron_trim_level + delta),
            )
        else:
            return

        self.update_trim_labels()
        self.play_sound_once(sound_name, volume=0.5)

    def _trim_level_to_channel_offset(self, level: int) -> int:
        """Convert a signed trim level into a CRSF channel offset."""

        channel_span = CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN
        return int(round(level * self.TRIM_STEP_NORMALIZED * channel_span * 0.5))

    def _apply_trim_to_channels(self, channels: list[int]) -> list[int]:
        """Offset roll/pitch channels by the current trim levels."""

        if len(channels) < 2:
            channels.extend([CRSF_CHANNEL_CENTER] * (2 - len(channels)))

        channels[0] = max(
            CRSF_CHANNEL_MIN,
            min(
                CRSF_CHANNEL_MAX,
                channels[0]
                + self._trim_level_to_channel_offset(self.aileron_trim_level),
            ),
        )
        channels[1] = max(
            CRSF_CHANNEL_MIN,
            min(
                CRSF_CHANNEL_MAX,
                channels[1]
                + self._trim_level_to_channel_offset(self.elevator_trim_level),
            ),
        )
        return channels

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


    def _normalize_sound_name(self, name: str) -> str:
        """Return the base name for a sound, ignoring its extension."""

        base, _ = os.path.splitext(name)
        return base if base else name

    def _is_sound_muted(self, name: str) -> bool:
        """Return ``True`` if the sound is currently muted."""

        normalized = self._normalize_sound_name(name)
        expiry = self._muted_sounds.get(normalized)
        if expiry is None:
            return False

        now = time.monotonic()
        if now >= expiry:
            self._muted_sounds.pop(normalized, None)
            return False

        return True

    def _mute_sounds(self, names, duration: float) -> None:
        """Mute ``names`` for ``duration`` seconds."""

        expiry = time.monotonic() + duration
        for name in names:
            normalized = self._normalize_sound_name(name)
            current = self._muted_sounds.get(normalized, 0.0)
            if expiry > current:
                self._muted_sounds[normalized] = expiry

    def play_sound(self, name: str):
        """Play a warning sound identified by ``name``.

        ``name`` may be provided without an extension (``elrsconnected``) or
        with a full filename (``elrsinitiated.mp3``). Files are loaded from the
        ``audio`` directory. Player instances are cached so repeated alerts
        reuse the same player.
        """

        if self._is_sound_muted(name):
            return

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

    def play_sound_once(self, name: str, volume: float = 1.0):
        """Play a sound on a throwaway player so repeated cues can overlap."""

        if self._is_sound_muted(name):
            return

        if os.path.splitext(name)[1]:
            file_path = os.path.join("audio", name)
        else:
            file_path = os.path.join("audio", f"{name}.mp3")

        if not os.path.exists(file_path):
            logging.warning("Sound file not found: %s", file_path)
            return

        player, output = QMediaPlayer(), QAudioOutput()
        player.setAudioOutput(output)
        player.setSource(QUrl.fromLocalFile(file_path))
        output.setVolume(max(0.0, min(1.0, float(volume))))

        players = getattr(self, "_overlapping_sound_players", None)
        if players is None:
            players = []
            self._overlapping_sound_players = players
        players.append((player, output))

        def cleanup():
            try:
                player.mediaStatusChanged.disconnect(handle_status)
            except Exception:
                pass
            try:
                players.remove((player, output))
            except ValueError:
                pass
            player.deleteLater()
            output.deleteLater()

        def handle_status(status):
            if status in (
                QMediaPlayer.MediaStatus.EndOfMedia,
                QMediaPlayer.MediaStatus.InvalidMedia,
            ):
                QTimer.singleShot(200, cleanup)

        player.mediaStatusChanged.connect(handle_status)
        player.play()

    def play_sound_sequence(self, names, finished_callback=None):
        """Play a sequence of warning sounds in order.

        ``finished_callback`` is called when the sequence has completed.
        """
        if not names:
            if finished_callback:
                finished_callback()
            return

        names = [name for name in list(names) if not self._is_sound_muted(name)]

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

    def check_warnings(self, now: Optional[float] = None):
        """Evaluate telemetry values against configured thresholds and play alarms."""
        if now is None:
            now = time.monotonic()

        if not self.warning_cfg.get("warning_alarms_enabled", True):
            self._clear_warning_alarm_state()
            return

        airspeed = self._safe_float(self.current_airspeed)
        altitude = self._safe_float(self.current_altitude)
        roll = self._safe_float(self.telemetry_roll)
        sink_rate = (
            self._safe_float(self.current_sink_rate_fps)
            if self._sink_rate_value_fresh(now)
            else None
        )
        if airspeed is None or altitude is None:
            return

        airborne_warnings_armed = (
            self._is_airborne() and self._airborne_landing_start_time is None
        )

        # Airspeed warning: low airspeed at high altitude while airborne and not
        # already satisfying the landing debounce.
        stall_enabled = self.warning_cfg.get("stall_alarm_enabled", True)
        if (
            airborne_warnings_armed
            and stall_enabled
            and airspeed < self.warning_cfg.get("stall_airspeed", 0)
            and altitude > self.warning_cfg.get("stall_altitude", 0)
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

        # Altitude warning: low altitude at high airspeed while airborne and not
        # already satisfying the landing debounce.
        altitude_enabled = self.warning_cfg.get("altitude_alarm_enabled", True)
        if (
            airborne_warnings_armed
            and altitude_enabled
            and altitude < self.warning_cfg.get("altitude_alarm_altitude", 0)
            and airspeed > self.warning_cfg.get("altitude_alarm_airspeed", 0)
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
            roll is not None
            and bank_enabled
            and abs(roll) > self.warning_cfg.get("roll_angle", 0)
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

        # Sink-rate warning: excessive descent rate while airborne and not already
        # satisfying the landing debounce.
        sink_rate_enabled = self.warning_cfg.get("sink_rate_alarm_enabled", True)
        sink_rate_threshold = self._safe_float(
            self.warning_cfg.get("sink_rate_threshold_fps", 10.0), 10.0
        )
        if (
            airborne_warnings_armed
            and sink_rate_enabled
            and sink_rate is not None
            and sink_rate > sink_rate_threshold
        ):
            if self.sink_rate_alarm_start_time is None:
                self.sink_rate_alarm_start_time = now
            elif (
                now - self.sink_rate_alarm_start_time > 1.0
                and not self.sink_rate_alarm_playing
            ):
                self.sink_rate_alarm_playing = True
                self.play_sound_sequence(
                    ["sinkalarm", "sinkratewarning"],
                    finished_callback=lambda: setattr(
                        self, "sink_rate_alarm_playing", False
                    ),
                )
        else:
            self.sink_rate_alarm_start_time = None
            self.sink_rate_alarm_playing = False


    def _set_crsf_raw_serial_debug(self, enabled: bool) -> None:
        """Ask the CRSF worker to emit raw serial chunks only when needed."""
        processor = getattr(self, "crsf_processor", None)
        if processor is not None:
            try:
                processor.raw_serial_debug_update.emit(bool(enabled))
            except Exception:
                logging.debug("Failed to update CRSF raw serial debug state", exc_info=True)

    def _log_control_update_frequency(self) -> None:
        """Print the GUI-to-worker channel update rate while debug monitoring."""

        if not self._debug_monitoring or "control" not in self._debug_packets:
            return

        now = time.monotonic()
        if self._last_control_update_debug_timestamp is None:
            self._last_control_update_debug_timestamp = now
            return

        interval = now - self._last_control_update_debug_timestamp
        self._last_control_update_debug_timestamp = now
        if interval > 0:
            print(f"GS control channel update frequency: {1.0 / interval:.2f} Hz", flush=True)

    @Slot(object)
    def _handle_transmit_debug(self, stats) -> None:
        """Forward CRSF worker transmit-rate diagnostics to the Debug tab."""

        if not self._debug_monitoring:
            return
        if "control_tx" not in self._debug_packets and "control" not in self._debug_packets:
            return

        self.debug_page.log_packet("control_tx", stats)

    @Slot(str)
    def _handle_parameter_query_update(self, message: str) -> None:
        """Append CRSF parameter-query diagnostics to the Debug tab."""

        self.debug_page.append_message(message)

    @Slot(object)
    def _handle_link_diagnostics_update(self, stats) -> None:
        """Append CRSF/ELRS link diagnostics to the Debug tab."""

        try:
            if isinstance(stats, dict) and stats.get("event") == "state":
                self._link_diagnostics_active = bool(stats.get("enabled"))
                self._update_link_diagnostics_button_state()
            self.debug_page.log_link_diagnostics(stats)
        except Exception:
            logging.debug("Failed to handle link diagnostics update", exc_info=True)

    def _update_link_diagnostics_button_state(self) -> None:
        """Enable the link diagnostics button when a CRSF processor is connected."""

        debug_page = getattr(self, "debug_page", None)
        if debug_page is None:
            return
        if not self._is_crsf_debug_available():
            self._link_diagnostics_active = False
            debug_page.set_link_diagnostics_enabled(
                False,
                False,
                "Connect the ELRS/CRSF transmitter before starting link diagnostics.",
            )
        else:
            debug_page.set_link_diagnostics_enabled(
                True,
                self._link_diagnostics_active,
                "Stream RX/TX CRSF link diagnostics to the debug terminal.",
            )

    def toggle_link_diagnostics_from_debug_page(self) -> None:
        """Start or stop CRSF/ELRS link diagnostics from the Debug tab."""

        if not self._is_crsf_debug_available():
            self.debug_page.append_message(
                "Link diagnostics failed: CRSF transmitter is not connected."
            )
            self._update_link_diagnostics_button_state()
            return

        next_state = not self._link_diagnostics_active
        self._link_diagnostics_active = next_state
        self._update_link_diagnostics_button_state()
        self.crsf_processor.diagnostic_enabled_update.emit(next_state)

    def _update_parameter_query_button_state(self) -> None:
        """Enable ELRS parameter querying only while RC packet transmission is stopped."""

        debug_page = getattr(self, "debug_page", None)
        if debug_page is None:
            return
        if self.transmission_active:
            debug_page.set_parameter_query_enabled(
                False,
                "Stop packet transmission before querying ELRS parameters.",
            )
        elif not self._is_crsf_debug_available():
            debug_page.set_parameter_query_enabled(
                False,
                "Connect the ELRS/CRSF transmitter before querying parameters.",
            )
        else:
            debug_page.set_parameter_query_enabled(
                True,
                "Query the ELRS TX module CRSF parameter table.",
            )

    def query_elrs_parameters_from_debug_page(self) -> None:
        """Request ELRS TX module parameter information from the Debug tab."""

        if self.transmission_active:
            self.debug_page.append_message(
                "ELRS parameter query blocked: stop packet transmission first."
            )
            self._update_parameter_query_button_state()
            return
        if not self._is_crsf_debug_available():
            self.debug_page.append_message(
                "ELRS parameter query failed: CRSF transmitter is not connected."
            )
            self._update_parameter_query_button_state()
            return

        self.debug_page.append_message("Starting ELRS TX module CRSF parameter query...")
        self.crsf_processor.parameter_query_request.emit()

    @Slot(object)
    def _handle_serial_debug(self, payload) -> None:
        """Log raw serial bytes to the Debug tab when requested."""

        if not self._debug_monitoring or not self._debug_serial_all:
            return

        try:
            data = bytes(payload)
        except Exception:
            try:
                data = bytes(payload.data())
            except Exception:
                return

        if not data:
            return

        self.debug_page.log_serial_data(data)

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
        self._last_telemetry_packet_times[packet_type] = self.last_telemetry_time
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

            self._log_attitude_frequency(now)
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
            gps_has_fix = (
                lat_value is not None
                and lon_value is not None
                and math.isfinite(lat_value)
                and math.isfinite(lon_value)
                and not (lat_value == 0.0 and lon_value == 0.0)
            )
            self.gps_lat = lat_value if lat_value is not None else lat
            self.gps_lon = lon_value if lon_value is not None else lon
            self.current_altitude = alt
            self.current_airspeed = speed
            # Barometric altitude is carried in the GPS telemetry frame even when
            # latitude/longitude report no GPS fix, so keep sink-rate and alarm
            # logic tied to altitude freshness rather than GPS lock state.
            self._update_sink_rate_from_altitude(self._safe_float(alt), now)
            try:
                speed_value = float(speed)
            except (TypeError, ValueError):
                self.last_airspeed_packet_time = None
            else:
                self.last_airspeed_packet_time = (
                    now if math.isfinite(speed_value) else None
                )
            self.telemetry_state["latitude"] = lat_value if lat_value is not None else lat
            self.telemetry_state["longitude"] = lon_value if lon_value is not None else lon
            self.telemetry_state["altitude_ft"] = alt
            self.telemetry_state["airspeed_mph"] = speed
            self.telemetry_state["ground_course"] = course
            self.telemetry_state["satellites"] = sats
            self.data_page.add_flight_metrics(alt, speed)
            self._update_airborne_state(now)
            if gps_has_fix:
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

        if self._debug_monitoring and (
            self._debug_telemetry_all or packet_type in self._debug_packets
        ):
            self.debug_page.log_packet(packet_type, values)

        self._update_packet_rates(packet_type, now)
        self._update_rate_labels()
        self._record_telemetry_sample(packet_type)

    def _log_attitude_frequency(self, timestamp: float) -> None:
        """Print the instantaneous attitude packet frequency to the terminal."""

        if self._last_attitude_packet_timestamp is None:
            self._last_attitude_packet_timestamp = timestamp
            return

        interval = timestamp - self._last_attitude_packet_timestamp
        self._last_attitude_packet_timestamp = timestamp

        if interval <= 0:
            return

        frequency = 1.0 / interval
        print(f"Attitude telemetry frequency: {frequency:.2f} Hz", flush=True)

    @staticmethod
    def _validated_fbw_limit(
        value: int | float, fc_limit: float, default_limit: float
    ) -> float:
        """Clamp a GS FBW attitude limit to the FC's redundant safety envelope."""

        try:
            limit = float(value)
        except (TypeError, ValueError):
            limit = float(default_limit)
        if not math.isfinite(limit) or limit < 0.0:
            limit = float(default_limit)
        return max(0.0, min(float(fc_limit), limit))

    def _fbw_limited_channel(
        self, raw_channel: int | float, gs_limit_deg: float, fc_limit_deg: float
    ) -> int:
        """Scale a raw stick channel so the FC sees the GS FBW limit as full stick."""

        input_norm = self._normalise_crsf_channel_for_fbw(raw_channel)
        gs_limit_deg = self._validated_fbw_limit(
            gs_limit_deg, fc_limit_deg, fc_limit_deg
        )
        if fc_limit_deg <= 0.0:
            return CRSF_CHANNEL_CENTER
        fc_norm = input_norm * (gs_limit_deg / fc_limit_deg)
        return self._map_axis_to_crsf(fc_norm)

    def _apply_fbw_command_limits(self, channels: list[int]) -> list[int]:
        """Apply GS-configured FBW roll/pitch limits to outgoing RC channels."""

        if len(channels) < 2:
            channels.extend([CRSF_CHANNEL_CENTER] * (2 - len(channels)))
        channels[0] = self._fbw_limited_channel(
            channels[0],
            self.fbw_max_roll_angle_deg,
            self.FBW_FC_MAX_ROLL_ANGLE_DEG,
        )
        channels[1] = self._fbw_limited_channel(
            channels[1],
            self.fbw_max_pitch_angle_deg,
            self.FBW_FC_MAX_PITCH_ANGLE_DEG,
        )
        return channels

    @staticmethod
    def _normalise_crsf_channel_for_fbw(value: int | float) -> float:
        """Mirror the FC's ``mapRcToNormalized`` helper for FBW setpoints."""

        in_min = float(CRSF_CHANNEL_MIN)
        in_max = float(CRSF_CHANNEL_MAX)
        clamped = max(in_min, min(in_max, float(value)))
        half_range = (in_max - in_min) * 0.5
        if half_range <= 0.0:
            return 0.0
        center = in_min + half_range
        normalized = (clamped - center) / half_range
        return max(-1.0, min(1.0, normalized))

    def _desired_fbw_attitude_from_channels(
        self, channels: list[int]
    ) -> tuple[float, float]:
        """Return desired roll/pitch degrees from the scaled command sent to the FC."""

        roll_raw = channels[0] if len(channels) > 0 else CRSF_CHANNEL_CENTER
        pitch_raw = channels[1] if len(channels) > 1 else CRSF_CHANNEL_CENTER
        roll_norm = self._normalise_crsf_channel_for_fbw(roll_raw)
        pitch_norm = self._normalise_crsf_channel_for_fbw(pitch_raw)
        desired_roll = roll_norm * self.FBW_FC_MAX_ROLL_ANGLE_DEG
        desired_pitch = pitch_norm * self.FBW_FC_MAX_PITCH_ANGLE_DEG
        return desired_roll, desired_pitch

    def _update_desired_fbw_attitude(
        self, channels: list[int], enabled: Optional[bool] = None
    ) -> None:
        """Cache and publish the desired FBW attitude cue for the OSD."""

        self._latest_control_channels = list(channels[:16])
        show_desired = (
            self.control_mode == "Fly-By-Wire" if enabled is None else enabled
        )
        if show_desired:
            self.desired_fbw_roll, self.desired_fbw_pitch = (
                self._desired_fbw_attitude_from_channels(channels)
            )
        else:
            self.desired_fbw_roll = None
            self.desired_fbw_pitch = None

        if hasattr(self, "rollpitch_osd"):
            self.rollpitch_osd.setDesiredRollPitch(
                self.desired_fbw_roll,
                self.desired_fbw_pitch,
                visible=show_desired,
            )

    def _update_desired_fbw_attitude_from_stick(
        self, joy_pitch: Optional[float], joy_roll: Optional[float]
    ) -> None:
        """Refresh the OSD cue from the same joystick-to-CRSF mapping as TX."""

        if self.control_mode != "Fly-By-Wire":
            self._update_desired_fbw_attitude(
                getattr(self, "_latest_control_channels", [CRSF_CHANNEL_CENTER] * 16),
                enabled=False,
            )
            return

        channels = list(
            getattr(self, "_latest_control_channels", [CRSF_CHANNEL_CENTER] * 16)
        )
        if len(channels) < 16:
            channels.extend([CRSF_CHANNEL_CENTER] * (16 - len(channels)))
        if joy_roll is not None and joy_pitch is not None:
            channels[0] = JoystickRawHandler._map_to_crsf(joy_roll)
            channels[1] = JoystickRawHandler._map_to_crsf(joy_pitch)
            self._apply_trim_to_channels(channels)
            self._apply_fbw_command_limits(channels)
        self._update_desired_fbw_attitude(channels, enabled=True)


    def _create_joystick_handler(self, port: str | None):
        """Create a joystick handler and wire all runtime error reporting."""

        if not validate_port("joystick", port):
            print("Joystick disabled due to unavailable port.")
            return None

        try:
            joystick = JoystickRawHandler(
                port=port,
                baudrate=self.joystick_cfg.get("baudrate"),
                deadzone=self.joystick_cfg.get("deadzone", 0),
                sensitivity=self.joystick_cfg.get("sensitivity", 100),
                smoothing=self.joystick_cfg.get("smoothing", 0),
            )
            joystick.error.connect(self.handle_worker_error)
            return joystick
        except Exception as e:
            print(f"Failed to initialize joystick: {e}")
            return None

    def _create_crsf_processor(
        self, port: str | None, *, transmission_enabled: bool
    ) -> Optional[CRSFPacketProcessor]:
        """Create a CRSF worker with safe initial channels and all signals wired."""

        if not self._is_serial_port_configured(port):
            print("CRSF port: Not connected")
            return None

        available_ports = set(self._available_serial_port_devices())
        if port not in available_ports:
            print(
                f"Warning: CRSF port '{port}' not found. "
                "Creating retryable worker for hot-plug reconnect."
            )

        initial_channels = (
            self._build_control_channels()
            if transmission_enabled
            else self._build_safe_control_channels()
        )
        try:
            processor = CRSFPacketProcessor(
                port=port,
                baudrate=self.crsf_cfg.get("baudrate"),
                channels=initial_channels,
                packet_interval_ms=self.crsf_cfg.get("packet_interval", 4),
                transmission_enabled=transmission_enabled,
                raw_serial_debug_enabled=self._debug_monitoring and self._debug_serial_all,
            )
            processor.telemetry_ready.connect(self.handle_telemetry_wrapper)
            processor.serial_data.connect(self._handle_serial_debug)
            processor.transmit_debug_update.connect(self._handle_transmit_debug)
            processor.parameter_query_update.connect(
                self._handle_parameter_query_update
            )
            processor.link_diagnostics_update.connect(
                self._handle_link_diagnostics_update
            )
            processor.safe_shutdown_complete.connect(
                self._handle_safe_shutdown_complete
            )
            processor.error.connect(self.handle_worker_error)
            return processor
        except Exception as e:
            print(f"Failed to initialize CRSF processor: {e}")
            return None

    def _build_safe_control_channels(self) -> list[int]:
        """Return a neutral, throttle-cut, disarmed channel frame."""

        channels = [CRSF_CHANNEL_CENTER] * 16
        channels[0] = CRSF_CHANNEL_CENTER
        channels[1] = CRSF_CHANNEL_CENTER
        channels[2] = CRSF_CHANNEL_MIN
        channels[3] = CRSF_CHANNEL_CENTER
        channels[self.elrs_arm_channel] = CRSF_CHANNEL_MIN
        channels[self.control_mode_channel] = 400
        channels[self.throttle_mode_channel] = 400
        return channels

    def _handle_safe_shutdown_complete(self, success: bool) -> None:
        if not success:
            logging.warning("Safe CRSF shutdown frames were not written successfully")

    @staticmethod
    def _map_axis_to_crsf(value: float) -> int:
        """Map a normalized control value (``-1`` to ``1``) to CRSF range."""

        value = max(-1.0, min(1.0, float(value)))
        return int(
            round(
                (value + 1.0)
                * 0.5
                * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)
                + CRSF_CHANNEL_MIN
            )
        )

    def _build_control_channels(self):
        """Build the current CRSF channel set using the UI's safe defaults."""

        channels = [CRSF_CHANNEL_CENTER] * 16
        joystick = getattr(self, "joystick", None)
        if joystick:
            try:
                mapped_roll, mapped_pitch = joystick.get_mapped_values()
                channels[0] = int(mapped_roll)
                channels[1] = int(mapped_pitch)
            except Exception as e:
                print(f"Error during transmission: {e}")

        self._apply_trim_to_channels(channels)

        # CH3 carries manual throttle percent in Manual mode. In Auto Throttle
        # mode it carries the configured target airspeed from the configuration
        # page; the FC applies the same converter in reverse before running its
        # local throttle PID.
        throttle_min = CRSF_CHANNEL_MIN
        throttle_max = CRSF_CHANNEL_MAX
        throttle_span = throttle_max - throttle_min
        if self.throttle_mode == "Auto Throttle":
            channel_fraction = self._clamp_auto_throttle_speed(
                self.throttle_target_airspeed_mph
            ) / self.auto_throttle_speed_channel_max_mph
        else:
            channel_fraction = max(
                0.0, min(100.0, float(getattr(self, "throttle_percent", 0)))
            ) / 100.0
        channels[2] = int(channel_fraction * throttle_span + throttle_min)

        # Map yaw input to channel 4 (index 3).
        channels[3] = self._map_axis_to_crsf(getattr(self, "yaw_value", 0.0))

        # Only active operator-started transmission drives ELRS AUX1/CH5 high.
        # Idle startup and shutdown frames use _build_safe_control_channels(),
        # which keeps this channel low/disarmed.
        channels[self.elrs_arm_channel] = CRSF_CHANNEL_MAX

        # Control mode channel: send low for Manual, high for Fly-By-Wire.
        mode_value = 1700 if self.control_mode == "Fly-By-Wire" else 400
        channels[self.control_mode_channel] = mode_value

        # Throttle mode channel: AUX3/CH7 avoids CH5/AUX1 arming and CH6/AUX2
        # flight-control mode. Low is manual throttle, high is FC auto throttle.
        throttle_mode_value = 1700 if self.throttle_mode == "Auto Throttle" else 400
        channels[self.throttle_mode_channel] = throttle_mode_value

        if self.control_mode == "Fly-By-Wire":
            self._apply_fbw_command_limits(channels)
        self._update_desired_fbw_attitude(channels)
        return channels

    def transmit_data(self):
        """
        Transmit CRSF packets using mapped joystick values.
        """
        if not self.crsf_processor:
            return

        channels = self._build_control_channels()
        try:
            self.crsf_processor.channel_update.emit(channels)
            self._log_control_update_frequency()
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
        self._update_desired_fbw_attitude(
            getattr(self, "_latest_control_channels", [CRSF_CHANNEL_CENTER] * 16)
        )
        sound_name = "fbw" if self.control_mode == "Fly-By-Wire" else "manual"
        self.play_sound(sound_name)

    def _setup_throttle_mode_indicator(self) -> None:
        """Make the throttle mode indicator act as the mode toggle."""
        for attr in ("throttleModeTitle", "throttleModeLabel"):
            widget = getattr(self.ui, attr, None)
            if widget is not None:
                widget.setCursor(Qt.CursorShape.PointingHandCursor)
                widget.setToolTip("Click or press Ctrl+B to toggle throttle mode")
                widget.mousePressEvent = self._handle_throttle_mode_indicator_click

    def _handle_throttle_mode_indicator_click(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self.toggle_throttle_mode()
            event.accept()
            return
        event.ignore()

    def update_throttle_mode_label(self):
        """Update the throttle mode indicator text and color."""
        if hasattr(self.ui, "throttleModeLabel"):
            is_manual = self.throttle_mode == "Manual"
            color = "rgb(0, 255, 0)" if is_manual else "rgb(255, 165, 0)"
            label = (
                self.throttle_mode
                if is_manual
                else f"Auto {self.throttle_target_airspeed_mph:.0f} mph"
            )
            self.ui.throttleModeLabel.setText(label)
            self.ui.throttleModeLabel.setStyleSheet(f"color: {color};")

    def toggle_throttle_mode(self):
        """Toggle between Manual and Auto Throttle modes."""
        self.throttle_mode = (
            "Auto Throttle" if self.throttle_mode == "Manual" else "Manual"
        )
        self.update_throttle_mode_label()
        self._update_throttle_indicator()
        sound_name = (
            "autothrottle"
            if self.throttle_mode == "Auto Throttle"
            else "manualthrottle"
        )
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
        if self.transmission_active:
            self._apply_transmission_button_style("active")
            self.transmission_control_button.setText("Terminate transmission")
        else:
            self._apply_transmission_button_style("inactive")
            self.transmission_control_button.setText("Start transmitting packets")
        layout.addWidget(self.transmission_control_button)
        layout.addSpacing(12)

        self.transmission_control_button.pressed.connect(
            self._on_transmission_button_pressed
        )
        self.transmission_control_button.released.connect(
            self._on_transmission_button_released
        )

        self.preflight_verification_button = QPushButton("Pre flight verification")
        self.preflight_verification_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.preflight_verification_button.setMinimumHeight(44)
        self.preflight_verification_button.setSizePolicy(button_policy)
        self.preflight_verification_button.setStyleSheet(
            "QPushButton {"
            "background-color: rgb(30, 136, 229);"
            "color: white;"
            "font-weight: bold;"
            "border-radius: 8px;"
            "padding: 8px 16px;"
            "}"
            "QPushButton:hover {background-color: rgb(66, 165, 245);}"
            "QPushButton:pressed {background-color: rgb(21, 101, 192);}"
            "QPushButton:disabled {background-color: rgb(80, 80, 80); color: rgb(180, 180, 180);}"
        )
        layout.addWidget(self.preflight_verification_button)

        self.preflight_status_label = QLabel(
            "Pre-flight verification has not been run yet."
        )
        self.preflight_status_label.setWordWrap(True)
        self.preflight_status_label.setStyleSheet(
            "color: rgb(220, 220, 220); padding: 4px 2px;"
        )
        layout.addWidget(self.preflight_status_label)
        layout.addSpacing(12)

        self.preflight_verification_button.clicked.connect(
            self._show_preflight_verification
        )

        available_ports = [p.device for p in list_ports.comports()]

        def port_choices(*configured_ports: str | None) -> list[str]:
            choices = ["Not connected"] + available_ports
            for configured_port in configured_ports:
                if (
                    self._is_serial_port_configured(configured_port)
                    and configured_port not in choices
                ):
                    choices.append(configured_port)
            return choices

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
                "Select the attitude/RC packet rate to match the ELRS link configuration."
            )
        )
        self.pico_rate_label = QLabel()
        rf_layout.addWidget(self.pico_rate_label)
        self.update_pico_rate_label()
        rf_port_row = QHBoxLayout()
        rf_port_row.addWidget(QLabel("Port"))
        self.elrs_port_combo = QComboBox()
        self.elrs_port_combo.addItems(port_choices(self.crsf_cfg.get("port")))
        rf_port_row.addWidget(self.elrs_port_combo)
        rf_layout.addLayout(rf_port_row)

        rate_row = QHBoxLayout()
        rate_row.addWidget(QLabel("Attitude/RC Packet Rate"))
        self.packet_rate_combo = QComboBox()
        for rate_hz in ALLOWED_ATTITUDE_PACKET_RATES_HZ:
            self.packet_rate_combo.addItem(f"{rate_hz} Hz", rate_hz)
        configured_rate = packet_rate_hz_from_interval(
            self.crsf_cfg.get("packet_interval", 4)
        )
        self.packet_rate_combo.setCurrentText(f"{configured_rate} Hz")
        rate_row.addWidget(self.packet_rate_combo)
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
        self.control_port_combo.addItems(port_choices(self.joystick_cfg.get("port")))
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

        fbw_roll_row = QHBoxLayout()
        fbw_roll_row.addWidget(QLabel("FBW max roll (deg)"))
        self.fbw_roll_limit_spin = QDoubleSpinBox()
        self.fbw_roll_limit_spin.setRange(0.0, self.FBW_FC_MAX_ROLL_ANGLE_DEG)
        self.fbw_roll_limit_spin.setDecimals(1)
        self.fbw_roll_limit_spin.setSingleStep(1.0)
        self.fbw_roll_limit_spin.setValue(self.fbw_max_roll_angle_deg)
        fbw_roll_row.addWidget(self.fbw_roll_limit_spin)
        control_layout.addLayout(fbw_roll_row)

        fbw_pitch_row = QHBoxLayout()
        fbw_pitch_row.addWidget(QLabel("FBW max pitch (deg)"))
        self.fbw_pitch_limit_spin = QDoubleSpinBox()
        self.fbw_pitch_limit_spin.setRange(0.0, self.FBW_FC_MAX_PITCH_ANGLE_DEG)
        self.fbw_pitch_limit_spin.setDecimals(1)
        self.fbw_pitch_limit_spin.setSingleStep(1.0)
        self.fbw_pitch_limit_spin.setValue(self.fbw_max_pitch_angle_deg)
        fbw_pitch_row.addWidget(self.fbw_pitch_limit_spin)
        control_layout.addLayout(fbw_pitch_row)

        auto_throttle_row = QHBoxLayout()
        auto_throttle_row.addWidget(QLabel("Auto throttle target airspeed (mph)"))
        self.auto_throttle_target_spin = QDoubleSpinBox()
        self.auto_throttle_target_spin.setRange(0.0, self.auto_throttle_speed_channel_max_mph)
        self.auto_throttle_target_spin.setDecimals(1)
        self.auto_throttle_target_spin.setSingleStep(1.0)
        self.auto_throttle_target_spin.setValue(self.throttle_target_airspeed_mph)
        auto_throttle_row.addWidget(self.auto_throttle_target_spin)
        control_layout.addLayout(auto_throttle_row)
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
        airborne_note = QLabel(
            "Stall and low-altitude/high-speed alarms sound only after the "
            "airborne detector confirms flight."
        )
        airborne_note.setWordWrap(True)
        airborne_note.setStyleSheet("color: rgb(154, 164, 178); font-size: 11px;")
        warn_layout.addWidget(airborne_note)

        airborne_takeoff_row = QHBoxLayout()
        airborne_takeoff_row.addWidget(QLabel("Airborne when AGL >"))
        self.airborne_takeoff_alt_spin = QDoubleSpinBox()
        self.airborne_takeoff_alt_spin.setRange(0.0, 200.0)
        self.airborne_takeoff_alt_spin.setDecimals(1)
        self.airborne_takeoff_alt_spin.setSingleStep(1.0)
        self.airborne_takeoff_alt_spin.setValue(
            self._airborne_config_value("takeoff_altitude_ft", 15.0)
        )
        self.airborne_takeoff_alt_spin.setSuffix(" ft")
        airborne_takeoff_row.addWidget(self.airborne_takeoff_alt_spin)
        warn_layout.addLayout(airborne_takeoff_row)

        airborne_landed_row = QHBoxLayout()
        airborne_landed_row.addWidget(QLabel("Grounded below"))
        self.airborne_landed_speed_spin = QDoubleSpinBox()
        self.airborne_landed_speed_spin.setRange(0.0, 50.0)
        self.airborne_landed_speed_spin.setDecimals(1)
        self.airborne_landed_speed_spin.setSingleStep(0.5)
        self.airborne_landed_speed_spin.setValue(
            self._airborne_config_value("landed_airspeed_mph", 7.0)
        )
        self.airborne_landed_speed_spin.setSuffix(" mph")
        airborne_landed_row.addWidget(self.airborne_landed_speed_spin)
        self.airborne_landed_alt_spin = QDoubleSpinBox()
        self.airborne_landed_alt_spin.setRange(0.0, 50.0)
        self.airborne_landed_alt_spin.setDecimals(1)
        self.airborne_landed_alt_spin.setSingleStep(0.5)
        self.airborne_landed_alt_spin.setValue(
            self._airborne_config_value("landed_altitude_ft", 5.0)
        )
        self.airborne_landed_alt_spin.setSuffix(" ft AGL")
        airborne_landed_row.addWidget(self.airborne_landed_alt_spin)
        warn_layout.addLayout(airborne_landed_row)

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

        warn_layout.addWidget(QLabel("Sink Rate Alarm"))
        sink_rate_row = QHBoxLayout()
        sink_rate_row.addWidget(QLabel("Descent rate >"))
        self.sink_rate_threshold_spin = QDoubleSpinBox()
        self.sink_rate_threshold_spin.setRange(0.0, 200.0)
        self.sink_rate_threshold_spin.setDecimals(1)
        self.sink_rate_threshold_spin.setSingleStep(0.5)
        self.sink_rate_threshold_spin.setValue(
            self._safe_float(
                self.warning_cfg.get("sink_rate_threshold_fps", 10.0), 10.0
            )
        )
        self.sink_rate_threshold_spin.setSuffix(" ft/s")
        sink_rate_row.addWidget(self.sink_rate_threshold_spin)
        warn_layout.addLayout(sink_rate_row)

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
        self.packet_rate_combo.currentIndexChanged.connect(self.on_packet_rate_changed)
        self.deadzone_slider.valueChanged.connect(self.on_deadzone_changed)
        self.sensitivity_slider.valueChanged.connect(self.on_sensitivity_changed)
        self.yaw_sensitivity_slider.valueChanged.connect(
            self.on_yaw_sensitivity_changed
        )
        self.smoothing_slider.valueChanged.connect(self.on_smoothing_changed)
        self.fbw_roll_limit_spin.valueChanged.connect(self.on_fbw_roll_limit_changed)
        self.fbw_pitch_limit_spin.valueChanged.connect(self.on_fbw_pitch_limit_changed)
        self.auto_throttle_target_spin.valueChanged.connect(
            self.on_auto_throttle_target_changed
        )
        self.airborne_takeoff_alt_spin.valueChanged.connect(
            self.on_airborne_takeoff_alt_changed
        )
        self.airborne_landed_speed_spin.valueChanged.connect(
            self.on_airborne_landed_speed_changed
        )
        self.airborne_landed_alt_spin.valueChanged.connect(
            self.on_airborne_landed_alt_changed
        )
        self.stall_speed_slider.valueChanged.connect(self.on_stall_speed_changed)
        self.stall_alt_slider.valueChanged.connect(self.on_stall_alt_changed)
        self.alt_alarm_alt_slider.valueChanged.connect(self.on_alt_alarm_alt_changed)
        self.alt_alarm_speed_slider.valueChanged.connect(self.on_alt_alarm_speed_changed)
        self.roll_angle_slider.valueChanged.connect(self.on_roll_angle_changed)
        self.sink_rate_threshold_spin.valueChanged.connect(
            self.on_sink_rate_threshold_changed
        )
        self.battery_type_combo.currentTextChanged.connect(
            self.on_battery_type_changed
        )

        # Initial connection status
        self.update_connection_status(self.control_status, self.joystick is not None)
        self.update_connection_status(self.rf_status, self._is_crsf_port_available())
        # Video connection status is derived from the video feed itself
        layout.addStretch()

        self.update_connection_status(self.vtx_status, False)
        # Ensure the port lists reflect currently connected devices
        self.update_port_lists()

        self.reinitialize_ports_button.clicked.connect(self.reinitialize_serial_ports)

    def _packet_rate(self, key: str) -> float:
        """Return the current one-second packet rate for a telemetry stream."""

        queue = self._packet_times.get(key)
        if not queue:
            return 0.0
        now = time.monotonic()
        self._prune_packet_times(queue, now)
        return len(queue) / self._rate_window_seconds

    def _telemetry_packet_age(self, packet_type: str) -> Optional[float]:
        """Return seconds since the last packet of ``packet_type`` was received."""

        timestamp = self._last_telemetry_packet_times.get(packet_type)
        if timestamp is None:
            return None
        return time.monotonic() - timestamp

    def _is_packet_fresh(self, packet_type: str, timeout: float = 2.0) -> bool:
        """Return whether the named telemetry packet arrived recently."""

        age = self._telemetry_packet_age(packet_type)
        return age is not None and age <= timeout

    def _format_preflight_age(self, packet_type: str) -> str:
        """Format packet age for display in the pre-flight report."""

        age = self._telemetry_packet_age(packet_type)
        if age is None:
            return "never received"
        if age < 1.0:
            return "received just now"
        return f"received {age:.1f}s ago"

    def _add_preflight_check(
        self,
        rows: list[tuple[str, str]],
        severity: str,
        message: str,
    ) -> None:
        """Append one formatted pre-flight check result."""

        rows.append((severity, message))

    def _run_preflight_verification(self) -> tuple[str, list[tuple[str, str]]]:
        """Evaluate telemetry, signal, port, and local safety readiness."""

        rows: list[tuple[str, str]] = []

        available_ports = {p.device for p in list_ports.comports()}
        crsf_port = self.crsf_cfg.get("port")
        joystick_port = self.joystick_cfg.get("port")

        crsf_ready = (
            self.crsf_processor is not None
            and bool(crsf_port)
            and str(crsf_port).lower() != "not connected"
            and crsf_port in available_ports
        )
        self._add_preflight_check(
            rows,
            "pass" if crsf_ready else "fail",
            (
                f"ELRS/CRSF radio port {crsf_port} is connected."
                if crsf_ready
                else "ELRS/CRSF radio port is not connected or is unavailable."
            ),
        )

        joystick_ready = (
            self.joystick is not None
            and bool(joystick_port)
            and str(joystick_port).lower() != "not connected"
            and joystick_port in available_ports
        )
        self._add_preflight_check(
            rows,
            "pass" if joystick_ready else "warn",
            (
                f"Control system port {joystick_port} is connected."
                if joystick_ready
                else "Control system port is not connected; verify keyboard fallback or reconnect joystick."
            ),
        )

        crsf_connected = self.crsf_processor is not None
        self._add_preflight_check(
            rows,
            "pass" if self.transmission_active else "fail",
            (
                "CRSF channel transmission is active."
                if self.transmission_active
                else (
                    "CRSF transmitter is not connected."
                    if not crsf_connected
                    else "CRSF channel transmission is terminated."
                )
            ),
        )

        throttle_safe = (
            self.throttle_mode == "Manual"
            and float(getattr(self, "throttle_percent", 0)) <= 0.1
        )
        self._add_preflight_check(
            rows,
            "pass" if throttle_safe else "fail",
            (
                "Throttle command is at idle."
                if throttle_safe
                else (
                    f"Throttle command is {self.throttle_percent:.0f}%; cut throttle before pre-flight."
                    if self.throttle_mode == "Manual"
                    else f"Auto throttle target is {self.throttle_target_airspeed_mph:.0f} mph; cut throttle before pre-flight."
                )
            ),
        )

        link_fresh = self._is_packet_fresh("link_stats")
        link_quality = self.telemetry_state.get("link_quality")
        downlink_quality = self.telemetry_state.get("downlink_quality")
        snr = self.telemetry_state.get("snr")
        downlink_snr = self.telemetry_state.get("downlink_snr")
        link_good = (
            link_fresh
            and link_quality is not None
            and downlink_quality is not None
            and snr is not None
            and downlink_snr is not None
            and link_quality >= 60
            and downlink_quality >= 60
            and snr >= 5
            and downlink_snr >= 5
        )
        self._add_preflight_check(
            rows,
            "pass" if link_good else "fail",
            (
                f"Signal is good: uplink LQ {link_quality}%, downlink LQ {downlink_quality}%, SNR {snr}/{downlink_snr} dB."
                if link_good
                else "Signal check failed: link statistics are stale/missing or below good thresholds (LQ >= 60%, SNR >= 5 dB)."
            ),
        )

        telemetry_streams = (
            ("attitude", "Attitude telemetry"),
            ("gps", "GPS telemetry"),
            ("battery", "Battery telemetry"),
        )
        for packet_type, label in telemetry_streams:
            fresh = self._is_packet_fresh(packet_type)
            rate = self._packet_rate(packet_type) if packet_type in self._packet_times else 0.0
            self._add_preflight_check(
                rows,
                "pass" if fresh else "fail",
                (
                    f"{label} is fresh ({self._format_preflight_age(packet_type)}, {rate:.1f} Hz)."
                    if fresh
                    else f"{label} is missing or stale ({self._format_preflight_age(packet_type)})."
                ),
            )

        telemetry_values = self.telemetry_state
        attitude_sane = all(
            value is not None and -180 <= float(value) <= 180
            for value in (
                telemetry_values.get("pitch"),
                telemetry_values.get("roll"),
                telemetry_values.get("yaw"),
            )
        )
        self._add_preflight_check(
            rows,
            "pass" if attitude_sane else "fail",
            (
                "Attitude values are within expected ranges."
                if attitude_sane
                else "Attitude values are missing or outside expected ranges."
            ),
        )

        gps_values_present = all(
            telemetry_values.get(field) is not None
            for field in ("latitude", "longitude", "altitude_ft", "airspeed_mph", "satellites")
        )
        gps_sane = False
        if gps_values_present:
            try:
                lat = float(telemetry_values.get("latitude"))
                lon = float(telemetry_values.get("longitude"))
                altitude = float(telemetry_values.get("altitude_ft"))
                airspeed = float(telemetry_values.get("airspeed_mph"))
                satellites = int(telemetry_values.get("satellites"))
                gps_sane = (
                    -90 <= lat <= 90
                    and -180 <= lon <= 180
                    and -2000 <= altitude <= 120000
                    and 0 <= airspeed <= 500
                    and satellites >= 4
                )
            except (TypeError, ValueError):
                gps_sane = False
        self._add_preflight_check(
            rows,
            "pass" if gps_sane else "warn",
            (
                f"GPS values are sane with {telemetry_values.get('satellites')} satellites."
                if gps_sane
                else "GPS values are missing, out of range, or have fewer than 4 satellites."
            ),
        )

        battery_voltage = telemetry_values.get("battery_voltage")
        battery_percent = telemetry_values.get("battery_percent")
        battery_good = False
        if battery_voltage is not None:
            try:
                voltage = float(battery_voltage)
                percent = float(battery_percent) if battery_percent is not None else (
                    voltage / self._battery_full_voltage * 100.0
                    if self._battery_full_voltage else 0.0
                )
                battery_good = voltage > 0 and percent >= 25.0
            except (TypeError, ValueError):
                battery_good = False
        self._add_preflight_check(
            rows,
            "pass" if battery_good else "warn",
            (
                f"Battery telemetry is usable: {float(battery_voltage):.2f} V, {float(battery_percent):.0f}% estimated."
                if battery_good and battery_percent is not None
                else (
                    f"Battery telemetry is usable: {float(battery_voltage):.2f} V."
                    if battery_good
                    else "Battery telemetry is missing or below 25% estimated charge."
                )
            ),
        )

        video_connected = False
        if hasattr(self, "video_feed") and getattr(self.video_feed, "label", None) is not None:
            pixmap = self.video_feed.label.pixmap()
            video_connected = pixmap is not None and not pixmap.isNull()
        self._add_preflight_check(
            rows,
            "pass" if video_connected else "warn",
            (
                "VTX video has produced a frame."
                if video_connected
                else "VTX video has not produced a frame yet."
            ),
        )

        if any(severity == "fail" for severity, _ in rows):
            overall = "fail"
        elif any(severity == "warn" for severity, _ in rows):
            overall = "warn"
        else:
            overall = "pass"

        return overall, rows

    def _preflight_report_html(
        self, overall: str, rows: list[tuple[str, str]]
    ) -> str:
        """Build rich-text pre-flight report content."""

        icons = {"pass": "✅", "warn": "⚠️", "fail": "❌"}
        summary = {
            "pass": "All pre-flight checks passed.",
            "warn": "Pre-flight checks passed with warnings.",
            "fail": "Pre-flight verification failed.",
        }[overall]
        items = "".join(
            f"<li>{icons.get(severity, '•')} {message}</li>"
            for severity, message in rows
        )
        return f"<b>{summary}</b><ul>{items}</ul>"

    def _set_message_box_icon_without_alert(
        self, message_box: QMessageBox, icon: QMessageBox.Icon
    ) -> None:
        """Show a standard QMessageBox icon without triggering native alert sounds."""

        pixmap_by_icon = {
            QMessageBox.Icon.Information: (
                QStyle.StandardPixmap.SP_MessageBoxInformation
            ),
            QMessageBox.Icon.Warning: QStyle.StandardPixmap.SP_MessageBoxWarning,
            QMessageBox.Icon.Critical: QStyle.StandardPixmap.SP_MessageBoxCritical,
        }
        standard_pixmap = pixmap_by_icon.get(icon)
        if standard_pixmap is None:
            message_box.setIcon(QMessageBox.Icon.NoIcon)
            return

        message_box.setIconPixmap(QApplication.style().standardPixmap(standard_pixmap))

    def _show_preflight_verification(self) -> None:
        """Run the pre-flight verification and display the result."""

        overall, rows = self._run_preflight_verification()
        report = self._preflight_report_html(overall, rows)

        if hasattr(self, "preflight_status_label"):
            color = {
                "pass": "rgb(76, 175, 80)",
                "warn": "rgb(255, 193, 7)",
                "fail": "rgb(244, 67, 54)",
            }[overall]
            plain_summary = {
                "pass": "Pre-flight verification passed.",
                "warn": "Pre-flight verification passed with warnings.",
                "fail": "Pre-flight verification failed.",
            }[overall]
            self.preflight_status_label.setText(plain_summary)
            self.preflight_status_label.setStyleSheet(
                f"color: {color}; font-weight: bold; padding: 4px 2px;"
            )

        message_box = QMessageBox(self)
        message_box.setWindowTitle("Pre-flight verification")
        message_box.setTextFormat(Qt.TextFormat.RichText)
        message_box.setText(report)
        if overall == "fail":
            self.play_sound("errorsound.mp3")

        self._set_message_box_icon_without_alert(
            message_box,
            QMessageBox.Icon.Information
            if overall == "pass"
            else QMessageBox.Icon.Warning,
        )
        message_box.exec()

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
        self._apply_transmission_button_style("hold")
        self._transmission_hold_elapsed_timer.start()
        self._update_transmission_hold_display(0.0)
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

        elapsed_ms = self._transmission_hold_elapsed_timer.elapsed()
        progress = min(
            elapsed_ms / self._transmission_hold_required_ms, 1.0
        )

        if progress >= 1.0:
            self._update_transmission_hold_display(1.0)
            self._transmission_hold_timer.stop()
            self._transmission_hold_in_progress = False
            self._terminate_transmission()
            return

        self._update_transmission_hold_display(progress)

    def _update_transmission_hold_display(self, progress: float) -> None:
        """Refresh the countdown text while the terminate button is held."""

        remaining_ms = max(
            self._transmission_hold_required_ms * (1.0 - progress), 0
        )
        remaining_seconds = remaining_ms / 1000
        self.transmission_control_button.setText(
            "Hold for "
            f"{remaining_seconds:.1f} seconds to terminate transmission"
        )
        self._set_transmission_hold_progress(progress)

    @staticmethod
    def _lighten_rgb(color: str, amount: float) -> str:
        """Blend an ``rgb(r, g, b)`` color toward white by ``amount``."""

        match = re.fullmatch(r"rgb\((\d+),\s*(\d+),\s*(\d+)\)", color.strip())
        if not match:
            return color

        r, g, b = (int(channel) for channel in match.groups())
        r = min(255, int(r + (255 - r) * amount))
        g = min(255, int(g + (255 - g) * amount))
        b = min(255, int(b + (255 - b) * amount))
        return f"rgb({r}, {g}, {b})"

    def _set_transmission_hold_progress(self, progress: float) -> None:
        """Shade the terminate button to represent hold progress."""

        progress = max(0.0, min(progress, 1.0))
        background, *_ = self._transmission_button_styles["hold"]
        fill_color = self._lighten_rgb(background, 0.55)
        gradient = (
            "qlineargradient(x1:0, y1:0, x2:1, y2:0,"
            f" stop:0 {fill_color},"
            f" stop:{progress:.3f} {fill_color},"
            f" stop:{progress:.3f} {background},"
            f" stop:1 {background})"
        )
        stylesheet = (
            "QPushButton {"
            f"background: {gradient};"
            "color: white;"
            "font-weight: bold;"
            "border-radius: 8px;"
            "padding: 8px 16px;"
            "}"
            f"QPushButton:hover {{background: {gradient};}}"
            f"QPushButton:pressed {{background: {gradient};}}"
            "QPushButton:disabled {background-color: rgb(80, 80, 80); color: rgb(180, 180, 180);}"
        )
        self.transmission_control_button.setStyleSheet(stylesheet)

    def _cancel_transmission_hold(self) -> None:
        """Cancel an in-progress hold-to-terminate action."""

        self._transmission_hold_timer.stop()
        self._transmission_hold_in_progress = False
        self._transmission_hold_elapsed_timer.invalidate()
        if self.transmission_active:
            self._apply_transmission_button_style("active")
            self.transmission_control_button.setText("Terminate transmission")


    def _send_safe_shutdown_frames(
        self, processor: Optional[CRSFPacketProcessor] = None
    ) -> bool:
        """Flush neutral/throttle-cut/disarmed frames before stopping TX."""

        if processor is None:
            processor = getattr(self, "crsf_processor", None)
        if processor is None:
            return False

        loop = QEventLoop(self)
        result = {"completed": False, "success": False}

        def finish(success: bool) -> None:
            result["completed"] = True
            result["success"] = bool(success)
            loop.quit()

        processor.safe_shutdown_complete.connect(finish)
        timeout = QTimer(self)
        timeout.setSingleShot(True)
        timeout.timeout.connect(loop.quit)
        try:
            processor.safe_shutdown_update.emit(
                self._build_safe_control_channels(), self._safe_shutdown_frame_count
            )
            timeout.start(self._safe_shutdown_timeout_ms)
            loop.exec()
        finally:
            timeout.stop()
            try:
                processor.safe_shutdown_complete.disconnect(finish)
            except Exception:
                pass

        if not result["completed"]:
            logging.warning("Timed out waiting for safe CRSF shutdown frames")
        return result["completed"] and result["success"]

    def _terminate_transmission(self) -> None:
        """Stop packet transmission and update button state."""

        if not self.transmission_active:
            return

        self.transmit_timer.stop()
        if self.crsf_processor:
            self._send_safe_shutdown_frames()
            self.crsf_processor.transmission_enabled_update.emit(False)
        self.transmission_active = False
        self._transmission_pressed_while_inactive = False
        self._transmission_hold_in_progress = False
        self._transmission_hold_timer.stop()
        self._apply_transmission_button_style("inactive")
        self.transmission_control_button.setText("Start transmitting packets")
        self._update_parameter_query_button_state()
        self._mute_sounds(["telemetryoffline", "disconnectedalarm", "disconnectalarm"], 2.0)
        self.play_sound("elrsterminated.mp3")

    def _start_transmission(self) -> None:
        """Resume packet transmission and reset button state."""

        if self.transmission_active:
            return

        if (
            not self.crsf_processor
            and self._is_serial_port_configured(self.crsf_cfg.get("port"))
        ):
            self.crsf_processor = self._create_crsf_processor(
                self.crsf_cfg.get("port"), transmission_enabled=False
            )
            if self.crsf_processor:
                self._set_crsf_raw_serial_debug(
                    self._debug_monitoring and self._debug_serial_all
                )
                self.update_connection_status(
                    self.rf_status, self._is_crsf_port_available()
                )

        if not self._can_start_crsf_transmission():
            self.update_connection_status(
                self.rf_status, self._is_crsf_port_available()
            )
            self.transmission_active = False
            self._transmission_pressed_while_inactive = False
            self._apply_transmission_button_style("inactive")
            self.transmission_control_button.setText("Start transmitting packets")
            self._update_parameter_query_button_state()
            self.play_sound("errorsound.mp3")
            message_box = QMessageBox(self)
            message_box.setWindowTitle("CRSF transmitter disconnected")
            message_box.setText(
                "Connect a CRSF transmitter before starting packet transmission."
            )
            self._set_message_box_icon_without_alert(
                message_box, QMessageBox.Icon.Warning
            )
            message_box.exec()
            return

        channels = self._build_control_channels()
        interval = self.crsf_cfg.get("channel_update_interval", 20)
        if self.crsf_processor:
            self.crsf_processor.transmission_start_update.emit(channels)
            if self._debug_monitoring and "control" in self._debug_packets:
                self.debug_page.log_packet("control", channels)
        self.transmit_timer.start(interval)
        self.transmission_active = True
        self._transmission_pressed_while_inactive = False
        self._apply_transmission_button_style("active")
        self.transmission_control_button.setText("Terminate transmission")
        self._update_parameter_query_button_state()
        self.play_sound("elrsinitiated.mp3")

    @staticmethod
    def _available_serial_port_devices() -> list[str]:
        """Return the serial device names currently reported by the OS."""

        return [p.device for p in list_ports.comports()]

    @staticmethod
    def _is_serial_port_configured(port: str | None) -> bool:
        """Return True when a saved port names a real selection to retry."""

        return bool(port) and str(port).lower() != "not connected"

    def _is_crsf_port_available(self) -> bool:
        """Return True when the configured CRSF port is currently enumerated."""

        port = self.crsf_cfg.get("port")
        return (
            self._is_serial_port_configured(port)
            and port in set(self._available_serial_port_devices())
        )

    def _can_start_crsf_transmission(self) -> bool:
        """Return True only when CRSF has a worker and an enumerated port."""

        return self.crsf_processor is not None and self._is_crsf_port_available()

    def _is_crsf_debug_available(self) -> bool:
        """Return True when CRSF debug actions have an available transmitter."""

        return self.crsf_processor is not None and self._is_crsf_port_available()

    def _start_serial_port_monitor(self) -> None:
        """Watch for hot-plugged serial ports and refresh the port dropdowns."""

        self._serial_port_devices = set(self._available_serial_port_devices())
        self._serial_port_monitor_timer = QTimer(self)
        self._serial_port_monitor_timer.setInterval(1000)
        self._serial_port_monitor_timer.timeout.connect(
            self._refresh_port_lists_if_serial_ports_changed
        )
        self._serial_port_monitor_timer.start()

    def _refresh_port_lists_if_serial_ports_changed(self) -> None:
        """Refresh the dropdowns only when the OS serial-port list changes."""

        current_ports = set(self._available_serial_port_devices())
        if current_ports == getattr(self, "_serial_port_devices", set()):
            return

        self._serial_port_devices = current_ports
        self.update_port_lists()

    def update_port_lists(self):
        """Refresh available serial ports and update the dropdowns."""
        available_ports = self._available_serial_port_devices()
        self._serial_port_devices = set(available_ports)
        ports = ["Not connected"] + available_ports

        def refresh(combo, handler, *, preserved_port: str | None = None):
            choices = list(ports)
            current = combo.currentText()
            if (
                self._is_serial_port_configured(preserved_port)
                and preserved_port not in choices
            ):
                choices.append(preserved_port)

            selected_port_available = current in choices
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(choices)
            combo.setCurrentText(current if selected_port_available else "Not connected")
            combo.blockSignals(False)
            if not selected_port_available:
                handler("Not connected")

        refresh(
            self.control_port_combo,
            self.on_control_port_selected,
            preserved_port=self.joystick_cfg.get("port"),
        )
        refresh(
            self.elrs_port_combo,
            self.on_elrs_port_selected,
            preserved_port=self.crsf_cfg.get("port"),
        )
        crsf_port_available = self._is_crsf_port_available()
        self.update_connection_status(self.rf_status, crsf_port_available)
        if crsf_port_available:
            if not self.crsf_processor:
                self.crsf_processor = self._create_crsf_processor(
                    self.crsf_cfg.get("port"), transmission_enabled=False
                )
                if self.crsf_processor:
                    self._set_crsf_raw_serial_debug(
                        self._debug_monitoring and self._debug_serial_all
                    )
            else:
                QMetaObject.invokeMethod(
                    self.crsf_processor, "reconnect_serial", Qt.QueuedConnection
                )
        self._update_parameter_query_button_state()
        self._update_link_diagnostics_button_state()
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
        self.joystick = self._create_joystick_handler(port)
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
        was_transmitting = self.transmission_active
        old_processor = self.crsf_processor
        if old_processor:
            if was_transmitting:
                self.transmit_timer.stop()
                self._send_safe_shutdown_frames(old_processor)
                old_processor.transmission_enabled_update.emit(False)
            try:
                thread = old_processor._thread
                QMetaObject.invokeMethod(
                    old_processor, "close_serial", Qt.BlockingQueuedConnection
                )
                thread.quit()
                thread.wait()
            except Exception:
                pass
            self.crsf_processor = None
            self._link_diagnostics_active = False
        self.crsf_processor = self._create_crsf_processor(
            port, transmission_enabled=was_transmitting
        )
        if self.crsf_processor:
            self._set_crsf_raw_serial_debug(
                self._debug_monitoring and self._debug_serial_all
            )

        self.transmission_active = bool(self.crsf_processor and was_transmitting)
        if self.transmission_active:
            self.transmit_timer.start(self.crsf_cfg.get("channel_update_interval", 20))
            if hasattr(self, "transmission_control_button"):
                self._apply_transmission_button_style("active")
                self.transmission_control_button.setText("Terminate transmission")
        else:
            self.transmit_timer.stop()
            if hasattr(self, "transmission_control_button"):
                self._apply_transmission_button_style("inactive")
                self.transmission_control_button.setText("Start transmitting packets")

        self.update_connection_status(self.rf_status, self._is_crsf_port_available())
        self._update_parameter_query_button_state()
        self._update_link_diagnostics_button_state()
        save_config(self.config)

    def on_packet_rate_changed(self, *_):
        rate_hz = self.packet_rate_combo.currentData()
        interval = packet_interval_ms_from_rate(rate_hz)
        self.crsf_cfg["packet_interval"] = interval
        if self.crsf_processor:
            self.crsf_processor.packet_interval_update.emit(interval)
        self.update_pico_rate_label()
        save_config(self.config)

    def update_pico_rate_label(self):
        interval = self.crsf_cfg.get("packet_interval", 4)
        rate_hz = packet_rate_hz_from_interval(interval)
        actual_freq = 1000 / packet_interval_ms_from_rate(rate_hz)
        self.pico_rate_label.setText(
            f"PICO writing RC packets at {actual_freq:.0f} Hz."
        )

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

    def on_fbw_roll_limit_changed(self, value: float):
        self.fbw_max_roll_angle_deg = self._validated_fbw_limit(
            value,
            self.FBW_FC_MAX_ROLL_ANGLE_DEG,
            self.DEFAULT_FBW_MAX_ROLL_ANGLE_DEG,
        )
        self.fbw_cfg["max_roll_angle_deg"] = self.fbw_max_roll_angle_deg
        self._update_desired_fbw_attitude(
            getattr(self, "_latest_control_channels", [CRSF_CHANNEL_CENTER] * 16)
        )
        save_config(self.config)

    def on_fbw_pitch_limit_changed(self, value: float):
        self.fbw_max_pitch_angle_deg = self._validated_fbw_limit(
            value,
            self.FBW_FC_MAX_PITCH_ANGLE_DEG,
            self.DEFAULT_FBW_MAX_PITCH_ANGLE_DEG,
        )
        self.fbw_cfg["max_pitch_angle_deg"] = self.fbw_max_pitch_angle_deg
        self._update_desired_fbw_attitude(
            getattr(self, "_latest_control_channels", [CRSF_CHANNEL_CENTER] * 16)
        )
        save_config(self.config)

    def on_auto_throttle_target_changed(self, value: float):
        self.set_auto_throttle_target_speed(value)

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

    def on_airborne_takeoff_alt_changed(self, value: float):
        self.airborne_cfg["takeoff_altitude_ft"] = float(value)
        self._airborne_takeoff_start_time = None
        save_config(self.config)

    def on_airborne_landed_speed_changed(self, value: float):
        self.airborne_cfg["landed_airspeed_mph"] = float(value)
        self._airborne_landing_start_time = None
        save_config(self.config)

    def on_airborne_landed_alt_changed(self, value: float):
        self.airborne_cfg["landed_altitude_ft"] = float(value)
        self._airborne_landing_start_time = None
        save_config(self.config)

    def on_stall_speed_changed(self, value: int):
        self.warning_cfg["stall_airspeed"] = value
        self.stall_speed_value.setText(str(value))
        self._airborne_takeoff_start_time = None
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

    def on_sink_rate_threshold_changed(self, value: float):
        self.warning_cfg["sink_rate_threshold_fps"] = float(value)
        save_config(self.config)

    def on_battery_type_changed(self, selection: str):
        self.aircraft_cfg["battery_cells"] = selection
        self._update_battery_full_voltage()
        save_config(self.config)

    def _stop_warning_alarm_audio(self):
        alarm_sounds = (
            "whoopalarm",
            "airspeedlowarning",
            "beepalarm",
            "altitudewarning",
            "pullupwarning",
            "downupalarm",
            "bankanglewarning",
            "sinkalarm",
            "sinkratewarning",
        )
        self._mute_sounds(alarm_sounds, 1.0)
        for sound_name in alarm_sounds:
            player_output = self.sound_players.pop(sound_name, None)
            if not player_output:
                continue
            player, output = player_output
            try:
                player.stop()
            except Exception:
                pass
            player.deleteLater()
            output.deleteLater()

    def _clear_warning_alarm_state(self, stop_audio: bool = True):
        self.stall_alarm_start_time = None
        self.altitude_alarm_start_time = None
        self.roll_alarm_start_time = None
        self.sink_rate_alarm_start_time = None
        self.stall_alarm_playing = False
        self.altitude_alarm_playing = False
        self.roll_alarm_playing = False
        self.sink_rate_alarm_playing = False
        if stop_audio:
            self._stop_warning_alarm_audio()

    def on_warning_alarms_toggled(self, checked: bool):
        self.warning_cfg["warning_alarms_enabled"] = checked
        if not checked:
            self._clear_warning_alarm_state()
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

    def on_sink_rate_alarm_toggled(self, checked: bool):
        self.warning_cfg["sink_rate_alarm_enabled"] = checked
        if not checked:
            self.sink_rate_alarm_start_time = None
            self.sink_rate_alarm_playing = False
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
        self._update_gps_fix_indicator()
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

        if btnName == "btn_preflight":
            widgets.stackedWidget.setCurrentWidget(widgets.preflight_page)
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

        if btnName == "btn_documentation":
            widgets.stackedWidget.setCurrentWidget(widgets.documentation_page)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

    def start_debug_monitoring(
        self,
        packets: set[str],
        include_joystick: bool,
        serial_all: bool,
        telemetry_all: bool,
    ) -> None:
        """Begin forwarding selected telemetry streams to the Debug tab."""

        self._debug_packets = set(packets)
        self._debug_include_joystick = include_joystick
        self._debug_serial_all = serial_all
        self._debug_telemetry_all = telemetry_all
        self._debug_monitoring = True
        self._last_control_update_debug_timestamp = None
        self._set_crsf_raw_serial_debug(self._debug_serial_all)
        self.debug_page.begin_monitoring(
            self._debug_packets,
            include_joystick,
            serial_all,
            telemetry_all,
        )
        if (self._debug_packets or serial_all or telemetry_all) and not self.crsf_processor:
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
        self._last_control_update_debug_timestamp = None
        self._debug_packets.clear()
        self._debug_include_joystick = False
        self._debug_serial_all = False
        self._debug_telemetry_all = False
        self._set_crsf_raw_serial_debug(False)
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
            if self.transmission_active:
                self.transmit_timer.stop()
                self._send_safe_shutdown_frames()
                self.crsf_processor.transmission_enabled_update.emit(False)
                self.transmission_active = False
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
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(40, 44, 52))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(221, 221, 221))
    palette.setColor(QPalette.ColorRole.Base, QColor(33, 37, 43))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(40, 44, 52))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(40, 44, 52))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.Text, QColor(221, 221, 221))
    palette.setColor(QPalette.ColorRole.Button, QColor(33, 37, 43))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(221, 221, 221))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(46, 125, 50))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    window = MainWindow()
    window.show()
    app.aboutToQuit.connect(window.cleanup)
    try:
        exit_code = app.exec()
    finally:
        window.cleanup()
    sys.exit(exit_code)
