import os
import time
import csv
import logging
import threading
import re
from datetime import datetime
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from functools import partial
from typing import Optional

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


class RangeRequestHandler(SimpleHTTPRequestHandler):
    """HTTP handler that adds Content-Length and Range support."""

    def send_head(self):
        path = self.translate_path(self.path)
        if os.path.isdir(path):
            return super().send_head()

        ctype = self.guess_type(path)
        try:
            f = open(path, "rb")
        except OSError:
            self.send_error(404, "File not found")
            return None

        fs = os.fstat(f.fileno())
        total_length = fs.st_size
        start = 0
        end = total_length - 1
        if "Range" in self.headers:
            m = re.match(r"bytes=(\d+)-(\d+)?", self.headers["Range"])
            if m:
                start = int(m.group(1))
                if m.group(2):
                    end = min(int(m.group(2)), end)
                length = end - start + 1
                f.seek(start)
                self.send_response(206)
                self.send_header("Content-Range", f"bytes {start}-{end}/{total_length}")
            else:
                length = total_length
                self.send_response(200)
        else:
            length = total_length
            self.send_response(200)

        self.send_header("Content-type", ctype)
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Last-Modified", self.date_time_string(fs.st_mtime))
        self.end_headers()
        self._range_length = length
        return f

    def do_GET(self):
        f = self.send_head()
        if f:
            try:
                self.copyfile_range(f, self.wfile, self._range_length)
            finally:
                f.close()

    def do_HEAD(self):
        f = self.send_head()
        if f:
            f.close()

    def copyfile_range(self, source, output, length, bufsize=64 * 1024):
        while length > 0:
            chunk = source.read(min(bufsize, length))
            if not chunk:
                break
            output.write(chunk)
            length -= len(chunk)


def start_static_server():
    """Start a tiny HTTP server to serve local map assets."""
    web_dir = os.path.dirname(__file__)
    handler = partial(RangeRequestHandler, directory=web_dir)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd

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
from PySide6.QtWebEngineCore import QWebEngineProfile
from PySide6.QtGui import QIcon, QColor, QShortcut, QKeySequence
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

# Temporarily disable the GPS map to troubleshoot memory usage
MAP_ENABLED = False

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

        # Remove unwanted side tabs
        self.ui.btn_save.hide()
        self.ui.btn_exit.hide()

        # Rename side tab buttons
        self.ui.btn_new.setText("Command")
        self.ui.btn_widgets.setText("Configuration")

        # Load configuration early so map defaults are available
        self.config = load_config()
        self.map_cfg = self.config.setdefault("map", {"center": [0, 0], "zoom": 2})

        # Optionally start the local map server and load the map if enabled
        self.httpd = None
        if MAP_ENABLED:
            self.map_view = self.ui.mapframe
            map_file = os.path.join(os.path.dirname(__file__), "map", "map1.pmtiles")
            if not os.path.exists(map_file):
                QMessageBox.warning(
                    self,
                    "Missing map tiles",
                    "Offline map tiles 'map1.pmtiles' not found in the map directory."
                    " The GPS map will remain blank.",
                )
            else:
                self.httpd = start_static_server()
                port = self.httpd.server_address[1]
                lat, lon = self.map_cfg.get("center", [0, 0])
                zoom = self.map_cfg.get("zoom", 2)
                map_url = QUrl(
                    f"http://127.0.0.1:{port}/map/index.html?lat={lat}&lon={lon}&zoom={zoom}"
                )
                self.map_view.setUrl(map_url)
                # Disable WebEngine HTTP caching to limit memory usage from map tiles
                profile = self.map_view.page().profile()
                profile.setHttpCacheType(QWebEngineProfile.NoCache)
                profile.clearHttpCache()
        else:
            # Hide the map widget when the map feature is disabled
            self.ui.mapframe.hide()

        # Add Data tab and associated graphs
        self.data_page = DataPage(self)

        # Use frameless window and translucent background
        # self.setWindowFlags(Qt.FramelessWindowHint)
        # self.setAttribute(Qt.WA_TranslucentBackground)

        # Configuration sections
        self.joystick_cfg = self.config.setdefault("joystick", {})
        self.crsf_cfg = self.config.setdefault("crsf", {})
        self.vtx_cfg = self.config.setdefault("vtx", {})
        self.warning_cfg = self.config.setdefault("warnings", {})
        self.warning_cfg.setdefault("stall_alarm_enabled", True)
        self.warning_cfg.setdefault("altitude_alarm_enabled", True)
        self.warning_cfg.setdefault("bank_angle_alarm_enabled", True)

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

        self.attitude_monitor_timer = QTimer(self)
        self.attitude_monitor_timer.timeout.connect(
            self.check_attitude_connection
        )
        self.attitude_monitor_timer.start(200)

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
        self.throttle_percent = 0
        self.target_throttle_percent = 0
        self.throttle_indicator = ThrottleWidget(self.ui.throttleInput)
        self.throttle_indicator.resize(self.ui.throttleInput.size())
        self.throttle_indicator.show()

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
        self.current_altitude = None
        self.current_airspeed = None
        self.telemetry_state = {field: None for field in self._sortie_fields}

        # Timer used to refresh labels/OSD widgets at a fixed rate. Telemetry
        # packets only update the cached values above; the GUI is refreshed by
        # this timer regardless of packet arrival rate.
        self.label_update_timer = QTimer(self)
        self.label_update_timer.timeout.connect(self.update_labels)

        self.label_update_timer.start(0)

        # Timer for transmitting data (default from config)
        self.transmit_timer = QTimer(self)
        self.transmit_timer.timeout.connect(self.transmit_data)
        # Throttle default packet rate to reduce GUI load
        self.transmit_timer.start(self.crsf_cfg.get("packet_interval", 15))

        # --------------------------------------------------------------------
        # OSD Overlay Setup - Create and initialize the RollPitchOSD widget
        # --------------------------------------------------------------------
        # Here we assume that in your .ui file there is a placeholder widget named "rollpitchosd"
        # We create our custom RollPitchOSD instance with that widget as its parent.
        self.rollpitch_osd = RollPitchOSD(self.ui.rollpitchosd)
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


    def _setup_sortie_section(self) -> None:
        """Create the Sorties section and recording controls on the command tab."""

        sorties_frame = QFrame(self.ui.frame_4)
        sorties_frame.setObjectName("sortiesFrame")
        sorties_frame.setGeometry(0, 150, 571, 170)

        layout = QVBoxLayout(sorties_frame)
        layout.setContentsMargins(20, 10, 20, 10)
        layout.setSpacing(10)

        sorties_title = QLabel("Sorties", sorties_frame)
        sorties_title.setObjectName("sortiesTitle")
        sorties_title.setAlignment(Qt.AlignCenter)
        sorties_title.setFont(self.ui.signalHealthTitle.font())
        sorties_title.setStyleSheet("color: white;")
        layout.addWidget(sorties_title)

        self.ui.sortieRecordButton = QPushButton("Start Recording", sorties_frame)
        self.ui.sortieRecordButton.setObjectName("sortieRecordButton")
        self.ui.sortieRecordButton.setCursor(Qt.PointingHandCursor)
        self.ui.sortieRecordButton.setMinimumHeight(36)
        layout.addWidget(self.ui.sortieRecordButton, alignment=Qt.AlignCenter)

        self.sortie_status_label = QLabel("Status: Waiting for telemetry", sorties_frame)
        self.sortie_status_label.setAlignment(Qt.AlignCenter)
        self.sortie_status_label.setStyleSheet("color: #9e9e9e;")
        layout.addWidget(self.sortie_status_label)

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

        self.ui.sortieRecordButton.clicked.connect(self.toggle_sortie_recording)
        self._update_sortie_ui_state()

    def _update_sortie_ui_state(self) -> None:
        """Refresh the sortie button appearance and status text."""

        if self.sortie_recording:
            self.ui.sortieRecordButton.setEnabled(True)
            self.ui.sortieRecordButton.setText("Stop Recording")
            self.ui.sortieRecordButton.setStyleSheet(self._sortie_active_style)
            self.ui.sortieRecordButton.setToolTip("Stop recording telemetry (Ctrl+R)")
            if self.sortie_filename:
                status = f"Status: Recording ({self.sortie_filename})"
            else:
                status = "Status: Recording"
            self.sortie_status_label.setText(status)
            self.sortie_status_label.setStyleSheet("color: #ff6e6e; font-weight: bold;")
            return

        ready = self._sortie_ready_state
        self.ui.sortieRecordButton.setStyleSheet(self._sortie_idle_style)
        self.ui.sortieRecordButton.setText("Start Recording")
        self.ui.sortieRecordButton.setEnabled(ready)
        if ready:
            self.ui.sortieRecordButton.setToolTip("Start recording telemetry (Ctrl+R)")
            self.sortie_status_label.setText("Status: Ready to record")
            self.sortie_status_label.setStyleSheet("color: #a5d6a7;")
        else:
            self.ui.sortieRecordButton.setToolTip(
                "Telemetry data required to start recording"
            )
            self.sortie_status_label.setText("Status: Waiting for telemetry")
            self.sortie_status_label.setStyleSheet("color: #9e9e9e;")

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

        os.makedirs(self.sortie_directory, exist_ok=True)
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

    @Slot(str)
    def handle_worker_error(self, message: str):
        """Handle errors emitted from worker threads."""
        logging.error("Worker error: %s", message)
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
        # Check for any telemetry-based warnings
        self.check_warnings()

        # ------------------------------------------------------------------
        # Joystick values update the label texts
        # ------------------------------------------------------------------
        joy_pitch = joy_roll = None
        if self.joystick:
            try:
                joy_pitch, joy_roll = self.joystick.get_raw_values()
            except Exception:
                pass

        if joy_pitch is None:
            self.pitch_indicator.setValue(0)
            self.roll_indicator.setValue(0)
            self.yaw_indicator.setValue(0)
        else:
            norm_pitch = (joy_pitch - 512) / 512
            norm_roll = (joy_roll - 512) / 512
            self.pitch_indicator.setValue(norm_pitch)
            self.roll_indicator.setValue(norm_roll)
            self.yaw_indicator.setValue(0)
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
        else:
            super().keyPressEvent(event)

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

    def play_sound(self, name: str):
        """Play a warning sound identified by ``name``.

        MP3 files are expected to reside in an ``audio`` directory and be
        named ``{name}.mp3``. The player instances are cached so repeated
        alerts reuse the same player.
        """
        file_path = os.path.join("audio", f"{name}.mp3")

        # Reuse an existing player if it hasn't been deleted; otherwise create
        # a fresh QMediaPlayer/QAudioOutput pair.
        player_output = self.sound_players.get(name)
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
        try:
            player.mediaStatusChanged.disconnect()
        except Exception:
            pass

        def handle_status(status, *, name=name, player=player, output=output):
            if status == QMediaPlayer.MediaStatus.EndOfMedia:
                try:
                    player.mediaStatusChanged.disconnect(handle_status)
                except Exception:
                    pass
                self.sound_players.pop(name, None)
                player.deleteLater()
                output.deleteLater()

        player.mediaStatusChanged.connect(handle_status)
        player.play()
        self.sound_players[name] = (player, output)

    def play_sound_sequence(self, names, finished_callback=None):
        """Play a sequence of warning sounds in order.

        ``finished_callback`` is called when the sequence has completed.
        """
        if not names:
            if finished_callback:
                finished_callback()
            return

        name = names[0]
        file_path = os.path.join("audio", f"{name}.mp3")

        player_output = self.sound_players.get(name)
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

        def handle_status(status, *, name=name, player=player, output=output):
            if status == QMediaPlayer.MediaStatus.EndOfMedia:
                try:
                    player.mediaStatusChanged.disconnect(handle_status)
                except Exception:
                    pass
                self.sound_players.pop(name, None)
                player.deleteLater()
                output.deleteLater()
                if len(names) > 1:
                    self.play_sound_sequence(names[1:], finished_callback)
                elif finished_callback:
                    finished_callback()

        player.mediaStatusChanged.connect(handle_status)
        player.play()
        self.sound_players[name] = (player, output)

    def check_attitude_connection(self):
        """Monitor attitude packet reception and play connection sounds."""
        now = time.monotonic()
        if self.attitude_connected:
            if (
                self.last_attitude_packet_time is None
                or now - self.last_attitude_packet_time > 1.0
            ):
                self.attitude_connected = False
                self.play_sound_sequence(["disconnectedalarm"])
                self.attitude_first_received_time = None
        else:
            if (
                self.last_attitude_packet_time is None
                or now - self.last_attitude_packet_time > 1.0
            ):
                self.attitude_first_received_time = None

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
                    ["beepalarm", "pullupwarning", "pullupwarning"],
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
        # Temporarily silence telemetry debug output
        # if packet_type != "link_stats":
        #     print(f"Telemetry {packet_type}: {values}")
        self.data_page.record_packet()
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
                    self.play_sound_sequence(["connectedalarm"])
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
            self.gps_lat = lat
            self.gps_lon = lon
            self.current_altitude = alt
            self.current_airspeed = speed
            self.telemetry_state["latitude"] = lat
            self.telemetry_state["longitude"] = lon
            self.telemetry_state["altitude_ft"] = alt
            self.telemetry_state["airspeed_mph"] = speed
            self.telemetry_state["ground_course"] = course
            self.telemetry_state["satellites"] = sats
            self.data_page.add_flight_metrics(alt, speed)
            if MAP_ENABLED and hasattr(self, "map_view"):
                self.map_view.page().runJavaScript(
                    f"updateMarker({lat}, {lon});"
                )
        elif packet_type == "link_stats":
            (
                rssi_a,
                rssi_b,
                link_quality,
                snr,
                downlink_lq,
                downlink_snr,
            ) = values
            self.telemetry_state["rssi_a"] = rssi_a
            self.telemetry_state["rssi_b"] = rssi_b
            self.telemetry_state["link_quality"] = link_quality
            self.telemetry_state["snr"] = snr
            self.telemetry_state["downlink_quality"] = downlink_lq
            self.telemetry_state["downlink_snr"] = downlink_snr
            self.data_page.add_link_stats(
                rssi_a,
                rssi_b,
                link_quality,
                downlink_lq,
                snr,
                downlink_snr,
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

        self._record_telemetry_sample(packet_type)

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
        # Control mode channel: send low for Manual, high for Fly-By-Wire
        mode_value = 1811 if self.control_mode == "Fly-By-Wire" else 172
        channels[self.control_mode_channel] = mode_value
        try:
            self.crsf_processor.channel_update.emit(channels)
        except Exception as e:
            print(f"Error during transmission: {e}")

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

    def setup_configuration_page(self):
        """Create configuration page for selecting settings."""
        self.ui.configuration_page = QWidget()
        widgets.configuration_page = self.ui.configuration_page
        widgets.stackedWidget.addWidget(self.ui.configuration_page)

        layout = QVBoxLayout(self.ui.configuration_page)
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
        # Connect signals
        self.control_port_combo.currentTextChanged.connect(self.on_control_port_selected)
        self.elrs_port_combo.currentTextChanged.connect(self.on_elrs_port_selected)
        self.packet_interval_edit.editingFinished.connect(self.on_packet_interval_changed)
        self.deadzone_slider.valueChanged.connect(self.on_deadzone_changed)
        self.sensitivity_slider.valueChanged.connect(self.on_sensitivity_changed)
        self.smoothing_slider.valueChanged.connect(self.on_smoothing_changed)
        self.stall_speed_slider.valueChanged.connect(self.on_stall_speed_changed)
        self.stall_alt_slider.valueChanged.connect(self.on_stall_alt_changed)
        self.alt_alarm_alt_slider.valueChanged.connect(self.on_alt_alarm_alt_changed)
        self.alt_alarm_speed_slider.valueChanged.connect(self.on_alt_alarm_speed_changed)
        self.roll_angle_slider.valueChanged.connect(self.on_roll_angle_changed)

        # Initial connection status
        self.update_connection_status(self.control_status, self.joystick is not None)
        self.update_connection_status(self.rf_status, self.crsf_processor is not None)
        # Video connection status is derived from the video feed itself
        self.update_connection_status(self.vtx_status, False)
        # Ensure the port lists reflect currently connected devices
        self.update_port_lists()

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

    def on_smoothing_changed(self, value: int):
        self.joystick_cfg["smoothing"] = value
        self.smoothing_value_label.setText(str(value))
        if self.joystick:
            self.joystick.set_smoothing(value)
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
        if self.joystick:
            self.joystick.close()
            self.joystick = None
        if getattr(self, "httpd", None):
            self.httpd.shutdown()
            self.httpd = None

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
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.aboutToQuit.connect(window.cleanup)
    try:
        exit_code = app.exec()
    finally:
        window.cleanup()
    sys.exit(exit_code)
