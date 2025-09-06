import sys
import os
import time
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
    QGridLayout,
    QPushButton,
)
from PySide6.QtCore import Qt, QTimer, QMetaObject, Slot
from PySide6.QtGui import QCursor
from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
from PySide6.QtCore import QUrl

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
from pico_modules.compass_osd import CompassOSD

from config import load_config, save_config

from collections import deque
import pyqtgraph as pg


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

        # Add Data tab and associated graphs
        self.setup_data_page()

        # Use frameless window and translucent background
        # self.setWindowFlags(Qt.FramelessWindowHint)
        # self.setAttribute(Qt.WA_TranslucentBackground)

        self.config = load_config()

        # Configuration sections
        self.joystick_cfg = self.config.setdefault("joystick", {})
        self.crsf_cfg = self.config.setdefault("crsf", {})
        self.vtx_cfg = self.config.setdefault("vtx", {})
        self.warning_cfg = self.config.setdefault("warnings", {})

        # Warning system state
        self.stall_alarm_playing = False
        self.altitude_alarm_playing = False
        self.roll_alarm_playing = False
        self.stall_alarm_start_time = None
        self.altitude_alarm_start_time = None
        self.roll_alarm_start_time = None
        self.sound_players = {}

        # Initialize the video feed using the configured device index
        self.video_port = self.vtx_cfg.get("port", "")
        video_index = self.vtx_cfg.get("device_index", 1)
        self.video_feed = VideoFeed(self.ui.VideoLabel, device_index=video_index)
        if validate_port("VTX", self.video_port):
            self.video_feed.start()
        else:
            print("VTX video disabled due to unavailable port.")
            self.ui.VideoLabel.setText("Not connected")
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
                )
                self.crsf_processor.telemetry_ready.connect(
                    lambda data: self.handle_telemetry(*data)
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
        }

        # Initialize the LabelManager with the labels
        self.label_manager = LabelManager(labels)

        # Variables updated from telemetry packets
        self.telemetry_pitch = None
        self.telemetry_roll = None
        self.telemetry_yaw = None
        self.current_altitude = None
        self.current_airspeed = None

        # Label/OSD updates are event-driven and triggered by telemetry
        # packets to avoid unnecessary 10ms polling. When telemetry data
        # arrives, ``handle_telemetry`` schedules ``update_labels`` via
        # ``QMetaObject.invokeMethod`` to keep the UI responsive.

        # Timer for transmitting data (default from config)
        self.transmit_timer = QTimer(self)
        self.transmit_timer.timeout.connect(self.transmit_data)
        self.transmit_timer.start(self.crsf_cfg.get("packet_interval", 3))

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


    def setup_data_page(self):
        """Create the Data tab with live telemetry graphs."""
        font = self.ui.btn_home.font()
        self.ui.btn_data = QPushButton(self.ui.topMenu)
        self.ui.btn_data.setObjectName("btn_data")
        size_policy = self.ui.btn_home.sizePolicy()
        self.ui.btn_data.setSizePolicy(size_policy)
        self.ui.btn_data.setMinimumSize(self.ui.btn_home.minimumSize())
        self.ui.btn_data.setFont(font)
        self.ui.btn_data.setCursor(QCursor(Qt.PointingHandCursor))
        self.ui.btn_data.setLayoutDirection(Qt.LeftToRight)
        self.ui.btn_data.setStyleSheet(
            "background-image: url(:/icons/images/icons/cil-chart-line.png);"
        )
        self.ui.btn_data.setText("Telemetry Data")
        self.ui.verticalLayout_8.addWidget(self.ui.btn_data)
        widgets.btn_data = self.ui.btn_data

        # Data page widget
        self.data_page = QWidget()
        widgets.data_page = self.data_page
        self.ui.stackedWidget.addWidget(self.data_page)

        layout = QVBoxLayout(self.data_page)

        title_label = QLabel("Telemetry Data")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title_label)

        attitude_label = QLabel("Attitude")
        attitude_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(attitude_label)

        attitude_layout = QHBoxLayout()
        layout.addLayout(attitude_layout, 1)
        plot_height = 150
        self.roll_plot = pg.PlotWidget()
        self.roll_plot.setTitle("Roll")
        self.roll_plot.setMinimumHeight(plot_height)
        attitude_layout.addWidget(self.roll_plot)
        self.pitch_plot = pg.PlotWidget()
        self.pitch_plot.setTitle("Pitch")
        self.pitch_plot.setMinimumHeight(plot_height)
        attitude_layout.addWidget(self.pitch_plot)
        self.yaw_plot = pg.PlotWidget()
        self.yaw_plot.setTitle("Yaw")
        self.yaw_plot.setMinimumHeight(plot_height)
        attitude_layout.addWidget(self.yaw_plot)

        flight_label = QLabel("Flight Telemetry")
        flight_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(flight_label)

        flight_layout = QHBoxLayout()
        layout.addLayout(flight_layout, 1)
        self.airspeed_plot = pg.PlotWidget()
        self.airspeed_plot.setTitle("Air speed")
        self.airspeed_plot.setMinimumHeight(plot_height)
        flight_layout.addWidget(self.airspeed_plot)
        self.altitude_plot = pg.PlotWidget()
        self.altitude_plot.setTitle("Altitude")
        self.altitude_plot.setMinimumHeight(plot_height)
        flight_layout.addWidget(self.altitude_plot)

        signal_label = QLabel("Signal Health")
        signal_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(signal_label)

        signal_layout = QGridLayout()
        layout.addLayout(signal_layout, 1)
        self.rssi_a_plot = pg.PlotWidget(); self.rssi_a_plot.setTitle("RSSI A")
        self.rssi_b_plot = pg.PlotWidget(); self.rssi_b_plot.setTitle("RSSI B")
        self.link_quality_plot = pg.PlotWidget(); self.link_quality_plot.setTitle("Link Quality")
        self.downlink_quality_plot = pg.PlotWidget(); self.downlink_quality_plot.setTitle("Downlink Quality")
        self.snr_plot = pg.PlotWidget(); self.snr_plot.setTitle("SNR")
        self.downlink_snr_plot = pg.PlotWidget(); self.downlink_snr_plot.setTitle("Downlink SNR")
        signal_layout.addWidget(self.rssi_a_plot, 0, 0)
        signal_layout.addWidget(self.rssi_b_plot, 0, 1)
        signal_layout.addWidget(self.link_quality_plot, 0, 2)
        signal_layout.addWidget(self.downlink_quality_plot, 1, 0)
        signal_layout.addWidget(self.snr_plot, 1, 1)
        signal_layout.addWidget(self.downlink_snr_plot, 1, 2)

        self.packet_rate_label = QLabel("Packets Received Rate: 0 Hz")
        layout.addWidget(self.packet_rate_label)

        # Data storage for plots
        max_points = 200
        self.roll_data = deque([0] * max_points, maxlen=max_points)
        self.pitch_data = deque([0] * max_points, maxlen=max_points)
        self.yaw_data = deque([0] * max_points, maxlen=max_points)
        self.airspeed_data = deque([0] * max_points, maxlen=max_points)
        self.altitude_data = deque([0] * max_points, maxlen=max_points)
        self.rssi_a_data = deque([0] * max_points, maxlen=max_points)
        self.rssi_b_data = deque([0] * max_points, maxlen=max_points)
        self.link_quality_data = deque([0] * max_points, maxlen=max_points)
        self.downlink_quality_data = deque([0] * max_points, maxlen=max_points)
        self.snr_data = deque([0] * max_points, maxlen=max_points)
        self.downlink_snr_data = deque([0] * max_points, maxlen=max_points)

        # Plot curves
        self.roll_curve = self.roll_plot.plot()
        self.pitch_curve = self.pitch_plot.plot()
        self.yaw_curve = self.yaw_plot.plot()
        self.airspeed_curve = self.airspeed_plot.plot()
        self.altitude_curve = self.altitude_plot.plot()
        self.rssi_a_curve = self.rssi_a_plot.plot()
        self.rssi_b_curve = self.rssi_b_plot.plot()
        self.link_quality_curve = self.link_quality_plot.plot()
        self.downlink_quality_curve = self.downlink_quality_plot.plot()
        self.snr_curve = self.snr_plot.plot()
        self.downlink_snr_curve = self.downlink_snr_plot.plot()

        # Timers for updating graphs and packet rate
        self.graph_timer = QTimer(self)
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.start(100)

        self.packet_rate = 0
        self.packet_count = 0
        self.packet_rate_timer = QTimer(self)
        self.packet_rate_timer.timeout.connect(self.update_packet_rate)
        self.packet_rate_timer.start(1000)


    def update_graphs(self):
        self.roll_curve.setData(self.roll_data)
        self.pitch_curve.setData(self.pitch_data)
        self.yaw_curve.setData(self.yaw_data)
        self.airspeed_curve.setData(self.airspeed_data)
        self.altitude_curve.setData(self.altitude_data)
        self.rssi_a_curve.setData(self.rssi_a_data)
        self.rssi_b_curve.setData(self.rssi_b_data)
        self.link_quality_curve.setData(self.link_quality_data)
        self.downlink_quality_curve.setData(self.downlink_quality_data)
        self.snr_curve.setData(self.snr_data)
        self.downlink_snr_curve.setData(self.downlink_snr_data)


    def update_packet_rate(self):
        self.packet_rate_label.setText(
            f"Packets Received Rate: {self.packet_rate} Hz"
        )
        self.packet_rate = self.packet_count
        self.packet_count = 0

    @Slot()
    def update_labels(self) -> None:
        """Update GUI labels using joystick inputs and refresh OSD widgets."""

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
            self.label_manager.update_labels(
                {"pitch": "N/A", "roll": "N/A", "yaw": "N/A"}
            )
        else:
            self.label_manager.update_labels(
                {
                    "pitch": f"{joy_pitch}",
                    "roll": f"{joy_roll}",
                    "yaw": "N/A",
                }
            )

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
        player, output = self.sound_players.get(name, (QMediaPlayer(), QAudioOutput()))
        player.setAudioOutput(output)
        player.setSource(QUrl.fromLocalFile(file_path))
        output.setVolume(1.0)
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
        player, output = self.sound_players.get(name, (QMediaPlayer(), QAudioOutput()))
        player.setAudioOutput(output)
        player.setSource(QUrl.fromLocalFile(file_path))
        output.setVolume(1.0)

        def handle_status(status):
            if status == QMediaPlayer.MediaStatus.EndOfMedia:
                player.mediaStatusChanged.disconnect(handle_status)
                if len(names) > 1:
                    self.play_sound_sequence(names[1:], finished_callback)
                elif finished_callback:
                    finished_callback()

        player.mediaStatusChanged.connect(handle_status)
        player.play()
        self.sound_players[name] = (player, output)

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
        if (
            self.current_airspeed < self.warning_cfg.get("stall_airspeed", 0)
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
        if (
            self.current_altitude < self.warning_cfg.get("altitude_alarm_altitude", 0)
            and self.current_airspeed > self.warning_cfg.get("altitude_alarm_airspeed", 0)
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
        if abs(self.telemetry_roll) > self.warning_cfg.get("roll_angle", 0):
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

    def handle_telemetry(self, packet_type, *values) -> None:
        """Receive decoded telemetry from ``CRSFPacketProcessor`` and cache it."""
        self.packet_count += 1
        if packet_type == "attitude":
            pitch, roll, yaw = values
            self.telemetry_pitch = pitch
            self.telemetry_roll = roll
            self.telemetry_yaw = yaw
            self.pitch_data.append(pitch)
            self.roll_data.append(roll)
            self.yaw_data.append(yaw)
        elif packet_type == "gps":
            _lat, _lon, speed, _course, alt, _sats = values
            self.current_airspeed = speed
            self.current_altitude = alt
            self.airspeed_data.append(speed)
            self.altitude_data.append(alt)
        elif packet_type == "link_stats":
            (
                rssi_a,
                rssi_b,
                link_quality,
                snr,
                downlink_lq,
                downlink_snr,
            ) = values
            self.rssi_a_data.append(rssi_a)
            self.rssi_b_data.append(rssi_b)
            self.link_quality_data.append(link_quality)
            self.downlink_quality_data.append(downlink_lq)
            self.snr_data.append(snr)
            self.downlink_snr_data.append(downlink_snr)
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

        # Schedule a label/OSD refresh on the GUI thread. This ensures that
        # updates triggered by telemetry packets do not block the interface.
        self.check_warnings()
        QMetaObject.invokeMethod(self, "update_labels", Qt.QueuedConnection)

    def transmit_data(self):
        """
        Transmit CRSF packets using mapped joystick values.
        """
        if not self.joystick or not self.crsf_processor:
            return

        try:
            mapped_roll, mapped_pitch = self.joystick.get_mapped_values()
            channels = [1500] * 16
            channels[0] = int(mapped_roll)
            channels[1] = int(mapped_pitch)
            self.crsf_processor.update_and_send_packet(channels)
        except Exception as e:
            print(f"Error during transmission: {e}")

    def setup_configuration_page(self):
        """Create configuration page for selecting settings."""
        self.ui.configuration_page = QWidget()
        widgets.configuration_page = self.ui.configuration_page
        widgets.stackedWidget.addWidget(self.ui.configuration_page)

        layout = QVBoxLayout(self.ui.configuration_page)
        ports = ["Not connected"] + [p.device for p in list_ports.comports()]

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
        rf_layout.addWidget(
            QLabel("ELRS transmitting at 300 Hz at 1:2 telemetry ratio")
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
        add_separator()

        # VTX settings
        vtx_layout, self.vtx_status = add_section("VTX System Settings")
        vtx_port_row = QHBoxLayout()
        vtx_port_row.addWidget(QLabel("Port"))
        self.video_port_combo = QComboBox()
        self.video_port_combo.addItems(ports)
        vtx_port_row.addWidget(self.video_port_combo)
        vtx_layout.addLayout(vtx_port_row)

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
        self.video_port_combo.setCurrentText(
            self.vtx_cfg.get("port", "Not connected")
        )

        # Connect signals
        self.control_port_combo.currentTextChanged.connect(self.on_control_port_selected)
        self.video_port_combo.currentTextChanged.connect(self.on_video_port_selected)
        self.elrs_port_combo.currentTextChanged.connect(self.on_elrs_port_selected)
        self.packet_interval_edit.editingFinished.connect(self.on_packet_interval_changed)
        self.deadzone_slider.valueChanged.connect(self.on_deadzone_changed)
        self.sensitivity_slider.valueChanged.connect(self.on_sensitivity_changed)
        self.stall_speed_slider.valueChanged.connect(self.on_stall_speed_changed)
        self.stall_alt_slider.valueChanged.connect(self.on_stall_alt_changed)
        self.alt_alarm_alt_slider.valueChanged.connect(self.on_alt_alarm_alt_changed)
        self.alt_alarm_speed_slider.valueChanged.connect(self.on_alt_alarm_speed_changed)
        self.roll_angle_slider.valueChanged.connect(self.on_roll_angle_changed)

        # Initial connection status
        self.update_connection_status(self.control_status, self.joystick is not None)
        self.update_connection_status(self.rf_status, self.crsf_processor is not None)
        self.update_connection_status(
            self.vtx_status, validate_port("VTX", self.video_port_combo.currentText())
        )
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
        refresh(self.video_port_combo, self.on_video_port_selected)

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
        # Always stop the current feed before switching
        self.video_feed.stop()
        if valid:
            self.video_feed.start()
        else:
            self.ui.VideoLabel.setText("Not connected")
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
                )
                self.crsf_processor.telemetry_ready.connect(
                    lambda data: self.handle_telemetry(*data)
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
