from PySide6.QtCore import QTimer
from pico_modules.pico_videofeed import VideoFeed
from pico_modules.pico_joystick2state import JoystickRateHandler
from pico_modules.pico_transmitpackets import CRSFPacketProcessor
from serial.tools import list_ports

from config import load_config


def validate_port(name: str, port: str) -> bool:
    available = [p.device for p in list_ports.comports()]
    if port not in available:
        print(
            f"Warning: {name} port '{port}' not found. Available ports: {', '.join(available) or 'None'}"
        )
        return False
    return True


class HardwareManager:
    def __init__(self, ui):
        """
        Initialize HardwareManager with UI reference for label updates.
        """
        self.ui = ui

        # Load configuration
        cfg = load_config()
        self.configs = {
            "joystick": cfg.get("joystick", {}),
            "crsf": cfg.get("crsf", {}),
        }

        # Initialize hardware components
        self.video_feed = VideoFeed(self.ui.VideoLabel)

        self.joystick = None
        joy_cfg = self.configs["joystick"]
        if validate_port("joystick", joy_cfg.get("port")):
            try:
                self.joystick = JoystickRateHandler(
                    port=joy_cfg.get("port"),
                    baudrate=joy_cfg.get("baudrate"),
                    update_interval=0.01,
                    roll_rate_max=100,
                    pitch_rate_max=100,
                    roll_sensitivity=0.3,
                    pitch_sensitivity=0.3,
                    roll_exponent=1.5,
                    pitch_exponent=1.5,
                )
            except Exception as exc:
                print(f"Failed to initialize joystick: {exc}")
        else:
            print("Joystick disabled due to unavailable port.")

        self.crsf_processor = None
        crsf_cfg = self.configs["crsf"]
        if validate_port("CRSF", crsf_cfg.get("port")):
            try:
                self.crsf_processor = CRSFPacketProcessor(
                    port=crsf_cfg.get("port"),
                    baudrate=crsf_cfg.get("baudrate"),
                )
            except Exception as exc:
                print(f"Failed to initialize CRSF processor: {exc}")
        else:
            print("CRSF disabled due to unavailable port.")

        # Timer to check hardware connections
        self.check_timer = QTimer()
        self.check_timer.timeout.connect(self.check_connections)
        self.check_timer.start(1000)  # Check every second

    def check_connections(self):
        """
        Check and manage connections to all hardware components.
        """
        for name, config in self.configs.items():
            port = config.get("port")
            if not port:
                continue
            if not self.is_port_connected(port):
                self.ui.transmitstatus1.setText(f"{name} Disconnected")
                print(f"{name} on {port} disconnected. Attempting to reconnect...")
                self.reconnect_hardware(name, config)
            else:
                self.ui.transmitstatus1.setText(f"{name} Connected")
                print(f"{name} on {port} connected.")

    def is_port_connected(self, port):
        """
        Check if a specific COM port is connected.
        """
        available_ports = [p.device for p in list_ports.comports()]
        return port in available_ports

    def reconnect_hardware(self, name, config):
        """
        Attempt to reconnect a specific hardware component.
        """
        try:
            if name == "joystick" and self.joystick and hasattr(self.joystick, "connect_serial"):
                self.joystick.connect_serial()
            elif name == "crsf" and self.crsf_processor:
                self.crsf_processor.connect_serial()
            print(f"Reconnected to {name} on {config['port']}")
        except Exception as e:
            print(f"Failed to reconnect {name}: {e}")

    def close_all(self):
        """
        Cleanly close all hardware connections.
        """
        self.video_feed.stop()
        if self.joystick and hasattr(self.joystick, "close_serial"):
            self.joystick.close_serial()
        if self.crsf_processor and hasattr(self.crsf_processor, "close_serial"):
            self.crsf_processor.close_serial()
        print("All hardware connections closed.")
