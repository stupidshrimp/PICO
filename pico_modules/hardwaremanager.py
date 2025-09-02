from PyQt5.QtCore import QTimer
from pico_modules.pico_videofeed import VideoFeed
from pico_modules.pico_joystick2state import JoystickRateHandler
from pico_modules.pico_transmitpackets import CRSFPacketProcessor
from serial.tools import list_ports


class HardwareManager:
    def __init__(self, ui):
        """
        Initialize HardwareManager with UI reference for label updates.
        """
        self.ui = ui

        # Define COM ports and baud rates for each device
        self.configs = {
            "video_feed": {"port": "COM5", "baudrate": 9600},
            "joystick": {"port": "COM14", "baudrate": 9600},
            "crsf": {"port": "COM3", "baudrate": 921600}
        }

        # Initialize hardware components
        self.video_feed = VideoFeed(self.ui.VideoLabel)
        self.joystick = JoystickRateHandler(
            port=self.configs["joystick"]["port"],
            baudrate=self.configs["joystick"]["baudrate"],
            update_interval=0.01,
            roll_rate_max=100,
            pitch_rate_max=100,
            roll_sensitivity=0.3,
            pitch_sensitivity=0.3,
            roll_exponent=1.5,
            pitch_exponent=1.5,
        )
        self.crsf_processor = CRSFPacketProcessor(
            port=self.configs["crsf"]["port"],
            baudrate=self.configs["crsf"]["baudrate"]
        )

        # Timer to check hardware connections
        self.check_timer = QTimer()
        self.check_timer.timeout.connect(self.check_connections)
        self.check_timer.start(1000)  # Check every second

    def check_connections(self):
        """
        Check and manage connections to all hardware components.
        """
        for name, config in self.configs.items():
            if not self.is_port_connected(config["port"]):
                self.ui.transmitstatus1.setText(f"{name} Disconnected")
                print(f"{name} on {config['port']} disconnected. Attempting to reconnect...")
                self.reconnect_hardware(name, config)
            else:
                self.ui.transmitstatus1.setText(f"{name} Connected")
                print(f"{name} on {config['port']} connected.")

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
            if name == "video_feed":
                self.video_feed.start()
            elif name == "joystick":
                self.joystick.connect_serial()
            elif name == "crsf":
                self.crsf_processor.connect_serial()
            print(f"Reconnected to {name} on {config['port']}")
        except Exception as e:
            print(f"Failed to reconnect {name}: {e}")

    def close_all(self):
        """
        Cleanly close all hardware connections.
        """
        self.video_feed.stop()
        self.joystick.close_serial()
        self.crsf_processor.close_serial()
        print("All hardware connections closed.")
