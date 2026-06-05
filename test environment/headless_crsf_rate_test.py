#!/usr/bin/env python3
"""Headless CRSF/ELRS rate test without the ground-station GUI.

This script sends a fixed RC channel packet stream through the same
``CRSFPacketProcessor`` used by the application and prints terminal diagnostics
for outbound control writes and inbound attitude telemetry frequency.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path
from typing import Any

from PySide6.QtCore import QCoreApplication, QMetaObject, QTimer, Qt
from PySide6.QtSerialPort import QSerialPortInfo

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pico_modules.pico_transmitpackets import (  # noqa: E402
    CRSF_CHANNEL_CENTER,
    CRSF_CHANNEL_COUNT,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MIN,
    CRSFPacketProcessor,
)


DEFAULT_CONFIG = REPO_ROOT / "config.json"


def _load_crsf_config(config_path: Path) -> dict[str, Any]:
    if not config_path.exists():
        return {}
    with config_path.open("r", encoding="utf-8") as config_file:
        return json.load(config_file).get("crsf", {})


def _normalised_axis_to_crsf(value: float) -> int:
    """Map a -1.0..1.0 roll/pitch/yaw command to the CRSF channel range."""

    clamped = max(-1.0, min(1.0, float(value)))
    return int(
        round(
            (clamped + 1.0)
            * 0.5
            * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)
            + CRSF_CHANNEL_MIN
        )
    )


def _percent_to_crsf(value: float) -> int:
    """Map a 0..100 percent throttle command to the CRSF channel range."""

    clamped = max(0.0, min(100.0, float(value)))
    return int(
        round(
            (clamped / 100.0) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)
            + CRSF_CHANNEL_MIN
        )
    )


def _build_static_channels(args: argparse.Namespace) -> list[int]:
    """Build the fixed channel set used for the entire test run."""

    channels = [CRSF_CHANNEL_CENTER] * CRSF_CHANNEL_COUNT
    channels[0] = _normalised_axis_to_crsf(args.roll)
    channels[1] = _normalised_axis_to_crsf(args.pitch)
    channels[2] = _percent_to_crsf(args.throttle_percent)
    channels[3] = _normalised_axis_to_crsf(args.yaw)
    channels[args.mode_channel - 1] = args.mode_value
    return channels


class HeadlessRateMonitor:
    """Collect and print TX/RX rate diagnostics from ``CRSFPacketProcessor``."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.channels = _build_static_channels(args)
        self.started_at = time.perf_counter()
        self.window_started_at = self.started_at
        self.attitude_count = 0
        self.frame_count = 0
        self.latest_attitude: tuple[float, float, float] | None = None
        self.latest_tx_stats: dict[str, Any] = {}
        self.latest_link_stats: dict[str, Any] = {}

        self.processor = CRSFPacketProcessor(
            port=args.port,
            baudrate=args.baudrate,
            channels=self.channels,
            packet_interval_ms=args.packet_interval_ms,
            transmission_enabled=True,
            raw_serial_debug_enabled=args.raw_serial,
        )
        self.processor.telemetry_ready.connect(self._on_telemetry)
        self.processor.transmit_debug_update.connect(self._on_tx_debug)
        self.processor.link_diagnostics_update.connect(self._on_link_debug)
        self.processor.serial_data.connect(self._on_raw_serial)
        self.processor.error.connect(self._on_error)
        self.processor.diagnostic_enabled_update.emit(True)

        self.print_timer = QTimer()
        self.print_timer.setInterval(args.print_interval_ms)
        self.print_timer.timeout.connect(self._print_window)
        self.print_timer.start()

        QTimer.singleShot(250, self._start_static_transmission)
        if args.duration_s > 0:
            QTimer.singleShot(int(args.duration_s * 1000), self.stop)

    def _start_static_transmission(self) -> None:
        self.processor.transmission_start_update.emit(self.channels)
        print("Static control packet stream started (no GUI, no joystick).")
        print(
            "Channels: "
            f"roll ch1={self.channels[0]}, pitch ch2={self.channels[1]}, "
            f"throttle ch3={self.channels[2]}, yaw ch4={self.channels[3]}, "
            f"mode ch{self.args.mode_channel}={self.channels[self.args.mode_channel - 1]}"
        )
        print(
            f"Target TX cadence: {1000.0 / max(1, self.args.packet_interval_ms):.1f} Hz "
            f"({self.args.packet_interval_ms} ms interval); expected ELRS config: "
            f"250 Hz, telemetry ratio 1:2."
        )

    def _on_telemetry(self, packet: object) -> None:
        if not isinstance(packet, tuple) or not packet:
            return
        packet_name = packet[0]
        self.frame_count += 1
        if packet_name == "attitude" and len(packet) >= 4:
            pitch, roll, yaw = packet[1:4]
            self.attitude_count += 1
            self.latest_attitude = (float(pitch), float(roll), float(yaw))

    def _on_tx_debug(self, stats: object) -> None:
        if isinstance(stats, dict):
            self.latest_tx_stats = stats

    def _on_link_debug(self, stats: object) -> None:
        if isinstance(stats, dict):
            self.latest_link_stats = stats

    def _on_raw_serial(self, data: object) -> None:
        if isinstance(data, (bytes, bytearray)):
            print(f"RAW RX {len(data)} bytes: {bytes(data).hex(' ')}")

    def _on_error(self, message: str) -> None:
        print(f"ERROR: {message}", file=sys.stderr)

    def _print_window(self) -> None:
        now = time.perf_counter()
        elapsed = max(now - self.window_started_at, 1e-6)
        total_elapsed = now - self.started_at
        attitude_hz = self.attitude_count / elapsed
        telemetry_hz = self.frame_count / elapsed
        tx = self.latest_tx_stats
        link = self.latest_link_stats
        attitude_text = "latest attitude: none decoded"
        if self.latest_attitude is not None:
            pitch, roll, yaw = self.latest_attitude
            attitude_text = (
                f"latest attitude: pitch={pitch:7.2f} deg, "
                f"roll={roll:7.2f} deg, yaw={yaw:7.2f} deg"
            )

        print(
            f"[{total_elapsed:7.2f}s] "
            f"TX serial_write={tx.get('serial_write_hz', 0.0):7.2f} Hz, "
            f"TX attempts={tx.get('send_attempt_hz', 0.0):7.2f} Hz, "
            f"queued_bytes={tx.get('bytes_to_write', 0)}, "
            f"RX attitude={attitude_hz:7.2f} Hz, "
            f"RX telemetry_events={telemetry_hz:7.2f} Hz, "
            f"decoder_frame_hz={link.get('rx_frame_hz', 0.0):7.2f} Hz, "
            f"crc_err={link.get('rx_crc_error_hz', 0.0):6.2f} Hz, "
            f"dropped_Bps={link.get('rx_dropped_bytes_per_s', 0.0):6.2f}; "
            f"{attitude_text}"
        )
        self.window_started_at = now
        self.attitude_count = 0
        self.frame_count = 0

    def stop(self) -> None:
        self.print_timer.stop()
        self.processor.transmission_enabled_update.emit(False)
        thread = self.processor._thread
        QMetaObject.invokeMethod(self.processor, "close_serial", Qt.BlockingQueuedConnection)
        thread.quit()
        thread.wait(2000)
        QCoreApplication.quit()


def parse_args(argv: list[str]) -> argparse.Namespace:
    config = _load_crsf_config(DEFAULT_CONFIG)
    parser = argparse.ArgumentParser(
        description=(
            "Send static CRSF control packets and print headless TX/RX attitude "
            "rate diagnostics. Roll, pitch, and yaw are normalized -1.0..1.0 "
            "control commands, not telemetry angles."
        )
    )
    parser.add_argument("--port", default=config.get("port", "Not connected"), help="CRSF serial port, e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=int(config.get("baudrate", 921600)), help="CRSF serial baud rate")
    parser.add_argument("--packet-interval-ms", type=int, default=int(config.get("packet_interval", 4)), help="TX packet interval; 4 ms targets 250 Hz")
    parser.add_argument("--roll", type=float, default=0.0, help="Static roll command, normalized -1.0..1.0")
    parser.add_argument("--pitch", type=float, default=0.0, help="Static pitch command, normalized -1.0..1.0")
    parser.add_argument("--yaw", type=float, default=0.0, help="Static yaw command, normalized -1.0..1.0")
    parser.add_argument("--throttle-percent", type=float, default=0.0, help="Static throttle command, 0..100 percent")
    parser.add_argument("--mode-channel", type=int, default=5, help="1-based control-mode channel to set")
    parser.add_argument("--mode-value", type=int, default=400, help="CRSF value for the control-mode channel")
    parser.add_argument("--duration-s", type=float, default=0.0, help="Run duration; 0 means until Ctrl+C")
    parser.add_argument("--print-interval-ms", type=int, default=1000, help="Terminal diagnostics print interval")
    parser.add_argument("--raw-serial", action="store_true", help="Print raw received serial byte chunks as hex")
    parser.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.list_ports:
        for port in QSerialPortInfo.availablePorts():
            print(f"{port.portName()}\t{port.systemLocation()}\t{port.description()}")
        return 0
    if args.port == "Not connected":
        print("ERROR: pass --port or set crsf.port in config.json", file=sys.stderr)
        return 2
    if not 1 <= args.mode_channel <= CRSF_CHANNEL_COUNT:
        print(f"ERROR: --mode-channel must be 1..{CRSF_CHANNEL_COUNT}", file=sys.stderr)
        return 2

    app = QCoreApplication([sys.argv[0], *sys.argv[1:]])
    monitor = HeadlessRateMonitor(args)

    def handle_signal(_signum: int, _frame: object) -> None:
        monitor.stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
