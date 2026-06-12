"""Documentation page providing detailed operational guides."""

from __future__ import annotations

from typing import Callable, Dict

from PySide6.QtCore import Qt
from PySide6.QtGui import QCursor
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


class DocumentationPage:
    """Create a documentation tab with richly formatted guidance."""

    def __init__(self, main_window) -> None:  # type: ignore[no-untyped-def]
        self._main_window = main_window
        self._ui = main_window.ui

        self._create_navigation_button()
        self._build_page()

    # ------------------------------------------------------------------
    # Navigation button setup
    # ------------------------------------------------------------------
    def _create_navigation_button(self) -> None:
        button = QPushButton(self._ui.topMenu)
        button.setObjectName("btn_documentation")
        button.setSizePolicy(self._ui.btn_home.sizePolicy())
        button.setMinimumSize(self._ui.btn_home.minimumSize())
        button.setFont(self._ui.btn_home.font())
        button.setCursor(QCursor(Qt.PointingHandCursor))
        button.setLayoutDirection(Qt.LeftToRight)
        button.setStyleSheet(
            "background-image: url(:/icons/images/icons/cil-library.png);"
        )
        button.setText("Documentation")
        self._ui.verticalLayout_8.addWidget(button)
        self._ui.btn_documentation = button

    # ------------------------------------------------------------------
    # Page construction
    # ------------------------------------------------------------------
    def _build_page(self) -> None:
        page = QWidget()
        page.setObjectName("documentation_page")

        outer_layout = QVBoxLayout(page)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        tab_widget = QTabWidget()
        tab_widget.setObjectName("documentation_tabs")
        tab_widget.setTabPosition(QTabWidget.North)
        tab_widget.setDocumentMode(True)
        tab_widget.setElideMode(Qt.ElideNone)
        outer_layout.addWidget(tab_widget)

        self._tab_widget = tab_widget

        self._add_tab("GCS", self._populate_gcs_tab)
        self._add_tab("Safety", self._populate_safety_tab)
        self._add_tab("ELRS", self._populate_elrs_tab)
        self._add_tab("Electronics", self._populate_electronics_tab)
        self._add_tab("Airframes", self._populate_airframe_tab)

        self._ui.documentation_page = page
        self._ui.stackedWidget.addWidget(page)

    # ------------------------------------------------------------------
    # Section builders
    # ------------------------------------------------------------------
    def _populate_gcs_tab(self, layout: QVBoxLayout) -> None:
        frame, frame_layout = self._create_section("Ground Control Station (GCS)")

        overview = (
            "The Ground Control Station is the orchestration hub that stitches together "
            "video, command, telemetry, and mission management into a single operator "
            "workspace. The left navigation column exposes role-based tabs—Home for "
            "status at a glance, Configuration for hardware bindings, Command for live "
            "flight operations, and specialist tools such as the pre-flight checklist, "
            "telemetry analytics, sorties review, debug feeds, and this documentation."
        )
        self._add_paragraph(frame_layout, overview)

        gui_structure = (
            "The Command tab dominates the layout during flight. The central video canvas "
            "hosts the real-time downlink, onto which we overlay roll/pitch, compass, "
            "airspeed, altitude, and throttle indicators. Telemetry-driven widgets are "
            "instantiated once at startup and bound to live data frames so the GUI can "
            "update without reallocating controls. The right-hand command sidebar is a "
            "stack of themed panels—signal health, battery diagnostics, autopilot cues, "
            "and auxiliary mission widgets—that reuse a shared style sheet for visual "
            "consistency."
        )
        self._add_paragraph(frame_layout, gui_structure)

        integrations = (
            "Below the video canvas a control console presents operator inputs, "
            "autopilot advisories, and telemetry state summaries. Each element is wired "
            "to the application's dispatcher: joystick updates feed the command mixer, "
            "telemetry packets are funneled through the parser to update statistics, and "
            "map updates post JavaScript commands to the embedded WebView. The map panel "
            "subscribes to the same navigation topic to keep the aircraft icon and GPS "
            "lock indicators synchronized."
        )
        self._add_paragraph(frame_layout, integrations)

        self._add_subsection(
            frame_layout,
            "Major interface regions",
            (
                "Navigation rail: collapsible sidebar with iconography to minimise mouse travel during mission ops.",
                "Command workspace: live video composited with OSD widgets and a flight control dashboard.",
                "Mission sidebar: scrollable column of health panels, alerts, and automation controls generated at runtime.",
                "Analytics pages: dedicated stacked widgets for telemetry graphs, sortie playback, and debug stream inspection.",
            ),
        )

        workflow = (
            "System state is persisted in JSON configuration profiles so operators can "
            "boot into known-good settings. When telemetry streams connect, the main "
            "window toggles the sortie recorder, recalculates battery warnings based on "
            "aircraft metadata, and begins pushing navigation solutions to the embedded "
            "map. When telemetry is lost, graceful degradation routines blank outdated "
            "data, preserve the last known fix for situational awareness, and present "
            "clear tooltips explaining the required recovery actions."
        )
        self._add_paragraph(frame_layout, workflow)

        layout.addWidget(frame)

    def _populate_safety_tab(self, layout: QVBoxLayout) -> None:
        frame, frame_layout = self._create_section(
            "Technical Safety Implementations"
        )

        overview = (
            "Feather uses layered safety defenses instead of relying on any single "
            "component. The Ground Control Station validates operator readiness, "
            "keeps command data fresh, limits the requested flight envelope, and "
            "annunciates hazardous telemetry. The flight controller independently "
            "enforces packet-age failsafes, neutral/throttle-cut outputs, bounded "
            "control loops, sensor rejection, and bench-only startup halts. These "
            "layers are intentionally redundant so a GUI stall, stale telemetry "
            "sample, missing sensor, or bad control mode cannot silently become a "
            "persistent unsafe command."
        )
        self._add_paragraph(frame_layout, overview)

        self._add_subsection(
            frame_layout,
            "Ground-station readiness checks",
            (
                "Pre-flight verification checks that the ELRS/CRSF serial port exists, "
                "the joystick/control port is available or consciously substituted, "
                "channel transmission is active, and throttle is at idle before launch.",
                "Signal readiness requires fresh link-statistics telemetry with uplink "
                "and downlink link quality at or above 60% and SNR at or above 5 dB, "
                "which catches weak RF conditions before control authority is needed.",
                "Attitude, GPS, and battery streams must be fresh and within sane "
                "numeric ranges; GPS latitude/longitude, altitude, airspeed, satellite "
                "count, and battery charge checks prevent the UI from treating stale or "
                "nonsensical telemetry as a valid flight state.",
                "The video-frame check warns when the VTX path has not produced an image "
                "so the operator does not launch without situational awareness.",
            ),
        )

        self._add_subsection(
            frame_layout,
            "Command freshness and CRSF transmit watchdogs",
            (
                "The CRSF worker runs a high-resolution pacer so RC frames keep a stable "
                "100/250/500 Hz cadence without depending on the GUI event loop.",
                "Only one serial write may be queued at a time; if writes are slow, ticks "
                "are coalesced instead of accumulating a backlog of old RC packets.",
                "Every channel update is timestamped. If the GUI/control pipeline stops "
                "refreshing channels for the configured stale timeout, the worker stops "
                "transmitting instead of replaying the last command forever.",
                "Stopping transmission deliberately lets the flight controller's faster "
                "packet-age failsafe take over, blending surfaces toward neutral and "
                "cutting throttle from firmware that is closest to the servos.",
            ),
        )

        self._add_subsection(
            frame_layout,
            "Authority limits and mode isolation",
            (
                "Fly-By-Wire roll and pitch requests are clamped in the GCS to operator "
                "limits and then scaled against the flight controller's redundant "
                "80-degree firmware envelope.",
                "The flight controller also constrains final servo outputs to the 1000-"
                "2000 us range, so PID overshoot or malformed channel values cannot "
                "command beyond calibrated actuator travel.",
                "Manual mode is a direct RC-to-servo path and explicitly resets FBW PID "
                "and attitude-filter state so old attitude corrections cannot bleed into "
                "manual aileron or elevator commands.",
                "Mode switches use separate AUX channels with guard bands, reducing "
                "chatter around thresholds and keeping ELRS arming independent from "
                "control-mode and throttle-mode selection.",
            ),
        )

        self._add_subsection(
            frame_layout,
            "Flight-controller failsafes",
            (
                "The firmware treats RC packets older than 250 ms as stale. On a stale "
                "link it resets roll, pitch, and throttle controllers, forces Manual and "
                "Manual Throttle modes, clears auto-throttle percentage, and cuts "
                "throttle.",
                "A short servo-hold blend window smooths the transition from the last "
                "known surface position toward neutral, but it is deliberately brief so "
                "high-deflection commands are not preserved after the link stops decoding.",
                "Throttle defaults to cut whenever RC is stale; auto throttle is allowed "
                "only with fresh RC and fresh airspeed, and stale airspeed causes a "
                "controlled throttle decay instead of an open-loop power increase.",
                "Servo writes are hysteresis-limited and force-refreshed on a schedule to "
                "reduce ISR load while still keeping actuators synchronized with the "
                "latest safe command.",
            ),
        )

        self._add_subsection(
            frame_layout,
            "Bounded controllers and sensor validation",
            (
                "Roll, pitch, and auto-throttle PID controllers clamp their outputs and "
                "integrators, preventing wind-up and limiting how fast closed-loop logic "
                "can change servo or throttle commands.",
                "Airspeed-hold output is interpreted as percent per second and integrated "
                "into a 0-100% throttle command, making throttle changes rate-bounded and "
                "auditable.",
                "The EKF rejects accelerometer measurements that do not look like gravity "
                "and magnetometer measurements that fail vector or innovation gates, then "
                "uses predicted values with high measurement noise instead of injecting "
                "bad sensor data into attitude estimates.",
                "Repeated EKF update failures reset the filter back to a normalized "
                "quaternion state, preserving a recoverable attitude estimate rather than "
                "continuing with diverged state.",
            ),
        )

        self._add_subsection(
            frame_layout,
            "Operator warnings and debounced alarms",
            (
                "Configurable alarms monitor low airspeed at altitude, low altitude at "
                "speed, excessive bank angle, and high sink rate; each can be disabled "
                "individually while the master alarm switch controls all warnings.",
                "Airspeed and altitude alerts arm only when the aircraft is considered "
                "airborne and not already in the landing debounce, preventing nuisance "
                "alarms during ground handling or normal touchdown.",
                "Each warning condition must persist for more than one second before "
                "audio plays, reducing false positives from single noisy telemetry frames.",
                "Sink rate is estimated over a sliding altitude/time window and expires "
                "when source data is stale, so bursty GPS delivery cannot create an "
                "artificial descent-rate emergency.",
            ),
        )

        self._add_subsection(
            frame_layout,
            "Bench-only diagnostics and safe startup",
            (
                "Magnetometer calibration and GPS diagnostic modes intentionally block "
                "normal flight startup after collecting bench data, preventing a debug "
                "build from being flown accidentally.",
                "If the IMU fails initialization, the firmware halts startup with neutral "
                "servos instead of proceeding with invalid attitude data.",
                "GPS, barometer, airspeed, and telemetry tasks run on independent timers "
                "so a slow low-rate sensor cannot block the high-rate attitude and servo "
                "control loops.",
            ),
        )

        closing = (
            "Together, these implementations create a fail-operational monitoring "
            "experience for the operator and a fail-safe actuation path in firmware. "
            "The GCS tries to prevent unsafe launch conditions; if the desktop, link, "
            "or telemetry pipeline fails anyway, the flight controller transitions the "
            "aircraft toward neutral controls and throttle cut using its own local "
            "timers and sensor validity checks."
        )
        self._add_paragraph(frame_layout, closing)

        layout.addWidget(frame)

    def _populate_elrs_tab(self, layout: QVBoxLayout) -> None:
        frame, frame_layout = self._create_section(
            "ExpressLRS (ELRS) Long Range System Implementation"
        )

        pipeline = (
            "Our control link builds on the CRSF packet format provided by ExpressLRS. "
            "Transmitter inputs are sampled at 250 Hz, filtered for dead-band removal, "
            "and quantized into 11-bit channel values. The encoder packs these channels "
            "into the CRSF payload structure, appends link statistics, and computes the "
            "frame CRC prior to modulation. All timings honour the ELRS dynamic rate "
            "scheduler so that command latency remains under 10 ms when link conditions "
            "permit."
        )
        self._add_paragraph(frame_layout, pipeline)

        telemetry_flow = (
            "Downlink telemetry is demultiplexed on arrival. The radio module feeds raw "
            "CRSF frames into the parser, which validates headers, checks CRC integrity, "
            "and routes payload types to dedicated handlers. Flight dynamics packets "
            "update the HUD overlays, receiver health fields drive the signal metrics "
            "panel, and GPS data propagates to the mission map. Each packet is timestamped "
            "so stale data can be culled before it reaches the GUI refresh loop."
        )
        self._add_paragraph(frame_layout, telemetry_flow)

        reliability = (
            "Encoding and decoding stages share a buffer pool to avoid heap churn. The "
            "uplink side implements sequence counters and watchdogs that flag skipped "
            "frames, while the downlink maintains moving averages for RSSI, link "
            "quality, and SNR to populate the command sidebar. Failsafe thresholds are "
            "exposed in configuration, letting operators tune when the autopilot should "
            "transition to Return-to-Home or loiter modes."
        )
        self._add_paragraph(frame_layout, reliability)

        arduino_integration = (
            "For embedded nodes we lean on the open-source CRSF for Arduino library to "
            "bridge ExpressLRS receivers with custom firmware. The sketch-facing class "
            "`CRSFforArduino` wraps the low-level serial parser so mission logic can stay "
            "focused on reacting to stick inputs and pushing telemetry upstream."
        )
        self._add_paragraph(frame_layout, arduino_integration)

        self._add_subsection(
            frame_layout,
            "Integrating CRSF for Arduino",
            (
                "Instantiate a `CRSFforArduino` object, call `begin()` to initialise the shared UART, "
                "and bail out if the port is unavailable.",
                "Invoke `update()` every loop to drain the 420 kbaud serial stream, decode CRSF frames, "
                "and service telemetry slots without overflowing the RX buffer.",
                "Register callbacks such as `setRcChannelsCallback()` or `setLinkStatisticsCallback()` to "
                "receive decoded RC channels, link state transitions, and telemetry metrics as soon as new "
                "frames arrive.",
                "Map flight modes by declaring channel ranges with `setFlightMode()` and use helpers like "
                "`rcToUs()` or `getChannel()` for microsecond-style control logic.",
                "Leverage telemetry writers (battery, GPS, status text) to queue outbound frames; the "
                "library's scheduler sequences them alongside ExpressLRS telemetry windows automatically.",
            ),
        )

        self._add_subsection(
            frame_layout,
            "Encoding and decoding checkpoints",
            (
                "Input shaping → channel scaling → CRSF serialization → CRC append → RF module (uplink)",
                "RF module → frame buffer → header validation → CRC verification → payload dispatch (downlink)",
                "Telemetry aggregator → GUI bindings → persistent sortie recorder", 
            ),
        )

        closing = (
            "ExpressLRS integration allows us to run unified control and telemetry over a "
            "single RF link. By conforming to the CRSF binary schema and leveraging the "
            "high update rates, the Ground Control Station receives richer telemetry for "
            "situational awareness while maintaining sub-10 ms control latency even at "
            "long ranges."
        )
        self._add_paragraph(frame_layout, closing)

        layout.addWidget(frame)

    def _populate_electronics_tab(self, layout: QVBoxLayout) -> None:
        frame, frame_layout = self._create_section("Electronics")
        frame_layout.addSpacing(4)
        layout.addWidget(frame)

    def _populate_airframe_tab(self, layout: QVBoxLayout) -> None:
        frame, frame_layout = self._create_section("Airframe Library")

        selector_row = QHBoxLayout()
        selector_row.setSpacing(10)
        selector_row.addWidget(QLabel("Select airframe:"))

        selector_row.addItem(
            QSpacerItem(10, 10, QSizePolicy.Expanding, QSizePolicy.Minimum)
        )

        self._airframe_selector = QComboBox()
        self._airframe_selector.addItems(
            (
                "Aeroscout Test Vehicle",
                "Ti-K 3 Arctic Tern",
            )
        )
        self._airframe_selector.currentTextChanged.connect(
            self._update_airframe_details
        )
        self._airframe_selector.setCursor(QCursor(Qt.PointingHandCursor))
        selector_row.addWidget(self._airframe_selector, 2)

        frame_layout.addLayout(selector_row)

        self._airframe_details_label = QLabel()
        self._airframe_details_label.setWordWrap(True)
        self._airframe_details_label.setTextFormat(Qt.RichText)
        self._airframe_details_label.setStyleSheet(
            "font-size: 12pt; line-height: 1.4em;"
        )
        self._airframe_details_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        frame_layout.addWidget(self._airframe_details_label)

        self._airframe_descriptions: Dict[str, str] = {
            "Aeroscout Test Vehicle": self._build_aeroscout_text(),
            "Ti-K 3 Arctic Tern": self._build_arctic_tern_text(),
        }
        self._update_airframe_details(self._airframe_selector.currentText())

        layout.addWidget(frame)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _add_tab(self, title: str, builder: Callable[[QVBoxLayout], None]) -> None:
        tab = QWidget()
        tab_layout = QVBoxLayout(tab)
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.setSpacing(0)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        tab_layout.addWidget(scroll_area)

        container = QWidget()
        scroll_area.setWidget(container)

        content_layout = QVBoxLayout(container)
        content_layout.setContentsMargins(20, 20, 20, 20)
        content_layout.setSpacing(20)

        builder(content_layout)

        content_layout.addStretch(1)

        self._tab_widget.addTab(tab, title)

    def _create_section(self, title: str) -> tuple[QFrame, QVBoxLayout]:
        frame = QFrame()
        frame.setStyleSheet(
            "background-color: rgba(33, 37, 43, 200);"
            "border: 1px solid rgb(62, 68, 82);"
            "border-radius: 10px;"
        )
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        header = QLabel(title)
        header.setStyleSheet("font-size: 18px; font-weight: bold; color: white;")
        header.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(header)

        return frame, layout

    def _add_paragraph(self, layout: QVBoxLayout, text: str) -> None:
        label = QLabel(text)
        label.setWordWrap(True)
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        label.setStyleSheet("font-size: 12pt; line-height: 1.4em;")
        layout.addWidget(label)

    def _add_subsection(self, layout: QVBoxLayout, title: str, bullets: tuple[str, ...]) -> None:
        subtitle = QLabel(title)
        subtitle.setStyleSheet("font-size: 14pt; font-weight: bold;")
        subtitle.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(subtitle)

        bullet_label = QLabel(
            "<ul>" + "".join(f"<li>{item}</li>" for item in bullets) + "</ul>"
        )
        bullet_label.setTextFormat(Qt.RichText)
        bullet_label.setWordWrap(True)
        bullet_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        bullet_label.setStyleSheet("font-size: 12pt; line-height: 1.4em;")
        layout.addWidget(bullet_label)

    def _update_airframe_details(self, airframe: str) -> None:
        text = self._airframe_descriptions.get(airframe, "")
        self._airframe_details_label.setText(text)

    def _build_aeroscout_text(self) -> str:
        return (
            "<p><b>Role:</b> Modular test-bed used for rapid avionics integration and "
            "software regression flights.</p>"
            "<p>The Aeroscout platform retains its high-lift trainer wing but is "
            "outfitted with reinforced servo trays, quick-disconnect payload rails, and "
            "a removable avionics deck. The Ground Control Station maps every control "
            "surface to a dedicated ELRS channel so firmware builds can exercise new "
            "mixing logic without rewiring.</p>"
            "<p><b>Configuration highlights</b></p>"
            "<ul>"
            "<li>Standard CRSF uplink with four primary control channels and two "
            "auxiliary toggles reserved for experiment payloads.</li>"
            "<li>Telemetry suite includes pitot-static airspeed, barometric altitude, and "
            "dual GPS receivers for comparative testing. Both feeds are logged and "
            "tagged inside the sorties view.</li>"
            "<li>Power system monitored with hall-effect current sensing; thresholds "
            "propagate to the command sidebar battery panel.</li>"
            "</ul>"
            "<p><b>Operational notes:</b> Use this airframe when trialling new autopilot "
            "features or validating hardware revisions. The documentation above on the "
            "GCS and ELRS implementation maps directly to this configuration.</p>"
        )

    def _build_arctic_tern_text(self) -> str:
        return (
            "<p><b>Role:</b> Long-endurance mission airframe optimised for cold-weather "
            "operations and extended telemetry ranges.</p>"
            "<p>The Ti-K 3 Arctic Tern pairs a high aspect-ratio wing with an insulated "
            "electronics bay to maintain battery temperature and radio performance. The "
            "GCS command sidebar enables bespoke widgets for polar sorties, including "
            "battery preheat monitoring and GPS dilution of precision tracking.</p>"
            "<p><b>Configuration highlights</b></p>"
            "<ul>"
            "<li>ExpressLRS uplink pinned at lower refresh rates during cruise to "
            "increase link budget; the encoder automatically advertises the negotiated "
            "rate to the HUD.</li>"
            "<li>Dual redundant flight controllers share telemetry over the ELRS "
            "downlink. The data aggregator merges health packets so the operator sees a "
            "single set of status gauges.</li>"
            "<li>Heated battery sled and de-icing bus reported through the electronics "
            "telemetry channels and surfaced in the battery diagnostics panel.</li>"
            "</ul>"
            "<p><b>Operational notes:</b> Select this profile when mission planning for "
            "Arctic deployments. The configuration preloads navigation and warning "
            "thresholds tuned for low-temperature flight envelopes.</p>"
        )

