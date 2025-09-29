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

