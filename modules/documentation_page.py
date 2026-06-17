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

        what_it_is = (
            "The Ground Control Station is the desktop program the pilot sits in front of. "
            "It is a single PySide6 (Qt) application built around one main window class, "
            "MainWindow, in main.py. Everything the operator sees and does on the ground "
            "lives here: the live video coming down from the aircraft, the instrument "
            "overlays painted on top of that video, the radio link that carries stick "
            "commands up to the plane, the telemetry that comes back down, the moving map, "
            "the warning sounds, and the flight recorder. The goal of this tab is to "
            "explain, in plain terms, what each of those parts does and how they fit "
            "together, so that someone who has never opened the code can still understand "
            "how a command leaves the joystick and how a number ends up on the screen."
        )
        self._add_paragraph(frame_layout, what_it_is)

        big_picture = (
            "At the highest level the GCS is a hub with many spokes. Independent helper "
            "objects each own one job and run on their own background thread so that no "
            "single slow task can freeze the screen: a video object grabs camera frames, a "
            "joystick object reads the pilot's stick, and a radio object (the CRSF "
            "processor) talks to the ExpressLRS module over a serial cable. The main window "
            "is the conductor. It does not poll these objects in a loop; instead it "
            "connects to their Qt signals, so each object simply announces 'new data is "
            "ready' and the matching handler in the main window runs. This is why the "
            "interface stays responsive even when video stutters or the link drops."
        )
        self._add_paragraph(frame_layout, big_picture)

        self._add_subsection(
            frame_layout,
            "The pages you can switch between",
            (
                "<b>Home</b> &mdash; the default landing page, showing the video canvas and instrument overlays for a quick status glance.",
                "<b>Command</b> &mdash; the main flight page. Live video sits in the centre with instruments painted on top, control widgets below, and a stack of status panels down the right side.",
                "<b>Configuration</b> &mdash; where you pick serial ports, set the radio packet rate, tune joystick deadzone and sensitivity, set Fly-By-Wire angle limits, and choose battery cell count.",
                "<b>Pre-flight</b> &mdash; an interactive checklist (modules/preflight_page.py) covering airframe, power, radio, sensors, environment, and final safety items.",
                "<b>Telemetry Data</b> &mdash; live graphs of the incoming numbers, drawn with PyQtGraph (modules/data_page.py).",
                "<b>Sorties</b> &mdash; review and play back the CSV flight logs the GCS records (modules/sorties_page.py).",
                "<b>Debug</b> &mdash; raw packet inspection and link diagnostics for troubleshooting (modules/debug_page.py).",
                "<b>Documentation</b> &mdash; this page. The left navigation rail switches between all of the above using one QStackedWidget.",
            ),
        )

        video_block = (
            "The video downlink is handled by the VideoFeed class in "
            "pico_modules/pico_videofeed.py. A FrameWorker runs on its own thread and owns "
            "the capture device so that grabbing frames never blocks the GUI. On startup it "
            "probes capture indices (preferring an external USB capture card) until it finds "
            "a working source. Because analog video from a VTX is often interlaced, each "
            "frame is de-interlaced by blending fields together, then scaled to fit the "
            "video panel while keeping its aspect ratio. If the feed drops, a fading 'no "
            "signal' message is animated over the panel instead of leaving a frozen image, "
            "so the operator always knows the video path itself has failed."
        )
        self._add_paragraph(frame_layout, video_block)

        self._add_subsection(
            frame_layout,
            "The instrument overlays (OSD) painted on the video",
            (
                "<b>Attitude / artificial horizon</b> (rollpitch_osd.py): a horizon line and pitch ladder show which way the aircraft is banked and pitched, plus a cyan cue that shows the attitude you are commanding in Fly-By-Wire so you can see commanded-versus-actual at a glance.",
                "<b>Compass / heading tape</b> (compass_osd.py): a scrolling strip across the bottom marked every 5&deg; with N/E/S/W labels, driven by the yaw telemetry.",
                "<b>Airspeed tape</b> (airspeed_osd.py): a vertical scale on the left showing speed in mph with a centred readout box.",
                "<b>Altitude tape</b> (altitude_osd.py): a vertical scale on the right showing altitude in feet with a centred readout box.",
                "Each overlay is a transparent widget that paints itself with QPainter, and each can be turned on or off independently with its own checkbox. All four are refreshed together about 70 times a second (a 14&nbsp;ms timer) so motion looks smooth.",
            ),
        )

        telemetry_block = (
            "Telemetry is the stream of numbers the aircraft sends back: attitude, GPS, "
            "battery, and radio link health. These arrive as CRSF frames on the same serial "
            "link used for control, are decoded on the radio worker thread, and are then "
            "delivered to the main window through the telemetry_ready signal. A single "
            "handler, handle_telemetry, looks at the packet type and routes it to the right "
            "place: attitude values go to the horizon and compass overlays, GPS goes to the "
            "altitude/airspeed tapes and the map, battery goes to the battery panel, and "
            "link statistics go to the signal-health panel. Every value is also copied into "
            "one telemetry-state dictionary, which is the single source of truth that the "
            "flight recorder and the warning system both read from."
        )
        self._add_paragraph(frame_layout, telemetry_block)

        self._add_subsection(
            frame_layout,
            "The status panels down the right side of the Command page",
            (
                "<b>Signal health</b>: a grid of RSSI A/B, link quality, SNR, and the downlink equivalents. Each value is colour-coded &mdash; for example RSSI is green when stronger than about -75&nbsp;dBm and turns yellow, orange, then red as it weakens &mdash; so signal trouble is obvious without reading exact numbers.",
                "<b>Battery</b>: a percentage bar. Percentage is estimated from measured pack voltage against the configured full-charge voltage for the chosen cell count (8.4&nbsp;V for 2S, 12.6&nbsp;V for 3S, 16.8&nbsp;V for 4S, 21.0&nbsp;V for 5S).",
                "<b>Flight status</b>: two indicators &mdash; GROUNDED/AIRBORNE and GPS NO&nbsp;FIX/FIX&nbsp;VALID &mdash; driven by a debounced state machine so a single noisy reading cannot flip them.",
                "<b>Sortie recorder</b>: a Start/Stop button that is greyed out until fresh telemetry is present, so you cannot accidentally record an empty log.",
            ),
        )

        control_block = (
            "Control input starts at the JoystickRawHandler in "
            "pico_modules/pico_joystick2state.py, which reads raw stick values over serial. "
            "Those raw numbers pass through a deadzone (to ignore tiny movements near "
            "centre), a sensitivity multiplier, and a smoothing filter before being mapped "
            "into the radio's channel range. The main window's channel builder then "
            "assembles all sixteen radio channels: roll, pitch, throttle, and yaw on the "
            "first four, an arm switch and the control/throttle mode switches on the "
            "auxiliary channels, and the rest centred. In Fly-By-Wire mode the sticks no "
            "longer move the surfaces directly &mdash; they command a desired bank and pitch "
            "angle, which the cyan overlay cue visualises. Keyboard keys and joystick "
            "buttons also feed in: yaw nudges, throttle presets, and elevator/aileron trim. "
            "The finished channel set is hidden behind the radio link, described on the ELRS "
            "tab."
        )
        self._add_paragraph(frame_layout, control_block)

        map_block = (
            "The moving map is an embedded web view (QWebEngineView) running Leaflet.js "
            "against map tiles stored locally in a map/ folder, so it works without an "
            "internet connection in the field. Python drives it by calling small JavaScript "
            "functions &mdash; one to set the initial view, one to update the aircraft "
            "marker. About once a second the GCS pushes the latest GPS fix to the map, and "
            "only when the fix has actually changed, so it does not waste effort redrawing "
            "the same position. The first valid fix recentres the map on the aircraft."
        )
        self._add_paragraph(frame_layout, map_block)

        persistence_block = (
            "Settings are saved to and loaded from a JSON file by config.py, so the station "
            "always boots into the last known-good configuration. This covers serial ports, "
            "the radio packet rate, joystick tuning, Fly-By-Wire limits, warning thresholds, "
            "airborne-detection timing, the map view, and battery cell count. Flight data is "
            "captured by the sortie recorder, which writes one CSV row per telemetry packet "
            "&mdash; timestamp, packet type, stick positions, attitude, GPS, link, and "
            "battery &mdash; into a dated file under a 'sortie data' folder, flushing to disk "
            "about twice a second so a crash never loses more than the last fraction of a "
            "second. The Sorties page reads these files back for review."
        )
        self._add_paragraph(frame_layout, persistence_block)

        degrade_block = (
            "Finally, the GCS is built to fail gracefully. Background workers report "
            "problems through an error signal instead of crashing the window. When the "
            "attitude or link-statistics streams go quiet for about a second, connection "
            "monitors play a clear audio cue, stop trusting the stale numbers, and keep the "
            "last known GPS position on the map for situational awareness rather than "
            "blanking it. None of this commands the aircraft &mdash; the real safety "
            "behaviour lives in the firmware &mdash; but it makes sure the operator is never "
            "misled by data that is no longer live. The Safety tab covers how the ground and "
            "air sides cooperate."
        )
        self._add_paragraph(frame_layout, degrade_block)

        layout.addWidget(frame)

    def _populate_safety_tab(self, layout: QVBoxLayout) -> None:
        frame, frame_layout = self._create_section(
            "Technical Safety Implementations"
        )

        philosophy = (
            "The single most important idea in this section is that safety does not "
            "depend on any one part working. There are two independent computers in the "
            "loop: the Ground Control Station (the desktop program on the ground) and the "
            "flight controller (the firmware on the aircraft). Each has its own timers, its "
            "own checks, and its own ability to fall back to a safe state. The ground side "
            "tries to stop an unsafe flight before it starts and to keep commands fresh "
            "while flying. The air side assumes the ground side might fail at any instant "
            "and is built to bring the aircraft to neutral controls and throttle cut on its "
            "own. Because the air side is physically closest to the servos and runs on its "
            "own clock, it always has the final say. The rest of this tab walks through "
            "each layer from the ground up."
        )
        self._add_paragraph(frame_layout, philosophy)

        overview = (
            "In short: the GCS validates operator readiness before launch, keeps command "
            "packets fresh while flying, clamps the flight envelope the operator can "
            "request, and sounds alarms on hazardous telemetry. The flight controller "
            "independently enforces packet-age failsafes, neutral and throttle-cut outputs, "
            "bounded control loops, sensor rejection, and bench-only startup halts. These "
            "layers are deliberately redundant so that a frozen GUI, a stale telemetry "
            "sample, a missing sensor, or a bad control mode cannot quietly turn into a "
            "lasting unsafe command."
        )
        self._add_paragraph(frame_layout, overview)

        layer1_intro = (
            "Layer 1 &mdash; before you launch. The pre-flight page (modules/preflight_page.py) "
            "presents a structured checklist grouped into airframe and surfaces, power and "
            "propulsion, electronics and radio, sensors and extras, environment, and a final "
            "safety group. It walks the operator through confirming the things software "
            "cannot see for itself: that the wings are attached, the propeller spins the "
            "right way, the control surfaces move correctly, the ELRS link is up, and a kill "
            "switch is configured. Alongside this human checklist, the live GCS continuously "
            "watches the incoming telemetry, so a weak link or stale stream is visible on "
            "screen before control authority is ever needed."
        )
        self._add_paragraph(frame_layout, layer1_intro)

        self._add_subsection(
            frame_layout,
            "Ground-station readiness checks",
            (
                "The checklist confirms airframe integrity, power and propulsion, antenna "
                "orientation, verified control-surface deflection, GPS lock, telemetry "
                "connectivity, camera/VTX operation, and final items like flight-mode setup "
                "and the throttle kill switch.",
                "While the checklist is being worked through, the GCS monitors live link "
                "statistics (RSSI, link quality, SNR) and colour-codes them, so the operator "
                "can see weak RF conditions rather than relying on a single pass/fail moment.",
                "Attitude, GPS, and battery streams are validated for freshness and sane "
                "numeric ranges before they are trusted, so the UI does not treat a stale or "
                "nonsensical reading as a valid flight state.",
                "Sortie recording is locked out until fresh telemetry is present, ensuring a "
                "flight is never logged against a dead link.",
            ),
        )

        layer2_intro = (
            "Layer 2 &mdash; keeping commands fresh in the air. The radio worker "
            "(pico_modules/pico_transmitpackets.py) does not lean on the GUI's clock to time "
            "RC frames. A dedicated high-resolution pacer thread schedules each frame against "
            "an absolute deadline, so the link holds a steady cadence even if the rest of the "
            "program is busy. Crucially, the worker refuses to keep transmitting commands "
            "that the operator is no longer actively producing."
        )
        self._add_paragraph(frame_layout, layer2_intro)

        self._add_subsection(
            frame_layout,
            "Command freshness and CRSF transmit watchdogs",
            (
                "The CRSF worker runs a high-resolution pacer so RC frames keep a stable "
                "cadence (100, 250, or 500&nbsp;Hz &mdash; 250&nbsp;Hz / one frame every "
                "4&nbsp;ms by default) without depending on the Qt event loop, which on some "
                "systems would otherwise collapse the rate to roughly 60&nbsp;Hz.",
                "Only one serial write may be in flight at a time; if writes are slow, extra "
                "ticks are coalesced (dropped) rather than queued, so a backlog of stale RC "
                "packets can never build up.",
                "Every channel update is timestamped. If the control pipeline stops refreshing "
                "the channels for the stale timeout (RC_CHANNEL_STALE_TIMEOUT_S, 2.0&nbsp;s by "
                "default), the worker <i>stops transmitting</i> instead of replaying the last "
                "command forever.",
                "Stopping transmission is deliberate: it lets the flight controller's much "
                "faster 250&nbsp;ms packet-age failsafe take over and bring the aircraft to "
                "neutral from the firmware that sits closest to the servos.",
            ),
        )

        layer3_intro = (
            "Layer 3 &mdash; limiting how much authority is even on the table. Both sides "
            "clamp the flight envelope. The GCS limits what the operator can ask for in "
            "Fly-By-Wire, and the firmware independently limits what the servos can actually "
            "be driven to, so a software bug or a malformed channel cannot translate into "
            "physical over-travel."
        )
        self._add_paragraph(frame_layout, layer3_intro)

        self._add_subsection(
            frame_layout,
            "Authority limits and mode isolation",
            (
                "In Fly-By-Wire, stick deflection commands a desired bank/pitch angle. The GCS "
                "applies the operator's softer limits (45&deg; roll and 30&deg; pitch by "
                "default) and scales them against the firmware's hard redundant envelope of "
                "80&deg; (FBW_MAX_ROLL_ANGLE_DEG / FBW_MAX_PITCH_ANGLE_DEG).",
                "Regardless of mode, the firmware constrains every final servo output to the "
                "1000&ndash;2000&nbsp;&micro;s range (centre 1500&nbsp;&micro;s), so PID "
                "overshoot or a bad channel value can never command beyond calibrated travel.",
                "Manual mode is a direct stick-to-servo path; switching into it resets the "
                "Fly-By-Wire PID controllers and the attitude filter so old attitude "
                "corrections cannot bleed into manual aileron or elevator commands.",
                "The mode switches live on separate auxiliary channels with guard bands "
                "(Fly-By-Wire engages only above ~1550, well away from centre), and the ELRS "
                "arm switch is a different channel again, keeping arming, control mode, and "
                "throttle mode independent of one another.",
            ),
        )

        layer4_intro = (
            "Layer 4 &mdash; the firmware's own failsafes (flight_controller/Main.ino). This is "
            "the layer that does not trust anything outside the aircraft. It runs on the "
            "flight controller's local clock and a hardware watchdog, and it assumes the link "
            "can vanish at any moment."
        )
        self._add_paragraph(frame_layout, layer4_intro)

        self._add_subsection(
            frame_layout,
            "Flight-controller failsafes",
            (
                "RC packets older than 250&nbsp;ms (RC_FAILSAFE_TIMEOUT_US) are treated as "
                "stale. On a stale link the firmware resets the roll, pitch, and throttle "
                "controllers, forces Manual and Manual-Throttle modes, clears any "
                "auto-throttle command, and cuts throttle to 1000&nbsp;&micro;s.",
                "Between 250 and 500&nbsp;ms a short servo-hold window linearly blends the "
                "surfaces from their last position toward neutral. It is deliberately brief so "
                "a large deflection is not frozen in place after the link stops decoding; past "
                "500&nbsp;ms everything is hard-centred.",
                "A separate raw-byte activity check (also 250&nbsp;ms) detects a stalled serial "
                "parser independently of frame decoding, so cached frames cannot keep the link "
                "looking alive.",
                "Auto-throttle is permitted only with fresh RC <i>and</i> fresh airspeed "
                "(within 100&nbsp;ms). If airspeed goes stale, throttle decays at 50% per "
                "second rather than holding or increasing power open-loop.",
                "Servo writes are hysteresis-limited (only updated on a meaningful change) and "
                "force-refreshed on a schedule, and a hardware watchdog (~100&nbsp;ms) resets "
                "the microcontroller outright if the main loop ever hangs.",
            ),
        )

        layer5_intro = (
            "Layer 5 &mdash; keeping the control math and the sensors honest. Even with a good "
            "link, the closed-loop controllers and the attitude estimator are bounded so that "
            "they cannot run away or be poisoned by a single bad sensor reading."
        )
        self._add_paragraph(frame_layout, layer5_intro)

        self._add_subsection(
            frame_layout,
            "Bounded controllers and sensor validation",
            (
                "The roll, pitch, and auto-throttle PID controllers clamp both their output "
                "and their integrator (&plusmn;400&nbsp;&micro;s output and &plusmn;100 "
                "integrator on roll/pitch), and only accumulate the integrator when not "
                "saturated &mdash; classic anti-windup that limits how fast and how far "
                "closed-loop logic can move a surface.",
                "Auto-throttle output is treated as percent-per-second and integrated into a "
                "0&ndash;100% command, so throttle changes are inherently rate-bounded and "
                "easy to reason about.",
                "The attitude estimator (an Extended Kalman Filter) rejects accelerometer "
                "readings whose magnitude deviates more than 35% from gravity, and rejects "
                "accelerometer/magnetometer readings that fail direction (innovation) gates. "
                "Rejected readings are replaced by the filter's own prediction with very high "
                "assumed noise instead of being fed in as truth.",
                "The innovation gates stay disabled for the first ~250 updates (about two "
                "seconds) so the estimate can converge at startup, and if updates fail 25 "
                "times in a row the filter resets to a clean normalized quaternion rather than "
                "continuing with a diverged state.",
            ),
        )

        layer6_intro = (
            "Layer 6 &mdash; telling the operator before it becomes an emergency. The GCS "
            "watches the telemetry for hazardous trends and plays distinct audio warnings, but "
            "it is careful not to cry wolf."
        )
        self._add_paragraph(frame_layout, layer6_intro)

        self._add_subsection(
            frame_layout,
            "Operator warnings and debounced alarms",
            (
                "Configurable alarms cover low airspeed at altitude (stall), low altitude at "
                "speed, excessive bank angle (default 45&deg;), and high sink rate (default "
                "10&nbsp;ft/s). Each can be switched off individually, and a master switch "
                "controls them all.",
                "The airspeed and altitude alarms arm only when the aircraft is judged "
                "airborne and not already in the landing debounce, so they stay quiet during "
                "ground handling and normal touchdowns.",
                "A warning condition must persist for more than one second before any audio "
                "plays, which filters out single noisy telemetry frames.",
                "Sink rate is estimated over a sliding altitude-versus-time window and is "
                "discarded when its source data goes stale, so a burst of late GPS packets "
                "cannot manufacture a false descent-rate emergency.",
            ),
        )

        layer7_intro = (
            "Layer 7 &mdash; safe on the bench and safe at power-on. Diagnostic builds and a "
            "bad sensor at startup must not be flyable by accident."
        )
        self._add_paragraph(frame_layout, layer7_intro)

        self._add_subsection(
            frame_layout,
            "Bench-only diagnostics and safe startup",
            (
                "Magnetometer-calibration and GPS-diagnostic modes are compile-time flags that "
                "deliberately block normal flight startup after collecting their bench data, "
                "so a debug build cannot be launched by mistake.",
                "If the IMU fails to initialize, the firmware centres the servos and halts "
                "rather than flying on invalid attitude data.",
                "GPS, barometer, airspeed, and telemetry run on independent timers (e.g. the "
                "attitude filter at ~125&nbsp;Hz, GPS far slower) so a slow low-rate sensor "
                "can never stall the high-rate attitude and servo loops.",
                "At power-on the servos are centred and throttle is held at cut until the very "
                "first valid RC packet arrives, so nothing moves before the link is confirmed.",
            ),
        )

        closing = (
            "Put together, these seven layers give the operator a fail-operational picture on "
            "the ground and the aircraft a fail-safe actuation path in the air. The GCS works "
            "to prevent an unsafe launch and to keep commands fresh; if the desktop, the link, "
            "or the telemetry pipeline fails anyway, the flight controller &mdash; on its own "
            "clock, with its own sensor checks &mdash; brings the aircraft to neutral controls "
            "and throttle cut without needing anything from the ground."
        )
        self._add_paragraph(frame_layout, closing)

        layout.addWidget(frame)

    def _populate_elrs_tab(self, layout: QVBoxLayout) -> None:
        frame, frame_layout = self._create_section(
            "ExpressLRS (ELRS) Long Range System Implementation"
        )

        what_is_elrs = (
            "ExpressLRS (ELRS) is the long-range radio system that links the ground station "
            "to the aircraft. CRSF (the Crossfire Serial Protocol) is the language spoken "
            "over that link &mdash; a compact binary format that carries both the stick "
            "commands going up to the plane and the telemetry coming back down on a single "
            "RF channel. The clever part is that both the ground side and the aircraft "
            "exchange CRSF with their ELRS modules over an ordinary serial (UART) cable; the "
            "ELRS modules then handle the actual radio transmission. This tab explains how a "
            "stick movement becomes bytes on a wire, how those bytes are framed and "
            "error-checked, and how the numbers that come back are turned into the gauges on "
            "screen. The aim is that someone unfamiliar with the code can follow the whole "
            "path end to end."
        )
        self._add_paragraph(frame_layout, what_is_elrs)

        frame_format = (
            "Every CRSF message is a small frame with the same shape, implemented in "
            "pico_modules/pico_transmitpackets.py (the CRSFPacketProcessor class). A frame is: "
            "a sync/address byte that marks the start (0xC8 for RC channels, 0xEA for "
            "telemetry coming from the handset), a length byte, a type byte that says what "
            "kind of data follows, the payload itself, and finally a one-byte checksum. The "
            "checksum is a CRC-8 using the DVB-S2 polynomial (0xD5) computed over the type "
            "and payload. The receiver recomputes it and throws the frame away if it does not "
            "match, which is how corrupted data is caught and discarded rather than acted on."
        )
        self._add_paragraph(frame_layout, frame_format)

        channel_encoding = (
            "Stick and switch positions travel in a single 'RC channels packed' frame (type "
            "0x16). It carries 16 channels, and to save airtime each channel is squeezed into "
            "just 11 bits &mdash; so 16 channels fit into exactly 22 bytes with no wasted "
            "space, the bits flowing across byte boundaries back-to-back. An 11-bit value "
            "spans 0&ndash;2047, but CRSF uses the range 172 (minimum) to 1811 (maximum) with "
            "992 as centre. Those familiar numbers are the standard CRSF stick endpoints; the "
            "firmware later maps them to servo microseconds."
        )
        self._add_paragraph(frame_layout, channel_encoding)

        self._add_subsection(
            frame_layout,
            "The uplink: from stick to radio",
            (
                "<b>Sample</b> &mdash; the joystick handler reads raw stick values and applies "
                "deadzone, sensitivity, and smoothing (pico_joystick2state.py).",
                "<b>Map</b> &mdash; the filtered values are linearly scaled into the CRSF "
                "172&ndash;1811 range; unused channels default to centre (992).",
                "<b>Pack</b> &mdash; all 16 channels are bit-packed into the 22-byte payload, "
                "naturally truncated to 11 bits each.",
                "<b>Checksum and send</b> &mdash; the CRC-8 is appended and the finished frame "
                "is written to the serial port for the ELRS module to transmit.",
                "<b>Pace</b> &mdash; frames are sent at a steady chosen rate (100, 250, or "
                "500&nbsp;Hz; 250&nbsp;Hz / every 4&nbsp;ms by default) by a high-resolution "
                "pacer thread, and transmission stops if the channels stop being refreshed for "
                "2&nbsp;seconds &mdash; see the Safety tab.",
            ),
        )

        baud_note = (
            "On the ground side the serial link to the ELRS module runs at 921600 baud by "
            "default (configurable in config.py and via the CRSF_BAUDRATE environment "
            "variable), with 8 data bits, no parity, and one stop bit. On the aircraft the "
            "flight controller talks to its ELRS receiver over a hardware UART, and a "
            "separate UART at 9600 baud is reserved for the GPS module. The GCS uses a "
            "generously sized read buffer so a burst of telemetry never overflows while the "
            "program is briefly busy."
        )
        self._add_paragraph(frame_layout, baud_note)

        telemetry_flow = (
            "Telemetry on the downlink is the reverse trip. Incoming bytes are accumulated in "
            "a buffer and a small state machine looks for a sync byte, reads the length, waits "
            "for the whole frame to arrive, and verifies the CRC before trusting a single "
            "byte of it. Only after the checksum passes is the payload decoded and routed by "
            "type. If the CRC fails, just the leading sync byte is dropped and the parser "
            "resynchronises &mdash; one bad frame never derails the stream. Each decoded "
            "packet is delivered to the GUI through a Qt signal and timestamped so stale "
            "values can be aged out."
        )
        self._add_paragraph(frame_layout, telemetry_flow)

        self._add_subsection(
            frame_layout,
            "Telemetry frame types the GCS decodes",
            (
                "<b>Attitude (0x1E)</b> &mdash; pitch, roll, and yaw (sent as scaled "
                "integers, converted to degrees). Drives the artificial horizon and compass.",
                "<b>GPS (0x02)</b> &mdash; latitude, longitude, altitude, ground speed, course, "
                "and satellite count. Feeds the altitude/airspeed tapes and the moving map.",
                "<b>Battery (0x08)</b> &mdash; pack voltage, current, used capacity, and "
                "(optionally) charge percent. Feeds the battery panel.",
                "<b>Link statistics (0x14)</b> &mdash; RSSI on two antennas, link quality, SNR, "
                "active antenna, RF mode/power, plus the downlink RSSI, link quality, and SNR "
                "as seen from the receiver. Feeds the signal-health panel.",
            ),
        )

        link_stats = (
            "The link-statistics frame is what tells the operator how healthy the radio link "
            "is, in both directions. Link quality is the percentage of packets getting "
            "through, RSSI is raw signal strength in dBm, and SNR is how far the signal sits "
            "above the noise. The GCS colour-codes these (for example RSSI is green above "
            "roughly -75&nbsp;dBm and steps through yellow, orange, and red as it weakens) so "
            "a degrading link is obvious at a glance. The radio worker can also emit "
            "once-per-second diagnostics &mdash; frame rates, byte throughput, and CRC error "
            "counts &mdash; which the Debug page uses for troubleshooting."
        )
        self._add_paragraph(frame_layout, link_stats)

        firmware_side = (
            "On the aircraft, the flight controller firmware (flight_controller/Main.ino) does "
            "not hand-roll the protocol &mdash; it uses the open-source CRSF for Arduino "
            "library. The firmware creates a CRSFforArduino object bound to its serial port, "
            "calls update() repeatedly in its service loop to pump the parser, and registers a "
            "callback that fires whenever a valid RC-channels frame arrives. That callback "
            "copies in the new channel values and, importantly, records the arrival time so "
            "the firmware's own 250&nbsp;ms packet-age failsafe (see the Safety tab) can act "
            "if frames stop coming. For the return trip, the firmware calls the library's "
            "telemetry writers &mdash; attitude at roughly 125&nbsp;Hz and GPS at a lower rate "
            "&mdash; and the library frames, checksums, and schedules those outbound packets "
            "automatically."
        )
        self._add_paragraph(frame_layout, firmware_side)

        self._add_subsection(
            frame_layout,
            "Working with CRSF for Arduino (firmware side)",
            (
                "Create a <code>CRSFforArduino</code> object bound to the receiver's UART and "
                "call <code>begin()</code> once at startup.",
                "Call <code>update()</code> every loop to drain the serial stream, decode "
                "frames, and service telemetry without overflowing the RX buffer.",
                "Register a channels callback to receive decoded RC channels the instant a "
                "valid frame arrives, and timestamp it for failsafe detection.",
                "Read channels with the library's helpers and translate them to servo "
                "microseconds for the control logic.",
                "Use the telemetry writers (attitude, GPS, battery) to queue outbound frames; "
                "the library sequences them into the ELRS telemetry windows for you.",
            ),
        )

        self._add_subsection(
            frame_layout,
            "End-to-end checkpoints",
            (
                "Uplink: stick &rarr; deadzone/sensitivity/smoothing &rarr; map to 172&ndash;1811 &rarr; pack 16&times;11 bits &rarr; append CRC-8 &rarr; serial &rarr; ELRS module &rarr; air.",
                "Downlink: air &rarr; ELRS module &rarr; serial &rarr; buffer &rarr; sync + length + CRC check &rarr; decode by type &rarr; Qt signal &rarr; gauges, map, and recorder.",
                "Failsafe: no fresh frame for 250&nbsp;ms &rarr; firmware neutralises surfaces and cuts throttle, regardless of what the ground side is doing.",
            ),
        )

        closing = (
            "By running both control and telemetry over one CRSF link, the system keeps the "
            "ground station richly informed for situational awareness while keeping control "
            "latency low. The protocol's compact 11-bit packing and per-frame CRC give it "
            "efficiency and robustness, and because the same framing is used in both "
            "directions, the ground and air software stay simple, symmetric, and easy to "
            "reason about."
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

