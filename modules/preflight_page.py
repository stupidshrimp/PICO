"""Pre-flight checklist page and navigation button."""

from __future__ import annotations

from typing import List

from PySide6.QtCore import Qt
from PySide6.QtGui import QCursor
from PySide6.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)


class PreFlightChecklistPage:
    """Add a pre-flight checklist tab with grouped checklist items."""

    def __init__(self, main_window) -> None:  # type: ignore[no-untyped-def]
        self._main_window = main_window
        self._ui = main_window.ui
        self._checkboxes: List[QCheckBox] = []

        self._create_navigation_button()
        self._build_page()

    # ------------------------------------------------------------------
    # Navigation button setup
    # ------------------------------------------------------------------
    def _create_navigation_button(self) -> None:
        button = QPushButton(self._ui.topMenu)
        button.setObjectName("btn_preflight")
        button.setSizePolicy(self._ui.btn_home.sizePolicy())
        button.setMinimumSize(self._ui.btn_home.minimumSize())
        button.setFont(self._ui.btn_home.font())
        button.setCursor(QCursor(Qt.PointingHandCursor))
        button.setLayoutDirection(Qt.LeftToRight)
        button.setStyleSheet(
            "background-image: url(:/icons/images/icons/cil-task.png);"
        )
        button.setText("Pre-Flight Checklist")
        self._ui.verticalLayout_8.addWidget(button)
        self._ui.btn_preflight = button

    # ------------------------------------------------------------------
    # Page construction
    # ------------------------------------------------------------------
    def _build_page(self) -> None:
        page = QWidget()
        page.setObjectName("preflight_page")

        layout = QVBoxLayout(page)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        title = QLabel("Pre-Flight Checklist")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        description = QLabel(
            "Complete each item before flight. Use the clear button to reset the list."
        )
        description.setWordWrap(True)
        description.setAlignment(Qt.AlignCenter)
        layout.addWidget(description)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)
        layout.addWidget(scroll_area, 1)

        container = QWidget()
        scroll_area.setWidget(container)
        container_layout = QVBoxLayout(container)
        container_layout.setSpacing(10)
        container_layout.setContentsMargins(0, 0, 0, 0)

        sections = (
            (
                "Airframe & Surfaces",
                (
                    "Inspect fuselage for cracks, loose parts, or damage",
                    "Check wings and tail are securely attached",
                    "Verify control horns, linkages, and pushrods are tight",
                    "Move control surfaces (ailerons, elevator, rudder) by hand—no binding or excessive play",
                ),
            ),
            (
                "Power & Propulsion",
                (
                    "Battery fully charged and securely fastened",
                    "ESC connections secure and correct polarity",
                    "Propeller tight and free of cracks/damage",
                    "Motor spins freely with no grinding",
                    "Verify prop orientation and correct rotation direction",
                ),
            ),
            (
                "Electronics & Radio",
                (
                    "Antennas oriented correctly, not obstructed by carbon/metal",
                    "Control surface check (stick movements = correct surface deflections)",
                    "Confirm ELRS link active (Tx/Rx telemetry confirmed)",
                    "RSSI and LQ at safe levels before flight",
                ),
            ),
            (
                "Sensors & Extras",
                (
                    "GPS lock confirmed (if using autopilot)",
                    "Telemetry/OSD connected and reading correctly",
                    "Camera/VTX powered, video feed confirmed (ensure camera and transmitter are not too hot)",
                ),
            ),
            (
                "Environment",
                (
                    "Verify wind is not excessive and rain will not occur for the next hour",
                    "Verify no planes in the area",
                ),
            ),
            (
                "Final Safety",
                (
                    "Flight mode switch in correct starting position",
                    "Throttle cut/kill switch engaged before handling",
                    "Verify correct CG (center of gravity)",
                    "Launch/takeoff area clear of people and obstacles",
                ),
            ),
        )

        for title_text, items in sections:
            container_layout.addWidget(self._create_section_label(title_text))
            for text in items:
                checkbox = QCheckBox(text)
                checkbox.setCursor(QCursor(Qt.PointingHandCursor))
                checkbox.setStyleSheet(
                    "QCheckBox { font-size: 12pt; }"
                )
                self._checkboxes.append(checkbox)
                container_layout.addWidget(checkbox)
            container_layout.addSpacing(8)

        container_layout.addStretch(1)

        controls_layout = QHBoxLayout()
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.addItem(
            QSpacerItem(20, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        )

        clear_button = QPushButton("Clear Checklist")
        clear_button.setCursor(QCursor(Qt.PointingHandCursor))
        clear_button.setStyleSheet(
            "QPushButton {"
            "    background-color: rgb(52, 59, 72);"
            "    padding: 8px 16px;"
            "    border-radius: 6px;"
            "}"
            "QPushButton:hover {"
            "    background-color: rgb(64, 71, 88);"
            "}"
            "QPushButton:pressed {"
            "    background-color: rgb(46, 125, 50);"
            "}"
        )
        clear_button.clicked.connect(self._clear_checklist)
        controls_layout.addWidget(clear_button)

        layout.addLayout(controls_layout)

        self._ui.preflight_page = page
        self._ui.stackedWidget.addWidget(page)

    def _create_section_label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setStyleSheet("font-size: 14px; font-weight: bold;")
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        return label

    def _clear_checklist(self) -> None:
        for checkbox in self._checkboxes:
            checkbox.setChecked(False)

