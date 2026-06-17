"""Compact pseudo-3D attitude model widget.

This widget renders a small, low-poly aircraft that rotates with the incoming
roll/pitch/yaw telemetry, giving the operator an at-a-glance 3D read on the
airframe's orientation (similar to the model shown on a flight-controller
setup screen).  Unlike the existing OSD overlays it is intended to live in the
command sidebar *below* the status indicators rather than over the video feed.

Two ambient effects layer behind the model:

* **Wind streaks** whose density and speed scale with the pitot airspeed
  telemetry, conveying how fast the airframe is moving through the air.
* **An altitude background** whose sky gradient darkens with height and whose
  visible ground band shrinks as the barometric altitude climbs.

The model is drawn entirely with :class:`QPainter` using a hand-rolled
orthographic-ish perspective projection (NumPy is already a project
dependency), keeping it consistent with the other custom OSD widgets and free
of any extra 3D toolkit.
"""

import math
import time

import numpy as np

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QLinearGradient,
    QPainter,
    QPen,
    QPolygonF,
)
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer

from pico_modules.osd_smoothing import time_scaled_weight


# Altitude (ft) at which the sky reaches its "high" colour and the ground band
# has fully receded.  Tuned for small fixed-wing operating heights; adjust to
# taste for the platform.
ALT_FULL_SCALE_FT = 400.0

# Airspeed (mph) at which the wind effect reaches its maximum density/speed.
AIRSPEED_FULL_SCALE_MPH = 45.0

# Fixed chase-camera orientation (degrees) applied on top of the live attitude
# so the neutral model is seen from behind and slightly above, like the
# reference setup view.
VIEW_PITCH_DEG = 34.0   # positive = look down onto the top of the airframe
VIEW_YAW_DEG = -30.0    # positive = orbit to the right

# Perspective projection constants.
_CAM_DIST = 3.2
_FOCAL = 2.6


def _rot_x(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _rot_y(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _rot_z(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


class Attitude3DOSD(QWidget):
    """A small 3D-looking aircraft driven by attitude/airspeed/altitude TLM."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Smoothed display state.
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._airspeed = 0.0
        self._altitude = 0.0
        self._initialized = False
        self._smoothing = 0.25  # Per-call EMA weight tuned at the ~30 Hz reference
        self._last_update_time = None

        # Pre-built model geometry and the fixed view transform.
        self._vertices, self._faces = self._build_model()
        self._view = _rot_x(math.radians(VIEW_PITCH_DEG)) @ _rot_y(
            math.radians(VIEW_YAW_DEG)
        )
        self._light_dir = self._normalize(np.array([-0.35, 0.82, 0.45]))

        # Wind streak particles.
        self._wind = []  # list of [x, y, vx, vy, length, alpha]
        self._wind_spawn_accum = 0.0
        self._last_anim_time = time.monotonic()

        self.setMinimumSize(180, 150)

        # Drive the wind animation independently of telemetry arrival so the
        # effect stays smooth between packets.
        self._anim_timer = QTimer(self)
        self._anim_timer.setInterval(33)
        self._anim_timer.timeout.connect(self._animate)
        self._anim_timer.start()

    # ------------------------------------------------------------------ #
    # Public telemetry setters
    # ------------------------------------------------------------------ #
    def setAttitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        """Update roll, pitch and yaw (degrees) with time-scaled smoothing."""

        if (
            roll_deg is None
            or pitch_deg is None
            or yaw_deg is None
            or not math.isfinite(roll_deg)
            or not math.isfinite(pitch_deg)
            or not math.isfinite(yaw_deg)
        ):
            return

        now = time.monotonic()
        if not self._initialized:
            self._roll = roll_deg
            self._pitch = pitch_deg
            self._yaw = yaw_deg % 360.0
            self._initialized = True
        else:
            alpha = time_scaled_weight(self._smoothing, now - self._last_update_time)
            self._roll = self._roll * (1 - alpha) + roll_deg * alpha
            self._pitch = self._pitch * (1 - alpha) + pitch_deg * alpha
            # Smooth yaw along the shortest arc so it never spins through 0/360.
            delta = (yaw_deg - self._yaw + 180.0) % 360.0 - 180.0
            self._yaw = (self._yaw + delta * alpha) % 360.0
        self._last_update_time = now
        self.update()

    def setAirspeed(self, airspeed_mph: float) -> None:
        """Update the airspeed (mph) that drives the wind effect."""

        if airspeed_mph is None or not math.isfinite(airspeed_mph):
            return
        self._airspeed = max(0.0, float(airspeed_mph))

    def setAltitude(self, altitude_ft: float) -> None:
        """Update the barometric altitude (ft) that drives the background."""

        if altitude_ft is None or not math.isfinite(altitude_ft):
            return
        self._altitude = float(altitude_ft)
        self.update()

    # ------------------------------------------------------------------ #
    # Geometry
    # ------------------------------------------------------------------ #
    @staticmethod
    def _normalize(v: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(v)
        return v / n if n else v

    def _build_model(self):
        """Return ``(vertices, faces)`` describing a low-poly aircraft.

        Body frame: ``+x`` right wing (starboard), ``+y`` up, ``+z`` aft so the
        nose points toward ``-z``.  Each face is ``(indices, base_color)`` where
        ``indices`` reference rows of the returned ``vertices`` array.
        """

        verts = []
        faces = []

        # Corner order for a unit box, reused by every part below.
        corner_signs = [
            (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
        ]
        box_faces = [
            (0, 3, 2, 1), (4, 5, 6, 7), (0, 4, 7, 3),
            (1, 2, 6, 5), (0, 1, 5, 4), (3, 7, 6, 2),
        ]

        def add_box(center, half, color):
            base = len(verts)
            cx, cy, cz = center
            hx, hy, hz = half
            for sx, sy, sz in corner_signs:
                verts.append((cx + sx * hx, cy + sy * hy, cz + sz * hz))
            for f in box_faces:
                faces.append(((base + f[0], base + f[1], base + f[2], base + f[3]), color))

        fuselage = QColor(214, 218, 226)
        wing = QColor(226, 230, 236)
        dark = QColor(44, 48, 58)
        red = QColor(232, 64, 58)
        green = QColor(58, 200, 92)

        add_box((0.0, 0.0, 0.06), (0.075, 0.090, 0.62), fuselage)   # fuselage
        add_box((0.0, -0.01, -0.62), (0.050, 0.060, 0.14), fuselage) # nose taper
        add_box((0.0, 0.04, -0.06), (0.66, 0.020, 0.26), wing)      # main wing
        add_box((0.0, 0.05, 0.58), (0.26, 0.016, 0.11), wing)       # h-stabilizer
        add_box((0.0, 0.14, 0.58), (0.018, 0.13, 0.11), dark)       # vertical fin
        add_box((-0.74, 0.04, -0.06), (0.085, 0.024, 0.20), red)    # port tip (red)
        add_box((0.74, 0.04, -0.06), (0.085, 0.024, 0.20), green)   # starboard tip

        return np.array(verts, dtype=float), faces

    # ------------------------------------------------------------------ #
    # Wind animation
    # ------------------------------------------------------------------ #
    def _airspeed_fraction(self) -> float:
        return max(0.0, min(1.0, self._airspeed / AIRSPEED_FULL_SCALE_MPH))

    def _animate(self) -> None:
        now = time.monotonic()
        dt = now - self._last_anim_time
        self._last_anim_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.033

        frac = self._airspeed_fraction()
        w = max(1, self.width())
        h = max(1, self.height())

        # Advance existing streaks; air streams toward the viewer (downward and
        # outward) to convey forward flight.
        survivors = []
        for p in self._wind:
            p[0] += p[2] * dt
            p[1] += p[3] * dt
            if p[1] - p[4] < h + 4:
                survivors.append(p)
        self._wind = survivors

        # Spawn new streaks at a rate proportional to airspeed (none when stopped).
        if frac > 0.02:
            spawn_rate = 4.0 + 56.0 * frac  # streaks per second
            self._wind_spawn_accum += spawn_rate * dt
            cx = w * 0.5
            while self._wind_spawn_accum >= 1.0 and len(self._wind) < 70:
                self._wind_spawn_accum -= 1.0
                x = np.random.uniform(0.0, w)
                y = np.random.uniform(-10.0, h * 0.35)
                # Emanate from the centre line so motion reads as "flying into" it.
                spread = (x - cx) / max(1.0, cx)
                speed = (90.0 + 320.0 * frac)
                vx = spread * speed * 0.45
                vy = speed
                length = 6.0 + 26.0 * frac
                alpha = np.random.uniform(0.10, 0.30) * (0.4 + 0.6 * frac)
                self._wind.append([x, y, vx, vy, length, alpha])

        self.update()

    # ------------------------------------------------------------------ #
    # Painting
    # ------------------------------------------------------------------ #
    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        w = self.width()
        h = self.height()

        self._paint_background(painter, w, h)
        self._paint_wind(painter, w, h)
        self._paint_model(painter, w, h)
        self._paint_readout(painter, w, h)

        painter.end()

    def _paint_background(self, painter: QPainter, w: int, h: int) -> None:
        t = max(0.0, min(1.0, self._altitude / ALT_FULL_SCALE_FT))

        def lerp(a: QColor, b: QColor, s: float) -> QColor:
            return QColor(
                int(a.red() + (b.red() - a.red()) * s),
                int(a.green() + (b.green() - a.green()) * s),
                int(a.blue() + (b.blue() - a.blue()) * s),
            )

        # Sky darkens and deepens in blue as altitude increases.
        top = lerp(QColor(96, 152, 208), QColor(8, 18, 52), t)
        horizon = lerp(QColor(196, 222, 244), QColor(60, 104, 168), t)

        # Ground band recedes (shrinks) with altitude; gone near full scale.
        ground_frac = (1.0 - t) * 0.30
        horizon_y = h * (1.0 - ground_frac)

        sky = QLinearGradient(0, 0, 0, horizon_y)
        sky.setColorAt(0.0, top)
        sky.setColorAt(1.0, horizon)
        painter.fillRect(QRectF(0, 0, w, horizon_y), QBrush(sky))

        if ground_frac > 0.001:
            ground_top = lerp(QColor(96, 120, 92), QColor(120, 150, 120), t)
            ground_bot = lerp(QColor(58, 78, 56), QColor(96, 124, 100), t)
            ground = QLinearGradient(0, horizon_y, 0, h)
            ground.setColorAt(0.0, ground_top)
            ground.setColorAt(1.0, ground_bot)
            painter.fillRect(QRectF(0, horizon_y, w, h - horizon_y), QBrush(ground))

            # A couple of perspective lines hint at ground distance.
            painter.setPen(QPen(QColor(255, 255, 255, 40), 1))
            for i in range(1, 4):
                gy = horizon_y + (h - horizon_y) * (i / 4.0) ** 1.6
                painter.drawLine(QPointF(0, gy), QPointF(w, gy))

    def _paint_wind(self, painter: QPainter, w: int, h: int) -> None:
        for x, y, vx, vy, length, alpha in self._wind:
            speed = math.hypot(vx, vy) or 1.0
            ux, uy = vx / speed, vy / speed
            color = QColor(235, 245, 255)
            color.setAlphaF(max(0.0, min(1.0, alpha)))
            painter.setPen(QPen(color, 1.4, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(
                QPointF(x - ux * length, y - uy * length), QPointF(x, y)
            )

    def _paint_model(self, painter: QPainter, w: int, h: int) -> None:
        # body -> world (live attitude) -> camera (fixed chase view).
        # Sign conventions: positive roll banks the right wing down, positive
        # pitch raises the nose, and increasing yaw turns the nose right.
        attitude = (
            _rot_y(math.radians(-self._yaw))
            @ _rot_x(math.radians(self._pitch))
            @ _rot_z(math.radians(-self._roll))
        )
        rot = self._view @ attitude
        pts = self._vertices @ rot.T  # (N, 3) rotated into camera space

        cx = w * 0.5
        cy = h * 0.46
        scale = min(w, h) * 0.62

        def project(p):
            depth = _CAM_DIST - p[2]
            if depth < 0.05:
                depth = 0.05
            f = _FOCAL / depth
            return QPointF(cx + p[0] * f * scale, cy - p[1] * f * scale)

        screen = [project(p) for p in pts]

        light = self._light_dir
        drawables = []
        for indices, base in self._faces:
            v0, v1, v2 = pts[indices[0]], pts[indices[1]], pts[indices[2]]
            normal = np.cross(v1 - v0, v2 - v0)
            n = np.linalg.norm(normal)
            if n == 0:
                continue
            normal = normal / n
            shade = 0.42 + 0.58 * abs(float(np.dot(normal, light)))
            depth = float(np.mean([pts[i][2] for i in indices]))
            drawables.append((depth, indices, base, shade))

        # Painter's algorithm: far (smaller camera-space z) first.
        drawables.sort(key=lambda d: d[0])

        for _depth, indices, base, shade in drawables:
            color = QColor(
                int(min(255, base.red() * shade)),
                int(min(255, base.green() * shade)),
                int(min(255, base.blue() * shade)),
            )
            poly = QPolygonF([screen[i] for i in indices])
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor(20, 24, 30, 160), 0.6))
            painter.drawPolygon(poly)

    def _paint_readout(self, painter: QPainter, w: int, h: int) -> None:
        painter.setFont(QFont("Arial", 8))
        painter.setPen(QColor(235, 245, 255, 220))
        painter.drawText(
            QRectF(6, 4, w - 12, 16),
            Qt.AlignLeft | Qt.AlignVCenter,
            f"{int(round(self._altitude))} ft",
        )
        painter.drawText(
            QRectF(6, 4, w - 12, 16),
            Qt.AlignRight | Qt.AlignVCenter,
            f"{int(round(self._airspeed))} mph",
        )
        painter.setPen(QColor(235, 245, 255, 200))
        painter.drawText(
            QRectF(6, h - 18, w - 12, 14),
            Qt.AlignLeft | Qt.AlignVCenter,
            f"R {self._roll:+.0f}°  P {self._pitch:+.0f}°  Y {self._yaw:03.0f}°",
        )
