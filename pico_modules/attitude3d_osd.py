"""Compact pseudo-3D attitude model widget.

This widget renders a small, smoothly-curved UAV that rotates with the incoming
roll/pitch/yaw telemetry, giving the operator an at-a-glance 3D read on the
airframe's orientation (similar to the model shown on a flight-controller
setup screen).  The silhouette borrows the cues of a medium-altitude UAV --
a rounded, lofted fuselage with a bulbous sensor nose and chin turret, long
high-aspect swept wings with dihedral and an upward V-tail -- so it reads as
an unmanned platform rather than a hobby aeroplane.
Unlike the existing OSD overlays it is intended to live in the command sidebar
*below* the status indicators rather than over the video feed.

Rather than a faceted box airframe the model is built by *lofting* elliptical
cross-section rings along each part's axis, so the fuselage and wings read as
continuous curved surfaces once flat-shaded.

Two ambient effects layer behind the model:

* **Wind streaks** whose density and speed scale with the pitot airspeed
  telemetry, conveying how fast the airframe is moving through the air.
* **An altitude background**, themed to match the application's dark slate
  chrome and accent green so it reads as part of the GUI rather than a bright
  outdoor scene, whose sky gradient darkens with height and whose visible
  ground band shrinks as the barometric altitude climbs.

The model is drawn entirely with :class:`QPainter` using a hand-rolled
perspective projection (NumPy is already a project dependency), keeping it
consistent with the other custom OSD widgets and free of any extra 3D toolkit.

A built-in **telemetry simulation** drives synthetic attitude/altitude/airspeed
through a gentle, looping flight manoeuvre so the model can be previewed without
a live link.  The simulation ticks at :data:`SIM_TLM_FREQUENCY_HZ` (60 Hz) to
mimic a real telemetry stream and is automatically superseded by live data via
:meth:`set_simulation_enabled`.
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

# Sky/ground background palette.  These mirror the application's dark theme so
# the widget's backdrop reads as part of the GUI rather than a bright outdoor
# scene: the sky borrows the slate tones of the window and side panels
# (rgb(33, 37, 43)/rgb(40, 44, 52) in modules/ui_main.py) and the ground borrows
# the signature accent green (#2E7D32 / rgb(46, 125, 50) from
# modules/app_settings.py).  Within each pair the colour is blended from its
# low-altitude value to its high-altitude value as the aircraft climbs, so the
# sky still deepens and the ground still lightens/recedes with height.
_SKY_TOP_LOW = QColor(40, 46, 56)        # slate near the sidebar/content panels
_SKY_TOP_HIGH = QColor(18, 20, 26)       # deep, near-black overhead at altitude
_SKY_HORIZON_LOW = QColor(70, 80, 96)    # lighter slate haze along the horizon
_SKY_HORIZON_HIGH = QColor(33, 37, 43)   # the app's base background tone
_GROUND_TOP_LOW = QColor(46, 125, 50)    # accent green meeting the horizon
_GROUND_TOP_HIGH = QColor(70, 150, 78)   # lighter green when viewed from height
_GROUND_BOT_LOW = QColor(24, 64, 30)     # deeper green underfoot
_GROUND_BOT_HIGH = QColor(46, 110, 52)

# Fixed chase-camera orientation (degrees) applied on top of the live attitude
# so the neutral model is seen from behind at a rear three-quarter angle, like
# the reference setup view.
#
# The view pitch is held at 0 so a level airframe (roll/pitch = 0) actually
# reads as level on screen.  A non-zero downward look raises the far nose in the
# projection -- the nose's on-screen height is sin(VIEW_PITCH) regardless of yaw
# -- which made a zero-attitude model appear permanently pitched up.  All of the
# three-dimensional read therefore comes from the yaw orbit plus perspective
# rather than a top-down tilt.
VIEW_PITCH_DEG = 0.0    # positive = look down onto the top of the airframe
VIEW_YAW_DEG = -28.0    # positive = orbit to the right

# Perspective projection constants.
_CAM_DIST = 3.2
_FOCAL = 2.6

# Telemetry-simulation tick rate (Hz).  Matches a real 60 Hz telemetry stream so
# the preview animation moves the way live data would.
SIM_TLM_FREQUENCY_HZ = 60.0


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
        self._view = _rot_y(math.radians(VIEW_YAW_DEG)) @ _rot_x(
            math.radians(VIEW_PITCH_DEG)
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

        # Built-in telemetry simulation: a 60 Hz timer that synthesises a gentle
        # looping flight so the model previews without a live link.  Live data
        # disables it via ``set_simulation_enabled(False)``.
        self._sim_enabled = False
        self._sim_time = 0.0
        self._sim_timer = QTimer(self)
        self._sim_timer.setInterval(int(round(1000.0 / SIM_TLM_FREQUENCY_HZ)))
        self._sim_timer.timeout.connect(self._simulate_tick)
        self.set_simulation_enabled(False)

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
        self._blend_attitude(roll_deg, pitch_deg, yaw_deg)
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
    # Telemetry simulation
    # ------------------------------------------------------------------ #
    def set_simulation_enabled(self, enabled: bool) -> None:
        """Enable/disable the built-in 60 Hz telemetry simulation.

        When enabled the widget free-runs a synthetic flight so the model can be
        previewed without a live link.  Callers should disable it as soon as real
        telemetry is available so live data drives the model instead.
        """

        enabled = bool(enabled)
        if enabled == self._sim_enabled:
            return
        self._sim_enabled = enabled
        if enabled:
            # Reset so the loop always restarts from a calm, level attitude.
            self._sim_time = 0.0
            self._sim_timer.start()
        else:
            self._sim_timer.stop()

    def is_simulation_enabled(self) -> bool:
        return self._sim_enabled

    def _simulate_tick(self) -> None:
        """Advance the synthetic flight by one 60 Hz telemetry frame."""

        dt = 1.0 / SIM_TLM_FREQUENCY_HZ
        self._sim_time += dt
        t = self._sim_time

        # A coordinated, looping manoeuvre: a slow heading change with the bank,
        # pitch and speed that would accompany it, plus a long altitude drift.
        # Independent, incommensurate periods keep the motion from looking like a
        # short repeating loop.
        yaw = (t * 9.0 + 26.0 * math.sin(t * 0.12)) % 360.0
        roll = 26.0 * math.sin(t * 0.23) + 6.0 * math.sin(t * 0.71)
        pitch = 7.0 * math.sin(t * 0.37) + 3.0 * math.sin(t * 0.17)
        altitude = 150.0 + 70.0 * math.sin(t * 0.09) + 12.0 * math.sin(t * 0.41)
        airspeed = 27.0 + 9.0 * math.sin(t * 0.27) + 3.0 * math.sin(t * 0.6)

        self._blend_attitude(roll, pitch, yaw)
        self._airspeed = max(0.0, airspeed)
        self._altitude = max(0.0, altitude)
        self.update()

    def _blend_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        """Smooth an attitude sample into the displayed state (EMA)."""

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

    # ------------------------------------------------------------------ #
    # Geometry
    # ------------------------------------------------------------------ #
    @staticmethod
    def _normalize(v: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(v)
        return v / n if n else v

    def _build_model(self):
        """Return ``(vertices, faces)`` describing a smoothly-curved UAV.

        Body frame: ``+x`` right wing (starboard), ``+y`` up, ``+z`` aft so the
        nose points toward ``-z``.  Each face is ``(indices, base_color)`` where
        ``indices`` reference rows of the returned ``vertices`` array.

        The airframe is assembled from *lofted* parts: a list of elliptical
        cross-section rings is threaded along each part's axis and consecutive
        rings are stitched with quads, giving continuous curved surfaces (a
        rounded fuselage, tapered swept wings) rather than a faceted box jet.
        """

        verts = []
        faces = []

        def add_vert(p) -> int:
            verts.append(tuple(np.asarray(p, dtype=float)))
            return len(verts) - 1

        def ellipse_ring(center, u_axis, v_axis, ru, rv, n):
            """A ring of ``n`` points on an ellipse spanning ``u_axis``/``v_axis``."""

            center = np.asarray(center, dtype=float)
            u_axis = np.asarray(u_axis, dtype=float)
            v_axis = np.asarray(v_axis, dtype=float)
            ring = []
            for k in range(n):
                a = 2.0 * math.pi * k / n
                ring.append(
                    add_vert(center + u_axis * (ru * math.cos(a)) + v_axis * (rv * math.sin(a)))
                )
            return ring

        def loft(rings, color, cap_start=None, cap_end=None):
            """Stitch consecutive equal-length rings into a quad tube.

            ``cap_start``/``cap_end`` optionally close the ends with a triangle
            fan to the supplied apex point (its colour matches ``color``).
            """

            n = len(rings[0])
            for r0, r1 in zip(rings[:-1], rings[1:]):
                for i in range(n):
                    j = (i + 1) % n
                    faces.append(((r0[i], r0[j], r1[j], r1[i]), color))
            if cap_start is not None:
                apex = add_vert(cap_start)
                r = rings[0]
                for i in range(n):
                    j = (i + 1) % n
                    faces.append(((apex, r[j], r[i]), color))
            if cap_end is not None:
                apex = add_vert(cap_end)
                r = rings[-1]
                for i in range(n):
                    j = (i + 1) % n
                    faces.append(((apex, r[i], r[j]), color))

        # Reusable box helper for small angular parts (turret, prop).
        corner_signs = [
            (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
        ]
        box_faces = [
            (0, 3, 2, 1), (4, 5, 6, 7), (0, 4, 7, 3),
            (1, 2, 6, 5), (0, 1, 5, 4), (3, 7, 6, 2),
        ]

        def add_box(center, half, color, rot=None):
            base = len(verts)
            center = np.asarray(center, dtype=float)
            hx, hy, hz = half
            for sx, sy, sz in corner_signs:
                offset = np.array([sx * hx, sy * hy, sz * hz], dtype=float)
                if rot is not None:
                    offset = rot @ offset
                add_vert(center + offset)
            for f in box_faces:
                faces.append(((base + f[0], base + f[1], base + f[2], base + f[3]), color))

        fuselage = QColor(206, 211, 219)
        wing = QColor(214, 218, 225)
        sensor = QColor(150, 157, 168)
        red = QColor(232, 64, 58)
        green = QColor(58, 200, 92)

        x_axis = np.array([1.0, 0.0, 0.0])
        y_axis = np.array([0.0, 1.0, 0.0])
        z_axis = np.array([0.0, 0.0, 1.0])

        # ---- Fuselage: lofted along z from a rounded sensor nose to the tail.
        # (station z, half-width, half-height, vertical centre offset)
        fuse_stations = [
            (-0.66, 0.020, 0.022, 0.012),   # nose tip
            (-0.58, 0.072, 0.078, 0.018),   # bulbous EO/IR sensor ball
            (-0.46, 0.082, 0.090, 0.020),   # SATCOM hump shoulder
            (-0.28, 0.070, 0.082, 0.014),   # forward fuselage / avionics bay
            (-0.05, 0.060, 0.072, 0.006),   # wing box
            (0.22, 0.048, 0.058, 0.000),    # mid tail boom
            (0.46, 0.034, 0.040, -0.004),   # aft boom
            (0.62, 0.024, 0.028, -0.006),   # tail
        ]
        fuse_rings = [
            ellipse_ring((0.0, cy, z), x_axis, y_axis, rx, ry, 14)
            for (z, rx, ry, cy) in fuse_stations
        ]
        loft(
            fuse_rings,
            fuselage,
            cap_start=(0.0, fuse_stations[0][3], fuse_stations[0][0] - 0.03),
            cap_end=(0.0, fuse_stations[-1][3], fuse_stations[-1][0] + 0.02),
        )

        # ---- Main wing: a tapered, swept, dihedral surface lofted spanwise.
        # Cross-sections live in the (chord=z, thickness=y) plane and march out
        # along +x, sweeping aft and rising for dihedral toward the tip.
        def wing_sections(sign):
            # (span fraction, chord half, thickness half)
            shape = [
                (0.00, 0.150, 0.020),
                (0.30, 0.140, 0.018),
                (0.62, 0.110, 0.013),
                (0.86, 0.072, 0.009),
                (1.00, 0.030, 0.005),
            ]
            span = 0.86
            rings = []
            for frac, chord, thick in shape:
                x = sign * (0.045 + frac * span)
                z = -0.02 + frac * 0.10          # sweep aft toward the tip
                y = 0.028 + frac * 0.055          # dihedral rise toward the tip
                rings.append(
                    ellipse_ring((x, y, z), z_axis, y_axis, chord, thick, 12)
                )
            return rings

        for sign in (1.0, -1.0):
            loft(wing_sections(sign), wing, cap_end=None)

        # Port/starboard nav-light tips.
        add_box((-0.930, 0.083, 0.064), (0.022, 0.010, 0.040), red)
        add_box((0.930, 0.083, 0.064), (0.022, 0.010, 0.040), green)

        # ---- Upward V-tail: two thin lofted surfaces splayed out and up.
        tail_dihedral = math.radians(40.0)
        tail_base = np.array([0.0, 0.030, 0.50])

        def vtail(sign):
            rot = _rot_z(sign * tail_dihedral)
            shape = [
                (0.00, 0.085, 0.010),
                (0.45, 0.072, 0.008),
                (0.78, 0.050, 0.006),
                (1.00, 0.026, 0.004),
            ]
            length = 0.24
            rings = []
            for frac, chord, thick in shape:
                local = np.array([sign * (0.01 + frac * length), 0.0, 0.0])
                center = tail_base + rot @ local
                u = rot @ z_axis
                v = rot @ y_axis
                rings.append(ellipse_ring(center, u, v, chord, thick, 10))
            return rings

        for sign in (1.0, -1.0):
            loft(vtail(sign), fuselage)

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

        # Sky darkens toward the deep slate of the app chrome as altitude rises.
        top = lerp(_SKY_TOP_LOW, _SKY_TOP_HIGH, t)
        horizon = lerp(_SKY_HORIZON_LOW, _SKY_HORIZON_HIGH, t)

        # Ground band recedes (shrinks) with altitude; gone near full scale.
        ground_frac = (1.0 - t) * 0.30
        horizon_y = h * (1.0 - ground_frac)

        sky = QLinearGradient(0, 0, 0, horizon_y)
        sky.setColorAt(0.0, top)
        sky.setColorAt(1.0, horizon)
        painter.fillRect(QRectF(0, 0, w, horizon_y), QBrush(sky))

        if ground_frac > 0.001:
            ground_top = lerp(_GROUND_TOP_LOW, _GROUND_TOP_HIGH, t)
            ground_bot = lerp(_GROUND_BOT_LOW, _GROUND_BOT_HIGH, t)
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
            # Match the outline to the fill so curved lofts read as smooth
            # surfaces instead of a wireframe of quads.
            painter.setPen(QPen(color, 0.4))
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
        if self._sim_enabled:
            painter.setPen(QColor(255, 214, 120, 220))
            painter.drawText(
                QRectF(6, h - 18, w - 12, 14),
                Qt.AlignRight | Qt.AlignVCenter,
                "SIM 60Hz",
            )
