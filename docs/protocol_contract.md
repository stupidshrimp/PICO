# Feather Ground Station / Flight Controller Protocol Contract

This document pins the CRSF/ELRS contract between the ground station (GS) and
flight controller (FC). Channel arrays in software are zero-indexed; radio
channel names below are one-indexed.

## Link and framing

| Direction | Frame | Address / sync | Type | Payload | Nominal rate |
| --- | --- | --- | --- | --- | --- |
| GS → FC | CRSF RC channels packed | `0xC8` | `0x16` | 16 × 11-bit CRSF channel values, 22 bytes packed little-endian bitstream | 250 Hz (`4 ms`) |
| FC → GS | CRSF attitude telemetry | `0xC8` or `0xEA` accepted by GS | `0x1E` | pitch, roll, yaw as signed big-endian radians × 10000 | 125 Hz (`8000 us`) |
| FC → GS | CRSF GPS telemetry | `0xC8` or `0xEA` accepted by GS | `0x02` | latitude, longitude, groundspeed/airspeed, course, altitude, satellites | 50 Hz (`20000 us`) |
| FC → GS | CRSF link statistics | `0xEA` | `0x14` | standard ELRS link stats; optional piggybacked telemetry stream | receiver-defined |

The GS validates every inbound frame with CRSF CRC-8/DVB-S2 over frame type and
payload. Unknown telemetry payloads are consumed only when their fixed length is
known; otherwise the parser drops/resynchronizes on the next valid sync byte.

## GS → FC channel map

CRSF channel values use the ELRS/CRSF raw range, not PWM microseconds:
minimum `172`, center `992`, maximum `1811`. The FC maps primary axes to servo
PWM `1000..2000 us` only after receiving the CRSF packet.

| Array index | Radio channel | Name | GS unit / encoding | FC interpretation |
| ---: | ---: | --- | --- | --- |
| `0` | CH1 | Roll | CRSF raw axis, `172..1811`, center `992` | Manual aileron command, or FBW desired roll = normalized axis × `80 deg` |
| `1` | CH2 | Pitch | CRSF raw axis, `172..1811`, center `992` | Manual elevator command, or FBW desired pitch = normalized axis × `80 deg` |
| `2` | CH3 | Throttle / auto-throttle setpoint | Manual: throttle percent mapped to `172..1811`; Auto Throttle: desired airspeed mapped linearly over `0..100 mph` | Manual throttle percent, or FC auto-throttle target airspeed |
| `3` | CH4 | Yaw | CRSF raw axis, `172..1811`, center `992` | Manual rudder command; held/blended to neutral during short RC dropouts |
| `4` | CH5 / AUX1 | ELRS arm/disarm state | Idle startup and shutdown safety frames drive low (`172`); operator-started active transmission drives high (`1811`) | Reserved for link/arm state; FC control modes do not use it |
| `5` | CH6 / AUX2 | Control mode | Low (`400`) = Manual, high (`1700`) = Fly-By-Wire | FBW enabled when value is at least `1550`; otherwise Manual |
| `6` | CH7 / AUX3 | Throttle mode | Low (`400`) = Manual Throttle, high (`1700`) = Auto Throttle | Auto throttle enabled when value is at least `1550`; otherwise Manual Throttle |
| `7..15` | CH8..CH16 | Reserved | Center (`992`) unless future features define them | Ignored by current FC firmware |

## FC mode thresholds and failsafes

| Contract item | Value | Behavior |
| --- | ---: | --- |
| RC fresh timeout | `250000 us` | RC input is fresh while the last decoded RC frame age is ≤ this timeout |
| Servo hold timeout | `3000000 us` | If raw CRSF bytes are still active but decoded RC frames are stale, roll, pitch, and yaw blend from the last command toward neutral over the remaining hold period |
| CRSF byte activity timeout | `250000 us` | Servo hold is allowed only while recent raw bytes indicate the receiver is still talking |
| Control-mode high threshold | `1550` | CH6 values at or above this threshold select Fly-By-Wire |
| Throttle-mode high threshold | `1550` | CH7 values at or above this threshold select Auto Throttle |
| Auto-throttle airspeed freshness timeout | `100000 us` (100 ms) | FC throttle PID is reset when pitot/airspeed data is stale, independent of GPS lock |
| GS shutdown safety burst | 3 CRSF RC frames before TX disable | Roll/pitch/yaw neutral, throttle cut, CH5 low/disarmed, Manual control mode, and Manual throttle mode |

## FC → GS telemetry units

### Attitude (`0x1E`)

Payload order is pitch, roll, yaw. Each value is a signed 16-bit big-endian
integer in radians × 10000. The FC starts from EKF degrees, caches decidegrees,
and the CRSF library converts decidegrees to radians × 10000. The GS converts
back to degrees and negates pitch to undo the CRSF library sign convention.

### GPS (`0x02`)

Payload order and units:

| Field | Encoding | GS unit after decode |
| --- | --- | --- |
| Latitude | signed big-endian `int32`, degrees × `1e7`; `0` when no GPS fix | degrees |
| Longitude | signed big-endian `int32`, degrees × `1e7`; `0` when no GPS fix | degrees |
| Speed | unsigned big-endian `uint16`, km/h | mph (`raw × 0.0621371`) |
| Course | unsigned big-endian `uint16`, degrees × `100` | degrees |
| Altitude | unsigned big-endian `uint16`, meters + `1000` CRSF offset | feet above MSL/baro reference |
| Satellites | unsigned byte | count |

The speed field carries FC pitot airspeed converted to CRSF GPS speed units.
Because this airspeed is sampled separately from GPS, the GS treats a finite
speed value as fresh airspeed telemetry even when latitude/longitude are zero
(no GPS lock). GPS lock state still depends only on finite, non-zero
coordinates.
