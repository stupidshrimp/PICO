# Feather Ground Station / Flight Controller Protocol Contract

This document pins the CRSF/ELRS contract between the ground station (GS) and
flight controller (FC). Channel arrays in software are zero-indexed; radio
channel names below are one-indexed.

The values below were reconciled against the current GS and FC code. When a
previous document value disagreed with code, this document now follows the code
and calls out the authoritative symbol/file so future updates can be verified.

## Link and framing

| Direction | Frame | Address / sync | Type | Payload | Nominal rate |
| --- | --- | --- | --- | --- | --- |
| GS → FC | CRSF RC channels packed | `0xC8` | `0x16` | 16 × 11-bit CRSF channel values, 22 bytes packed little-endian bitstream | Configurable: `100 Hz` (`10 ms`), `250 Hz` (`4 ms`, default), or `500 Hz` (`2 ms`) |
| FC → GS | CRSF attitude telemetry | `0xC8` or `0xEA` accepted by GS | `0x1E` | pitch, roll, yaw as signed big-endian radians × 10000 | 125 Hz (`8000 us`) |
| FC → GS | CRSF GPS telemetry | `0xC8` or `0xEA` accepted by GS | `0x02` | latitude, longitude, groundspeed/airspeed, course, altitude, satellites | 50 Hz (`20000 us`) |
| FC → GS | CRSF battery telemetry | `0xC8` or `0xEA` accepted by GS | `0x08` | voltage, current, capacity, optional percent | receiver-/producer-defined |
| FC → GS | CRSF link statistics | `0xEA` | `0x14` | standard 10-byte CRSF/ELRS link stats | receiver-defined |
| RX/handset → GS | Handset timing synchronization | `0xC8` or `0xEA` accepted by GS | `0x3A` | optional destination/origin bytes, then subtype plus rate/offset timing fields | receiver-defined |

The GS validates every inbound frame with CRSF CRC-8/DVB-S2 over frame type and
payload. Inbound frame lengths must be at least `2` bytes and at most `64`
bytes. The GS accepts telemetry addressed as either `0xC8` or `0xEA`, drops
invalid length/CRC frames one byte at a time to resynchronize, decodes known
fixed-length payloads, and counts unknown payloads without emitting telemetry.

## GS → FC channel map

CRSF channel values use the ELRS/CRSF raw range, not PWM microseconds:
minimum `172`, center `991` on the FC (`992` on the GS), maximum `1811`. This
one-count center discrepancy is intentional in the code today: the FC computes
`(172 + 1811) / 2` with integer truncation, while the GS constant is `992`.
Both sides clamp incoming channel values to `172..1811`; the FC maps primary
axes to servo PWM `1000..2000 us` only after receiving the CRSF packet.

| Array index | Radio channel | Name | GS unit / encoding | FC interpretation |
| ---: | ---: | --- | --- | --- |
| `0` | CH1 | Roll | CRSF raw axis, `172..1811`, center `992`; optional trim and GS FBW command limiting apply before transmit | Manual aileron command, or FBW desired roll = normalized axis × `80 deg` (FC hard limit) |
| `1` | CH2 | Pitch | CRSF raw axis, `172..1811`, center `992`; optional trim and GS FBW command limiting apply before transmit | Manual elevator command, or FBW desired pitch = normalized axis × `80 deg` (FC hard limit) |
| `2` | CH3 | Throttle / auto-throttle setpoint | Manual: throttle percent mapped to `172..1811`; Auto Throttle: desired airspeed mapped linearly over `0..100 mph` | Manual throttle percent, or FC auto-throttle target airspeed |
| `3` | CH4 | Yaw | CRSF raw axis, `172..1811`, center `992` | Manual rudder command; held/blended to neutral during short RC decode gaps |
| `4` | CH5 / AUX1 | ELRS arm keepalive | GS drives high (`1811`) | Reserved for link/arm state; FC control modes do not use it |
| `5` | CH6 / AUX2 | Control mode | Low (`400`) = Manual, high (`1700`) = Fly-By-Wire | FBW enabled when value is at least `1550`; otherwise Manual |
| `6` | CH7 / AUX3 | Throttle mode | Low (`400`) = Manual Throttle, high (`1700`) = Auto Throttle | Auto throttle enabled when value is at least `1550`; otherwise Manual Throttle |
| `7..15` | CH8..CH16 | Reserved | Center (`992`) unless future features define them | Ignored by current FC firmware |

### Command limits and tuning ownership

| Item | Current value | Authority / note |
| --- | ---: | --- |
| GS-supported packet rates | `100`, `250`, `500 Hz` | `config.py` (`ALLOWED_ATTITUDE_PACKET_RATES_HZ`); invalid intervals are normalized to the nearest supported rate |
| GS default packet rate | `250 Hz` (`4 ms`) | `config.py` (`DEFAULT_ATTITUDE_PACKET_RATE_HZ`) and `CRSFPacketProcessor(packet_interval_ms=4)` |
| GS FBW roll command limit | `45 deg` | `config.py` default `fbw.max_roll_angle_deg`; this limits the transmitted stick command before the FC applies its `80 deg` safety clamp |
| GS FBW pitch command limit | `30 deg` | `config.py` default `fbw.max_pitch_angle_deg`; this limits the transmitted stick command before the FC applies its `80 deg` safety clamp |
| FC FBW roll/pitch hard limit | `80 deg` | `flight_controller/Main.ino` (`FBW_MAX_ROLL_ANGLE_DEG`, `FBW_MAX_PITCH_ANGLE_DEG`) |
| FC FBW PID gains | Roll `Kp=5.0`, `Ki=0.25`, `Kd=0.9`; Pitch `Kp=6.0`, `Ki=0.30`, `Kd=1.1` | `flight_controller/Main.ino` |
| FC FBW PID output limit | `±400 us` | `flight_controller/Main.ino` (`FBW_PID_OUTPUT_LIMIT_US`) |
| FC FBW attitude filter | _none_ | EKF attitude is fed straight into the FBW PID; no output low-pass (removed to minimize control-loop latency) |
| FC auto-throttle target range | `0..100 mph` | `flight_controller/Main.ino` (`AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH`) |
| FC auto-throttle default target | `20 mph` | `flight_controller/Main.ino` (`AUTO_THROTTLE_DEFAULT_TARGET_MPH`); GS default `throttle.target_airspeed_mph` is also `20.0` |
| FC auto-throttle PID gains | `Kp=0.8`, `Ki=0.04`, `Kd=0.15` | `flight_controller/Main.ino` |
| FC auto-throttle output limit | `±100 percent/s` | `flight_controller/Main.ino` (`AUTO_THROTTLE_OUTPUT_LIMIT_PERCENT_PER_S`) |
| FC auto-throttle stale decay | `50 percent/s` | `flight_controller/Main.ino` (`AUTO_THROTTLE_STALE_DECAY_PERCENT_PER_S`) |

## FC mode thresholds and failsafes

| Contract item | Value | Behavior | Code authority / discrepancy note |
| --- | ---: | --- | --- |
| RC fresh timeout | `250000 us` (250 ms) | RC input is fresh while the last decoded RC frame age is ≤ this timeout | `flight_controller/Main.ino` (`RC_FAILSAFE_TIMEOUT_US`) |
| Servo hold timeout | `500000 us` (500 ms) | If raw CRSF bytes are still active but decoded RC frames are stale, roll, pitch, and yaw blend from the last command toward neutral after the RC fresh timeout until this total age expires | `flight_controller/Main.ino` (`RC_SERVO_HOLD_TIMEOUT_US`); this replaces the older documented `3000000 us` value |
| CRSF byte activity timeout | `250000 us` (250 ms) | Servo hold is allowed only while recent raw bytes indicate the receiver is still talking | `flight_controller/Main.ino` (`CRSF_BYTE_ACTIVITY_TIMEOUT_US = RC_FAILSAFE_TIMEOUT_US`) |
| Control-mode high threshold | `1550` | CH6 values at or above this threshold select Fly-By-Wire | `CONTROL_MODE_FLY_BY_WIRE_TARGET` `1700` minus `CONTROL_MODE_SWITCH_DEADBAND` `150` |
| Throttle-mode high threshold | `1550` | CH7 values at or above this threshold select Auto Throttle | `THROTTLE_MODE_AUTO_TARGET` `1700` minus `THROTTLE_MODE_SWITCH_DEADBAND` `150` |
| Auto-throttle airspeed freshness timeout | `100000 us` (100 ms) | FC throttle PID is reset when pitot/airspeed data is stale, independent of GPS lock | `flight_controller/Main.ino` (`AIRSPEED_FAILSAFE_TIMEOUT_US`) |
| Auto-throttle stale behavior | Decays by `50 percent/s` toward `0%` | If Auto Throttle is active but airspeed is stale, the FC resets the throttle PID and ramps down `autoThrottlePercent`; full RC loss still cuts throttle immediately | `flight_controller/Main.ino` (`AUTO_THROTTLE_STALE_DECAY_PERCENT_PER_S`) |

## GS-side failsafes

The FC failsafes above only engage when RC frames or raw CRSF bytes stop
arriving. Because the GS transmit pacer repeats the last channel set at the
CRSF rate, the GS must stop producing frames when its own control pipeline
goes stale; otherwise the FC would keep seeing a healthy link carrying frozen
commands.

| Contract item | Value | Behavior | Code authority / discrepancy note |
| --- | ---: | --- | --- |
| GS channel staleness timeout | `2.0 s` by default | The GS transmit pacer stops writing RC frames when the last fresh channel update from the GUI thread is older than this window (e.g. a GUI stall). Transmission resumes immediately on the next update. Halting TX lets the FC RC-fresh timeout engage. Configurable via `CRSFPacketProcessor(channel_stale_timeout_s=…)`; a non-positive value disables the watchdog. | `config.py` and `pico_modules/pico_transmitpackets.py` (`RC_CHANNEL_STALE_TIMEOUT_S`); this replaces the older documented `200 ms` value |
| Joystick loss | n/a | On joystick serial loss the GS centres roll/pitch (CH1/CH2) and cuts throttle to `0`, reverting CH7 to Manual Throttle, so the aircraft glides rather than holding the last commanded power. | `main.py` control-channel construction and joystick-loss handling |
| Raw channel sanitation | `16` channels, clamped to `172..1811` | The GS truncates extra channels, pads missing channels with center, coerces invalid values to center, and clamps all values before packing. | `CRSF_CHANNEL_COUNT`, `CRSF_CHANNEL_MIN`, `CRSF_CHANNEL_MAX`, `CRSF_CHANNEL_CENTER` in `pico_modules/pico_transmitpackets.py` |

## FC → GS telemetry units

### Attitude (`0x1E`)

Payload order is pitch, roll, yaw. Each value is a signed 16-bit big-endian
integer in radians × 10000. The FC starts from EKF degrees, caches decidegrees,
and the CRSF library converts decidegrees to radians × 10000. The GS converts
back to degrees and negates pitch to undo the CRSF library sign convention.

Current FC attitude production details:

- EKF/cache cadence is `8 ms` (`SS_DT_MILIS`), matching the `125 Hz` attitude
  telemetry period.
- Roll is sign-inverted before being cached so right rolls are negative and left
  rolls are positive.
- Pitch is cached directly from the EKF but emitted through the CRSF attitude
  helper, whose sign convention is undone in the GS decoder.

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

Current FC telemetry cache/update rates:

| Sensor/cache | Period | Rate | Notes |
| --- | ---: | ---: | --- |
| EKF attitude / attitude telemetry | `8000 us` | `125 Hz` | Attitude telemetry is sent only after a valid attitude sample exists |
| GPS UART drain/cache | `20000 us` | `50 Hz` | Updates cached coordinates, satellites, and course when the parser reports a valid fix |
| GPS telemetry frame | `20000 us` | `50 Hz` | Sends cached GPS plus latest airspeed/barometer data |
| Barometer cache | `16667 us` | ~`60 Hz` | Produces `sensorAltitudeCm` for CRSF GPS altitude |
| Airspeed cache | `16667 us` | ~`60 Hz` | Produces `airSpeedCms` for CRSF GPS speed and `latestAirspeedMph` for auto throttle |

### Battery (`0x08`)

The GS decoder accepts a minimum 6-byte payload and emits battery telemetry as:

| Field | Encoding | GS unit after decode |
| --- | --- | --- |
| Voltage | little-endian `uint16` | volts, `(raw + 5) / 10` |
| Current | little-endian `uint16` | amps, `raw / 10` |
| Capacity | little-endian `uint16` | mAh / producer-defined capacity units |
| Percent | optional unsigned byte | percent |

### Link statistics (`0x14`)

The GS decodes the standard 10-byte CRSF link-statistics payload and emits:
RSSI A, RSSI B, uplink link quality, uplink SNR, downlink link quality, and
downlink SNR. Extra trailing bytes are ignored.

### Handset timing synchronization (`0x3A`)

The GS accepts either a compact 9-byte timing payload or an extended payload
with destination and origin bytes prepended. It emits the subtype, raw rate,
raw offset, and any decoded destination/origin addresses.

### Custom telemetry (`0xF0`)

The GS consumes 16-byte custom telemetry payloads to maintain stream
synchronization, but it does not currently emit application telemetry for this
frame type.
