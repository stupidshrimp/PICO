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
| `6` | CH7 / AUX3 | Throttle mode / compass-cal request | Low (`400`) = Manual Throttle, high (`1700`) = Auto Throttle, center (`992`, GS "Calibrate compass" button) = on-ground magnetometer-calibration request | Auto throttle enabled when value is at least `1550`; otherwise Manual Throttle. Values inside `891..1091` additionally request the in-field magnetometer calibration, honored only on the ground (not airborne-latched, Manual control mode, throttle stick at minimum) after the band is held for `1 s`; the calibration holds the surfaces in a distinctive pose while sampling, forces throttle cut, and leaving the band ends the run. Ending the run computes a fit and plays exactly one completion signal: a valid fit reaches the flash save, which stalls the FC ~1-2 s while the sector is erased (surfaces freeze at the pose) before signalling, and — once it verifies — is applied live + persisted and acknowledged with one continuous SLOW glide (pose -> min -> max -> center, ~3.5 s, never stepping); any failure keeps the previous calibration and plays a rapid full-travel flutter with no preceding stall on a rejected run — 4 wags = run rejected/aborted (coverage, samples, ground gates), 8 wags = fit was good but the flash save did not verify. While the GS request is active it also forces CH3 to minimum |
| `7` | CH8 / AUX4 | Board-alignment roll trim | Center (`992`) = no delta | Interim on-ground level trim (`FC_BOARD_ALIGN_TRIM_RC`) |
| `8` | CH9 / AUX5 | Board-alignment pitch trim | Center (`992`) = no delta | Interim on-ground level trim (`FC_BOARD_ALIGN_TRIM_RC`) |
| `9` | CH10 / AUX6 | Loiter request | Low (`400`) = off, high (`1700`) = request the fixed-bank orbit | Loiter requested when value is at least `1550`; the FC additionally requires RC fresh, Fly-By-Wire, a fresh and converged attitude estimate, and the airborne latch before the orbit actually flies |
| `10..15` | CH11..CH16 | Reserved | Center (`992`) unless future features define them | Ignored by current FC firmware |

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
| FC FBW attitude filter | _none on P path_ | EKF attitude is fed straight into the FBW PID proportional term (no input prefilter, so command latency is unchanged); only the derivative term is low-pass filtered |
| FC FBW derivative filter | `20 Hz` first-order | `flight_controller/Main.ino` (`FBW_PID_DERIVATIVE_CUTOFF_HZ`); low-passes the measurement-derivative term so EKF jitter is not differentiated into servo dither |
| FC auto-throttle derivative filter | `5 Hz` first-order | `flight_controller/Main.ino` (`AUTO_THROTTLE_DERIVATIVE_CUTOFF_HZ`); rejects pitot noise on the airspeed-rate term |
| FC auto-throttle target range | `0..100 mph` | `flight_controller/Main.ino` (`AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH`) |
| FC auto-throttle default target | `30 mph` | `flight_controller/Main.ino` (`AUTO_THROTTLE_DEFAULT_TARGET_MPH`); GS default `throttle.target_airspeed_mph` is also `30.0`, from `config.DEFAULT_AUTO_THROTTLE_TARGET_MPH`. Raised from `20` so loiter's airspeed floor can sit between the configured stall warning (`20`) and cruise — at a `20` target there was no room for it at all |
| FC auto-throttle PID gains | `Kp=0.8`, `Ki=0.0`, `Kd=0.15` | `flight_controller/Main.ino`; output is integrated into `autoThrottlePercent` (velocity form), so the P term already provides integral action — `Ki` is `0` to avoid a double integrator |
| FC auto-throttle output limit | `±100 percent/s` | `flight_controller/Main.ino` (`AUTO_THROTTLE_OUTPUT_LIMIT_PERCENT_PER_S`) |
| FC auto-throttle stale decay | `50 percent/s` | `flight_controller/Main.ino` (`AUTO_THROTTLE_STALE_DECAY_PERCENT_PER_S`) |

### Loiter (fixed-bank orbit)

Loiter is flown by the **flight controller**. It substitutes a constant desired
bank and a level desired pitch for the pilot's stick inside the existing
Fly-By-Wire branch, so the same attitude PIDs fly the orbit and the aircraft
circles wherever it happens to be. It performs no navigation and drifts
downwind; holding the circle over a point needs a position loop, which does not
exist yet.

The ground station only *requests* the mode on CH10 and never commands an
attitude. That split is deliberate: a pilot reaches for loiter when the model is
far away and the link is weakest, so the loop that holds the wings over has to
live on the aircraft. What stays on the GS is everything involving a joystick.

| Item | Current value | Authority / note |
| --- | ---: | --- |
| Request channel | CH10 / AUX6, index `9` | `flight_controller/loiter_nav.h` (`LOITER_REQUEST_TARGET` `1700`, `LOITER_REQUEST_DEADBAND` `150`, threshold `1550`); GS values in `modules/loiter.py` |
| Commanded bank | `15 deg` | `flight_controller/loiter_nav.h` (`LOITER_BANK_ANGLE_DEG`), clamped into `FBW_MAX_ROLL_ANGLE_DEG`. Radius ≈ `v²/(g·tan θ)`, so about `68 m` (a `137 m` circle) at the `30 mph` auto-throttle default — it was `30 m` at the old `20 mph` target, so raising cruise for stall margin more than doubled the orbit. Fly it somewhere with room, or raise `LOITER_BANK_ANGLE_DEG` to tighten it back up at the cost of more lift shortfall. The shallow bank also halves the lift shortfall the altitude loop has to fight, to about `3.4%` |
| Commanded pitch | altitude hold, P-only | `flight_controller/loiter_nav.h`. The target is the barometric altitude captured **at engage** — not a ground reference and not a ground-station value, so there is nothing for the two sides to disagree about and it works from a watchdog-recovery boot that never captured a ground reference. `pitch = clamp(0.5 deg/m × error, ±10 deg)`, then clamped again into the FBW envelope |
| Altitude loop: no integrator | deliberate | An integrator against an airframe whose phugoid period has not been measured will wind up and porpoise. The price is steady-state droop: the orbit settles a few metres low, because holding height in a bank needs a standing nose-up command and only a standing error can produce one without an integrator |
| Airspeed floor | `24 mph`, outranks altitude | `flight_controller/loiter_nav.h` (`LOITER_MIN_AIRSPEED_MPH`). A loop that pitches up to hold height while the throttle cannot sustain the climb flies the wing to a stall. Below the floor the hold is abandoned and pitch returns to **level**, which descends and regains speed — deliberately not a nose-down command, because level pitch is exactly the un-held orbit and therefore never worse than not having the feature. It must sit **between** stall and cruise: `24` is `1.2×` the configured `20 mph` stall warning and `0.8×` the `30 mph` cruise target. In the `15°` bank stall rises `1.8%` to about `20.4 mph`, leaving roughly `1.18×`. A test reads all three from the files that own them and fails if the ordering breaks. **`20` is a configured warning threshold, not a measured stall speed** — if a flight test measures the real number, reset the floor, the cruise target and `warnings.stall_airspeed` together |
| Barometer gates the airborne latch too | `aircraftAirborne && barometerInputFresh` | `flight_controller/Main.ino` (`loiterAirborneTrustworthy`). The latch clears on **height** alone, and height comes from a `sensorAltitudeCm` that a failed barometer leaves frozen — so the latch pins set, `updateAirborneState` never sees the disengage height, and the landing is never detected. Passing the raw flag would let the orbit go on commanding its bank after touchdown, on the runway. Without a trustworthy altitude the FC cannot tell flight from a rollout, so it drops the orbit; that costs nothing real, since the same failure has already disabled altitude hold |
| Barometer freshness | `250 ms` | `flight_controller/Main.ino` (`BAROMETER_FAILSAFE_TIMEOUT_US`, `barometerInputFresh`). `barometerHealthy` only reports an **initialisation** failure and is never cleared at runtime, and a failed `readAdc()` returns the state machine to IDLE without touching `sensorAltitudeCm` — so a sensor that stops answering mid-flight leaves the last altitude cached forever while looking healthy. Anything closing a loop on altitude must check freshness explicitly. The window clears the normal gap caused by the `500 ms` temperature conversion by an order of magnitude |
| Altitude-hold degraded paths | fall back to level pitch | Reachable in this firmware: **stale pitot**, or **airspeed below the floor**. In both the orbit itself continues and only the hold is abandoned. The two barometer paths in `loiterDesiredAttitude` (no target captured at engage, unusable barometer now) are **unreachable here** and must not be read as describing this aircraft: `loiterAirborneTrustworthy` folds barometer freshness into the airborne gate (row above), so an unusable barometer ends the orbit outright rather than flying it un-held — without a trustworthy altitude the FC cannot detect a landing at all. Those branches are kept only because `loiter_nav.h` is caller-agnostic |
| Bank direction | `+1` | `flight_controller/loiter_nav.h` (`LOITER_BANK_DIRECTION`); follows the FC roll convention (left roll positive), **not** a compass direction. Which way the model circles is a first-flight observation — flip the sign and reflash if it turns the wrong way |
| FC engage gates | request AND RC fresh AND FBW AND attitude fresh+converged AND airborne (**with a live barometer**) | `flight_controller/loiter_nav.h` (`loiterMayEngage`); re-evaluated every control cycle, so losing any condition hands the stick straight back with no transition step |
| FC rising-edge requirement | a request must be flyable **when it arrives** | `flight_controller/loiter_nav.h` (`loiterUpdate`), advanced once per control cycle *before* the servo-mode branch chain and with the real gate values — not inside the Fly-By-Wire branch, which would skip the latch entirely whenever a failing gate routed control elsewhere (an attitude outage takes the pass-through branch, so the orbit would resume silently on recovery, and a request raised during watchdog convergence would first be seen only after it, reading as a valid edge). The gate above is a continuous predicate, so a request left standing on CH10 while the aircraft was grounded would otherwise begin the orbit the instant the airborne latch set after takeoff, with no operator action. A request that was not flyable on its rising edge stays refused until CH10 drops and rises again; likewise a gate lost mid-orbit does not silently resume when it returns |
| FC behaviour on RC failsafe | orbit stops, request stays latched | `flight_controller/Main.ino`. The `rcFresh` gate in `loiterUpdate` stops the orbit; `navMode` is deliberately **not** forced off and the latch is **not** re-armed. Forcing it off would make the standing CH10 look released, so the first recovered packet would read as a fresh rising edge and the aircraft would resume a 20 degree orbit on its own — attitude disturbed by the failsafe blend, throttle cut, pilot given no say. Loiter does **not** auto-resume after a dropout: the operator must cycle CH10. This deliberately differs from Fly-By-Wire, which does resume, because resuming an assistance mode only restores what the pilot's stick means whereas resuming an autonomous mode flies the aircraft with no input |
| PID handling on transition | roll/pitch integrators reset on **effective** state change | `flight_controller/Main.ino` (`loiterState.transitioned`), deliberately not on `setNavMode`. The requested mode and the flown state are not the same thing: the airborne latch can start or stop an orbit under a standing request, and a request raised on the ground never flies at all. Resetting on the request would miss those transitions and would also dump the integrator while the pilot is still hand-flying Fly-By-Wire |
| GS engage gesture | hold `2.0 s` | `config.py` (`loiter.hold_seconds`); the control-mode toggle (Ctrl+M or joystick button `13`) held this long. A short tap of the same control still toggles Manual/Fly-By-Wire, so that toggle now fires on the RELEASE edge |
| GS engage gates | transmitting, FBW, attitude fresh, joystick live, airborne **by both detectors**, flight metrics fresh | `main.py` (`_loiter_gates`); the airborne check is what makes a hold on the ground fail *audibly* at the gesture instead of quietly raising CH10 for the FC to refuse |
| GS/FC airborne agreement | asymmetric: engage re-derives, hold uses a latch | `modules/loiter.py` (`fc_airborne_latched`, mirroring `AIRBORNE_ENGAGE_AIRSPEED_MPS`; the test suite reads that macro out of `Main.ino` so the mirror cannot drift). Airspeed only **sets** the flag — the firmware clears it on the disengage HEIGHT alone, and keeps orbiting below its airspeed floor by design, abandoning only altitude hold. Re-testing the engage threshold every cycle would therefore drop a running orbit the FC was happy to keep flying. Hence the split: `fc_airborne_engage_ok` re-derives **both halves** of the FC's engage condition — airspeed `≥ 17.9 mph` AND height `≥ 9.84 ft` above the ground reference (`AIRBORNE_ENGAGE_HEIGHT_M`, also read from the firmware by the tests) — to authorise a **new** request (a request the FC refuses is stranded permanently by the rising-edge rule, so a stale belief must never authorise one), while `fc_airborne_latched` decides whether a **running** orbit continues (a false negative there interrupts flight on a ground-station guess, which is the worse error).
Height is where the detectors diverge most: the FC clears its latch at `AIRBORNE_DISENGAGE_HEIGHT_M` (`1.5 m`) and will not set it again below `3 m`, while the GS needs low speed **and** low altitude sustained for its landing debounce — so through a fast low pass or touch-and-go the GS stays airborne at cruise speed while the FC has already gone grounded.
**The latch is knowingly imperfect and cannot be made exact.** After a watchdog reset in flight the firmware runs a separate branch with no ground reference, where airspeed below `AIRBORNE_RECOVERY_DISENGAGE_AIRSPEED_MPS` (`6 m/s`) *does* clear the flag — and nothing in the downlink reports `watchdogRecoveryBoot`, so the GS cannot follow it. The consequence is confined to a stale display, which is the same blind spot that makes the indicator read "Loiter req"; it cannot authorise anything, because engagement does not consult the latch. The two detectors are independent and the GS one can be the **laxer**: with the default warning config it latches at `12 mph` (`stall_airspeed` `10` × `takeoff_airspeed_multiplier` `1.2`) while the FC waits for `17.9`. A hold in that window raises CH10, the FC refuses it as grounded, and the rising-edge rule means it stays refused even after the FC latches airborne — the orbit never flies while the operator has been told it is. Requiring the stricter of the two closes the window |
| GS indication | **"Loiter req"**, never "Loiter" | The ground station drives CH10 but nothing in the downlink reports the FC's nav state, so it cannot know the orbit is flying and must not claim to. The `loiter_requested` blackbox column is named on the same principle. Confirmation is watching the aircraft turn, or the FC's `nav=` / `loiter_hz` debug output (row below) |
| GS synchronous teardown | loiter aborted at the teardown itself | `main.py` (`_abort_loiter`), called when the CRSF transport is replaced, the joystick handler errors out or is reselected, and when transmission stops. The periodic gates only catch a condition still false on the next tick, and a transport rebuilt inside one GUI callback never presents one — so a running orbit would survive the swap, and the rebuilt transport would be seeded with CH10 already high for a reconnecting FC to read as a fresh request edge. Cancelling a pending press is not sufficient here: that is deliberately a no-op once the hold has matured |
| GS transmission gate | a genuinely open uplink, not just intent | `main.py` reuses the TX indicator's own test (`transmission_active` **and** `_crsf_serial_link_up()`). Terminating transmission from the configuration page stops RC frames while telemetry keeps arriving, so every other gate can stay satisfied with nothing reaching the aircraft: engaging then would arm CH10 locally and the next "start transmitting" click would carry it high as a fresh FC-side edge, making that click — not a loiter gesture — the thing that starts the orbit. A pending hold is also cancelled when transmission stops, when the joystick handler errors out, and when the joystick port is reselected, because the release edge that would have ended the hold dies with the old handler |
| GS-side drop conditions | toggle press, stick moved, FBW lost, attitude stale, joystick lost or stalled, no longer airborne, timeout | `modules/loiter.py`; these lower CH10. They are one of two independent ways the orbit ends — the FC gates above are the other |
| GS flight-metric freshness | GPS packet and airspeed within `airborne.gps_fresh_timeout_s` (`2 s`) | `main.py` (`_loiter_gates`) using the same `_is_packet_fresh` / `_airspeed_value_fresh` helpers as the airborne detector and auto-trim. When GPS telemetry stops while attitude and the uplink stay live, `_update_airborne_state` deliberately freezes `airborne_state` and the cached speed/altitude rather than guessing — so a landing during that outage leaves the GS reporting the last airborne values it saw. Stale readings are treated as **unavailable**, which `fc_airborne_engage_ok` already refuses on |
| GS joystick liveness | fresh sample within `0.5 s` | `main.py` uses `_last_stick_sample_time` against `AUTO_TRIM_STICK_STALE_S`, **not** the existence of a handler object: `get_raw_values()` returns cached axes when the serial stream stalls, so an existence check would leave loiter engaged while stick movement and button releases stopped being observable — disabling the pilot's primary way out |
| GS stick-break threshold | `0.25` normalized, clamped to `0.02..0.75` | `config.py` (`loiter.stick_break_norm`); measured against where the stick sat at engage time, not against centre, and against the **physical** stick position rather than the sensitivity-scaled command (`get_physical_values`). Sensitivity shapes how much command a deflection produces and must not decide whether the pilot can take over: at 25% a full deflection only reaches `0.25` of the processed range, so a threshold applied there is uncrossable. The upper clamp is below full travel on purpose: a centred stick has exactly `1.0` of travel and the comparison is strictly greater-than, so a cap of `1.0` would leave full deflection unable to break out — the very failure the cap exists to prevent |
| GS hold validation | non-positive → default, floored at `0.5 s` | `modules/loiter.py`; `config.json` is hand-editable and a zero or negative hold makes the first poll tick satisfy the gesture, so an ordinary tap spanning one GUI refresh would engage an autonomous mode |
| GS input-source matching | releases matched to the control that pressed | `modules/loiter.py` (`PRESS_SOURCE_KEY` / `PRESS_SOURCE_JOYSTICK`); an unmatched joystick release arriving while Ctrl+M is held would otherwise consume the keyboard press and toggle the flight mode with the key still down. Presses are honoured from either source regardless, because a press is the disengage path and must never be blocked |
| FC debug nav state | `OFF` / `LOITER_REQ` / `LOITER` | `flight_controller/Main.ino`. `LOITER_REQ` means CH10 is high but a gate refused the orbit; `LOITER` means it is actually flying (`loiterState.running`). The field deliberately does **not** print the requested mode, for the same reason the GS indicator reads "Loiter req": it is the one readout that can tell a refused request from a flown orbit, and the two rows above send operators here to do exactly that. `loiter_hz` corroborates it — non-zero only while the orbit runs |
| GS attitude freshness window | `200 ms`, matching the FC | `main.py` (`LOITER_ATTITUDE_STALE_S`) = `ATTITUDE_STALE_TIMEOUT_US`; the test suite reads the firmware value so the two cannot drift. Deliberately **not** the `1.0 s` window `check_attitude_connection` uses — that answers "is the link up". The FC drops the orbit as soon as its estimate goes stale and the rising-edge rule then refuses to restart it, so a laxer GS window leaves a band (`200 ms`–`1 s` of lost attitude) where the FC has handed the stick back while the GS holds CH10 high and never cycles it: the orbit is stranded, unannounced. Matching exactly is sound rather than another approximation, because the FC **stops sending** attitude when the estimate goes stale (`telemetryWriteAttitude` is gated on `attitudeSampleValid` = `attitudeEstimateFresh`, the same predicate the loiter gate uses), so a gap in arrivals means what a gap means there |
| GS maximum orbit duration | disabled (`0`) | `config.py` (`loiter.max_duration_s`). Defaulted off now that the FC holds altitude: the orbit no longer runs out of height, so an arbitrary clock is a worse bound than the gates that track conditions actually worth stopping for. Set a positive value to re-enable |
| Blackbox column | `loiter_requested` | `main.py`; records the GS **request**, not the flown orbit. The FC's gates (airborne latch, converged attitude) are not reported down, so "requested and refused" is indistinguishable from "requested and flown" in a sortie log. The FC's own `loiter_hz` / `nav=` debug counters are the authority on whether an orbit actually flew (row below) |

## FC mode thresholds and failsafes

| Contract item | Value | Behavior | Code authority / discrepancy note |
| --- | ---: | --- | --- |
| RC fresh timeout | `250000 us` (250 ms) | RC input is fresh while the last decoded RC frame age is ≤ this timeout | `flight_controller/Main.ino` (`RC_FAILSAFE_TIMEOUT_US`) |
| Servo hold timeout | `500000 us` (500 ms) | If raw CRSF bytes are still active but decoded RC frames are stale, roll, pitch, and yaw blend from the last command toward neutral after the RC fresh timeout until this total age expires | `flight_controller/Main.ino` (`RC_SERVO_HOLD_TIMEOUT_US`); this replaces the older documented `3000000 us` value |
| CRSF byte activity timeout | `250000 us` (250 ms) | Servo hold is allowed only while recent raw bytes indicate the receiver is still talking | `flight_controller/Main.ino` (`CRSF_BYTE_ACTIVITY_TIMEOUT_US = RC_FAILSAFE_TIMEOUT_US`) |
| Control-mode high threshold | `1550` | CH6 values at or above this threshold select Fly-By-Wire | `CONTROL_MODE_FLY_BY_WIRE_TARGET` `1700` minus `CONTROL_MODE_SWITCH_DEADBAND` `150` |
| Throttle-mode high threshold | `1550` | CH7 values at or above this threshold select Auto Throttle | `THROTTLE_MODE_AUTO_TARGET` `1700` minus `THROTTLE_MODE_SWITCH_DEADBAND` `150` |
| Compass-cal request band | `891..1091` | CH7 values inside this band (below the Auto Throttle threshold, so throttle mode stays Manual) request the on-ground magnetometer calibration; the FC starts sampling only after the band is held `1 s` with every ground gate satisfied, and ends the run when the band is left | `flight_controller/Main.ino` (`MAG_CAL_REQUEST_MIN/MAX` = `RC_INPUT_CENTER ± 100`, `MAG_CAL_TRIGGER_HOLD_US`); GS value `992` in `modules/compass_cal.py` (`COMPASS_CAL_CHANNEL_VALUE`) |
| FBW stale-attitude fallback | `200000 us` (200 ms) attitude staleness | While CH6 selects Fly-By-Wire but the EKF attitude estimate is stale (dead IMU, wedged bus, persistent EKF failures) or not yet converged after a watchdog-recovery boot, the FC passes the roll/pitch channels straight through to the servos instead of closing the PID loop. Because the GS scales those channels by its FBW command limits (default 45/80 roll, 30/80 pitch of the FC hard limit), manual authority in this fallback is limited to that fraction of full servo travel; switching the GS to Manual restores full-range channels. GS-side indication differs by cause: in the stale case attitude telemetry stops (`attitudeSampleValid` tracks freshness), raising the GS "telemetry offline" alarm about a second later — but in the convergence case (watchdog-recovery boot, innovation-gate warmup, ~2 s at the 125 Hz correction rate) the estimate is fresh, attitude telemetry keeps flowing, and the GS gets no alarm while the limited-authority pass-through is active. | `flight_controller/Main.ino` (`ATTITUDE_STALE_TIMEOUT_US`, `attitudeEstimateConvergedForFbw()`, `EKF_INNOVATION_GATE_WARMUP_UPDATES`); GS scaling in `main.py` (`_apply_fbw_command_limits`) |
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
| GS channel staleness timeout | `2.0 s` by default | The GS transmit pacer stops writing RC frames when the last fresh channel update from the GUI thread is older than this window (e.g. a GUI stall). Transmission resumes immediately on the next update. Halting TX lets the FC RC-fresh timeout engage. Configurable via `CRSFPacketProcessor(channel_stale_timeout_s=…)`; a non-positive value disables the watchdog. | `config.py` (`channel_stale_timeout_s` default) and `pico_modules/pico_transmitpackets.py` (`CRSFPacketProcessor.RC_CHANNEL_STALE_TIMEOUT_S` fallback); this replaces the older documented `200 ms` value |
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
- The cached angle conventions are unchanged: right rolls are negative, left
  rolls are positive, pitch is nose-up positive, yaw is compass-style. Since
  the EKF body frame moved to Z-down (proper right-handed), these signs come
  directly out of the quaternion-to-Euler conversion
  (`quaternionToEulerDeg` in `Main.ino`) with no post-hoc roll flip; the
  emitted values are numerically identical to earlier firmware (proven in
  `flight_controller/tests/frame_consistency_test.cpp`).
- Pitch is emitted through the CRSF attitude helper, whose sign convention is
  undone in the GS decoder.

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
