# Maiden Flight Verification & Test Plan

Custom ELRS/CRSF radio system — PICO Ground Station (GS) + Feather Flight
Controller (FC)

**Status:** Draft for first flight · **Aircraft:** fixed-wing ·
**Airframe/serial:** _____________ · **Date:** _____________ ·
**Pilot-in-command:** _____________ · **GS operator:** _____________

---

## 1. Purpose and scope

This plan defines the requirements the maiden flight must satisfy and the
verification steps that prove them, for the custom radio system that links the
PICO ground station to the Feather flight controller over **ELRS / CRSF**.

The system under test is **the radio link and its behavior**, not the airframe's
aerodynamics. The maiden flight's job is to prove that:

- commands the operator sends reach the aircraft correctly and with acceptable
  latency,
- telemetry the aircraft sends is decoded correctly on the GS,
- every failsafe behaves exactly as the protocol contract specifies, and
- the two control modes (Manual / Fly-By-Wire) and two throttle modes
  (Manual / Auto Throttle) each do what they should and hand off cleanly.

The airframe is assumed already trimmed for a conventional maiden (CG, control
throws, thrust line). Aerodynamic first-flight practice is out of scope except
where it interacts with the radio system.

Authority for every numeric value below is `docs/protocol_contract.md` and the
cited source symbols in `flight_controller/Main.ino`, `config.py`, and
`pico_modules/pico_transmitpackets.py`. If code and this plan ever disagree,
code wins and this plan is stale — re-verify against the contract before flying.

### 1.1 System summary (context for the tests)

| Path | Frame | Rate | Notes |
| --- | --- | --- | --- |
| GS → FC RC channels | CRSF `0x16`, 16×11-bit | 250 Hz default (100/250/500 selectable) | commands |
| FC → GS attitude | CRSF `0x1E` | 125 Hz | pitch/roll/yaw |
| FC → GS GPS+air | CRSF `0x02` | 50 Hz | lat/lon/speed/course/alt/sats + pitot airspeed |
| FC → GS battery | CRSF `0x08` | RX-defined | V/I/mAh/% |
| FC → GS link stats | CRSF `0x14` | RX-defined | RSSI A/B, uplink/downlink LQ + SNR |

Control mode on **CH6** (AUX2): Manual `<1550`, Fly-By-Wire `≥1550`.
Throttle mode on **CH7** (AUX3): Manual `<1550`, Auto Throttle `≥1550`.
**CH5** (AUX1) is the ELRS arm keepalive driven high by the GS; the FC control
modes do not use it (arming is at the ELRS/ESC level).

---

## 2. Requirements

Each requirement is testable and has a verification method: **I**nspection,
**A**nalysis, **D**emonstration, or **T**est. IDs are referenced by the
procedures in §4. A requirement is **satisfied** only when its pass criterion is
met and recorded.

### 2.1 Link establishment and integrity

| ID | Requirement | Method | Pass criterion |
| --- | --- | --- | --- |
| **R-LNK-01** | The GS establishes a bidirectional CRSF link to the FC's ELRS receiver. | D | GS shows link stats (`0x14`) updating; FC receives RC frames (surfaces respond). |
| **R-LNK-02** | Uplink RC frames are transmitted at the configured rate and accepted by the FC. | T | Selected rate (default 250 Hz) confirmed; surfaces track sticks with no dropouts on the bench. |
| **R-LNK-03** | The GS validates every inbound frame (CRC-8/DVB-S2) and resynchronizes on corruption without crashing or freezing telemetry. | A/T | No decoder stall under induced noise; invalid frames dropped, stream re-locks. |
| **R-LNK-04** | Link margin is adequate at the intended operating range. | T | Range check (§4.3): LQ stays 100% and RSSI within spec while walking to ≥ intended max range with the model powered. |
| **R-LNK-05** | RSSI, LQ, and SNR are displayed on the GS and are sane (LQ ~100%, RSSI strong) at close range. | D | Values present and plausible before flight. |

### 2.2 Command path (GS → FC)

| ID | Requirement | Method | Pass criterion |
| --- | --- | --- | --- |
| **R-CMD-01** | Each control axis drives the correct surface in the correct direction. | D | Roll/pitch/yaw stick → correct aileron/elevator/rudder deflection; verified per the pre-flight "control surface check". |
| **R-CMD-02** | Channel values are clamped to `172..1811` and center is respected on both sides. | A | GS sanitizes to 16 channels, clamps range, pads/centers missing channels; FC clamps identically. (Note the intentional 991/992 one-count center difference.) |
| **R-CMD-03** | End-to-end command latency (stick → surface) is low enough for manual flight. | T | Subjectively immediate on bench; measured latency test (`tests/test_joystick_latency.py`) within its bound. |
| **R-CMD-04** | Throttle maps correctly and the throttle-cut/kill path drives the motor to zero. | D | Throttle low → motor off; kill engaged before all handling. |
| **R-CMD-05** | AUX arm keepalive (CH5) is driven high by the GS so the ELRS/ESC arm path stays alive during operation. | I/D | CH5 high throughout; loss of keepalive disarms as designed. |

### 2.3 Telemetry path (FC → GS)

| ID | Requirement | Method | Pass criterion |
| --- | --- | --- | --- |
| **R-TLM-01** | Attitude telemetry (`0x1E`) is decoded to correct degrees with correct sign conventions. | D/A | Physically pitch/roll/yaw the airframe: GS attitude matches (right roll negative, left positive, nose-up positive pitch, compass-style yaw). |
| **R-TLM-02** | GPS telemetry (`0x02`) decodes lat/lon/speed/course/alt/sats correctly; GPS lock is reported only with a real fix. | D | Position matches known location; sats climb; lock indicated only on non-zero coordinates. |
| **R-TLM-03** | Pitot airspeed is shown even without GPS lock (speed field treated as fresh airspeed). | D | Blow into pitot → airspeed reads on GS with no GPS fix. |
| **R-TLM-04** | Barometric altitude decodes with the +1000 m CRSF offset removed and tracks real altitude changes. | D | Raise/lower aircraft → altitude tracks; ground reference sane. |
| **R-TLM-05** | Battery telemetry (`0x08`) voltage/current read correctly. | D | GS voltage within ±0.2 V of a metered pack voltage. |
| **R-TLM-06** | Attitude estimate is fresh and stable before flight (EKF converged, no drift). | T | After boot + TRIAD alignment, attitude is stable and matches the true resting attitude with no slow drift; telemetry fresh (no "telemetry offline" alarm). |
| **R-TLM-07** | GS telemetry alarms (stall, altitude, bank-angle, sink-rate) are configured and fire on their thresholds. | T | Each enabled alarm triggers when its condition is forced on the bench/ground. |

### 2.4 Control modes

| ID | Requirement | Method | Pass criterion |
| --- | --- | --- | --- |
| **R-MODE-01** | Manual mode (CH6 `<1550`) passes stick commands straight to the surfaces at full authority. | D | Full-range surface travel in Manual. |
| **R-MODE-02** | Fly-By-Wire (CH6 `≥1550`) closes the attitude PID loop; stick commands desired roll/pitch, GS-limited (45° roll / 30° pitch) under the FC's 80° hard clamp. | T | In FBW on the bench, tilting the airframe drives surfaces to correct the attitude (correct restoring sense); stick offsets the target. |
| **R-MODE-03** | Mode switching (CH6) is clean and unambiguous with the 150-count deadband; no oscillation at the threshold. | D | Toggle CH6 repeatedly: single, crisp transitions each way. |
| **R-MODE-04** | Auto Throttle (CH7 `≥1550`) commands airspeed (0–100 mph, default 20 mph target); Manual Throttle (CH7 `<1550`) is direct. | T | In Auto Throttle with forced airspeed error, throttle output moves in the correcting direction; switching CH7 low restores direct throttle. |
| **R-MODE-05** | FBW restoring direction is confirmed **correct** (not inverted) on every axis before flight. | T | Roll right → right aileron corrects toward level; pitch up → down-elevator corrects; verified on bench in FBW. **A sign error here is a crash — this gate is mandatory.** |

### 2.5 Failsafe behavior — the core of the maiden

All values below are contractual (`docs/protocol_contract.md`). Each must be
demonstrated on the ground before flight (§4.2), not discovered in the air.

| ID | Requirement | Trigger | Expected behavior | Pass criterion |
| --- | --- | --- | --- | --- |
| **R-FS-01** | RC-fresh timeout | No decoded RC frame for `250 ms` (`RC_FAILSAFE_TIMEOUT_US`) | RC input marked stale | Surfaces begin the hold/blend sequence at 250 ms. |
| **R-FS-02** | Servo hold/blend | RC stale but raw CRSF bytes still active | Roll/pitch/yaw blend from last command toward neutral, completing by `500 ms` total age (`RC_SERVO_HOLD_TIMEOUT_US`) | Surfaces glide to neutral, not frozen, not snapped. |
| **R-FS-03** | Full RC loss | No RC frames **and** no raw CRSF bytes (`250 ms`, `CRSF_BYTE_ACTIVITY_TIMEOUT_US`) | Surfaces to neutral; **throttle cut immediately** | Motor stops immediately on total link loss. |
| **R-FS-04** | GS channel-staleness watchdog | GS control pipeline stale `> 2.0 s` (`RC_CHANNEL_STALE_TIMEOUT_S`) | GS **stops** transmitting RC frames so the FC failsafe can engage (prevents frozen commands on a "healthy" link) | Kill/stall the GS control source → TX halts → FC failsafe engages. |
| **R-FS-05** | Joystick loss | GS joystick serial lost | GS centers CH1/CH2, cuts throttle to 0, reverts CH7 to Manual Throttle (aircraft glides, does not hold power) | Unplug joystick → GS commands center + zero throttle. |
| **R-FS-06** | FBW stale-attitude fallback | CH6 = FBW but EKF attitude stale `> 200 ms` (`ATTITUDE_STALE_TIMEOUT_US`) or not yet converged | FC passes roll/pitch straight through (limited GS-scaled authority); in the *stale* case attitude telemetry stops and the GS raises "telemetry offline" ~1 s later; in the *convergence* case telemetry keeps flowing with no alarm | Verify switching GS to Manual restores full-range authority in the fallback. |
| **R-FS-07** | Auto-throttle airspeed-stale | Airspeed data stale `> 100 ms` (`AIRSPEED_FAILSAFE_TIMEOUT_US`) in Auto Throttle | Throttle PID reset; `autoThrottlePercent` decays at `50 %/s` toward 0; full RC loss still cuts throttle immediately | Occlude/disconnect pitot in Auto Throttle → throttle ramps down, not held. |
| **R-FS-08** | Watchdog (IWDG) | Main loop stalls `> 100 ms` (`WATCHDOG_TIMEOUT_US`) | Independent hardware watchdog resets the FC | (Analysis/bench only — do **not** induce in air.) Confirmed by test/analysis, not on the maiden. |
| **R-FS-09** | Watchdog-recovery boot | IWDG reset while airborne | FC defaults to **airborne recovery** (skips gyro cal / fail-stop); drops to cold boot only if GPS/airspeed positively confirm stationary | Verified by `flight_controller` host tests + bench reset while stationary → cold boot; airborne path is analysis-only. |
| **R-FS-10** | Startup fault fail-stop | Sensor/init fault on cold boot | `haltStartupWithNeutralServos()` — surfaces neutral, throttle cut, aircraft does not arm | Force a sensor fault on the bench → FC halts safe, GS shows no valid attitude. |

### 2.6 Boot and initialization

| ID | Requirement | Method | Pass criterion |
| --- | --- | --- | --- |
| **R-BOOT-01** | Cold boot requires a motionless airframe for gyro calibration. | D | FC boots only when still; movement during boot is rejected/retried. |
| **R-BOOT-02** | TRIAD coarse alignment starts the EKF near true attitude (no large initial offset that slowly drifts away). | T | At boot, GS attitude is immediately close to true resting attitude — no slow multi-second convergence from level. |
| **R-BOOT-03** | Servo/attitude indicator confirms init completion before handling. | D | Indicator sequence observed; surfaces live only after successful init. |
| **R-BOOT-04** | On the maiden, the FC is cold-booted on the ground, stationary, with the ESC/motor safe. | D | Boot performed with prop clear / motor disabled. |

### 2.7 Abort / go-no-go authority

| ID | Requirement | Method | Pass criterion |
| --- | --- | --- | --- |
| **R-ABT-01** | The pilot can take full manual control instantly at any time (CH6 → Manual). | D | Manual override verified pre-flight and used as the primary abort. |
| **R-ABT-02** | Any unsatisfied §4.1/§4.2 gate is a **No-Go**. | I | All mandatory gates recorded PASS before launch. |

---

## 3. Verification methods and instrumentation

| Item | Needed |
| --- | --- |
| GS running with link-stats, attitude, GPS, battery, and alarm displays live | ✔ |
| Prop **removed** or motor disabled for all bench/ground electrical tests | ✔ |
| Metered reference for battery voltage (R-TLM-05) | ✔ |
| Means to occlude/disconnect the pitot (R-FS-07, R-TLM-03) | ✔ |
| Means to stop GS control source and joystick (R-FS-04, R-FS-05) | ✔ |
| Open area for range check with model powered (R-LNK-04) | ✔ |
| Second observer / spotter for the flight (R-ABT-01) | ✔ |
| Data logging: keep the GS session/telemetry log for post-flight review | ✔ |

Host-side tests that back the analysis items (run before travelling to the
field): `tests/` (GS: `test_crsf_handset.py`, `test_frame_consistency.py`,
`test_config_packet_rate.py`, `test_joystick_latency.py`,
`test_port_reconnect.py`, `test_osd_smoothing.py`, `test_attitude_init.py`,
`test_ekf_decouple_mag.py`) and `flight_controller/tests/`
(`attitude_init_test.cpp`, `frame_consistency_test.cpp`,
`ekf_decouple_mag_test.cpp`). All must pass green as a precondition (§4.1).

---

## 4. Test procedure

Run phases in order. Do not advance to a later phase until every mandatory gate
in the current phase is PASS. **Prop off / motor disabled** for all of Phase 0
except where a live motor is explicitly required, and never point the aircraft
at anyone.

### Phase 0 — Bench verification (prop off / motor safe)

**0.1 Software preconditions (do before the field)**
- [ ] All GS `tests/` and `flight_controller/tests/` pass green. → gates R-CMD-03, R-LNK-03, R-TLM-01, R-BOOT-02, R-FS-09
- [ ] Firmware built with the intended `konfig.h` flags; note `FC_EKF_DECOUPLE_MAG`, `FC_EKF_FAST_PREDICT`, and confirm `FC_MAG_CALIBRATION_MODE = 0`.
- [ ] Magnetometer calibrated for this airframe (mag disturbances bleed into heading; do a bench cal run if not current).
- [ ] Confirm CPU/I²C headroom and IWDG margin at the chosen predict/telemetry rates (see `FC_EKF_FAST_PREDICT` note).

**0.2 Power-up and boot** → R-BOOT-01..04
- [ ] Cold boot stationary; confirm gyro-cal-still requirement and init/indicator sequence.
- [ ] Attitude appears near true resting attitude immediately (TRIAD), no slow drift from level.

**0.3 Link and telemetry** → R-LNK-01/02/05, R-TLM-01..06
- [ ] Link stats live: LQ ~100%, RSSI strong, SNR sane.
- [ ] Pitch/roll/yaw the airframe by hand; GS attitude matches sign & magnitude.
- [ ] GPS acquires; lat/lon/sats/course/alt sane; lock only on real fix.
- [ ] Blow into pitot → airspeed reads (even with no GPS lock).
- [ ] Battery V/I within tolerance of meter.

**0.4 Command path and surfaces** → R-CMD-01/02/04/05, R-MODE-01/03
- [ ] Control surface check: each stick → correct surface, correct direction, full travel in Manual.
- [ ] Throttle low = motor off (motor may be enabled only here, restrained, prop off); kill/cut works.
- [ ] Toggle CH6/CH7 at threshold: clean single transitions, no chatter.

**0.5 Fly-By-Wire and Auto Throttle sense** → R-MODE-02/04, **R-MODE-05 (mandatory)**
- [ ] In FBW, tilt airframe each axis → surfaces move in the **restoring** direction (roll right → correcting aileron; nose-up → down-elevator). Confirm **not inverted**. A failure here is a No-Go.
- [ ] Stick input in FBW offsets the target within GS limits (45° roll / 30° pitch).
- [ ] In Auto Throttle, force an airspeed error (pitot) → throttle output moves to correct; CH7 low restores direct throttle.

**0.6 Failsafe demonstration** → R-FS-01..07, R-FS-10
- [ ] Power off the transmitter/receiver link: surfaces blend to neutral by ~500 ms; **throttle cuts immediately** on total loss (R-FS-01/02/03).
- [ ] Stall/stop the GS control source > 2 s → GS halts TX, FC failsafe engages (R-FS-04).
- [ ] Unplug joystick → GS centers roll/pitch, throttle 0, CH7 → Manual (R-FS-05).
- [ ] In FBW, induce attitude staleness → limited-authority pass-through; switching GS to Manual restores full range; confirm GS "telemetry offline" behavior matches the stale vs. convergence cases (R-FS-06).
- [ ] In Auto Throttle, occlude/disconnect pitot → throttle decays at ~50 %/s, not held (R-FS-07).
- [ ] Force a startup sensor fault → FC fail-stops with neutral servos, does not arm (R-FS-10).

> R-FS-08 (IWDG timeout) and R-FS-09 airborne path are **analysis/bench only** —
> never induce a watchdog reset in the air. Confirm via host tests and a
> stationary bench reset that cold-boots.

**Phase 0 gate:** every box above checked. Any FAIL → stop, fix, re-run.

### Phase 1 — Ground / taxi (optional, if the airframe taxis)
- [ ] Low-speed taxi in Manual: surfaces track sticks under vibration; no telemetry dropouts; attitude stays fresh under motor vibration and current draw (watch for mag/EKF disturbance from motor current).
- [ ] Confirm no unexpected mode changes and LQ stays high with motor running.

### Phase 2 — Range check → R-LNK-04
- [ ] Model powered, motor armed (restrained or held), walk to ≥ intended max operating range.
- [ ] LQ stays 100% / RSSI within spec the whole way; note the range at first degradation. Must comfortably exceed the planned flight distance. Below margin → No-Go.

### Phase 3 — Maiden flight
1. **Launch in Manual mode** (CH6 low), Manual Throttle. Fly the aircraft on the radio system exactly as a conventional maiden: trim, establish safe altitude, confirm handling and that commands/telemetry stay solid.
2. **Telemetry sanity in the air:** confirm attitude tracks real aircraft attitude, GPS position/speed/alt plausible, battery drains sanely, LQ/RSSI hold. Watch for any drift or "telemetry offline" alarm.
3. **FBW evaluation** (only after Manual is trimmed and stable, at safe altitude, ready to revert instantly to Manual): switch CH6 → FBW. Confirm the aircraft holds/commands attitude correctly and the restoring sense matches the bench (R-MODE-02/05). At the **first** sign of wrong-sense or oscillation, switch straight back to Manual (R-ABT-01).
4. **Auto Throttle evaluation** (after FBW is trusted, safe altitude): switch CH7 → Auto Throttle with a conservative target airspeed. Confirm throttle regulates airspeed toward target and reverts cleanly to Manual Throttle on CH7 low.
5. **Recover in Manual mode.** Land on the radio system with direct manual control.

---

## 5. Abort criteria (any one → revert to Manual and land)

- Wrong-sense or oscillating FBW response (R-MODE-05).
- Attitude telemetry drift, freeze, or "telemetry offline" alarm in flight.
- LQ drops / RSSI collapses / repeated dropouts.
- Any unexpected mode or throttle change.
- Any GS alarm (stall / bank-angle / sink-rate / altitude) that reflects a real
  condition.
- Any behavior not matching a Phase 0 result.

**Primary abort is always CH6 → Manual, then land.** Manual authority is the
last line of defense and must be confirmed working before every mode experiment.

---

## 6. Pass / fail record

| Phase | Result (PASS/FAIL) | Notes / anomalies |
| --- | --- | --- |
| 0 — Bench | | |
| 1 — Taxi | | |
| 2 — Range check | | |
| 3 — Maiden (Manual) | | |
| 3 — FBW eval | | |
| 3 — Auto Throttle eval | | |

**Overall Go / No-Go for FBW & Auto Throttle in flight:** all of Phase 0
mandatory gates + R-MODE-05 + Phase 2 range margin = **Go**. Any gap = **Manual
only**, defer autonomy features to a later flight.

### Post-flight
- [ ] Save GS telemetry/session log.
- [ ] Note any anomaly against the requirement ID above.
- [ ] If any failsafe or mode behaved differently from Phase 0, ground the
      system and re-verify against `docs/protocol_contract.md` before flying
      again.

---

## 7. In-flight automated verification

"Automated in flight" means **passive monitoring and assertion by GS code while
the aircraft flies** — never active fault injection. You cannot safely cut the
link, unplug the joystick, or freeze the IMU in the air, so every destructive
failsafe test (R-FS-01/02/03/05/06/07/08/09/10 and the deliberate FBW
sign-injection) stays a **Phase 0 ground gate**. But their *occurrence* can be
auto-detected and logged if they happen unexpectedly, and most of the
non-destructive requirements can be continuously checked in flight.

The GS already provides the hooks: a **telemetry-offline indicator**, four live
**alarms** (stall, altitude, bank-angle, sink-rate), and **sortie (blackbox) CSV
recording** with a review/plot page. The items below extend that infrastructure.

### 7.1 Already automated today (turn on and record)

| Requirement | Existing GS mechanism |
| --- | --- |
| R-TLM-06 (attitude freshness) | Telemetry-offline indicator + attitudeSampleValid |
| R-TLM-07 (alarms) | Stall, altitude, bank-angle, sink-rate alarms |
| All (post-flight review) | Start sortie recording before launch → CSV of the full flight for offline pass/fail |

**Action:** enable all four alarms and start sortie recording before every
flight so the maiden is captured end-to-end.

### 7.2 Automatable in flight (passive assertions — recommended additions)

| # | In-flight auto-check | Verifies | Method (all from telemetry the GS already receives) |
| --- | --- | --- | --- |
| 1 | **Link-health monitor** | R-LNK-04/05 | Threshold LQ/RSSI/SNR from `0x14`; count and timestamp every dropout / LQ-drop event. |
| 2 | **Telemetry-rate & decode-health monitor** | R-LNK-03, R-TLM-06 | Measure actual attitude (~125 Hz) and GPS (~50 Hz) arrival rates; track CRC/length-reject rate; flag frozen (unchanging) fields. |
| 3 | **Attitude sanity monitor** | R-TLM-01/06 | Assert values finite and in range; detect stuck/NaN; log every staleness event (also flags an unexpected R-FS-06 fallback). |
| 4 | **GPS ↔ airspeed cross-check** | R-TLM-02/03 | Plausibility of pitot airspeed vs GPS ground speed; log fix-state transitions and sat count. |
| 5 | **Battery sag monitor** | R-TLM-05 | Min-voltage / sag-under-load alarm from `0x08`. |
| 6 | **Mode-transition logger** | R-MODE-03 | Timestamp every CH6/CH7 crossing of the 1550 threshold (with the 150 deadband); confirm no chatter. |
| 7 | **FBW closed-loop monitor** ⭐ | R-MODE-02/05 (partial) | The GS knows the commanded roll/pitch (its own transmitted CH1/CH2 scaled through the 45°/30° FBW limits) **and** the measured attitude (`0x1E`). Compute tracking error; auto-flag **divergence**, **sustained wrong-sign correlation** (inverted loop), or **limit-cycle oscillation**. This is the one check that can catch a bad FBW loop *in the air* — but it is a monitor, not a substitute for the mandatory R-MODE-05 ground gate. |
| 8 | **Auto-throttle monitor** | R-MODE-04, R-FS-07 (detect) | Compare commanded target airspeed (CH3 mapping) to measured airspeed; assert error trends toward zero; detect the 50 %/s decay signature if pitot goes stale. |
| 9 | **GS TX-watchdog event logger** | R-FS-04 (detect) | Log whenever the GS transmit pacer halts on the 2 s channel-staleness watchdog. |

### 7.3 Not automatable in flight (must stay Phase 0 ground gates)

R-FS-01/02/03 (RC/link loss), R-FS-05 (joystick loss), R-FS-06 (attitude-stale
fallback, *inducing* it), R-FS-07 (pitot occlusion, *inducing* it),
R-FS-08/09 (watchdog), R-FS-10 (startup fault), and the deliberate **R-MODE-05
sign injection** — all require a destructive stimulus that would risk the
aircraft. Verify these on the bench; in the air the GS only *detects and logs*
them if they occur.

R-CMD-03 (command latency) also stays off the maiden: the GS receives no echo of
the applied surface command (only attitude, which is coupled to airframe
dynamics), so true end-to-end latency is measured by the host test
(`tests/test_joystick_latency.py`), not in flight.

### 7.4 Priority if you automate one thing

Item **7 (FBW closed-loop monitor)** and items **1–3** (link + telemetry
health) give the most safety value for the least code, and all run purely off
telemetry the GS already decodes. Everything else is incremental logging on top
of the existing sortie recorder.

---

## 8. Traceability

Every requirement traces to the protocol contract and firmware:

- Link/framing, rates, channel map, telemetry units → `docs/protocol_contract.md`.
- Failsafe timeouts and mode thresholds → `flight_controller/Main.ino`
  (`RC_FAILSAFE_TIMEOUT_US`, `RC_SERVO_HOLD_TIMEOUT_US`,
  `CRSF_BYTE_ACTIVITY_TIMEOUT_US`, `ATTITUDE_STALE_TIMEOUT_US`,
  `AIRSPEED_FAILSAFE_TIMEOUT_US`, `WATCHDOG_TIMEOUT_US`,
  `CONTROL_MODE_*`, `THROTTLE_MODE_*`, `haltStartupWithNeutralServos`,
  `attitudeEstimateConvergedForFbw`).
- GS-side limits and watchdogs → `config.py`
  (`ALLOWED_ATTITUDE_PACKET_RATES_HZ`, `fbw.max_*_angle_deg`,
  `throttle.target_airspeed_mph`, alarm thresholds) and
  `pico_modules/pico_transmitpackets.py` (`RC_CHANNEL_*`,
  `channel_stale_timeout_s`).
- EKF/boot behavior → `attitude_init.h`, `konfig.h`
  (`FC_EKF_DECOUPLE_MAG`, `FC_EKF_FAST_PREDICT`), and the host tests in
  `tests/` and `flight_controller/tests/`.

This document is a snapshot. Re-reconcile against the code before each flight
campaign; the contract file is the single source of truth.
