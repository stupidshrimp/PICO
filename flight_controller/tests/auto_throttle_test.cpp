/*************************************************************************************************************
 * Host-side verification of the FC auto-throttle (airspeed hold) loop.
 *
 * Motivated by a bench report: with Auto Throttle enabled the motor sat at a
 * constant slow speed and did not respond to blowing into the pitot tube.
 * That is exactly the pre-ae746d1 failure mode ("FC: stop transient airspeed
 * dropouts from idling auto-throttle"), so this test proves the CURRENT code
 * behaves correctly on the bench scenarios and demonstrates that the old
 * dropout handling reproduces the reported symptom.
 *
 * The PID controller, the RC/throttle unit converters, and every
 * AUTO_THROTTLE_* / RC_INPUT_* constant are extracted VERBATIM from Main.ino
 * at build time (see the awk lines below), so this exercises the shipped
 * arithmetic rather than a copy that could drift. The two small pieces that
 * cannot be lifted out of the sketch without an Arduino toolchain -- the
 * updateAirspeedCache() validity/freshness handling and the Auto Throttle
 * branch of the servo loop -- are transcribed here line-for-line with
 * references back to Main.ino.
 *
 * Scenarios:
 *   1. Bench spin-up: target 20 mph, pitot reads ~0     -> ramps to 100%.
 *   2. Blow into pitot (40 mph) while at full           -> throttle ramps down, recovers after.
 *   3. Intermittent MS4525D0 stale frames (1 in 3 bad)  -> still reaches target (ae746d1 fix).
 *   4. Sensor death mid-run                             -> decays to 0% at the stale-decay rate.
 *   5. Target set to 0 mph                              -> percent inherited from Manual is held
 *                                                          (bumpless transfer), blowing drives it to 0.
 *   6. PRE-ae746d1 dropout handling (old firmware)      -> motor sits near idle and ignores the
 *                                                          pitot: the reported bench symptom.
 *
 * Build & run (from flight_controller/tests/):
 *   awk '/^const uint16_t RC_INPUT_MIN/,/^};/' ../Main.ino                       >  /tmp/at_extract.h
 *   awk '/^float mapRcToPercent/,/^}/' ../Main.ino                               >> /tmp/at_extract.h
 *   awk '/^uint16_t mapPercentToThrottleUs/,/^}/' ../Main.ino                    >> /tmp/at_extract.h
 *   awk '/^float mapRcToAutoThrottleTargetMph/,/^}/' ../Main.ino                 >> /tmp/at_extract.h
 *   c++ -std=c++17 -O2 -I/tmp -o /tmp/auto_throttle_test auto_throttle_test.cpp && /tmp/auto_throttle_test
 ************************************************************************************************************/

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

/* ---- Minimal Arduino shims needed by the extracted firmware code. ---- */
template <typename T>
static T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
using std::isnan;

/* ---- Verbatim firmware extraction (constants + PIDController + converters). ---- */
#include "at_extract.h"

static int failures = 0;
#define CHECK(cond, msg)                                                        \
  do {                                                                          \
    if (cond) {                                                                 \
      std::printf("  PASS: %s\n", msg);                                         \
    } else {                                                                    \
      std::printf("  FAIL: %s\n", msg);                                         \
      ++failures;                                                               \
    }                                                                           \
  } while (0)

/* GS-side CH3 encoding of a target airspeed (main.py _build_control_channels):
 * channels[2] = fraction * (CRSF_MAX - CRSF_MIN) + CRSF_MIN, range 172..1811. */
static uint16_t gsEncodeTargetMph(float mph) {
  float fraction = constrain(mph, 0.0f, AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH) /
                   AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH;
  return static_cast<uint16_t>(fraction * (RC_INPUT_MAX - RC_INPUT_MIN) + RC_INPUT_MIN);
}

/* Simulation of the firmware state machine around the throttle PID.
 * The airspeed cache mirrors updateAirspeedCache() (Main.ino ~1743-1810,
 * validity/freshness handling only; the EKF-only airspeed-rate filter is
 * irrelevant to throttle and omitted). The control step mirrors the
 * THROTTLE_MODE_AUTO branch of the servo loop (Main.ino ~4590-4602). */
struct Sim {
  bool preAe746d1;  // emulate the OLD dropout handling (one NaN read zeroes + invalidates)

  uint32_t nowUs = 0;
  float autoThrottlePercent = 0.0f;
  float latestAutoThrottleTargetMph = AUTO_THROTTLE_DEFAULT_TARGET_MPH;
  float latestAirspeedMph = 0.0f;
  bool latestAirspeedValid = false;
  uint32_t lastAirspeedUpdateUs = 0;
  PIDController throttlePid;

  explicit Sim(bool oldFirmware = false)
      : preAe746d1(oldFirmware),
        // Same constructor arguments as the throttlePid instance in Main.ino.
        throttlePid(AUTO_THROTTLE_KP, AUTO_THROTTLE_KI, AUTO_THROTTLE_KD,
                    -AUTO_THROTTLE_OUTPUT_LIMIT_PERCENT_PER_S,
                    AUTO_THROTTLE_OUTPUT_LIMIT_PERCENT_PER_S,
                    -AUTO_THROTTLE_INTEGRAL_LIMIT,
                    AUTO_THROTTLE_INTEGRAL_LIMIT,
                    AUTO_THROTTLE_ERROR_DEADBAND_MPH,
                    AUTO_THROTTLE_DERIVATIVE_CUTOFF_HZ) {}

  // Main.ino airspeedInputFresh()
  bool airspeedInputFresh(uint32_t t) const {
    return latestAirspeedValid && lastAirspeedUpdateUs != 0 &&
           (uint32_t)(t - lastAirspeedUpdateUs) <= AIRSPEED_FAILSAFE_TIMEOUT_US;
  }

  // Main.ino updateAirspeedCache(): sensorMph is getAirspeed()'s return (NaN = failed read).
  void updateAirspeedCache(float sensorMph) {
    if (std::isnan(sensorMph)) {
      if (preAe746d1) {
        // OLD behavior (commit ae746d1's "before"): one bad read zeroes the
        // cached airspeed and clears the valid flag immediately.
        latestAirspeedValid = false;
        latestAirspeedMph = 0.0f;
        return;
      }
      // CURRENT behavior: dropped sample. Keep the last good value/flag, do not
      // advance the freshness timestamp; only a dropout outlasting the
      // freshness window fails the cache safe.
      if (latestAirspeedValid && lastAirspeedUpdateUs != 0 &&
          (uint32_t)(nowUs - lastAirspeedUpdateUs) > AIRSPEED_FAILSAFE_TIMEOUT_US) {
        latestAirspeedValid = false;
        latestAirspeedMph = 0.0f;
      }
      return;
    }
    latestAirspeedValid = true;
    latestAirspeedMph = sensorMph;
    lastAirspeedUpdateUs = nowUs;
  }

  // THROTTLE_MODE_AUTO branch of the servo loop, controlDt = 8 ms (125 Hz).
  uint16_t controlStepAuto(uint16_t rcThrottleRaw, float controlDt) {
    latestAutoThrottleTargetMph = mapRcToAutoThrottleTargetMph(rcThrottleRaw);
    if (!airspeedInputFresh(nowUs)) {
      throttlePid.reset();
      autoThrottlePercent = std::max(
          0.0f,
          autoThrottlePercent - (AUTO_THROTTLE_STALE_DECAY_PERCENT_PER_S * controlDt));
    } else {
      float throttleAdjustment = throttlePid.update(
          latestAutoThrottleTargetMph, latestAirspeedMph, controlDt) * controlDt;
      autoThrottlePercent = constrain(autoThrottlePercent + throttleAdjustment, 0.0f, 100.0f);
    }
    return mapPercentToThrottleUs(autoThrottlePercent);
  }

  /* Seed one valid pitot sample so the sim starts (like the real FC, which
   * refreshes the cache continuously before Auto Throttle engages) with a
   * fresh cache rather than tripping the stale branch on the first steps. */
  void seedAirspeed(float mph) {
    nowUs += 1000;
    updateAirspeedCache(mph);
  }

  /* Run the loop for `seconds`, control at 125 Hz, airspeed reads at ~60 Hz.
   * sensor(t_us, read_index) returns the pitot reading (NaN = failed read). */
  template <typename SensorFn>
  void run(float seconds, uint16_t rcThrottleRaw, SensorFn sensor) {
    static const uint32_t CONTROL_PERIOD_US = 8000;     // 125 Hz servo/control loop
    static const uint32_t AIRSPEED_PERIOD_US_SIM = 16667;  // ~60 Hz cache refresh
    uint32_t endUs = nowUs + (uint32_t)(seconds * 1e6f);
    uint32_t nextControl = nowUs + CONTROL_PERIOD_US;
    uint32_t nextAirspeed = nowUs + AIRSPEED_PERIOD_US_SIM;
    static uint32_t readIndex = 0;
    while (nowUs < endUs) {
      uint32_t next = std::min(nextControl, nextAirspeed);
      nowUs = next;
      if (nowUs == nextAirspeed) {
        updateAirspeedCache(sensor(nowUs, readIndex++));
        nextAirspeed += AIRSPEED_PERIOD_US_SIM;
      }
      if (nowUs == nextControl) {
        controlStepAuto(rcThrottleRaw, 0.008f);
        nextControl += CONTROL_PERIOD_US;
      }
    }
  }
};

int main() {
  const uint16_t ch3Target20 = gsEncodeTargetMph(20.0f);

  /* Round-trip sanity: GS CH3 encoding -> FC target decode. */
  {
    std::printf("Converter round-trip:\n");
    float decoded = mapRcToAutoThrottleTargetMph(ch3Target20);
    std::printf("  GS 20.0 mph -> CH3 %u -> FC %.2f mph\n", ch3Target20, decoded);
    CHECK(std::fabs(decoded - 20.0f) < 0.1f, "CH3 target round-trips within 0.1 mph");
    CHECK(mapPercentToThrottleUs(0.0f) == THROTTLE_MIN_US, "0%% maps to THROTTLE_MIN_US (motor off)");
    CHECK(mapPercentToThrottleUs(100.0f) == THROTTLE_MAX_US, "100%% maps to THROTTLE_MAX_US");
  }

  /* 1. Bench spin-up: pitot ~0, target 20 mph -> throttle must ramp to 100%. */
  {
    std::printf("\nScenario 1: bench spin-up (target 20 mph, pitot 0)\n");
    Sim sim;
    sim.seedAirspeed(0.0f);
    sim.run(10.0f, ch3Target20, [](uint32_t, uint32_t) { return 0.0f; });
    std::printf("  after 10 s: %.1f%%\n", sim.autoThrottlePercent);
    CHECK(sim.autoThrottlePercent > 95.0f, "ramps to full throttle chasing an unreachable target");
  }

  /* 2. Blowing into the pitot must pull the throttle back down, then recover. */
  {
    std::printf("\nScenario 2: blow into pitot at full throttle\n");
    Sim sim;
    sim.seedAirspeed(0.0f);
    sim.run(10.0f, ch3Target20, [](uint32_t, uint32_t) { return 0.0f; });
    float before = sim.autoThrottlePercent;
    sim.run(3.0f, ch3Target20, [](uint32_t, uint32_t) { return 40.0f; });  // ~40 mph blow
    float duringBlow = sim.autoThrottlePercent;
    std::printf("  before blow: %.1f%%, after 3 s at 40 mph: %.1f%%\n", before, duringBlow);
    CHECK(duringBlow < before - 30.0f, "throttle ramps down while airspeed exceeds target");
    sim.run(6.0f, ch3Target20, [](uint32_t, uint32_t) { return 0.0f; });
    std::printf("  6 s after release: %.1f%%\n", sim.autoThrottlePercent);
    CHECK(sim.autoThrottlePercent > 90.0f, "throttle recovers after the blow ends");
  }

  /* 3. Intermittent stale frames (1 in 3 reads bad) must NOT idle the motor. */
  {
    std::printf("\nScenario 3: intermittent MS4525D0 stale frames (ae746d1 fix)\n");
    Sim sim;
    sim.seedAirspeed(0.0f);
    sim.run(10.0f, ch3Target20, [](uint32_t, uint32_t i) {
      return (i % 3 == 0) ? NAN : 0.0f;
    });
    std::printf("  after 10 s: %.1f%%\n", sim.autoThrottlePercent);
    CHECK(sim.autoThrottlePercent > 95.0f, "dropped samples do not reset/decay the command");
  }

  /* 4. Genuine sensor death -> decay to 0% (stale failsafe). */
  {
    std::printf("\nScenario 4: sensor dies at full throttle\n");
    Sim sim;
    sim.seedAirspeed(0.0f);
    sim.run(10.0f, ch3Target20, [](uint32_t, uint32_t) { return 0.0f; });
    sim.run(4.0f, ch3Target20, [](uint32_t, uint32_t) { return NAN; });
    std::printf("  4 s after death: %.1f%%\n", sim.autoThrottlePercent);
    CHECK(sim.autoThrottlePercent == 0.0f, "sustained dropout decays throttle to 0%%");
  }

  /* 5. Target 0 mph: percent inherited at engage is held (deadband), but a
   *    real airspeed error still drives it down. */
  {
    std::printf("\nScenario 5: target 0 mph with inherited manual percent\n");
    Sim sim;
    sim.autoThrottlePercent = 15.0f;  // Manual mode continuously mirrors the stick (Main.ino ~4605)
    sim.seedAirspeed(0.0f);
    const uint16_t ch3Target0 = gsEncodeTargetMph(0.0f);
    sim.run(5.0f, ch3Target0, [](uint32_t, uint32_t) { return 0.0f; });
    std::printf("  after 5 s at 0 mph error: %.1f%%\n", sim.autoThrottlePercent);
    CHECK(std::fabs(sim.autoThrottlePercent - 15.0f) < 0.01f,
          "zero error holds the inherited percent (bumpless transfer)");
    sim.run(3.0f, ch3Target0, [](uint32_t, uint32_t) { return 20.0f; });
    std::printf("  after 3 s blowing 20 mph: %.1f%%\n", sim.autoThrottlePercent);
    CHECK(sim.autoThrottlePercent == 0.0f, "airspeed above a 0 mph target drives throttle to 0%%");
  }

  /* 6. OLD firmware dropout handling reproduces the reported bench symptom:
   *    constant slow spin, unresponsive to the pitot. */
  {
    std::printf("\nScenario 6: PRE-ae746d1 firmware, same intermittent stale frames\n");
    Sim sim(/*oldFirmware=*/true);
    sim.seedAirspeed(0.0f);
    sim.run(10.0f, ch3Target20, [](uint32_t, uint32_t i) {
      return (i % 3 == 0) ? NAN : 0.0f;
    });
    float idlePct = sim.autoThrottlePercent;
    sim.run(3.0f, ch3Target20, [](uint32_t, uint32_t i) {
      return (i % 3 == 0) ? NAN : 40.0f;  // blowing hard, dropouts continue
    });
    float blowPct = sim.autoThrottlePercent;
    std::printf("  after 10 s: %.1f%% (near idle); after 3 s blowing 40 mph: %.1f%%\n",
                idlePct, blowPct);
    CHECK(idlePct < 25.0f, "old firmware equilibrates near idle instead of reaching target");
    CHECK(std::fabs(blowPct - idlePct) < 10.0f,
          "old firmware barely reacts to the pitot (reported symptom)");
  }

  std::printf("\n%s (%d failure%s)\n", failures == 0 ? "ALL SCENARIOS PASSED" : "FAILURES",
              failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}
