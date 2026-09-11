// Host-side verification of the airspeed-hold throttle controller
// (flight_controller/auto_throttle.h), compiled from the exact flight header
// with the exact flight gains. It pins the behaviour the reported bug was
// about -- "the motor does not spin fast enough no matter what the airspeed
// reads, and blowing into the pitot does not move it":
//
//   * BENCH DIAGNOSTIC: with no airflow and a non-zero target, a working loop
//     MUST drive the throttle to 100% (error never closes, so the velocity form
//     integrates Kp*error forever). A bench motor that instead holds one static
//     speed therefore proves the loop is not running at all -- it is not a
//     tuning problem.
//   * the loop converges on a plant and responds to airspeed in the right
//     direction (blowing into the pitot must back the throttle off),
//   * a SHORT RC gap keeps the trim so the loop resumes instead of re-ramping
//     from idle, while a genuine link loss drops it,
//   * the derivative is differentiated against the pitot SAMPLE interval, so a
//     60 Hz sensor read at a 125 Hz control rate reports the true airspeed rate
//     instead of roughly twice it on the cycles a sample landed,
//   * deadband, output limit, command clamping and bumpless transfer.
//
// Build: c++ -std=c++17 -I flight_controller -O2 -Wall -Wextra -Werror
//          -o auto_throttle_test flight_controller/tests/auto_throttle_test.cpp

#include <cmath>
#include <cstdio>

#include "../auto_throttle.h"

static int failures = 0;

#define CHECK(cond, msg)                                            \
  do {                                                              \
    if (!(cond)) {                                                  \
      std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);   \
      ++failures;                                                   \
    }                                                               \
  } while (0)

static bool nearf(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// The exact flight configuration (flight_controller/Main.ino AUTO_THROTTLE_*),
// written positionally exactly as Main.ino writes it so a field reorder in
// either file shows up here.
static const AutoThrottleConfig CFG = {
  0.8f,    // kp, percent-per-second per mph
  0.15f,   // kd
  100.0f,  // output limit, percent-per-second
  0.2f,    // error deadband, mph
  5.0f,    // derivative cutoff, Hz
  50.0f    // stale decay, percent-per-second
};

// Positional init is only safe while the field order is what it claims to be.
static_assert(&CFG.kp < &CFG.kd && &CFG.kd < &CFG.outputLimitPercentPerS &&
                  &CFG.outputLimitPercentPerS < &CFG.errorDeadbandMph &&
                  &CFG.errorDeadbandMph < &CFG.derivativeCutoffHz &&
                  &CFG.derivativeCutoffHz < &CFG.staleDecayPercentPerS,
              "AutoThrottleConfig field order changed; update every positional "
              "initializer (flight_controller/Main.ino AUTO_THROTTLE_CONFIG)");

static const float CONTROL_DT = 1.0f / 125.0f;   // EKF_PERIOD_US
static const float SAMPLE_DT = 1.0f / 60.0f;     // AIRSPEED_PERIOD_US

// Drive `seconds` of control cycles, folding a new airspeed sample in whenever
// the ~60 Hz pitot cache would have produced one. `airspeed(t)` supplies the
// reading the FC would see.
template <typename AirspeedFn>
static void run(AutoThrottleController &c, float targetMph, float seconds,
                AirspeedFn airspeed) {
  const int cycles = static_cast<int>(seconds / CONTROL_DT);
  float t = 0.0f;
  float nextSampleT = 0.0f;
  float lastSampleT = 0.0f;
  float reading = airspeed(0.0f);
  for (int i = 0; i < cycles; ++i) {
    bool newSample = false;
    float sampleDt = 0.0f;
    if (t >= nextSampleT) {
      reading = airspeed(t);
      newSample = true;
      sampleDt = (i == 0) ? 0.0f : (t - lastSampleT);
      lastSampleT = t;
      nextSampleT += SAMPLE_DT;
    }
    c.update(targetMph, reading, newSample, sampleDt, CONTROL_DT, CFG);
    t += CONTROL_DT;
  }
}

// ---------------------------------------------------------------------------

static void testBenchNoAirflowGoesToFullThrottle() {
  // The decisive bench case: propeller spinning, aircraft stationary, so the
  // pitot reads ~0 and the error never closes. Kp * 20 mph = 16 percent/s, so
  // the command must sweep 0 -> 100% in about 6.3 s and then stay pinned.
  AutoThrottleController c;
  run(c, 20.0f, 1.0f, [](float) { return 0.0f; });
  CHECK(nearf(c.percent, 16.0f, 1.0f), "bench: ~16% after 1 s at a 20 mph error");

  run(c, 20.0f, 6.0f, [](float) { return 0.0f; });
  CHECK(c.percent >= 99.9f, "bench: full throttle within ~7 s with no airflow");

  run(c, 20.0f, 5.0f, [](float) { return 0.0f; });
  CHECK(c.percent >= 99.9f, "bench: stays at full throttle, clamped at 100%");
}

static void testRespondsToAirspeedInTheRightDirection() {
  // Parked at full throttle by a 20 mph error, then "blow into the pitot": the
  // reading jumps above the target and the command must come DOWN promptly.
  AutoThrottleController c;
  run(c, 20.0f, 8.0f, [](float) { return 0.0f; });
  CHECK(c.percent >= 99.9f, "blow test: starts saturated at 100%");

  run(c, 20.0f, 1.0f, [](float) { return 35.0f; });
  CHECK(c.percent < 90.0f, "blow test: 1 s above target backs the throttle off");

  run(c, 20.0f, 8.0f, [](float) { return 35.0f; });
  CHECK(c.percent <= 0.0f, "blow test: sustained overspeed drives it to idle");
}

static void testConvergesOnAPlant() {
  // First-order airspeed plant: steady state 45 mph at full throttle, ~2.5 s
  // time constant. Checks the loop actually holds a commanded airspeed rather
  // than oscillating or parking at a clamp.
  AutoThrottleController c;
  const float target = 25.0f;
  const float vMax = 45.0f;
  const float tau = 2.5f;
  float v = 0.0f;
  float lastSampleV = 0.0f;
  float t = 0.0f;
  float nextSampleT = 0.0f;
  float lastSampleT = 0.0f;
  const int cycles = static_cast<int>(60.0f / CONTROL_DT);
  for (int i = 0; i < cycles; ++i) {
    bool newSample = false;
    float sampleDt = 0.0f;
    if (t >= nextSampleT) {
      lastSampleV = v;
      newSample = true;
      sampleDt = (i == 0) ? 0.0f : (t - lastSampleT);
      lastSampleT = t;
      nextSampleT += SAMPLE_DT;
    }
    const float u = c.update(target, lastSampleV, newSample, sampleDt, CONTROL_DT, CFG) / 100.0f;
    v += ((u - (v / vMax) * (v / vMax)) * vMax / tau) * CONTROL_DT;
    if (v < 0.0f) v = 0.0f;
    t += CONTROL_DT;
  }
  CHECK(nearf(v, target, 0.6f), "plant: settles on the commanded airspeed");
  CHECK(c.percent > 1.0f && c.percent < 99.0f, "plant: settles off both clamps");
}

static void testShortGapKeepsTrimAndLossDropsIt() {
  // The failsafe policy the control loop relies on. resetHistory() is what a
  // short RC gap (or a pitot dropout, or an engage) uses: the loop must not
  // differentiate across the gap, but the commanded percent IS its whole trim
  // state, so throwing that away costs a full re-ramp from idle.
  AutoThrottleController c;
  run(c, 20.0f, 3.0f, [](float) { return 0.0f; });
  const float trim = c.percent;
  CHECK(trim > 10.0f, "gap test: built up some trim first");

  c.resetHistory();
  CHECK(nearf(c.percent, trim, 1e-6f), "short gap keeps the commanded trim");
  CHECK(!c.hasPrevAirspeed, "short gap forgets the airspeed sample history");
  CHECK(c.airspeedRateMphPerS == 0.0f, "short gap clears the rate estimate");

  c.resetCommand();
  CHECK(c.percent == 0.0f, "genuine link loss drops the trim to zero");
}

static void testBumplessTransferFromManual() {
  AutoThrottleController c;
  c.setCommand(42.5f);
  CHECK(nearf(c.percent, 42.5f, 1e-6f), "engage seeds the manual throttle");
  c.setCommand(140.0f);
  CHECK(c.percent == 100.0f, "seed clamps above 100%");
  c.setCommand(-5.0f);
  CHECK(c.percent == 0.0f, "seed clamps below 0%");
}

static void testDerivativeUsesTheSampleInterval() {
  // A steady 3 mph/s acceleration, with the pitot delivering a usable reading
  // only every fourth cache refresh (~15 Hz -- what a run of MS4525D0 "stale
  // data" status frames looks like) while the loop keeps running at 125 Hz.
  //
  // Dividing each sample's change by the 8 ms CONTROL period reports
  // 3 * 0.0667 / 0.008 = 25 mph/s on the cycles a sample landed and 0 on the
  // rest. The low-pass recovers the right MEAN either way -- total change over
  // total time -- so the mean alone proves nothing. What the sample interval
  // fixes is the sawtooth: stepped at the interval that actually produced the
  // derivative, a constant acceleration yields a constant rate estimate, so the
  // peak-to-peak ripple (which Kd feeds straight into the throttle command)
  // collapses to nothing.
  const float accelMphPerS = 3.0f;
  AutoThrottleController c;
  float t = 0.0f;
  float nextSampleT = 0.0f;
  float lastSampleT = 0.0f;
  float reading = 0.0f;
  int refresh = 0;
  float minRate = 1e9f;
  float maxRate = -1e9f;
  const int cycles = static_cast<int>(8.0f / CONTROL_DT);
  for (int i = 0; i < cycles; ++i) {
    bool newSample = false;
    float sampleDt = 0.0f;
    if (t >= nextSampleT) {
      nextSampleT += SAMPLE_DT;
      if ((refresh++ % 4) == 0) {   // three reads in four are rejected
        reading = accelMphPerS * t;
        newSample = true;
        sampleDt = (lastSampleT == 0.0f) ? 0.0f : (t - lastSampleT);
        lastSampleT = t;
      }
    }
    c.update(1000.0f, reading, newSample, sampleDt, CONTROL_DT, CFG);
    if (t > 4.0f) {   // after the filter has settled
      if (c.airspeedRateMphPerS < minRate) minRate = c.airspeedRateMphPerS;
      if (c.airspeedRateMphPerS > maxRate) maxRate = c.airspeedRateMphPerS;
    }
    t += CONTROL_DT;
  }
  CHECK(nearf(c.airspeedRateMphPerS, accelMphPerS, 0.1f),
        "derivative converges on the true airspeed rate");
  CHECK(maxRate - minRate < 0.05f,
        "a constant acceleration gives a ripple-free rate estimate");
  CHECK(maxRate < 2.0f * accelMphPerS,
        "dropped reads never inflate the rate toward the control-period value");

  // A sample interval outside the trusted window must re-seed the reference
  // instead of differentiating across the gap.
  AutoThrottleController g;
  g.update(100.0f, 10.0f, true, SAMPLE_DT, CONTROL_DT, CFG);
  g.update(100.0f, 60.0f, true, 4.0f /* s: beyond the window */, CONTROL_DT, CFG);
  CHECK(g.airspeedRateMphPerS == 0.0f, "a long sample gap produces no derivative");
  g.update(100.0f, 60.0f + accelMphPerS * SAMPLE_DT, true, SAMPLE_DT, CONTROL_DT, CFG);
  CHECK(g.airspeedRateMphPerS > 0.0f, "differentiation resumes after the gap");
}

static void testDeadbandAndOutputLimit() {
  AutoThrottleController c;
  c.setCommand(50.0f);
  // Inside the deadband and at a steady airspeed: the command must not drift.
  run(c, 20.1f, 2.0f, [](float) { return 20.0f; });
  CHECK(nearf(c.percent, 50.0f, 1e-3f), "error inside the deadband holds the command");

  // A huge error must be clamped to the configured percent-per-second limit.
  AutoThrottleController l;
  const float before = l.percent;
  l.update(10000.0f, 0.0f, false, 0.0f, CONTROL_DT, CFG);
  CHECK(nearf(l.percent - before, CFG.outputLimitPercentPerS * CONTROL_DT, 1e-4f),
        "rate command is clamped to the output limit");
}

static void testStaleAirspeedDecaysToIdle() {
  AutoThrottleController c;
  c.setCommand(80.0f);
  c.update(20.0f, 15.0f, true, SAMPLE_DT, CONTROL_DT, CFG);
  CHECK(c.hasPrevAirspeed, "a usable sample was folded in");

  // 50 percent/s: 80% must reach 30% after 1 s, and 0% (not negative) by 2 s.
  const int oneSecond = static_cast<int>(1.0f / CONTROL_DT);
  float percent = c.percent;
  for (int i = 0; i < oneSecond; ++i) {
    percent = c.updateStale(CONTROL_DT, CFG);
  }
  CHECK(nearf(percent, 30.0f, 0.5f), "stale airspeed decays at 50 percent/s");
  CHECK(!c.hasPrevAirspeed, "stale airspeed forgets the sample history");

  for (int i = 0; i < 2 * oneSecond; ++i) {
    percent = c.updateStale(CONTROL_DT, CFG);
  }
  CHECK(percent == 0.0f, "stale decay clamps at idle, never negative");
}

int main() {
  testBenchNoAirflowGoesToFullThrottle();
  testRespondsToAirspeedInTheRightDirection();
  testConvergesOnAPlant();
  testShortGapKeepsTrimAndLossDropsIt();
  testBumplessTransferFromManual();
  testDerivativeUsesTheSampleInterval();
  testDeadbandAndOutputLimit();
  testStaleAirspeedDecaysToIdle();

  if (failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", failures);
  return 1;
}
