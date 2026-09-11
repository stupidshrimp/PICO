#ifndef FEATHER_AUTO_THROTTLE_H
#define FEATHER_AUTO_THROTTLE_H

// Airspeed-hold (auto throttle) controller.
//
// Kept in a header with no Arduino dependencies so
// flight_controller/tests/auto_throttle_test.cpp compiles the EXACT flight code
// on the host (same idiom as board_align.h / attitude_init.h / mag_cal_fit.h).
//
// VELOCITY (incremental) FORM: update() accumulates percent-per-second into the
// commanded throttle percent. Because the output is integrated, the proportional
// term ALREADY supplies the integral action that drives steady-state airspeed
// error to zero -- each cycle it adds kp*error percent-per-second to the
// standing throttle -- so there is deliberately no Ki. A non-zero Ki here would
// integrate the error a SECOND time (a double integrator on throttle), adding
// phase lag that invites overshoot and limit-cycling around the target airspeed.
// After integration the derivative term behaves like a proportional
// (rate-damping) term on airspeed.
//
// DERIVATIVE SAMPLE TIMING: the derivative is differentiated against the
// AIRSPEED SAMPLE interval, not the control period. The pitot cache refreshes at
// ~60 Hz while the control loop runs at 125 Hz, so dividing one sample's change
// by an 8 ms control step reported roughly twice the true acceleration on the
// cycles where a new sample had landed and zero on all the others -- a 60/125 Hz
// beat injected straight into the throttle command. Samples are therefore folded
// in only when they are NEW, each with its own dt.
//
// COMMAND RETENTION is the caller's policy, expressed through the three reset
// entry points below, and it matters a lot in flight: the commanded percent is
// the loop's entire trim state (there is no separate integrator), so zeroing it
// costs a full re-ramp from idle at kp*error percent-per-second. Short RC gaps
// must therefore use resetHistory() (which only forgets the airspeed sample
// history, so the loop does not differentiate across the gap) and NOT
// resetCommand(); the throttle output is hard-cut for as long as the link is
// stale, so holding the trim commands nothing.

struct AutoThrottleConfig {
  float kp;                      // percent-per-second per mph of airspeed error
  float kd;                      // percent-per-second per (mph/s) of airspeed rate
  float outputLimitPercentPerS;  // clamp on the integrated rate command
  float errorDeadbandMph;        // airspeed error treated as zero
  float derivativeCutoffHz;      // first-order low-pass on the airspeed rate; <=0 disables
  float staleDecayPercentPerS;   // ramp-down rate while the airspeed input is unusable
};

// Bounds on a usable airspeed sample interval, matching the window
// updateAirspeedCache() uses for its own derivative: reject a glitched or
// duplicated timestamp, and do not differentiate across a long gap.
constexpr float AUTO_THROTTLE_MIN_SAMPLE_DT_S = 0.001f;
constexpr float AUTO_THROTTLE_MAX_SAMPLE_DT_S = 0.5f;

struct AutoThrottleController {
  float percent;               // commanded throttle, 0..100 (the loop's trim state)
  float airspeedRateMphPerS;   // low-passed airspeed derivative
  float prevAirspeedMph;
  bool hasPrevAirspeed;

  AutoThrottleController()
    : percent(0.0f), airspeedRateMphPerS(0.0f), prevAirspeedMph(0.0f),
      hasPrevAirspeed(false) {}

  // Forget the airspeed sample history (and the rate estimate) while KEEPING the
  // commanded percent. Use this across any interruption the loop should resume
  // from without re-ramping: a short RC gap, a pitot dropout, an engage.
  void resetHistory() {
    airspeedRateMphPerS = 0.0f;
    prevAirspeedMph = 0.0f;
    hasPrevAirspeed = false;
  }

  // Drop the trim as well: the aircraft must not resume a standing power
  // setting (reverting to manual throttle, or a genuine RC link loss).
  void resetCommand() {
    percent = 0.0f;
    resetHistory();
  }

  // Bumpless transfer: seed the trim from the throttle that is already being
  // commanded so engaging auto throttle does not step the motor.
  void setCommand(float commandedPercent) {
    percent = clampPercent(commandedPercent);
  }

  // One control cycle with a usable airspeed input. `newSample` is true only on
  // the cycles where the pitot cache produced a fresh reading, and
  // `sampleDtS` is that sample's own interval. Returns the commanded percent.
  float update(float targetMph, float airspeedMph,
               bool newSample, float sampleDtS,
               float controlDtS, const AutoThrottleConfig &cfg) {
    if (newSample) {
      foldAirspeedSample(airspeedMph, sampleDtS, cfg);
    }

    float error = targetMph - airspeedMph;
    if (absf(error) < cfg.errorDeadbandMph) {
      error = 0.0f;
    }

    const float ratePercentPerS =
        clampf(cfg.kp * error - cfg.kd * airspeedRateMphPerS,
               -cfg.outputLimitPercentPerS, cfg.outputLimitPercentPerS);

    percent = clampPercent(percent + ratePercentPerS * controlDtS);
    return percent;
  }

  // One control cycle with no usable airspeed input (pitot stale or failed).
  // Ramp the command down rather than holding a power setting that is no longer
  // being closed on anything, and forget the sample history so the loop does not
  // differentiate across the outage when the sensor returns. Returns the
  // commanded percent.
  float updateStale(float controlDtS, const AutoThrottleConfig &cfg) {
    resetHistory();
    percent = clampPercent(percent - cfg.staleDecayPercentPerS * controlDtS);
    return percent;
  }

private:
  static float absf(float v) { return v < 0.0f ? -v : v; }

  static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  static float clampPercent(float v) { return clampf(v, 0.0f, 100.0f); }

  void foldAirspeedSample(float airspeedMph, float sampleDtS,
                          const AutoThrottleConfig &cfg) {
    const bool usableDt = sampleDtS >= AUTO_THROTTLE_MIN_SAMPLE_DT_S &&
                          sampleDtS <= AUTO_THROTTLE_MAX_SAMPLE_DT_S;
    if (!hasPrevAirspeed || !usableDt) {
      // First sample after a reset, or an interval we cannot trust: adopt the
      // reading as the new reference without producing a derivative from it.
      prevAirspeedMph = airspeedMph;
      hasPrevAirspeed = true;
      return;
    }

    const float rawRate = (airspeedMph - prevAirspeedMph) / sampleDtS;
    prevAirspeedMph = airspeedMph;

    if (cfg.derivativeCutoffHz > 0.0f) {
      // First-order low-pass, stepped at the sample interval that produced the
      // derivative (3.14159265f inlined: this header stays free of <math.h>).
      const float rc = 1.0f / (2.0f * 3.14159265f * cfg.derivativeCutoffHz);
      const float alpha = sampleDtS / (rc + sampleDtS);
      airspeedRateMphPerS += alpha * (rawRate - airspeedRateMphPerS);
    } else {
      airspeedRateMphPerS = rawRate;
    }
  }
};

#endif  // FEATHER_AUTO_THROTTLE_H
