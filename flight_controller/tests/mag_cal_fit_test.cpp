// Host-side verification of the shared magnetometer-calibration fit
// (flight_controller/mag_cal_fit.h) used by both the bench MAGCAL helper and
// the in-field runtime calibration:
//   * the min/max fit recovers a known hard-iron offset and diagonal
//     soft-iron scaling from synthetic full-coverage rotation data,
//   * applying the fitted constants (the same math as applyMagCalibration)
//     restores a constant field magnitude across orientations,
//   * runs with too few samples or insufficient axis coverage are rejected
//     with the matching status instead of producing constants.
//
// Build: c++ -std=c++17 -I flight_controller -O2 -Wall -Wextra -Werror
//          -o mag_cal_fit_test flight_controller/tests/mag_cal_fit_test.cpp

#include <cmath>
#include <cstdio>
#include <cstdint>

#include "../mag_cal_fit.h"

static int failures = 0;

#define CHECK(cond, msg)                                            \
  do {                                                              \
    if (!(cond)) {                                                  \
      std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);   \
      ++failures;                                                   \
    }                                                               \
  } while (0)

static bool nearf(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// Synthetic sensor model: a unit-direction earth field of magnitude
// FIELD_UT is scaled per-axis (soft iron, as an axis gain 1/softTrue --
// the fit returns the inverse gain) and offset (hard iron).
static const float FIELD_UT = 50.0f;
static const float HARD_TRUE[3] = {-33.9f, 12.4f, 5.7f};
// Per-axis attenuation of the true field: raw radius = FIELD_UT * ATTEN.
static const float ATTEN[3] = {0.8f, 1.0f, 1.25f};

static void rawSample(float ux, float uy, float uz, float out[3]) {
  out[0] = HARD_TRUE[0] + FIELD_UT * ATTEN[0] * ux;
  out[1] = HARD_TRUE[1] + FIELD_UT * ATTEN[1] * uy;
  out[2] = HARD_TRUE[2] + FIELD_UT * ATTEN[2] * uz;
}

// Sweep directions over the sphere (full coverage, like a proper rotation).
static void accumulateSphere(MagCalAccumulator &acc, int steps) {
  for (int i = 0; i < steps; ++i) {
    const float theta = static_cast<float>(M_PI) * (i + 0.5f) / steps;  // 0..pi
    for (int j = 0; j < steps; ++j) {
      const float phi = 2.0f * static_cast<float>(M_PI) * j / steps;    // 0..2pi
      float raw[3];
      rawSample(std::sin(theta) * std::cos(phi),
                std::sin(theta) * std::sin(phi),
                std::cos(theta), raw);
      acc.add(raw[0], raw[1], raw[2]);
    }
  }
}

static void testRecoversKnownCalibration() {
  MagCalAccumulator acc;
  acc.reset();
  accumulateSphere(acc, 60);  // 3600 samples, full coverage

  MagCalFit fit;
  const MagCalFitStatus status = magCalComputeFit(acc, 500, 20.0f, fit);
  CHECK(status == MAG_CAL_FIT_OK, "full-coverage fit accepted");

  for (int axis = 0; axis < 3; ++axis) {
    CHECK(nearf(fit.hardIron[axis], HARD_TRUE[axis], 0.05f),
          "hard iron recovered");
    CHECK(nearf(fit.radius[axis], FIELD_UT * ATTEN[axis], 0.1f),
          "axis radius recovered");
  }
  // Soft-iron gains rescale each axis radius to the three-axis average.
  const float avgRadius =
      FIELD_UT * (ATTEN[0] + ATTEN[1] + ATTEN[2]) / 3.0f;
  for (int axis = 0; axis < 3; ++axis) {
    CHECK(nearf(fit.softIronDiag[axis], avgRadius / (FIELD_UT * ATTEN[axis]),
                0.01f),
          "soft iron gain recovered");
  }

  // Applying the constants (same math as applyMagCalibration with a diagonal
  // matrix) must restore a direction-independent magnitude.
  float minNorm = 1e9f, maxNorm = 0.0f;
  const int steps = 24;
  for (int i = 0; i < steps; ++i) {
    const float theta = static_cast<float>(M_PI) * (i + 0.5f) / steps;
    for (int j = 0; j < steps; ++j) {
      const float phi = 2.0f * static_cast<float>(M_PI) * j / steps;
      float raw[3];
      rawSample(std::sin(theta) * std::cos(phi),
                std::sin(theta) * std::sin(phi),
                std::cos(theta), raw);
      float cal[3];
      for (int axis = 0; axis < 3; ++axis) {
        cal[axis] = fit.softIronDiag[axis] * (raw[axis] - fit.hardIron[axis]);
      }
      const float norm =
          std::sqrt(cal[0]*cal[0] + cal[1]*cal[1] + cal[2]*cal[2]);
      if (norm < minNorm) minNorm = norm;
      if (norm > maxNorm) maxNorm = norm;
    }
  }
  CHECK(maxNorm - minNorm < 0.5f,
        "calibrated field magnitude constant across orientations");
}

static void testRejectsTooFewSamples() {
  MagCalAccumulator acc;
  acc.reset();
  accumulateSphere(acc, 10);  // 100 samples < 500 minimum

  MagCalFit fit;
  CHECK(magCalComputeFit(acc, 500, 20.0f, fit) == MAG_CAL_FIT_TOO_FEW_SAMPLES,
        "short run rejected");

  MagCalAccumulator empty;
  empty.reset();
  CHECK(magCalComputeFit(empty, 1, 20.0f, fit) == MAG_CAL_FIT_TOO_FEW_SAMPLES,
        "empty accumulator rejected");
}

static void testRejectsInsufficientCoverage() {
  // Rotation confined to the X/Y plane: the Z axis span stays ~0, which is
  // exactly what an operator who never noses the aircraft up/down produces.
  MagCalAccumulator acc;
  acc.reset();
  for (int j = 0; j < 1000; ++j) {
    const float phi = 2.0f * static_cast<float>(M_PI) * j / 1000.0f;
    float raw[3];
    rawSample(std::cos(phi), std::sin(phi), 0.0f, raw);
    acc.add(raw[0], raw[1], raw[2]);
  }

  MagCalFit fit;
  CHECK(magCalComputeFit(acc, 500, 20.0f, fit) == MAG_CAL_FIT_SPAN_TOO_SMALL,
        "planar-only rotation rejected");
  // Diagnostics are still filled for logging a rejected run.
  CHECK(nearf(fit.hardIron[0], HARD_TRUE[0], 0.1f),
        "diagnostic hard iron still reported for rejected run");
  CHECK(fit.span[2] < 1.0f, "unswept axis span reported near zero");
  // Soft-iron gains stay at the safe identity for a rejected run.
  CHECK(fit.softIronDiag[2] == 1.0f, "rejected run keeps identity soft iron");
}

int main() {
  testRecoversKnownCalibration();
  testRejectsTooFewSamples();
  testRejectsInsufficientCoverage();

  if (failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", failures);
  return 1;
}
