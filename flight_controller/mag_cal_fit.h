#ifndef FEATHER_MAG_CAL_FIT_H
#define FEATHER_MAG_CAL_FIT_H

#include <stdint.h>

// Shared magnetometer-calibration accumulator and hard-iron/diagonal
// soft-iron fit. Used by the bench-only MAGCAL helper
// (FC_MAG_CALIBRATION_MODE) and the in-field runtime calibration in Main.ino,
// and exercised host-side by tests/mag_cal_fit_test.cpp, so both calibration
// paths compute constants from one implementation.
//
// The fit is the same min/max model the bench helper has always used: the
// hard-iron bias is the midpoint of each axis's observed extremes and the
// diagonal soft-iron gain rescales each axis radius to the three-axis
// average. Off-diagonal soft-iron terms require a full ellipsoid fit and are
// intentionally not estimated here (see the MAGCAL notes in Main.ino).
//
// Header-only and dependency-free (no Arduino types) so the host tests can
// include it directly.

struct MagCalAccumulator {
  float minX, minY, minZ;
  float maxX, maxY, maxZ;
  uint32_t sampleCount;
  bool haveSample;

  void reset() {
    minX = minY = minZ = 0.0f;
    maxX = maxY = maxZ = 0.0f;
    sampleCount = 0;
    haveSample = false;
  }

  // Track per-axis extremes. Callers are responsible for gating out invalid
  // samples (failed reads, saturated magnetometer, near-zero norms) before
  // calling; every added sample counts toward the fit.
  void add(float x, float y, float z) {
    if (!haveSample) {
      minX = maxX = x;
      minY = maxY = y;
      minZ = maxZ = z;
      haveSample = true;
    } else {
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
    }
    ++sampleCount;
  }
};

enum MagCalFitStatus {
  MAG_CAL_FIT_OK = 0,
  // Not enough valid samples were accumulated to trust the extremes.
  MAG_CAL_FIT_TOO_FEW_SAMPLES,
  // At least one axis span is below the coverage threshold: the aircraft was
  // not rotated through enough orientations for the min/max model to see the
  // full field on every axis.
  MAG_CAL_FIT_SPAN_TOO_SMALL,
};

struct MagCalFit {
  float hardIron[3];      // uT, midpoint of each axis's extremes
  float softIronDiag[3];  // unitless per-axis gain (average radius / axis radius)
  float span[3];          // uT, max - min per axis (diagnostic)
  float radius[3];        // uT, span / 2 per axis (diagnostic)
};

// Compute the hard-iron/diagonal soft-iron fit from accumulated extremes.
// The diagnostic fields (and the hard iron) are filled in whenever at least
// one sample was seen so callers can log them even for a rejected run; the
// soft-iron gains are only meaningful when the status is MAG_CAL_FIT_OK
// (a degenerate axis radius would otherwise divide by ~zero).
inline MagCalFitStatus magCalComputeFit(const MagCalAccumulator &acc,
                                        uint32_t minSamples,
                                        float minAxisSpan,
                                        MagCalFit &out) {
  out.hardIron[0] = out.hardIron[1] = out.hardIron[2] = 0.0f;
  out.softIronDiag[0] = out.softIronDiag[1] = out.softIronDiag[2] = 1.0f;
  out.span[0] = out.span[1] = out.span[2] = 0.0f;
  out.radius[0] = out.radius[1] = out.radius[2] = 0.0f;

  if (!acc.haveSample || acc.sampleCount < minSamples) {
    return MAG_CAL_FIT_TOO_FEW_SAMPLES;
  }

  out.hardIron[0] = (acc.maxX + acc.minX) * 0.5f;
  out.hardIron[1] = (acc.maxY + acc.minY) * 0.5f;
  out.hardIron[2] = (acc.maxZ + acc.minZ) * 0.5f;
  out.span[0] = acc.maxX - acc.minX;
  out.span[1] = acc.maxY - acc.minY;
  out.span[2] = acc.maxZ - acc.minZ;
  for (int axis = 0; axis < 3; ++axis) {
    out.radius[axis] = out.span[axis] * 0.5f;
  }

  if (out.span[0] < minAxisSpan || out.span[1] < minAxisSpan ||
      out.span[2] < minAxisSpan) {
    return MAG_CAL_FIT_SPAN_TOO_SMALL;
  }

  const float averageRadius =
      (out.radius[0] + out.radius[1] + out.radius[2]) / 3.0f;
  for (int axis = 0; axis < 3; ++axis) {
    out.softIronDiag[axis] = averageRadius / out.radius[axis];
  }
  return MAG_CAL_FIT_OK;
}

#endif  // FEATHER_MAG_CAL_FIT_H
