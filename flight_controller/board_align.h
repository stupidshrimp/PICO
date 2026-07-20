/*************************************************************************************************************
 * Board-alignment (level) trim.
 *
 * The attitude EKF reports roll/pitch relative to the IMU's mounted frame, so a
 * chip/board that sits a few degrees off the airframe's reference plane shows a
 * fixed roll/pitch offset even when the aircraft is perfectly level. This header
 * removes that offset by rotating every body-frame sensor vector out of the
 * IMU's mounted frame and into the airframe reference frame: the accel and gyro
 * in the imuBodyAccel/Gyro helpers, and the magnetometer inside
 * applyMagCalibration -- AFTER its hard/soft-iron removal, so a stored MAGCAL
 * fit (which is done in the raw sensor frame) stays valid when the trim changes.
 *
 * Doing it pre-fusion (rather than subtracting a constant from the output Euler
 * angles) fixes the reference for EVERYTHING downstream at once and identically:
 * the TRIAD boot alignment, the accelerometer correction and its norm/innovation
 * gates, the tilt-compensated heading, the control loop's notion of "level", and
 * telemetry. It is also correct at any bank/pitch, not just near level, because
 * it is a proper rotation rather than a small-angle Euler subtraction.
 *
 * The SAME rotation is applied to accel, gyro, and magnetometer. A proper
 * rotation (det +1) transforms the gyro pseudovector identically to the
 * accel/mag polar vectors, so the right-handed body frame the fusion relies on
 * (see the frame-mapping comment in Main.ino) is preserved -- unlike a
 * reflection, this cannot desynchronize prediction from correction.
 *
 * Everything here is pure math on plain floats -- no Arduino, matrix library, or
 * konfig dependencies -- so the exact flight code is compiled and verified on the
 * host by tests/board_align_test.cpp.
 *
 * Convention matches quaternionToEulerDeg in Main.ino, whose reported
 * (roll, pitch) equal (phi, -theta) of the earth->body DCM M = Rx(phi)*Ry(theta).
 ************************************************************************************************************/
#ifndef BOARD_ALIGN_H
#define BOARD_ALIGN_H

#include <math.h>

/* Precomputed rotation mapping a body-frame sensor vector from the IMU's
 * physically-mounted frame into the airframe reference frame. */
typedef struct {
    float R[3][3];
} BoardAlign;

/* Build the trim from the roll/pitch the firmware REPORTS when the airframe is
 * actually level -- i.e. the observed static offset -- both in degrees and in
 * the quaternionToEulerDeg convention.
 *
 * The reported offset (rollOffsetDeg, pitchOffsetDeg) corresponds to the
 * earth->body offset attitude M = Rx(phi) * Ry(theta) with
 *   phi   =  rollOffsetDeg          (firmware roll == standard roll), and
 *   theta = -pitchOffsetDeg         (firmware pitch == -standard pitch).
 * The stored rotation is M^T, the inverse of that offset attitude, so a sensor
 * vector measured while the airframe is level is rotated back onto the level
 * reference and the reported roll/pitch become zero.
 *
 * Zero offsets yield exactly the identity, so an untrimmed build is bit-for-bit
 * unchanged. */
static inline void boardAlignInit(BoardAlign* ba, float rollOffsetDeg, float pitchOffsetDeg)
{
    const float phi   =  rollOffsetDeg  * (float)M_PI / 180.0f;   /* standard roll  */
    const float theta = -pitchOffsetDeg * (float)M_PI / 180.0f;   /* standard pitch */
    const float cphi = cosf(phi),   sphi = sinf(phi);
    const float cth  = cosf(theta), sth  = sinf(theta);

    /* M = Rx(phi) * Ry(theta)  (earth->body);  R_align = M^T. */
    ba->R[0][0] = cth;      ba->R[0][1] = sphi * sth;   ba->R[0][2] = cphi * sth;
    ba->R[1][0] = 0.0f;     ba->R[1][1] = cphi;         ba->R[1][2] = -sphi;
    ba->R[2][0] = -sth;     ba->R[2][1] = sphi * cth;   ba->R[2][2] = cphi * cth;
}

/* Apply the trim to one body-frame vector (in/out may be distinct). Used
 * identically for accel, gyro, and mag. */
static inline void boardAlignApply(const BoardAlign* ba, float x, float y, float z,
                                   float* ox, float* oy, float* oz)
{
    *ox = ba->R[0][0]*x + ba->R[0][1]*y + ba->R[0][2]*z;
    *oy = ba->R[1][0]*x + ba->R[1][1]*y + ba->R[1][2]*z;
    *oz = ba->R[2][0]*x + ba->R[2][1]*y + ba->R[2][2]*z;
}

#endif /* BOARD_ALIGN_H */
