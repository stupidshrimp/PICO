/*************************************************************************************************************
 * IMU -> aircraft/EKF axis alignment for the attitude filter.
 *
 * The MPU9250 on this airframe is mounted rotated 90 degrees about its Z (yaw)
 * axis relative to the aircraft body frame, so its raw X/Y axes must be rotated
 * into the aircraft/EKF frame before the attitude EKF consumes the accelerometer,
 * gyroscope and magnetometer. (The driver already aligns the AK8963 magnetometer
 * into the accel/gyro frame, so all three sensors arrive here in one consistent,
 * right-handed frame.)
 *
 * This transform MUST be a proper rotation (determinant +1). The firmware
 * previously exchanged X and Y with no sign change, (x,y,z) -> (y,x,z), which is a
 * REFLECTION (determinant -1) and is invalid for the right-handed quaternion
 * attitude filter: angular velocity is a pseudovector, so under a reflection the
 * gyro would need an extra sign the accel/mag do not, and the model has none.
 * The observable result was that the reported attitude
 *   - swapped roll and pitch,
 *   - put a large offset on the reported heading and reversed its direction, and
 *   - made the gyro prediction fight the accel/mag correction, so all three axes
 *     showed an offset that slowly drifted.
 * See flight_controller/tests/imu_axis_mount_test.cpp for the guard against the
 * reflection ever coming back.
 *
 * Two 90-degree mountings are physically possible; they differ only by the sign
 * of the rotation, and choosing the wrong one inverts BOTH roll and pitch and
 * offsets the reported heading by 180 degrees (it cannot corrupt only one axis).
 * Set FC_IMU_MOUNT_YAW_ROTATION to match the airframe:
 *   -1 : board rotated 90 deg CW  -> recover with R_z(-90): (x,y,z) -> ( y,-x, z)
 *   +1 : board rotated 90 deg CCW -> recover with R_z(+90): (x,y,z) -> (-y, x, z)
 *
 * VERIFY ON THE BENCH BEFORE FLIGHT (the CW/CCW mounting is a hardware fact this
 * header cannot know):
 *   - pitch the nose up            -> reported pitch must go positive,
 *   - drop the right wing          -> reported roll must move to its intended sign,
 *   - yaw the nose to the right    -> the compass heading must INCREASE.
 * If roll and pitch both read backwards (and the heading is ~180 deg off), flip
 * FC_IMU_MOUNT_YAW_ROTATION.
 *
 * NOTE: the stored HARD_IRON_BIAS / SOFT_IRON_MATRIX magnetometer constants in
 * Main.ino were captured in the old (invalid) frame. Re-run the magnetometer
 * calibration (FC_MAG_CALIBRATION_MODE) after applying this fix; heading accuracy
 * may be off until then, but roll, pitch and the drift are corrected regardless.
 ************************************************************************************************************/
#ifndef IMU_AXIS_MOUNT_H
#define IMU_AXIS_MOUNT_H

#include "konfig.h"   /* float_prec */

#ifndef FC_IMU_MOUNT_YAW_ROTATION
#define FC_IMU_MOUNT_YAW_ROTATION (-1)   /* default: 90 deg CW; flip to +1 if roll & pitch read inverted */
#endif

#if (FC_IMU_MOUNT_YAW_ROTATION != 1) && (FC_IMU_MOUNT_YAW_ROTATION != -1)
#error "FC_IMU_MOUNT_YAW_ROTATION must be +1 (90 deg CCW) or -1 (90 deg CW)"
#endif

/* Rotate a raw IMU-frame axis triplet (x,y,z) into the aircraft/EKF frame by the
 * configured proper 90-degree yaw rotation. Applied identically to gyro, accel
 * and magnetometer so the fused measurement set stays right-handed. Inputs are
 * taken by value, so passing an output that also feeds an input (e.g. a matrix
 * element) is safe. */
static inline void imuAxesToBody(float_prec x, float_prec y, float_prec z,
                                 float_prec& bx, float_prec& by, float_prec& bz) {
#if (FC_IMU_MOUNT_YAW_ROTATION < 0)
    bx =  y; by = -x; bz = z;   /* R_z(-90): board mounted 90 deg CW  */
#else
    bx = -y; by =  x; bz = z;   /* R_z(+90): board mounted 90 deg CCW */
#endif
}

#endif /* IMU_AXIS_MOUNT_H */
