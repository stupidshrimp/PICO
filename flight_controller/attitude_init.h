/*************************************************************************************************************
 * Coarse initial attitude alignment (TRIAD).
 *
 * The attitude EKF used to boot at the identity quaternion (level, facing the
 * magnetic reference) no matter how the airframe actually sat, then spend the
 * startup window converging toward the true attitude -- the operator saw that
 * as a large initial roll/pitch/yaw offset that slowly "drifted" away. Worse,
 * the innovation gates arm on an update *count*, so a slow convergence eats
 * into the window where large corrections are still accepted.
 *
 * The industry-standard fix is a deterministic coarse alignment before the
 * filter starts: solve Wahba's problem for the boot attitude from one
 * accelerometer observation of the earth specific-force reference and one
 * magnetometer observation of the local field, using the TRIAD method. The
 * EKF then starts at (roughly) the true attitude and only has to track.
 *
 * Everything here is pure math on plain float arrays -- no Arduino, matrix
 * library, or konfig dependencies -- so the exact flight code is compiled and
 * verified on the host by tests/attitude_init_test.cpp.
 *
 * Conventions match the EKF measurement model in Main.ino: quaternions
 * parameterize the earth->body direction cosine matrix M(q) with
 *   M(q) * accRef = predicted body-frame specific-force direction (accel rows),
 *   M(q) * magRef = predicted body-frame magnetic field           (mag rows),
 * i.e. the same M used by Main_bUpdateNonlinearY. The earth references are
 * caller-supplied so the alignment works in any earth convention (the firmware
 * passes (0,0,IMU_ACC_Z0) and IMU_MAG_B0).
 ************************************************************************************************************/
#ifndef ATTITUDE_INIT_H
#define ATTITUDE_INIT_H

#include <math.h>

#define ATTITUDE_INIT_VECTOR_EPSILON (1e-6f)

static inline void attitudeInitCross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

static inline bool attitudeInitNormalize3(float v[3])
{
    const float norm = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm < ATTITUDE_INIT_VECTOR_EPSILON) {
        return false;
    }
    v[0] /= norm;
    v[1] /= norm;
    v[2] /= norm;
    return true;
}

/* Extract the quaternion from an earth->body direction cosine matrix M laid
 * out in the EKF's parameterization:
 *   M = [ q0^2+q1^2-q2^2-q3^2   2(q1q2+q0q3)          2(q1q3-q0q2)
 *         2(q1q2-q0q3)          q0^2-q1^2+q2^2-q3^2   2(q2q3+q0q1)
 *         2(q1q3+q0q2)          2(q2q3-q0q1)          q0^2-q1^2-q2^2+q3^2 ]
 * Shepperd's method: branch on the largest of {trace, M00, M11, M22} so the
 * divisor is always well away from zero for any attitude (including inverted).
 */
static inline void attitudeInitQuatFromDcm(const float M[3][3], float quat[4])
{
    const float trace = M[0][0] + M[1][1] + M[2][2];
    float s;
    if (trace > 0.0f) {
        s = sqrtf(trace + 1.0f) * 2.0f;                       /* s = 4*q0 */
        quat[0] = 0.25f * s;
        quat[1] = (M[1][2] - M[2][1]) / s;
        quat[2] = (M[2][0] - M[0][2]) / s;
        quat[3] = (M[0][1] - M[1][0]) / s;
    } else if ((M[0][0] > M[1][1]) && (M[0][0] > M[2][2])) {
        s = sqrtf(1.0f + M[0][0] - M[1][1] - M[2][2]) * 2.0f; /* s = 4*q1 */
        quat[0] = (M[1][2] - M[2][1]) / s;
        quat[1] = 0.25f * s;
        quat[2] = (M[0][1] + M[1][0]) / s;
        quat[3] = (M[2][0] + M[0][2]) / s;
    } else if (M[1][1] > M[2][2]) {
        s = sqrtf(1.0f + M[1][1] - M[0][0] - M[2][2]) * 2.0f; /* s = 4*q2 */
        quat[0] = (M[2][0] - M[0][2]) / s;
        quat[1] = (M[0][1] + M[1][0]) / s;
        quat[2] = 0.25f * s;
        quat[3] = (M[1][2] + M[2][1]) / s;
    } else {
        s = sqrtf(1.0f + M[2][2] - M[0][0] - M[1][1]) * 2.0f; /* s = 4*q3 */
        quat[0] = (M[0][1] - M[1][0]) / s;
        quat[1] = (M[2][0] + M[0][2]) / s;
        quat[2] = (M[1][2] + M[2][1]) / s;
        quat[3] = 0.25f * s;
    }
}

/* TRIAD alignment: build the earth->body DCM that maps
 *   accRefEarth -> accBody (normalized accelerometer specific force) and
 *   magRefEarth -> magBody (as closely as the accel anchor allows),
 * then extract the quaternion. The accelerometer is the anchor observation
 * (exact), the magnetometer only resolves the rotation about it -- so mag
 * error cannot tilt the alignment, matching the decoupled fusion philosophy.
 *
 * accBody must be the specific force at rest (the gravity reaction), magBody
 * the iron-calibrated field. accRefEarth/magRefEarth are the earth-frame
 * references the EKF measurement model predicts against ((0,0,IMU_ACC_Z0) and
 * IMU_MAG_B0 in the firmware). Vectors need not be pre-normalized.
 *
 * Returns false (quat untouched) when the geometry is degenerate: a near-zero
 * accel, mag, or reference vector, or a field (measured or reference)
 * parallel to gravity so heading is unobservable.
 */
static inline bool bTriadAttitudeInit(const float accBody[3], const float magBody[3],
                                      const float accRefEarth[3], const float magRefEarth[3],
                                      float quat[4])
{
    /* Body-frame triad: anchor on the gravity direction. */
    float b1[3] = { accBody[0], accBody[1], accBody[2] };
    if (!attitudeInitNormalize3(b1)) {
        return false;
    }
    float b2[3];
    attitudeInitCross3(b1, magBody, b2);
    if (!attitudeInitNormalize3(b2)) {
        return false;
    }
    float b3[3];
    attitudeInitCross3(b1, b2, b3);

    /* Earth-frame triad from the same construction. */
    float r1[3] = { accRefEarth[0], accRefEarth[1], accRefEarth[2] };
    if (!attitudeInitNormalize3(r1)) {
        return false;
    }
    float r2[3];
    attitudeInitCross3(r1, magRefEarth, r2);
    if (!attitudeInitNormalize3(r2)) {
        return false;
    }
    float r3[3];
    attitudeInitCross3(r1, r2, r3);

    /* M = b1*r1' + b2*r2' + b3*r3' maps earth vectors to body vectors. */
    float M[3][3];
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            M[row][col] = b1[row]*r1[col] + b2[row]*r2[col] + b3[row]*r3[col];
        }
    }

    attitudeInitQuatFromDcm(M, quat);

    /* Unit norm + the q0 >= 0 hemisphere convention the filter boots with. */
    float norm = sqrtf(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
    if (norm < ATTITUDE_INIT_VECTOR_EPSILON || !isfinite(norm)) {
        return false;
    }
    if (quat[0] < 0.0f) {
        norm = -norm;
    }
    quat[0] /= norm;
    quat[1] /= norm;
    quat[2] /= norm;
    quat[3] /= norm;
    return true;
}

#endif /* ATTITUDE_INIT_H */
