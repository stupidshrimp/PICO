/*************************************************************************************************************
 * Host-side numerical verification for the boot-time coarse attitude alignment
 * (attitude_init.h). Compiles the EXACT flight header (it is pure math with no
 * Arduino dependencies) and proves:
 *   1. Round trip: for random attitudes, TRIAD on the model-generated accel/mag
 *      observations recovers the original quaternion (in the EKF's earth->body
 *      M(q) parameterization, q0 >= 0 hemisphere), including inverted and
 *      near-gimbal-lock attitudes that exercise every Shepperd branch.
 *   2. Anchor property: the alignment reproduces the accelerometer gravity
 *      direction EXACTLY regardless of magnetometer disturbance -- mag error
 *      can only rotate the solution about gravity (yaw), never tilt it. This
 *      matches the decoupled in-flight fusion philosophy.
 *   3. Degenerate observations (zero vectors, field parallel to gravity) are
 *      rejected instead of producing a garbage attitude.
 *
 * Build & run:
 *   c++ -std=c++17 -I.. -O2 -o /tmp/attitude_init_test attitude_init_test.cpp && /tmp/attitude_init_test
 ************************************************************************************************************/

#include "../attitude_init.h"

#include <cmath>
#include <cstdio>

/* ---- Magnetic reference for the firmware's default site, |B0| == 1. ---- */
static const double DECL = -0.05640509;
static const double INCL =  1.17209583;
static const float B0[3] = {
    (float)(cos(INCL) * cos(DECL)),
    (float)(cos(INCL) * sin(DECL)),
    (float)(-sin(INCL))
};

static int g_fail = 0;

/* Earth->body DCM in the EKF's parameterization (matches Main_bUpdateNonlinearY). */
static void MofQ(const double q[4], double M[3][3]) {
    const double q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    M[0][0] = q0*q0+q1*q1-q2*q2-q3*q3; M[0][1] = 2*(q1*q2+q0*q3);         M[0][2] = 2*(q1*q3-q0*q2);
    M[1][0] = 2*(q1*q2-q0*q3);         M[1][1] = q0*q0-q1*q1+q2*q2-q3*q3; M[1][2] = 2*(q2*q3+q0*q1);
    M[2][0] = 2*(q1*q3+q0*q2);         M[2][1] = 2*(q2*q3-q0*q1);         M[2][2] = q0*q0-q1*q1-q2*q2+q3*q3;
}

static void MtimesV(const double M[3][3], const float v[3], float out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = (float)(M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2]);
}

static unsigned g_seed = 24680;
static double frand(double lo, double hi) {
    g_seed = g_seed*1103515245u + 12345u;
    double u = ((g_seed >> 16) & 0x7fff) / 32767.0;
    return lo + (hi - lo) * u;
}

/* Angle between two unit quaternions (attitude error, rad), sign-invariant. */
static double quatAngleError(const double qa[4], const float qb[4]) {
    double dot = qa[0]*qb[0] + qa[1]*qb[1] + qa[2]*qb[2] + qa[3]*qb[3];
    if (dot < 0) dot = -dot;
    if (dot > 1.0) dot = 1.0;
    return 2.0 * acos(dot);
}

/* Test 1: round trip over random attitudes (uniform on the quaternion sphere,
 * so inverted and steep attitudes -- every Shepperd branch -- are covered). */
static void test_round_trip() {
    std::printf("[test_round_trip] TRIAD recovers random attitudes\n");
    double worst = 0.0;
    for (int t = 0; t < 2000; t++) {
        double q[4] = { frand(-1,1), frand(-1,1), frand(-1,1), frand(-1,1) };
        double n = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        if (n < 0.1) continue;
        for (int i = 0; i < 4; i++) q[i] /= n;

        double M[3][3];
        MofQ(q, M);
        const float up[3] = {0.0f, 0.0f, 1.0f};
        float acc[3], mag[3];
        MtimesV(M, up, acc);
        MtimesV(M, B0, mag);
        /* physical magnitudes: the header must not require unit inputs */
        for (int i = 0; i < 3; i++) { acc[i] *= 9.80665f; mag[i] *= 45.0f; }

        float qhat[4];
        if (!bTriadAttitudeInit(acc, mag, B0, qhat)) {
            g_fail++;
            std::printf("  FAIL trial %d: alignment rejected a valid observation\n", t);
            continue;
        }
        const double err = quatAngleError(q, qhat);
        if (err > worst) worst = err;
        if (err > 2e-3) {   /* float32 pipeline: allow ~0.1 deg */
            g_fail++;
            std::printf("  FAIL trial %d: attitude error %.6f rad\n", t, err);
        }
        if (qhat[0] < 0.0f) {
            g_fail++;
            std::printf("  FAIL trial %d: q0 hemisphere convention violated\n", t);
        }
    }
    std::printf("  ok   2000 random attitudes, worst error %.2e rad\n", worst);
}

/* Test 2: magnetometer disturbance cannot tilt the alignment. */
static void test_mag_disturbance_anchor() {
    std::printf("[test_mag_disturbance_anchor] mag error only rotates about gravity\n");
    double worstTilt = 0.0;
    for (int t = 0; t < 500; t++) {
        double q[4] = { frand(-1,1), frand(-1,1), frand(-1,1), frand(-1,1) };
        double n = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        if (n < 0.1) continue;
        for (int i = 0; i < 4; i++) q[i] /= n;

        double M[3][3];
        MofQ(q, M);
        const float up[3] = {0.0f, 0.0f, 1.0f};
        float acc[3], mag[3];
        MtimesV(M, up, acc);
        MtimesV(M, B0, mag);
        /* heavy disturbance: hard-iron-like offset up to ~60% of the field */
        float magBad[3] = { mag[0] + (float)frand(-0.6, 0.6),
                            mag[1] + (float)frand(-0.6, 0.6),
                            mag[2] + (float)frand(-0.6, 0.6) };

        float qhat[4];
        if (!bTriadAttitudeInit(acc, magBad, B0, qhat)) continue;  /* degenerate combos may reject */

        /* gravity direction predicted by the aligned attitude must equal acc */
        const double qh[4] = { qhat[0], qhat[1], qhat[2], qhat[3] };
        double Mh[3][3];
        MofQ(qh, Mh);
        float gpred[3];
        MtimesV(Mh, up, gpred);
        const double tilt = sqrt( (gpred[0]-acc[0])*(gpred[0]-acc[0])
                                + (gpred[1]-acc[1])*(gpred[1]-acc[1])
                                + (gpred[2]-acc[2])*(gpred[2]-acc[2]) );
        if (tilt > worstTilt) worstTilt = tilt;
        if (tilt > 1e-4) {
            g_fail++;
            std::printf("  FAIL trial %d: mag disturbance tilted the alignment by %.6f\n", t, tilt);
        }
    }
    std::printf("  ok   500 disturbed-mag alignments, worst tilt leak %.2e\n", worstTilt);
}

/* Test 3: degenerate observations are rejected. */
static void test_degenerate_rejection() {
    std::printf("[test_degenerate_rejection] unusable observations rejected\n");
    float quat[4] = {9, 9, 9, 9};
    const float acc[3] = {0.1f, -0.2f, 9.7f};
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float magAlongGravity[3] = {0.02f, -0.04f, 1.94f};  /* parallel to acc */
    const float upRef[3] = {0.0f, 0.0f, 1.0f};

    struct { bool got; const char* name; } cases[] = {
        { bTriadAttitudeInit(zero, B0, B0, quat),               "zero accel" },
        { bTriadAttitudeInit(acc, zero, B0, quat),              "zero mag" },
        { bTriadAttitudeInit(acc, magAlongGravity, B0, quat),   "mag parallel to gravity" },
        { bTriadAttitudeInit(acc, B0, upRef, quat),             "reference parallel to earth-up" },
        { bTriadAttitudeInit(acc, B0, zero, quat),              "zero reference" },
    };
    for (const auto& c : cases) {
        if (c.got) {
            g_fail++;
            std::printf("  FAIL %s: expected rejection\n", c.name);
        } else {
            std::printf("  ok   %s rejected\n", c.name);
        }
    }
}

int main() {
    std::printf("B0 = [% .5f % .5f % .5f]\n\n", (double)B0[0], (double)B0[1], (double)B0[2]);
    test_round_trip();
    test_mag_disturbance_anchor();
    test_degenerate_rejection();
    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
