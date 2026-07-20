/*************************************************************************************************************
 * Host-side numerical verification for the board-alignment (level) trim
 * (board_align.h). Compiles the EXACT flight headers (both are pure math with no
 * Arduino dependencies) and proves:
 *   1. Header sanity: the built rotation is orthonormal with det +1 (a proper
 *      rotation), equals the transpose of the offset attitude it inverts, and is
 *      exactly the identity for a zero offset (untrimmed build unchanged).
 *   2. Target case: an IMU mounted with the operator's reported offset
 *      (-6.9 deg roll, -4.3 deg pitch) reports exactly that offset UNCORRECTED
 *      on a level airframe, and reads level once the trim is applied -- i.e. the
 *      constants mean "the roll/pitch the FC shows when the aircraft is level".
 *   3. End to end: for random airframe attitudes and random mount offsets, the
 *      trimmed accel/mag stream fed through the real flight TRIAD init
 *      (attitude_init.h) recovers the TRUE AIRFRAME attitude, at any bank/pitch,
 *      not just near level. The gyro rate transforms consistently (same
 *      rotation), so prediction and correction stay in one frame.
 *
 * Build & run:
 *   c++ -std=c++17 -I.. -O2 -o /tmp/board_align_test board_align_test.cpp && /tmp/board_align_test
 ************************************************************************************************************/

#include "../board_align.h"
#include "../attitude_init.h"

#include <cmath>
#include <cstdio>

/* Magnetic reference for the firmware's default site, |B0| == 1 (NED Z-down). */
static const double DECL = -0.05640509;
static const double INCL =  1.17209583;
static const float  B0_NED[3]      = { (float)(cos(INCL)*cos(DECL)), (float)(cos(INCL)*sin(DECL)), (float)sin(INCL) };
static const float  ACC_REF_NED[3] = { 0.0f, 0.0f, -1.0f };

static int g_fail = 0;

/* Earth->body DCM in the EKF's parameterization (matches Main_bUpdateNonlinearY). */
static void MofQ(const double q[4], double M[3][3]) {
    const double q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    M[0][0]=q0*q0+q1*q1-q2*q2-q3*q3; M[0][1]=2*(q1*q2+q0*q3);         M[0][2]=2*(q1*q3-q0*q2);
    M[1][0]=2*(q1*q2-q0*q3);         M[1][1]=q0*q0-q1*q1+q2*q2-q3*q3; M[1][2]=2*(q2*q3+q0*q1);
    M[2][0]=2*(q1*q3+q0*q2);         M[2][1]=2*(q2*q3-q0*q1);         M[2][2]=q0*q0-q1*q1-q2*q2+q3*q3;
}

static void matVec(const double M[3][3], const double v[3], double out[3]) {
    for (int i = 0; i < 3; i++) out[i] = M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2];
}
static void matMul(const double A[3][3], const double B[3][3], double out[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
}

/* Firmware quaternionToEulerDeg convention (degrees). */
static void quatToEulerDeg(const float q[4], double e[3]) {
    const double q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    double pitchArg = 2.0*(q3*q1 - q0*q2);
    if (pitchArg >  1.0) pitchArg =  1.0;
    if (pitchArg < -1.0) pitchArg = -1.0;
    e[0] = atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)) * 180.0/M_PI;
    e[1] = asin(pitchArg) * 180.0/M_PI;
    e[2] = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3)) * 180.0/M_PI;
}

/* Offset attitude M_mount = Rx(phi)*Ry(theta) (earth->body), built independently
 * of the header so the header's rotation can be cross-checked. phi = rollOffDeg,
 * theta = -pitchOffDeg (firmware pitch == -standard pitch). R_align == M_mount^T. */
static void mountDcm(double rollOffDeg, double pitchOffDeg, double M[3][3]) {
    const double phi   =  rollOffDeg  * M_PI/180.0;
    const double theta = -pitchOffDeg * M_PI/180.0;
    const double cphi=cos(phi), sphi=sin(phi), cth=cos(theta), sth=sin(theta);
    M[0][0]=cth;        M[0][1]=0.0;   M[0][2]=-sth;
    M[1][0]=sphi*sth;   M[1][1]=cphi;  M[1][2]=sphi*cth;
    M[2][0]=cphi*sth;   M[2][1]=-sphi; M[2][2]=cphi*cth;
}

static unsigned g_seed = 0xB0A1D;
static double frand(double lo, double hi) {
    g_seed = g_seed*1103515245u + 12345u;
    return lo + (hi - lo) * (((g_seed >> 16) & 0x7fff) / 32767.0);
}

/* ---------------- Test 1: header sanity ---------------- */
static void test_header_sanity() {
    std::printf("[test_header_sanity] rotation orthonormal, det +1, == M_mount^T, identity at zero\n");
    double worstOrtho = 0.0, worstDet = 0.0, worstVsMount = 0.0;

    /* Identity at zero offset, bit-for-bit. */
    BoardAlign z; boardAlignInit(&z, 0.0f, 0.0f);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            const float want = (i == j) ? 1.0f : 0.0f;
            if (z.R[i][j] != want) {
                g_fail++;
                std::printf("  FAIL zero offset R[%d][%d]=%.9g, expected %.0f\n", i, j, z.R[i][j], want);
            }
        }

    for (double roll = -20.0; roll <= 20.0; roll += 2.5) {
        for (double pitch = -20.0; pitch <= 20.0; pitch += 2.5) {
            BoardAlign ba; boardAlignInit(&ba, (float)roll, (float)pitch);

            /* R R^T == I */
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    double dot = 0.0;
                    for (int k = 0; k < 3; k++) dot += (double)ba.R[i][k]*ba.R[j][k];
                    const double want = (i == j) ? 1.0 : 0.0;
                    worstOrtho = std::fmax(worstOrtho, std::fabs(dot - want));
                }
            /* det == +1 */
            const double det =
                (double)ba.R[0][0]*(ba.R[1][1]*ba.R[2][2] - ba.R[1][2]*ba.R[2][1])
              - (double)ba.R[0][1]*(ba.R[1][0]*ba.R[2][2] - ba.R[1][2]*ba.R[2][0])
              + (double)ba.R[0][2]*(ba.R[1][0]*ba.R[2][1] - ba.R[1][1]*ba.R[2][0]);
            worstDet = std::fmax(worstDet, std::fabs(det - 1.0));

            /* R == M_mount^T */
            double M[3][3]; mountDcm(roll, pitch, M);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    worstVsMount = std::fmax(worstVsMount, std::fabs((double)ba.R[i][j] - M[j][i]));
        }
    }
    std::printf("  ok   worst |RR^T-I|=%.2e  |det-1|=%.2e  |R-M_mount^T|=%.2e\n",
                worstOrtho, worstDet, worstVsMount);
    if (worstOrtho > 1e-5 || worstDet > 1e-5 || worstVsMount > 1e-5) {
        g_fail++;
        std::printf("  FAIL header rotation is not the expected proper rotation\n");
    }
}

/* ---------------- Test 2: the operator's target case ---------------- */
static void test_target_offset() {
    std::printf("[test_target_offset] mount offset (-6.9 roll, -4.3 pitch): reads it uncorrected, level trimmed\n");
    const double rollOff = -6.9, pitchOff = -4.3;

    /* Level airframe seen through the mount: sensors = M_mount * (earth refs). */
    double M[3][3]; mountDcm(rollOff, pitchOff, M);
    const double accRef[3] = {0, 0, -1};
    const double magRef[3] = {B0_NED[0], B0_NED[1], B0_NED[2]};
    double accImu[3], magImu[3];
    matVec(M, accRef, accImu);
    matVec(M, magRef, magImu);

    /* Uncorrected: TRIAD should report the raw offset. */
    float accU[3] = {(float)accImu[0], (float)accImu[1], (float)accImu[2]};
    float magU[3] = {(float)magImu[0], (float)magImu[1], (float)magImu[2]};
    float qU[4]; double eU[3];
    if (!bTriadAttitudeInit(accU, magU, ACC_REF_NED, B0_NED, qU)) { g_fail++; std::printf("  FAIL uncorrected TRIAD rejected\n"); return; }
    quatToEulerDeg(qU, eU);

    /* Corrected: apply the trim, TRIAD should read level. */
    BoardAlign ba; boardAlignInit(&ba, (float)rollOff, (float)pitchOff);
    float accC[3], magC[3];
    boardAlignApply(&ba, accU[0], accU[1], accU[2], &accC[0], &accC[1], &accC[2]);
    boardAlignApply(&ba, magU[0], magU[1], magU[2], &magC[0], &magC[1], &magC[2]);
    float qC[4]; double eC[3];
    if (!bTriadAttitudeInit(accC, magC, ACC_REF_NED, B0_NED, qC)) { g_fail++; std::printf("  FAIL corrected TRIAD rejected\n"); return; }
    quatToEulerDeg(qC, eC);

    std::printf("  info uncorrected roll/pitch = %.3f/%.3f deg (expect %.1f/%.1f)\n", eU[0], eU[1], rollOff, pitchOff);
    std::printf("  info corrected   roll/pitch = %.3f/%.3f deg (expect 0/0)\n", eC[0], eC[1]);
    if (std::fabs(eU[0] - rollOff) > 0.05 || std::fabs(eU[1] - pitchOff) > 0.05) {
        g_fail++; std::printf("  FAIL uncorrected output does not match the reported offset\n");
    }
    if (std::fabs(eC[0]) > 0.05 || std::fabs(eC[1]) > 0.05) {
        g_fail++; std::printf("  FAIL trim did not level the roll/pitch\n");
    } else {
        std::printf("  ok   trim removes the offset; roll/pitch read level\n");
    }
}

/* ---------------- Test 3: end to end over random attitudes & offsets ---------------- */
static void test_recovers_true_attitude() {
    std::printf("[test_recovers_true_attitude] trimmed stream through flight TRIAD recovers true airframe attitude\n");
    double worstRP = 0.0, worstGyro = 0.0;
    int tested = 0;

    for (int t = 0; t < 4000; t++) {
        /* random true airframe attitude */
        double q[4] = { frand(-1,1), frand(-1,1), frand(-1,1), frand(-1,1) };
        double n = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        if (n < 0.1) continue;
        for (int i = 0; i < 4; i++) q[i] /= n;
        double Mtrue[3][3]; MofQ(q, Mtrue);

        /* random mount offset */
        const double rollOff  = frand(-15.0, 15.0);
        const double pitchOff = frand(-15.0, 15.0);
        double Mmount[3][3]; mountDcm(rollOff, pitchOff, Mmount);
        double Mimu[3][3]; matMul(Mmount, Mtrue, Mimu);   /* earth -> IMU-body */

        const double accRef[3] = {0, 0, -1};
        const double magRef[3] = {B0_NED[0], B0_NED[1], B0_NED[2]};

        /* what the airframe SHOULD report (reference), and what the mounted IMU reads */
        double accTrue[3], accImu[3], magImu[3];
        matVec(Mtrue,  accRef, accTrue);
        matVec(Mimu,   accRef, accImu);
        matVec(Mimu,   magRef, magImu);
        /* physical magnitudes: the trim/init must not require unit inputs */
        float accImuf[3] = {(float)(accImu[0]*9.80665), (float)(accImu[1]*9.80665), (float)(accImu[2]*9.80665)};
        float magImuf[3] = {(float)(magImu[0]*45.0),    (float)(magImu[1]*45.0),    (float)(magImu[2]*45.0)};

        BoardAlign ba; boardAlignInit(&ba, (float)rollOff, (float)pitchOff);
        float accC[3], magC[3];
        boardAlignApply(&ba, accImuf[0], accImuf[1], accImuf[2], &accC[0], &accC[1], &accC[2]);
        boardAlignApply(&ba, magImuf[0], magImuf[1], magImuf[2], &magC[0], &magC[1], &magC[2]);

        float qhat[4];
        if (!bTriadAttitudeInit(accC, magC, ACC_REF_NED, B0_NED, qhat)) continue;

        double eEst[3], eTrue[3];
        quatToEulerDeg(qhat, eEst);
        float qtf[4] = {(float)q[0], (float)q[1], (float)q[2], (float)q[3]};
        quatToEulerDeg(qtf, eTrue);
        if (std::fabs(std::sin(eTrue[1]*M_PI/180.0)) > 0.98) continue;   /* skip gimbal lock */
        ++tested;

        worstRP = std::fmax(worstRP, std::fabs(eEst[0] - eTrue[0]));
        worstRP = std::fmax(worstRP, std::fabs(eEst[1] - eTrue[1]));

        /* gyro: a rate in airframe axes, measured in IMU axes, is recovered */
        const double w[3] = { frand(-3,3), frand(-3,3), frand(-3,3) };
        double wImu[3]; matVec(Mmount, w, wImu);
        float wc[3];
        boardAlignApply(&ba, (float)wImu[0], (float)wImu[1], (float)wImu[2], &wc[0], &wc[1], &wc[2]);
        for (int i = 0; i < 3; i++) worstGyro = std::fmax(worstGyro, std::fabs((double)wc[i] - w[i]));
    }

    std::printf("  ok   %d attitudes, worst roll/pitch err = %.4f deg, worst gyro err = %.2e rad/s\n",
                tested, worstRP, worstGyro);
    if (worstRP > 0.05) { g_fail++; std::printf("  FAIL trimmed attitude does not match the true airframe attitude\n"); }
    if (worstGyro > 1e-4) { g_fail++; std::printf("  FAIL gyro not recovered by the same rotation\n"); }
}

int main() {
    std::printf("board-alignment trim: reported (roll,pitch) == (phi,-theta) of Rx(phi)Ry(theta); R_align = M_mount^T\n\n");
    test_header_sanity();
    test_target_offset();
    test_recovers_true_attitude();
    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
