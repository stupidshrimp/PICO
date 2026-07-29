/*************************************************************************************************************
 * Host-side finite-difference verification for the attitude EKF analytic
 * Jacobians F and (legacy 3-axis) H.
 *
 * The EKF in Main.ino propagates the state with a hand-written nonlinear model
 * (Main_bUpdateNonlinearX = f, Main_bUpdateNonlinearY = h) and feeds the filter
 * hand-derived analytic Jacobians (Main_bCalcJacobianF = F = df/dx,
 * Main_bCalcJacobianH = H = dh/dx). A wrong sign or term in F/H does NOT crash or
 * NaN in flight -- the quaternion still normalizes and the covariance still
 * updates -- it just computes a slightly-wrong Kalman gain every step, silently
 * degrading accuracy. This test makes that class of error fail the build instead:
 * it compares each analytic Jacobian against a central finite difference of the
 * model it is supposed to differentiate.
 *
 *   - test_F:         F (df/dx) for the quaternion + gyro-bias state transition.
 *                     The finite difference uses the UN-normalized propagation,
 *                     because F is the Jacobian of that map; Main.ino applies the
 *                     quaternion normalization afterward and it is not part of F.
 *   - test_H_coupled: H (dh/dx) for the legacy accel(3)+mag(3) measurement
 *                     (FC_EKF_DECOUPLE_MAG == 0). The decoupled yaw Jacobian is
 *                     already covered by ekf_decouple_mag_test.cpp.
 *
 * For these polynomial models the central difference is exact up to floating
 * point round-off, so the tolerance is round-off-limited; a genuine sign/term
 * error produces an O(1) mismatch and is caught with wide margin.
 *
 * The f/F/h/H expressions below are copied VERBATIM from Main.ino. Main.ino can
 * not be compiled on the host (it pulls in Servo/Wire/CRSF/GPS), so -- as in the
 * sibling ekf_decouple_mag_test.cpp -- the model is replicated here. Keep these
 * copies in sync with Main_bUpdateNonlinearX / Main_bCalcJacobianF /
 * Main_bUpdateNonlinearY / Main_bCalcJacobianH if that model ever changes.
 *
 * Build & run:
 *   c++ -std=c++17 -O2 -o /tmp/ekf_jacobian_test ekf_jacobian_test.cpp && /tmp/ekf_jacobian_test
 ************************************************************************************************************/
#include <cstdio>
#include <cmath>

/* double precision for tight finite-difference tolerances. Flight runs single
 * float, but this checks the algebra of the Jacobian, not the float rounding. */

/* Representative integration step (Main.ino's gEkfRuntimeDt == SS_DT). F scales
 * linearly with it, so any single positive value exercises every term. */
static const double DT = 0.008;
static const double IMU_ACC_Z0 = 1.0;

/* Magnetic reference field, built exactly as Main.ino builds IMU_MAG_B0 from the
 * firmware default site (central Illinois). |B0| == 1 by construction. */
static const double DECL = -0.05640509;
static const double INCL =  1.17209583;
static double B0[3] = { cos(INCL) * cos(DECL), cos(INCL) * sin(DECL), -sin(INCL) };

/* ---- deterministic PRNG (same scheme as ekf_decouple_mag_test.cpp) ---- */
static unsigned g_seed = 12345;
static double frand(double lo, double hi) {
    g_seed = g_seed * 1103515245u + 12345u;
    double u = ((g_seed >> 16) & 0x7fff) / 32767.0;
    return lo + (hi - lo) * u;
}
static void qnorm(double q[4]) {
    double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; i++) q[i] /= n;
}

/* ===================== EXACT replica of the firmware model ===================== */

/* f: state propagation, copied verbatim from Main_bUpdateNonlinearX WITHOUT the
 * trailing Main_bNormalizeState call. State is [quaternion(4), gyro_bias(3)];
 * bias-corrected gyro rates drive the quaternion and the bias is held constant. */
static void f_propagate(const double X[7], const double U[3], double Xn[7]) {
    double q0 = X[0], q1 = X[1], q2 = X[2], q3 = X[3];
    double bp = X[4], bq = X[5], br = X[6];
    double p = U[0] - bp;
    double q = U[1] - bq;
    double r = U[2] - br;

    Xn[0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3)) * DT + q0;
    Xn[1] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3)) * DT + q1;
    Xn[2] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3)) * DT + q2;
    Xn[3] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00)) * DT + q3;
    Xn[4] = bp;
    Xn[5] = bq;
    Xn[6] = br;
}

/* F = df/dx, copied verbatim from Main_bCalcJacobianF. F[i][j] = d Xn[i] / d X[j]. */
static void F_analytic(const double X[7], const double U[3], double F[7][7]) {
    double q0 = X[0], q1 = X[1], q2 = X[2], q3 = X[3];
    double p = U[0] - X[4];
    double q = U[1] - X[5];
    double r = U[2] - X[6];

    for (int i = 0; i < 7; i++) for (int j = 0; j < 7; j++) F[i][j] = 0.0;

    F[0][0] =  1.000;
    F[1][0] =  0.5*p * DT;
    F[2][0] =  0.5*q * DT;
    F[3][0] =  0.5*r * DT;

    F[0][1] = -0.5*p * DT;
    F[1][1] =  1.000;
    F[2][1] = -0.5*r * DT;
    F[3][1] =  0.5*q * DT;

    F[0][2] = -0.5*q * DT;
    F[1][2] =  0.5*r * DT;
    F[2][2] =  1.000;
    F[3][2] = -0.5*p * DT;

    F[0][3] = -0.5*r * DT;
    F[1][3] = -0.5*q * DT;
    F[2][3] =  0.5*p * DT;
    F[3][3] =  1.000;

    F[0][4] =  0.5*q1 * DT;
    F[1][4] = -0.5*q0 * DT;
    F[2][4] = -0.5*q3 * DT;
    F[3][4] =  0.5*q2 * DT;

    F[0][5] =  0.5*q2 * DT;
    F[1][5] =  0.5*q3 * DT;
    F[2][5] = -0.5*q0 * DT;
    F[3][5] = -0.5*q1 * DT;

    F[0][6] =  0.5*q3 * DT;
    F[1][6] = -0.5*q2 * DT;
    F[2][6] =  0.5*q1 * DT;
    F[3][6] = -0.5*q0 * DT;

    F[4][4] = 1.000;
    F[5][5] = 1.000;
    F[6][6] = 1.000;
}

/* h: legacy 3-axis measurement (accel direction + body-frame magnetic field),
 * copied verbatim from Main_bUpdateNonlinearY (FC_EKF_DECOUPLE_MAG == 0 branch). */
static void h_coupled(const double X[7], double Y[6]) {
    double q0 = X[0], q1 = X[1], q2 = X[2], q3 = X[3];
    double q0_2 = q0*q0, q1_2 = q1*q1, q2_2 = q2*q2, q3_2 = q3*q3;

    Y[0] = (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;
    Y[1] = (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;
    Y[2] = (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;

    Y[3] = (+(q0_2)+(q1_2)-(q2_2)-(q3_2)) * B0[0]
         +(2*(q1*q2+q0*q3)) * B0[1]
         +(2*(q1*q3-q0*q2)) * B0[2];

    Y[4] = (2*(q1*q2-q0*q3)) * B0[0]
         +(+(q0_2)-(q1_2)+(q2_2)-(q3_2)) * B0[1]
         +(2*(q2*q3+q0*q1)) * B0[2];

    Y[5] = (2*(q1*q3+q0*q2)) * B0[0]
         +(2*(q2*q3-q0*q1)) * B0[1]
         +(+(q0_2)-(q1_2)-(q2_2)+(q3_2)) * B0[2];
}

/* H = dh/dx, copied verbatim from Main_bCalcJacobianH (FC_EKF_DECOUPLE_MAG == 0
 * branch). H[i][j] = d Y[i] / d X[j]. Columns 4..6 (gyro bias) are zero: the
 * measurement does not depend on bias. */
static void H_coupled(const double X[7], double H[6][7]) {
    double q0 = X[0], q1 = X[1], q2 = X[2], q3 = X[3];

    for (int i = 0; i < 6; i++) for (int j = 0; j < 7; j++) H[i][j] = 0.0;

    H[0][0] = -2*q2 * IMU_ACC_Z0;
    H[1][0] = +2*q1 * IMU_ACC_Z0;
    H[2][0] = +2*q0 * IMU_ACC_Z0;

    H[0][1] = +2*q3 * IMU_ACC_Z0;
    H[1][1] = +2*q0 * IMU_ACC_Z0;
    H[2][1] = -2*q1 * IMU_ACC_Z0;

    H[0][2] = -2*q0 * IMU_ACC_Z0;
    H[1][2] = +2*q3 * IMU_ACC_Z0;
    H[2][2] = -2*q2 * IMU_ACC_Z0;

    H[0][3] = +2*q1 * IMU_ACC_Z0;
    H[1][3] = +2*q2 * IMU_ACC_Z0;
    H[2][3] = +2*q3 * IMU_ACC_Z0;

    H[3][0] =  2*q0*B0[0] + 2*q3*B0[1] - 2*q2*B0[2];
    H[4][0] = -2*q3*B0[0] + 2*q0*B0[1] + 2*q1*B0[2];
    H[5][0] =  2*q2*B0[0] - 2*q1*B0[1] + 2*q0*B0[2];

    H[3][1] =  2*q1*B0[0] + 2*q2*B0[1] + 2*q3*B0[2];
    H[4][1] =  2*q2*B0[0] - 2*q1*B0[1] + 2*q0*B0[2];
    H[5][1] =  2*q3*B0[0] - 2*q0*B0[1] - 2*q1*B0[2];

    H[3][2] = -2*q2*B0[0] + 2*q1*B0[1] - 2*q0*B0[2];
    H[4][2] =  2*q1*B0[0] + 2*q2*B0[1] + 2*q3*B0[2];
    H[5][2] =  2*q0*B0[0] + 2*q3*B0[1] - 2*q2*B0[2];

    H[3][3] = -2*q3*B0[0] + 2*q0*B0[1] + 2*q1*B0[2];
    H[4][3] = -2*q0*B0[0] - 2*q3*B0[1] + 2*q2*B0[2];
    H[5][3] =  2*q1*B0[0] + 2*q2*B0[1] + 2*q3*B0[2];
}

/* ===================== Tests ===================== */

static int g_fail = 0;

/* F (df/dx) vs central finite difference of the un-normalized propagation. */
static void test_F() {
    std::printf("[test_F] F (df/dx) vs central finite difference\n");
    const double eps = 1e-6, tol = 1e-6;
    double worst = 0.0; int wi = -1, wj = -1;
    for (int t = 0; t < 200; t++) {
        double q[4] = { frand(-1,1), frand(-1,1), frand(-1,1), frand(-1,1) };
        if (q[0] < 0) for (int i = 0; i < 4; i++) q[i] = -q[i];
        qnorm(q);
        double X[7] = { q[0], q[1], q[2], q[3], frand(-0.1,0.1), frand(-0.1,0.1), frand(-0.1,0.1) };
        double U[3] = { frand(-3,3), frand(-3,3), frand(-3,3) };   /* gyro rates, rad/s */

        double F[7][7];
        F_analytic(X, U, F);
        for (int j = 0; j < 7; j++) {
            double Xp[7], Xm[7];
            for (int k = 0; k < 7; k++) { Xp[k] = X[k]; Xm[k] = X[k]; }
            Xp[j] += eps; Xm[j] -= eps;
            double fp[7], fm[7];
            f_propagate(Xp, U, fp);
            f_propagate(Xm, U, fm);
            for (int i = 0; i < 7; i++) {
                double fd = (fp[i] - fm[i]) / (2*eps);
                double err = fabs(fd - F[i][j]);
                if (err > worst) { worst = err; wi = i; wj = j; }
                if (err > tol) {
                    g_fail++;
                    std::printf("  FAIL trial %d F[%d][%d]: analytic=% .8f fd=% .8f\n",
                                t, i, j, F[i][j], fd);
                }
            }
        }
    }
    std::printf("  ok   200 random states, all 49 entries within %.0e (worst=%.2e at F[%d][%d])\n",
                tol, worst, wi, wj);
}

/* H (dh/dx) vs central finite difference of the legacy 3-axis measurement. */
static void test_H_coupled() {
    std::printf("[test_H_coupled] H (dh/dx) vs finite difference (accel + 3-axis mag)\n");
    const double eps = 1e-6, tol = 1e-6;
    double worst = 0.0; int wi = -1, wj = -1;
    for (int t = 0; t < 200; t++) {
        double q[4] = { frand(-1,1), frand(-1,1), frand(-1,1), frand(-1,1) };
        if (q[0] < 0) for (int i = 0; i < 4; i++) q[i] = -q[i];
        qnorm(q);
        double X[7] = { q[0], q[1], q[2], q[3], frand(-0.1,0.1), frand(-0.1,0.1), frand(-0.1,0.1) };

        double H[6][7];
        H_coupled(X, H);
        for (int j = 0; j < 7; j++) {
            double Xp[7], Xm[7];
            for (int k = 0; k < 7; k++) { Xp[k] = X[k]; Xm[k] = X[k]; }
            Xp[j] += eps; Xm[j] -= eps;
            double yp[6], ym[6];
            h_coupled(Xp, yp);
            h_coupled(Xm, ym);
            for (int i = 0; i < 6; i++) {
                double fd = (yp[i] - ym[i]) / (2*eps);
                double err = fabs(fd - H[i][j]);
                if (err > worst) { worst = err; wi = i; wj = j; }
                if (err > tol) {
                    g_fail++;
                    std::printf("  FAIL trial %d H[%d][%d]: analytic=% .8f fd=% .8f\n",
                                t, i, j, H[i][j], fd);
                }
            }
        }
    }
    std::printf("  ok   200 random states, all 42 entries within %.0e (worst=%.2e at H[%d][%d])\n",
                tol, worst, wi, wj);
}

int main() {
    std::printf("B0 = [% .5f % .5f % .5f], |B0|=%.6f  DT=%.4f\n\n",
                B0[0], B0[1], B0[2],
                sqrt(B0[0]*B0[0]+B0[1]*B0[1]+B0[2]*B0[2]), DT);
    test_F();
    test_H_coupled();
    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
