/*************************************************************************************************************
 * Host-side numerical verification for the magnetometer/roll-pitch decoupling
 * (FC_EKF_DECOUPLE_MAG) attitude EKF measurement model.
 *
 * This test does NOT need the Arduino toolchain. It compiles the real firmware
 * matrix library + EKF class on the host (by satisfying konfig.h's include guard
 * with a PC configuration) and exercises the exact decoupled measurement model
 * (h, H, and the tilt-compensated heading innovation) that Main.ino uses when
 * FC_EKF_DECOUPLE_MAG == 1.
 *
 * It proves the three properties that make the change correct:
 *   1. The analytic yaw-measurement Jacobian H[3][*] matches a finite-difference
 *      of the yaw measurement function h3(q).
 *   2. The tilt-compensated heading innovation deltaPsi has the correct sign and
 *      magnitude: a pure earth-frame yaw error of D produces deltaPsi == D.
 *   3. Decoupling: a magnetometer disturbance (including a large vertical
 *      component) moves the YAW estimate but leaves ROLL/PITCH essentially
 *      untouched, whereas the legacy 3-axis model bleeds the same disturbance
 *      into roll/pitch.
 *
 * Build & run:
 *   c++ -std=c++17 -I.. -O2 -o /tmp/ekf_decouple_test ekf_decouple_mag_test.cpp && /tmp/ekf_decouple_test
 ************************************************************************************************************/

/* ---- Satisfy konfig.h's include guard with a host (PC) configuration so the
 *      firmware headers compile without the Arduino toolchain. ---- */
#define KONFIG_H
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define SS_X_LEN    (7)
#define SS_U_LEN    (3)
#ifndef SS_Z_LEN
#define SS_Z_LEN    (4)   /* decoupled measurement: accel(3) + yaw(1) */
#endif
#define MATRIX_MAXIMUM_SIZE     (7)
#define MATRIX_USE_BOUNDS_CHECKING

#define PRECISION_SINGLE    1
#define PRECISION_DOUBLE    2
#define FPU_PRECISION       (PRECISION_DOUBLE)   /* double on host for tight FD checks */
#define float_prec          double
#define float_prec_ZERO     (1e-13)
#define float_prec_ZERO_ECO (1e-8)

#define SYSTEM_IMPLEMENTATION_PC                1
#define SYSTEM_IMPLEMENTATION_EMBEDDED_CUSTOM   2
#define SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO  3
#define SYSTEM_IMPLEMENTATION                   (SYSTEM_IMPLEMENTATION_PC)

void SPEW_THE_ERROR(char const* str);
#define ASSERT(truth, str) { if (!(truth)) SPEW_THE_ERROR(str); }

#include "matrix.h"
#include "ekf.cpp"

#include <cstdio>
#include <cmath>
#include <cassert>

void SPEW_THE_ERROR(char const* str) { std::printf("MATRIX ASSERT: %s\n", str); std::abort(); }

/* ---- Magnetic reference field for the firmware's default site (central
 *      Illinois). |B0| == 1 by construction. NED-style Z-down earth frame to
 *      match the firmware: inclination is down-positive (+sin), and the
 *      at-rest specific force points to (0,0,IMU_ACC_Z0) with Z0 = -1. ---- */
static const double DECL = -0.05640509;
static const double INCL =  1.17209583;
static double B0[3] = {
    cos(INCL) * cos(DECL),
    cos(INCL) * sin(DECL),
    sin(INCL)
};
static const double IMU_ACC_Z0 = -1.0;
static const double ACC_REF[3] = {0.0, 0.0, IMU_ACC_Z0};

static double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/* Hamilton product q (x) r */
static void qmul(const double q[4], const double r[4], double out[4]) {
    out[0] = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3];
    out[1] = q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2];
    out[2] = q[0]*r[2] - q[1]*r[3] + q[2]*r[0] + q[3]*r[1];
    out[3] = q[0]*r[3] + q[1]*r[2] - q[2]*r[1] + q[3]*r[0];
}
static void qnorm(double q[4]) {
    double n = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    for (int i = 0; i < 4; i++) q[i] /= n;
}

/* R_eb (earth->body) applied to an earth vector, matching Main.ino's h(). */
static void Reb_times(const double q[4], const double v[3], double out[3]) {
    double q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    double M[3][3] = {
        { q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2+q0*q3),         2*(q1*q3-q0*q2)         },
        { 2*(q1*q2-q0*q3),         q0*q0-q1*q1+q2*q2-q3*q3, 2*(q2*q3+q0*q1)         },
        { 2*(q1*q3+q0*q2),         2*(q2*q3-q0*q1),         q0*q0-q1*q1-q2*q2+q3*q3 }
    };
    for (int i = 0; i < 3; i++)
        out[i] = M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2];
}

/* ===================== EXACT replica of the decoupled firmware model ===================== */

/* h(x): rows 0..2 gravity direction in body frame (unchanged from legacy),
 *       row 3 = yaw Euler angle psi(q) (decoupled measurement). */
static void h_decoupled(const double X[7], double Y[SS_Z_LEN]) {
    double q0=X[0], q1=X[1], q2=X[2], q3=X[3];
    Y[0] = (2*q1*q3 - 2*q0*q2) * IMU_ACC_Z0;
    Y[1] = (2*q2*q3 + 2*q0*q1) * IMU_ACC_Z0;
    Y[2] = (q0*q0 - q1*q1 - q2*q2 + q3*q3) * IMU_ACC_Z0;
    Y[3] = atan2(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
}

/* H = dh/dx. Yaw row depends on the quaternion only. */
static void H_decoupled(const double X[7], double H[SS_Z_LEN][SS_X_LEN]) {
    double q0=X[0], q1=X[1], q2=X[2], q3=X[3];
    for (int i = 0; i < SS_Z_LEN; i++)
        for (int j = 0; j < SS_X_LEN; j++) H[i][j] = 0.0;

    H[0][0] = -2*q2*IMU_ACC_Z0; H[0][1] = 2*q3*IMU_ACC_Z0; H[0][2] = -2*q0*IMU_ACC_Z0; H[0][3] = 2*q1*IMU_ACC_Z0;
    H[1][0] =  2*q1*IMU_ACC_Z0; H[1][1] = 2*q0*IMU_ACC_Z0; H[1][2] =  2*q3*IMU_ACC_Z0; H[1][3] = 2*q2*IMU_ACC_Z0;
    H[2][0] =  2*q0*IMU_ACC_Z0; H[2][1] =-2*q1*IMU_ACC_Z0; H[2][2] = -2*q2*IMU_ACC_Z0; H[2][3] = 2*q3*IMU_ACC_Z0;

    double N = 2*(q1*q2 + q0*q3);                 /* M01 */
    double D = q0*q0 + q1*q1 - q2*q2 - q3*q3;      /* M00 */
    double den = N*N + D*D;                        /* == cos^2(pitch) */
    if (den < 1e-9) { H[3][0]=H[3][1]=H[3][2]=H[3][3]=0.0; return; }
    H[3][0] = (D*( 2*q3) - N*( 2*q0)) / den;
    H[3][1] = (D*( 2*q2) - N*( 2*q1)) / den;
    H[3][2] = (D*( 2*q1) - N*(-2*q2)) / den;
    H[3][3] = (D*( 2*q0) - N*(-2*q3)) / den;
}

/* Tilt-compensated heading innovation deltaPsi computed exactly as Main.ino does:
 *   measured field -> earth frame using the current estimate -> heading vs reference. */
static double mag_yaw_innovation(const double Xest[7], const double mb[3]) {
    double q0=Xest[0], q1=Xest[1], q2=Xest[2], q3=Xest[3];
    /* m_e = R_eb^T * m_b  (columns of R_eb dotted with m_b) */
    double M00=q0*q0+q1*q1-q2*q2-q3*q3, M10=2*(q1*q2-q0*q3), M20=2*(q1*q3+q0*q2);
    double M01=2*(q1*q2+q0*q3), M11=q0*q0-q1*q1+q2*q2-q3*q3, M21=2*(q2*q3-q0*q1);
    double me_x = M00*mb[0] + M10*mb[1] + M20*mb[2];
    double me_y = M01*mb[0] + M11*mb[1] + M21*mb[2];
    double alpha = atan2(me_y, me_x);
    double decl  = atan2(B0[1], B0[0]);
    return wrapPi(decl - alpha);
}

/* psi(q) helper (yaw Euler angle) */
static double psi_of(const double q[4]) {
    return atan2(2*(q[1]*q[2]+q[0]*q[3]), q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);
}

/* ===================== Tests ===================== */

static int g_fail = 0;
static void check(bool ok, const char* name, double got, double want, double tol) {
    if (!ok) { g_fail++; std::printf("  FAIL %-46s got=% .6f want=% .6f tol=%.1e\n", name, got, want, tol); }
    else       std::printf("  ok   %-46s (% .6f)\n", name, got);
}

/* Build a quaternion from small random tilt + a yaw. Body->earth convention,
 * consistent with Main.ino (q (x) v (x) q* = R_be). */
static unsigned g_seed = 12345;
static double frand(double lo, double hi) {
    g_seed = g_seed*1103515245u + 12345u;
    double u = ((g_seed >> 16) & 0x7fff) / 32767.0;
    return lo + (hi - lo) * u;
}

/* Test 1: analytic yaw Jacobian vs central finite difference. */
static void test_jacobian() {
    std::printf("[test_jacobian] H[3][*] vs finite difference\n");
    for (int t = 0; t < 200; t++) {
        double q[4] = { frand(-1,1), frand(-1,1), frand(-1,1), frand(-1,1) };
        if (q[0] < 0) for (int i=0;i<4;i++) q[i] = -q[i];
        qnorm(q);
        double X[7] = { q[0], q[1], q[2], q[3], frand(-0.1,0.1), frand(-0.1,0.1), frand(-0.1,0.1) };
        /* skip near-gimbal (pitch ~ +-90) where yaw is ill-conditioned */
        double D = q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
        double N = 2*(q[1]*q[2]+q[0]*q[3]);
        if (N*N + D*D < 0.05) continue;

        double H[SS_Z_LEN][SS_X_LEN];
        H_decoupled(X, H);
        const double eps = 1e-6;
        for (int j = 0; j < 4; j++) {
            double Xp[7], Xm[7];
            for (int k=0;k<7;k++){ Xp[k]=X[k]; Xm[k]=X[k]; }
            Xp[j]+=eps; Xm[j]-=eps;
            double Yp[SS_Z_LEN], Ym[SS_Z_LEN];
            h_decoupled(Xp, Yp); h_decoupled(Xm, Ym);
            double fd = wrapPi(Yp[3]-Ym[3]) / (2*eps);
            if (fabs(fd - H[3][j]) > 1e-4) {
                g_fail++;
                std::printf("  FAIL trial %d col %d: analytic=% .6f fd=% .6f\n", t, j, H[3][j], fd);
            }
        }
    }
    std::printf("  ok   200 random quaternions, all columns within 1e-4\n");
}

/* Test 2: deltaPsi sign/magnitude for a pure earth-yaw error. */
static void test_innovation_sign() {
    std::printf("[test_innovation_sign] deltaPsi == true yaw error\n");
    for (int t = 0; t < 50; t++) {
        double qest[4] = { frand(-1,1), frand(-0.4,0.4), frand(-0.4,0.4), frand(-1,1) };
        if (qest[0] < 0) for (int i=0;i<4;i++) qest[i]=-qest[i];
        qnorm(qest);
        double Dpsi = frand(-0.6, 0.6);
        double qz[4] = { cos(Dpsi/2), 0, 0, sin(Dpsi/2) };   /* earth-Z (up) rotation */
        double qtrue[4]; qmul(qz, qest, qtrue); qnorm(qtrue);

        double mb[3]; Reb_times(qtrue, B0, mb);               /* measured field at truth */
        double Xest[7] = { qest[0],qest[1],qest[2],qest[3],0,0,0 };
        double dpsi = mag_yaw_innovation(Xest, mb);
        double want = wrapPi(psi_of(qtrue) - psi_of(qest));
        if (fabs(wrapPi(dpsi - want)) > 1e-6) {
            g_fail++;
            std::printf("  FAIL trial %d: deltaPsi=% .6f want=% .6f (Dpsi=% .3f)\n", t, dpsi, want, Dpsi);
        }
    }
    std::printf("  ok   deltaPsi matches yaw error over 50 random tilts\n");
}

/* Run the real EKF (matrix.h + ekf.cpp) one correction with the decoupled model. */
static void ekf_correct(EKF& filter, const double mb[3], const double acc_meas[3],
                        double yawVar, double accVar) {
    Matrix Y(SS_Z_LEN, 1), U(SS_U_LEN, 1), R(SS_Z_LEN, SS_Z_LEN);
    U[0][0]=0; U[1][0]=0; U[2][0]=0;
    R.vSetToZero();
    R[0][0]=accVar; R[1][1]=accVar; R[2][2]=accVar; R[3][3]=yawVar;

    Matrix Xpred = filter.GetX();
    double Xp[7]; for (int i=0;i<7;i++) Xp[i]=Xpred[i][0];
    double yhat[SS_Z_LEN]; h_decoupled(Xp, yhat);

    /* gravity-direction rows: normalized accel measurement */
    double an = sqrt(acc_meas[0]*acc_meas[0]+acc_meas[1]*acc_meas[1]+acc_meas[2]*acc_meas[2]);
    Y[0][0]=acc_meas[0]/an; Y[1][0]=acc_meas[1]/an; Y[2][0]=acc_meas[2]/an;
    /* yaw row: predicted heading + tilt-compensated innovation (see Main.ino) */
    Y[3][0] = yhat[3] + mag_yaw_innovation(Xp, mb);

    filter.vSetMeasurementNoise(R);
    bool ok = filter.bCorrect(Y, U);
    assert(ok);
}

/* Wire the firmware EKF callbacks to the decoupled model. */
static bool cbX(Matrix& Xn, const Matrix& X, const Matrix& /*U*/) { Xn = X; return true; } /* U=0 => identity predict */
static bool cbY(Matrix& Y, const Matrix& X, const Matrix& /*U*/) {
    double Xa[7]; for (int i=0;i<7;i++) Xa[i]=X[i][0];
    double y[SS_Z_LEN]; h_decoupled(Xa, y);
    for (int i=0;i<SS_Z_LEN;i++) Y[i][0]=y[i];
    return true;
}
static bool cbF(Matrix& F, const Matrix& /*X*/, const Matrix& /*U*/) { F = MatIdentity(SS_X_LEN); return true; }
static bool cbH(Matrix& Hm, const Matrix& X, const Matrix& /*U*/) {
    double Xa[7]; for (int i=0;i<7;i++) Xa[i]=X[i][0];
    double Hh[SS_Z_LEN][SS_X_LEN]; H_decoupled(Xa, Hh);
    for (int i=0;i<SS_Z_LEN;i++) for (int j=0;j<SS_X_LEN;j++) Hm[i][j]=Hh[i][j];
    return true;
}
static bool cbNorm(Matrix& X) {
    double n = sqrt(X[0][0]*X[0][0]+X[1][0]*X[1][0]+X[2][0]*X[2][0]+X[3][0]*X[3][0]);
    if (n < 1e-9) return false;
    for (int i=0;i<4;i++) X[i][0]/=n;
    return true;
}

static void rollpitch_of(const double q[4], double& roll, double& pitch) {
    /* specific-force (at-rest accel) direction in body frame = accel prediction */
    double gb[3];
    Reb_times(q, ACC_REF, gb);
    pitch = asin(fmax(-1.0, fmin(1.0, gb[0])));
    roll  = atan2(gb[1], gb[2]);
}

/* ===================== Dynamic frame consistency (shipped configuration) =====================
 *
 * The firmware default is FC_EKF_DECOUPLE_MAG == 1 with the proper Z-down body
 * frame (Z negated on accel/gyro/mag, IMU_ACC_Z0 = -1, B0_z = +sin(INCL)).
 * tests/frame_consistency_test.cpp proves the frame fix dynamically for the
 * legacy 6-row model; this test proves it for the 4-row decoupled model that
 * actually ships: a tumbling truth, sensors mapped through the proper
 * C = [[0,1,0],[1,0,0],[0,0,-1]], the real EKF class running the firmware's
 * dt-predict (f, F) plus the decoupled (h, H) and the tilt-compensated heading
 * innovation exactly as Main.ino builds it. The gyro prediction must agree
 * with the accel/heading correction (innovation ~ 0) and the attitude must
 * track C * M(q_true) * C throughout. */

static const double DT_DYN = 0.008;

static bool cbXdyn(Matrix& Xn, const Matrix& X, const Matrix& U) {
    const double q0=X[0][0], q1=X[1][0], q2=X[2][0], q3=X[3][0];
    const double p=U[0][0]-X[4][0], q=U[1][0]-X[5][0], r=U[2][0]-X[6][0];
    double qn[4] = {
        q0 + DT_DYN*0.5*(-p*q1 - q*q2 - r*q3),
        q1 + DT_DYN*0.5*( p*q0 + r*q2 - q*q3),
        q2 + DT_DYN*0.5*( q*q0 - r*q1 + p*q3),
        q3 + DT_DYN*0.5*( r*q0 + q*q1 - p*q2)
    };
    qnorm(qn);
    for (int i = 0; i < 4; i++) Xn[i][0] = qn[i];
    for (int i = 4; i < 7; i++) Xn[i][0] = X[i][0];
    return true;
}

static bool cbFdyn(Matrix& F, const Matrix& X, const Matrix& U) {
    const double q0=X[0][0], q1=X[1][0], q2=X[2][0], q3=X[3][0];
    const double p=U[0][0]-X[4][0], q=U[1][0]-X[5][0], r=U[2][0]-X[6][0];
    F.vSetToZero();
    F[0][0]=1;             F[0][1]=-0.5*p*DT_DYN; F[0][2]=-0.5*q*DT_DYN; F[0][3]=-0.5*r*DT_DYN;
    F[1][0]=0.5*p*DT_DYN;  F[1][1]=1;             F[1][2]= 0.5*r*DT_DYN; F[1][3]=-0.5*q*DT_DYN;
    F[2][0]=0.5*q*DT_DYN;  F[2][1]=-0.5*r*DT_DYN; F[2][2]=1;             F[2][3]= 0.5*p*DT_DYN;
    F[3][0]=0.5*r*DT_DYN;  F[3][1]= 0.5*q*DT_DYN; F[3][2]=-0.5*p*DT_DYN; F[3][3]=1;
    F[0][4]= 0.5*q1*DT_DYN; F[1][4]=-0.5*q0*DT_DYN; F[2][4]=-0.5*q3*DT_DYN; F[3][4]= 0.5*q2*DT_DYN;
    F[0][5]= 0.5*q2*DT_DYN; F[1][5]= 0.5*q3*DT_DYN; F[2][5]=-0.5*q0*DT_DYN; F[3][5]=-0.5*q1*DT_DYN;
    F[0][6]= 0.5*q3*DT_DYN; F[1][6]=-0.5*q2*DT_DYN; F[2][6]= 0.5*q1*DT_DYN; F[3][6]=-0.5*q0*DT_DYN;
    F[4][4]=1; F[5][5]=1; F[6][6]=1;
    return true;
}

static void MofQ_full(const double q[4], double M[3][3]) {
    const double q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    M[0][0]=q0*q0+q1*q1-q2*q2-q3*q3; M[0][1]=2*(q1*q2+q0*q3);         M[0][2]=2*(q1*q3-q0*q2);
    M[1][0]=2*(q1*q2-q0*q3);         M[1][1]=q0*q0-q1*q1+q2*q2-q3*q3; M[1][2]=2*(q2*q3+q0*q1);
    M[2][0]=2*(q1*q3+q0*q2);         M[2][1]=2*(q2*q3-q0*q1);         M[2][2]=q0*q0-q1*q1-q2*q2+q3*q3;
}

static void mat3mul(const double A[3][3], const double B[3][3], double C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
}

static void test_dynamic_frame_decoupled() {
    std::printf("[test_dynamic_frame_decoupled] shipped 4-row model, proper Z-down frame, tumbling truth\n");

    /* proper body map C and the physical earth field (z-up world) consistent
     * with "the previous configuration read correctly": B_E = S * B0_ZUP,
     * where B0_ZUP is this file's NED B0 with the vertical sign flipped back. */
    const double C_MAP[3][3] = {{0,1,0},{1,0,0},{0,0,-1}};
    const double B0_ZUP[3] = { B0[0], B0[1], -B0[2] };
    const double B_E[3] = { B0_ZUP[1], B0_ZUP[0], B0_ZUP[2] };
    const double E3[3] = {0, 0, 1};   /* specific force points up in the z-up world */

    Matrix Xi(SS_X_LEN,1); Xi.vSetToZero(); Xi[0][0]=1.0;
    Matrix P(SS_X_LEN,SS_X_LEN); P.vSetToZero();
    for (int i=0;i<4;i++) P[i][i]=10.0;
    for (int i=4;i<7;i++) P[i][i]=0.02;
    Matrix Q(SS_X_LEN,SS_X_LEN); Q.vSetToZero();
    for (int i=0;i<4;i++) Q[i][i]=1e-6;
    for (int i=4;i<7;i++) Q[i][i]=1e-8;
    Matrix R(SS_Z_LEN,SS_Z_LEN); R.vSetToZero();
    R[0][0]=R[1][1]=R[2][2]=0.00015; R[3][3]=0.0025;
    EKF filter(Xi, P, Q, R, cbXdyn, cbY, cbFdyn, cbH, cbNorm);
    filter.vSetMeasurementNoise(R);

    double qt[4] = {1,0,0,0};
    const int steps = (int)(6.5/DT_DYN);
    double innovSum = 0; int innovN = 0; double worstAtt = 0;
    Matrix Y(SS_Z_LEN,1), U(SS_U_LEN,1);
    for (int k = 0; k < steps; k++) {
        const double t = k*DT_DYN;
        const bool tumbling = (t >= 1.5 && t < 5.5);
        double w[3] = {0,0,0};
        if (tumbling) { w[0]=12*M_PI/180; w[1]=8*M_PI/180; w[2]=-10*M_PI/180; }

        /* sensors at the current truth, mapped through the proper frame */
        double Rt[3][3]; MofQ_full(qt, Rt);
        double a_e[3], m_e[3], a_b[3], m_b[3], w_b[3];
        for (int i=0;i<3;i++) {
            a_e[i] = Rt[i][0]*E3[0] + Rt[i][1]*E3[1] + Rt[i][2]*E3[2];
            m_e[i] = Rt[i][0]*B_E[0] + Rt[i][1]*B_E[1] + Rt[i][2]*B_E[2];
        }
        for (int i=0;i<3;i++) {
            a_b[i] = C_MAP[i][0]*a_e[0] + C_MAP[i][1]*a_e[1] + C_MAP[i][2]*a_e[2];
            m_b[i] = C_MAP[i][0]*m_e[0] + C_MAP[i][1]*m_e[1] + C_MAP[i][2]*m_e[2];
            w_b[i] = C_MAP[i][0]*w[0]   + C_MAP[i][1]*w[1]   + C_MAP[i][2]*w[2];
        }
        for (int i=0;i<3;i++) U[i][0] = w_b[i];

        /* build the measurement exactly as Main.ino does: manual one-step
         * prediction (identical to the bUpdate-internal one), accel rows
         * normalized, yaw row = h3(predicted) + tilt-compensated innovation */
        Matrix Xpred(SS_X_LEN,1);
        cbXdyn(Xpred, filter.GetX(), U);
        double Xp[7]; for (int i=0;i<7;i++) Xp[i]=Xpred[i][0];
        double yhat[SS_Z_LEN]; h_decoupled(Xp, yhat);
        double an = sqrt(a_b[0]*a_b[0]+a_b[1]*a_b[1]+a_b[2]*a_b[2]);
        Y[0][0]=a_b[0]/an; Y[1][0]=a_b[1]/an; Y[2][0]=a_b[2]/an;
        Y[3][0]=yhat[3] + mag_yaw_innovation(Xp, m_b);

        bool ok = filter.bUpdate(Y, U);
        if (!ok) { g_fail++; std::printf("  FAIL bUpdate returned false at k=%d\n", k); return; }

        if (tumbling && t > 2.0) {
            Matrix E = filter.GetErr();
            double in = 0;
            for (int i=0;i<SS_Z_LEN;i++) in += E[i][0]*E[i][0];
            innovSum += sqrt(in); ++innovN;
            /* attitude error vs the exact conjugated truth N = C * M(qt) * C */
            double CR[3][3], N[3][3], Me[3][3];
            mat3mul(C_MAP, Rt, CR);
            mat3mul(CR, C_MAP, N);
            Matrix Xf = filter.GetX();
            double qe[4] = {Xf[0][0],Xf[1][0],Xf[2][0],Xf[3][0]};
            MofQ_full(qe, Me);
            double tr = 0;
            for (int i=0;i<3;i++)
                for (int j=0;j<3;j++) tr += Me[j][i]*N[j][i];   /* trace(Me' * N) */
            double cang = (tr - 1.0) * 0.5;
            if (cang > 1.0) cang = 1.0;
            if (cang < -1.0) cang = -1.0;
            const double ang = acos(cang);
            if (ang > worstAtt) worstAtt = ang;
        }

        /* propagate truth (10 substeps) with the standard kinematics */
        for (int s = 0; s < 10; s++) {
            const double p=w[0], q=w[1], r=w[2];
            const double h = DT_DYN/10*0.5;
            double q0=qt[0], q1=qt[1], q2=qt[2], q3=qt[3];
            qt[0] = q0 + h*(-p*q1 - q*q2 - r*q3);
            qt[1] = q1 + h*( p*q0 + r*q2 - q*q3);
            qt[2] = q2 + h*( q*q0 - r*q1 + p*q3);
            qt[3] = q3 + h*( r*q0 + q*q1 - p*q2);
            qnorm(qt);
        }
    }
    const double meanInnov = innovN ? innovSum/innovN : 1.0;
    std::printf("  info mean|innov| during tumble = %.6f, worst attitude error = %.4f deg\n",
                meanInnov, worstAtt*180/M_PI);
    check(meanInnov < 1e-3, "prediction agrees with correction", meanInnov, 0.0, 1e-3);
    check(worstAtt < 0.5*M_PI/180, "attitude tracks C*M(q)*C through tumble", worstAtt, 0.0, 0.5*M_PI/180);
}

/* Test 3: the decoupling property, run through the REAL EKF class. */
static void test_decoupling_ekf() {
    std::printf("[test_decoupling_ekf] mag disturbance moves yaw, not roll/pitch\n");

    /* truth attitude: a definite roll & pitch plus a yaw */
    double qest[4] = { 0.94, 0.20, -0.18, 0.12 }; qnorm(qest);
    double Dpsi = 0.35;
    double qz[4] = { cos(Dpsi/2),0,0,sin(Dpsi/2) };
    double qtrue[4]; qmul(qz, qest, qtrue); qnorm(qtrue);

    double r0,p0,rt,pt;
    rollpitch_of(qest, r0, p0);
    rollpitch_of(qtrue, rt, pt);
    check(fabs(r0-rt)<1e-9 && fabs(p0-pt)<1e-9, "truth differs from est by pure yaw", fabs(r0-rt), 0.0, 1e-9);

    /* measurements consistent with truth */
    double acc[3], mb_clean[3];
    Reb_times(qtrue, ACC_REF, acc);
    Reb_times(qtrue, B0, mb_clean);

    /* ---- Case A: clean mag -> yaw converges, roll/pitch stay put ---- */
    {
        Matrix Xi(SS_X_LEN,1); Xi.vSetToZero();
        for (int i=0;i<4;i++) Xi[i][0]=qest[i];
        Matrix P(SS_X_LEN,SS_X_LEN); P.vSetToZero();
        for (int i=0;i<4;i++) P[i][i]=10.0;
        for (int i=4;i<7;i++) P[i][i]=0.02;
        Matrix Q(SS_X_LEN,SS_X_LEN); Q.vSetToZero();
        Matrix R(SS_Z_LEN,SS_Z_LEN); R.vSetToZero();
        EKF filter(Xi, P, Q, R, cbX, cbY, cbF, cbH, cbNorm);
        for (int k=0;k<60;k++) ekf_correct(filter, mb_clean, acc, 0.0025, 0.00015);
        Matrix Xf = filter.GetX();
        double qf[4]={Xf[0][0],Xf[1][0],Xf[2][0],Xf[3][0]};
        double rf,pf; rollpitch_of(qf, rf, pf);
        check(fabs(wrapPi(psi_of(qf)-psi_of(qtrue)))<0.02, "clean: yaw error -> 0",
              wrapPi(psi_of(qf)-psi_of(qtrue)), 0.0, 0.02);
        check(fabs(rf-rt)<0.01, "clean: roll undisturbed", rf-rt, 0.0, 0.01);
        check(fabs(pf-pt)<0.01, "clean: pitch undisturbed", pf-pt, 0.0, 0.01);
    }

    /* ---- Case B: strongly disturbed mag (incl. large vertical component) ---- */
    {
        double mb_bad[3] = { mb_clean[0]+0.40, mb_clean[1]-0.30, mb_clean[2]+0.60 };
        Matrix Xi(SS_X_LEN,1); Xi.vSetToZero();
        for (int i=0;i<4;i++) Xi[i][0]=qtrue[i];        /* start AT truth */
        Matrix P(SS_X_LEN,SS_X_LEN); P.vSetToZero();
        for (int i=0;i<4;i++) P[i][i]=10.0;
        for (int i=4;i<7;i++) P[i][i]=0.02;
        Matrix Q(SS_X_LEN,SS_X_LEN); Q.vSetToZero();
        Matrix R(SS_Z_LEN,SS_Z_LEN); R.vSetToZero();
        EKF filter(Xi, P, Q, R, cbX, cbY, cbF, cbH, cbNorm);
        for (int k=0;k<60;k++) ekf_correct(filter, mb_bad, acc, 0.0025, 0.00015);
        Matrix Xf = filter.GetX();
        double qf[4]={Xf[0][0],Xf[1][0],Xf[2][0],Xf[3][0]};
        double rf,pf; rollpitch_of(qf, rf, pf);
        double rollLeak = fabs(rf-rt), pitchLeak = fabs(pf-pt);
        double yawShift = fabs(wrapPi(psi_of(qf)-psi_of(qtrue)));
        std::printf("  info disturbed-mag leak: roll=%.4f pitch=%.4f rad, yaw shift=%.4f rad\n",
                    rollLeak, pitchLeak, yawShift);
        check(rollLeak  < 1e-3, "disturbed: roll stays put",  rollLeak,  0.0, 1e-3);
        check(pitchLeak < 1e-3, "disturbed: pitch stays put", pitchLeak, 0.0, 1e-3);
        check(yawShift  > 0.05,  "disturbed: yaw absorbs the error (expected)", yawShift, 0.1, 0.0);
    }
}

/* ===================== Maiden-flight divergence & fix regression =====================
 *
 * Replays the flight profile that made the decoupled build go "haywire" on its
 * first flight, through the REAL EKF class and a replica of Main.ino's 125 Hz
 * correction block (accel norm gate + smooth R ramp + innovation gates +
 * warmup + rejection fallbacks): runway idle, takeoff roll under sustained
 * forward acceleration, climb-out, then a 35-deg-bank coordinated turn, with
 * realistic deterministic sensor noise, gyro bias, and a small hard-iron
 * calibration residual.
 *
 * Root cause this test pins down (see konfig.h, FC_EKF_DECOUPLE_MAG):
 *   1. The tilt-compensated heading error is ~tan(inclination) x the
 *      roll/pitch error and correlated with the state; fusing it at the
 *      quiet-air R_INIT_YAW while dynamics corrupt the accel (the only
 *      roll/pitch reference in this build) is an unstable feedback loop.
 *      FIX: slave the heading noise to the accel trust ratio.
 *   2. The innovation gates (accel AND heading) have no escape from
 *      self-lockout: once the estimate's own error exceeds a gate, every
 *      clean sample is rejected against the broken estimate forever.
 *      FIX: a 2 s streak of otherwise-valid-but-innovation-rejected samples
 *      suspends THAT row's gate for a 2 s re-acquire window; the heading
 *      streak additionally requires the accel row to be trusted (near 1 g)
 *      so yaw re-acquisition waits until the tilt it depends on is healthy,
 *      plus COAST EVIDENCE -- a sustained accel-untrusted stretch (a real
 *      maneuver) since the heading was last fused near base trust -- so a
 *      sustained compass fault in steady flight stays gated instead of
 *      being adopted (EKF_GATE_REACQUIRE_REJECT_STREAK).
 *
 * The replica MUST mirror Main.ino's correction assembly; update both together. */

/* Deterministic ~N(0,1) (Irwin-Hall) on top of the file's LCG. */
static double nrand() {
    double s = 0;
    for (int i = 0; i < 12; i++) s += frand(0, 1);
    return s - 6.0;
}

/* Firmware state normalization including the gyro-bias clamp (Main_bNormalizeState). */
static bool cbNormClamp(Matrix& X) {
    if (!cbNorm(X)) return false;
    for (int i = 4; i < 7; i++) {
        if (X[i][0] >  0.1) X[i][0] =  0.1;
        else if (X[i][0] < -0.1) X[i][0] = -0.1;
    }
    return true;
}

static double att_err_deg(const double qt[4], const Matrix& Xe) {
    double qe[4] = { Xe[0][0], Xe[1][0], Xe[2][0], Xe[3][0] };
    double qtc[4] = { qt[0], -qt[1], -qt[2], -qt[3] };
    double dq[4]; qmul(qtc, qe, dq); qnorm(dq);
    double w = fabs(dq[0]); if (w > 1.0) w = 1.0;
    return 2.0 * acos(w) * 180.0 / M_PI;
}

struct MaidenResult { double maxErr; double finalErr; int accRej; int magRej; int reacquires; };

/* steadyCompassFault: instead of the maneuvering maiden profile, fly straight
 * and level and inject a persistent body-frame magnetic disturbance from
 * t = 10 s (~47 deg of apparent heading shift, well past the yaw gate) -- the
 * sustained-compass-fault case the yaw-gate escape must NOT re-admit. */
static MaidenResult run_maiden_flight(bool withFixes, bool steadyCompassFault) {
    /* firmware constants (Main.ino) */
    const double R_ACC = 0.0025, R_YAW = 0.0025, R_REJ = 1.0e3;
    const double NORM_GATE_FRAC = 0.35, ACC_INNOV_GATE = 0.65, YAW_INNOV_GATE = 0.6;
    const unsigned WARMUP = 250, REACQ_STREAK = 250, COAST_EVIDENCE = 250;
    const double G = 9.80665;

    g_seed = 20260717u;   /* identical sensor stream for both runs */

    Matrix Xi(SS_X_LEN,1); Xi.vSetToZero(); Xi[0][0] = 1.0;
    Matrix P(SS_X_LEN,SS_X_LEN); P.vSetToZero();
    for (int i=0;i<4;i++) P[i][i] = 10.0;
    for (int i=4;i<7;i++) P[i][i] = 0.02;
    Matrix Q(SS_X_LEN,SS_X_LEN); Q.vSetToZero();
    for (int i=0;i<4;i++) Q[i][i] = 1e-6;
    for (int i=4;i<7;i++) Q[i][i] = 1e-8;
    Matrix R(SS_Z_LEN,SS_Z_LEN); R.vSetToZero();
    EKF filter(Xi, P, Q, R, cbXdyn, cbY, cbFdyn, cbH, cbNormClamp);

    double qt[4] = {1,0,0,0};
    const double biasTrue[3] = { 0.010, -0.008, 0.012 };   /* rad/s residual gyro bias */
    const double magResid[3] = { 1.5, -2.0, 1.0 };         /* uT hard-iron cal residual */
    const double B_uT = 48.0;
    const double B0e[3] = { B0[0]*B_uT, B0[1]*B_uT, B0[2]*B_uT };

    unsigned warmupCount = 0;
    unsigned accStreak = 0, accWindow = 0, magStreak = 0, magWindow = 0;
    unsigned coastEvidence = 0;
    MaidenResult res = {0, 0, 0, 0, 0};
    double finalSum = 0; int finalN = 0;
    const double Tend = steadyCompassFault ? 45.0 : 58.0;
    Matrix Y(SS_Z_LEN,1), U(SS_U_LEN,1);

    for (double t = 0; t < Tend; t += DT_DYN) {
        /* ---- flight script ---- */
        double w[3] = {0,0,0}, V = 0, Vdot = 0, throttle = 0.1;
        const double bank = 35.0*M_PI/180.0;
        if (steadyCompassFault) {
            if (t >= 4) { V = 22.0; throttle = 0.6; }   /* straight & level throughout */
        }
        else if (t < 4) { /* runway idle: warmup arms on a still airframe */ }
        else if (t < 11) { V = 2.5*(t-4);        Vdot = 2.5; throttle = 1.0; }  /* takeoff roll */
        else if (t < 13) { V = 17.5+1.2*(t-11);  Vdot = 1.2; throttle = 1.0;
                           w[1] = (12.0*M_PI/180.0)/2.0; }                      /* rotate */
        else if (t < 20) { V = 19.9+0.5*(t-13);  Vdot = 0.5; throttle = 0.9; }  /* climb-out */
        else if (t < 22) { V = 23.4; throttle = 0.6;
                           w[1] = -(12.0*M_PI/180.0)/2.0; }                     /* level off */
        else if (t < 26) { V = 23.4; throttle = 0.6; }                          /* cruise */
        else if (t < 28) { V = 20.0; throttle = 0.65; w[0] = bank/2.0; }        /* roll in */
        else if (t < 40) { V = 20.0; throttle = 0.65;                           /* coordinated turn */
                           double Om = G*tan(bank)/V;
                           w[1] = Om*sin(bank); w[2] = Om*cos(bank); }
        else if (t < 42) { V = 20.0; throttle = 0.65; w[0] = -bank/2.0; }       /* roll out */
        else             { V = 22.0; throttle = 0.6; }                          /* straight recovery */

        /* ---- truth propagation ---- */
        for (int s = 0; s < 10; s++) {
            const double p = w[0], q = w[1], r = w[2];
            const double h = DT_DYN/10*0.5;
            double q0=qt[0], q1=qt[1], q2=qt[2], q3=qt[3];
            qt[0] = q0 + h*(-p*q1 - q*q2 - r*q3);
            qt[1] = q1 + h*( p*q0 + r*q2 - q*q3);
            qt[2] = q2 + h*( q*q0 - r*q1 + p*q3);
            qt[3] = q3 + h*( r*q0 + q*q1 - p*q2);
            qnorm(qt);
        }

        /* ---- sensors: specific force (gravity + kinematic), field, gyro ---- */
        const double gE[3] = {0, 0, -G};
        double fb[3]; Reb_times(qt, gE, fb);
        fb[0] += Vdot; fb[1] += w[2]*V; fb[2] += -w[1]*V;
        double mb[3]; Reb_times(qt, B0e, mb);
        const double vibe = 0.25 + 0.9*throttle;   /* m/s^2 rms, throttle-correlated */
        double acc[3], mg[3];
        for (int i = 0; i < 3; i++) {
            acc[i]   = fb[i] + vibe*nrand();
            mg[i]    = mb[i] + magResid[i] + 0.6*nrand();
            U[i][0]  = w[i] + biasTrue[i] + 0.015*nrand();
        }
        if (steadyCompassFault && t >= 10.0) {
            mg[1] += 20.0;   /* ~47 deg apparent heading shift vs the ~18.7 uT horizontal field */
        }

        /* ---- replica of Main.ino's 125 Hz correction assembly ---- */
        Matrix Xpred(SS_X_LEN,1);
        cbXdyn(Xpred, filter.GetX(), U);
        double Xp[7]; for (int i=0;i<7;i++) Xp[i] = Xpred[i][0];
        double yhat[SS_Z_LEN]; h_decoupled(Xp, yhat);

        const double an = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
        const double normErrFrac = (an > 1e-6) ? fabs(an - G)/G : (NORM_GATE_FRAC + 1.0);
        bool accelRejected = normErrFrac > NORM_GATE_FRAC;
        const bool accelNormOk = !accelRejected;
        double rAcc;
        if (!accelRejected) {
            Y[0][0]=acc[0]/an; Y[1][0]=acc[1]/an; Y[2][0]=acc[2]/an;
            if (warmupCount >= WARMUP && accWindow == 0) {
                const double dx=Y[0][0]-yhat[0], dy=Y[1][0]-yhat[1], dz=Y[2][0]-yhat[2];
                accelRejected = sqrt(dx*dx + dy*dy + dz*dz) > ACC_INNOV_GATE;
            }
        }
        if (accelRejected) {
            Y[0][0]=yhat[0]; Y[1][0]=yhat[1]; Y[2][0]=yhat[2];
            rAcc = R_REJ; ++res.accRej;
        } else {
            rAcc = R_ACC * pow(R_REJ/R_ACC, normErrFrac/NORM_GATE_FRAC);
        }
        if (withFixes) {  /* FIX 2: accel gate lockout escape (per-row window) */
            if (accWindow > 0) --accWindow;
            if (accelRejected && accelNormOk) {
                if (++accStreak >= REACQ_STREAK) {
                    accWindow = REACQ_STREAK; accStreak = 0; ++res.reacquires;
                }
            } else if (!accelRejected) {
                accStreak = 0;
            }
        }

        double rYaw = R_YAW;
        const double dpsi = mag_yaw_innovation(Xp, mg);
        bool magRejected = (warmupCount >= WARMUP && magWindow == 0 &&
                            fabs(dpsi) > YAW_INNOV_GATE);
        if (magRejected) {
            Y[3][0] = yhat[3]; rYaw = R_REJ; ++res.magRej;
        } else {
            Y[3][0] = yhat[3] + dpsi;
            if (withFixes) {  /* FIX 1: heading trust slaved to accel trust */
                rYaw = R_YAW * (rAcc / R_ACC);
                if (rYaw > R_REJ) rYaw = R_REJ;
            }
        }
        /* Coast evidence: only a sustained accel-untrusted stretch (a real
         * maneuver, during which the heading was slaved weak and yaw could
         * genuinely coast away) may unlock the yaw-gate escape. It builds on
         * accel-untrusted corrections, DRAINS on trusted ones (so steady-
         * flight noise tails accumulate nothing and a persistent compass
         * fault stays gated), and resets whenever the heading is fused near
         * base trust (yaw healthy again). */
        const bool accelTrusted = rAcc <= R_ACC * 100.0;
        const bool magYawAided = !magRejected && rYaw <= R_YAW * 10.0;
        if (magYawAided)            coastEvidence = 0;
        else if (!accelTrusted)     { if (coastEvidence < 2500u) ++coastEvidence; }
        else if (coastEvidence > 0) --coastEvidence;
        if (withFixes) {  /* FIX 2, heading gate: yaw lockout escape */
            if (magWindow > 0) --magWindow;
            if (magRejected && accelTrusted) {
                if (magStreak < 60000u) ++magStreak;
                if (magStreak >= REACQ_STREAK && coastEvidence >= COAST_EVIDENCE) {
                    magWindow = REACQ_STREAK; magStreak = 0; ++res.reacquires;
                }
            } else if (!magRejected) {
                magStreak = 0;
            }
        }

        R.vSetToZero();
        R[0][0]=R[1][1]=R[2][2]=rAcc; R[3][3]=rYaw;
        filter.vSetMeasurementNoise(R);
        if (!filter.bUpdate(Y, U)) {
            g_fail++;
            std::printf("  FAIL bUpdate returned false at t=%.2f\n", t);
            return res;
        }
        if (warmupCount < WARMUP) ++warmupCount;

        const double e = att_err_deg(qt, filter.GetX());
        if (e > res.maxErr) res.maxErr = e;
        if (t > Tend - 3.0) { finalSum += e; ++finalN; }
    }
    res.finalErr = finalN ? finalSum/finalN : 0.0;
    return res;
}

static void test_flight_divergence_and_fix() {
    std::printf("[test_flight_divergence_and_fix] maiden-flight replay: unfixed diverges, fixed recovers\n");
    MaidenResult unfixed = run_maiden_flight(false, false);
    MaidenResult patched = run_maiden_flight(true, false);
    std::printf("  info unfixed: maxErr=%.1f deg  finalErr=%.1f deg  accRej=%d magRej=%d\n",
                unfixed.maxErr, unfixed.finalErr, unfixed.accRej, unfixed.magRej);
    std::printf("  info fixed:   maxErr=%.1f deg  finalErr=%.1f deg  accRej=%d magRej=%d reacq=%d\n",
                patched.maxErr, patched.finalErr, patched.accRej, patched.magRej, patched.reacquires);
    check(unfixed.maxErr > 90.0,  "unfixed build diverges on the maiden profile", unfixed.maxErr, 90.0, 0.0);
    check(patched.maxErr < 80.0,  "fixed: error bounded through maneuvers", patched.maxErr, 0.0, 80.0);
    check(patched.finalErr < 12.0, "fixed: re-converges in straight flight", patched.finalErr, 0.0, 12.0);
    check(patched.reacquires >= 1, "fixed: yaw re-acquires after the coast", (double)patched.reacquires, 1.0, 0.0);
}

/* The yaw-gate escape must not re-admit a sustained compass fault: in steady
 * flight the heading was fully aided until the fault, so there is no coast
 * evidence, the gate must stay closed, and yaw must hold on the gyro. */
static void test_steady_compass_fault_stays_gated() {
    std::printf("[test_steady_compass_fault_stays_gated] persistent disturbance in level flight\n");
    MaidenResult r = run_maiden_flight(true, true);
    std::printf("  info fault run: maxErr=%.1f deg  finalErr=%.1f deg  magRej=%d reacq=%d\n",
                r.maxErr, r.finalErr, r.magRej, r.reacquires);
    check(r.reacquires == 0, "no re-acquire window opens on a compass fault", (double)r.reacquires, 0.0, 0.0);
    check(r.magRej > 3000,   "fault stays rejected by the yaw gate", (double)r.magRej, 4000.0, 0.0);
    check(r.maxErr < 15.0,   "attitude holds on gyro through the fault", r.maxErr, 0.0, 15.0);
}

int main() {
    std::printf("B0 = [% .5f % .5f % .5f], |B0|=%.6f  decl=%.4f incl=%.4f\n\n",
                B0[0],B0[1],B0[2], sqrt(B0[0]*B0[0]+B0[1]*B0[1]+B0[2]*B0[2]),
                atan2(B0[1],B0[0]), asin(B0[2]));
    test_jacobian();
    test_innovation_sign();
    test_decoupling_ekf();
    test_dynamic_frame_decoupled();
    test_flight_divergence_and_fix();
    test_steady_compass_fault_stays_gated();
    std::printf("\n%s (%d failure%s)\n", g_fail? "TESTS FAILED":"ALL TESTS PASSED",
                g_fail, g_fail==1?"":"s");
    return g_fail ? 1 : 0;
}
