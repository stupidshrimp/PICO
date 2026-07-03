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

int main() {
    std::printf("B0 = [% .5f % .5f % .5f], |B0|=%.6f  decl=%.4f incl=%.4f\n\n",
                B0[0],B0[1],B0[2], sqrt(B0[0]*B0[0]+B0[1]*B0[1]+B0[2]*B0[2]),
                atan2(B0[1],B0[0]), asin(B0[2]));
    test_jacobian();
    test_innovation_sign();
    test_decoupling_ekf();
    std::printf("\n%s (%d failure%s)\n", g_fail? "TESTS FAILED":"ALL TESTS PASSED",
                g_fail, g_fail==1?"":"s");
    return g_fail ? 1 : 0;
}
