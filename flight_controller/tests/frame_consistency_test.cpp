/*************************************************************************************************************
 * Host-side proof for the body-frame handedness fix.
 *
 * The firmware used to map IMU axes into the fusion frame with a bare X<->Y
 * swap S = [[0,1,0],[1,0,0],[0,0,1]] -- det(S) = -1, a REFLECTION no physical
 * mounting can produce. Angular velocity is a pseudovector: under a reflection
 * it needs the opposite sign from the accel/mag polar vectors, so the gyro
 * prediction rotated the attitude estimate in the wrong sense and the accel/
 * mag correction dragged it back every step (roll/pitch swapped in the dynamic
 * response, heading mirrored, bias states polluted -> offset + drift). The fix
 * keeps the X<->Y swap and negates Z on ALL THREE sensors:
 * C = [[0,1,0],[1,0,0],[0,0,-1]], det(C) = +1, with the earth references moved
 * to the Z-down convention (IMU_ACC_Z0 = -1, B0_z = +sin(INCL)) and the Euler
 * output stage adjusted (roll sign flip absorbed by the frame, pitch negated,
 * yaw unchanged).
 *
 * This test proves the two properties the fix must have:
 *   A. OUTPUT EQUIVALENCE: for every attitude, the new pipeline's static
 *      (roll, pitch, yaw) output is numerically identical to the old
 *      pipeline's converged static output -- the behavior the operator has
 *      verified as correct. Display/control/telemetry conventions unchanged.
 *   B. DYNAMIC CONSISTENCY: through the REAL firmware EKF (matrix.h+ekf.cpp,
 *      exact f/F/h/H and tuning), the new mapping tracks a tumbling truth with
 *      near-zero innovation, while the old bare-swap mapping shows the
 *      persistent prediction/correction fight and a mirrored heading response.
 *
 * Build & run:
 *   c++ -std=c++17 -I.. -O2 -o /tmp/frame_test frame_consistency_test.cpp && /tmp/frame_test
 ************************************************************************************************************/

/* ---- Host (PC) konfig stub, legacy 6-row measurement vector. ---- */
#define KONFIG_H
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define SS_X_LEN    (7)
#define SS_Z_LEN    (6)   /* accel(3) + 3-axis mag: the model that was flying */
#define SS_U_LEN    (3)
#define MATRIX_MAXIMUM_SIZE     (7)
#define MATRIX_USE_BOUNDS_CHECKING

#define PRECISION_SINGLE    1
#define PRECISION_DOUBLE    2
#define FPU_PRECISION       (PRECISION_DOUBLE)
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
#include "../attitude_init.h"

#include <cstdio>
#include <cmath>

void SPEW_THE_ERROR(char const* str) { std::printf("MATRIX ASSERT: %s\n", str); std::abort(); }

/* ---------------- shared constants & helpers ---------------- */
static const double DECL = -0.05640509;
static const double INCL =  1.17209583;
static const double DT   = 0.008;

/* legacy (z-up) and new (z-down) earth references */
static const double B0_ZUP[3] = { cos(INCL)*cos(DECL), cos(INCL)*sin(DECL), -sin(INCL) };
static const double B0_NED[3] = { cos(INCL)*cos(DECL), cos(INCL)*sin(DECL),  sin(INCL) };
/* physical earth field consistent with "the current config reads correctly":
 * at the old pipeline's zero attitude the swapped mag reading equals B0_ZUP. */
static const double B_E[3] = { B0_ZUP[1], B0_ZUP[0], B0_ZUP[2] };   /* = S * B0_ZUP */

static const double S_MAP[3][3] = {{0,1,0},{1,0,0},{0,0, 1}};   /* det -1, as flown */
static const double C_MAP[3][3] = {{0,1,0},{1,0,0},{0,0,-1}};   /* det +1, the fix */

static void matVec(const double M[3][3], const double v[3], double out[3]) {
    for (int i = 0; i < 3; i++) out[i] = M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2];
}

static void MofQ(const double q[4], double M[3][3]) {
    const double q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    M[0][0]=q0*q0+q1*q1-q2*q2-q3*q3; M[0][1]=2*(q1*q2+q0*q3);         M[0][2]=2*(q1*q3-q0*q2);
    M[1][0]=2*(q1*q2-q0*q3);         M[1][1]=q0*q0-q1*q1+q2*q2-q3*q3; M[1][2]=2*(q2*q3+q0*q1);
    M[2][0]=2*(q1*q3+q0*q2);         M[2][1]=2*(q2*q3-q0*q1);         M[2][2]=q0*q0-q1*q1-q2*q2+q3*q3;
}

static void qdot(const double q[4], const double w[3], double out[4]) {
    const double p=w[0], qq=w[1], r=w[2];
    out[0] = 0.5*(-p*q[1] - qq*q[2] - r*q[3]);
    out[1] = 0.5*( p*q[0] +  r*q[2] - qq*q[3]);
    out[2] = 0.5*(qq*q[0] -  r*q[1] +  p*q[3]);
    out[3] = 0.5*( r*q[0] + qq*q[1] -  p*q[2]);
}

static void qnormalize(double q[4]) {
    double n = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    for (int i = 0; i < 4; i++) q[i] /= n;
}

static double wrapPi(double a) {
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

/* Output stages. OLD: the roll-sign hack the firmware used to apply.
 * NEW: hack removed (the frame's Z negation supplies it), pitch negated. */
static void eulerOld(const double q[4], double e[3]) {
    e[0] = -atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
    e[1] = asin(fmax(-1.0, fmin(1.0, 2*(q[0]*q[2]-q[3]*q[1]))));
    e[2] = atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
}
static void eulerNew(const double q[4], double e[3]) {
    e[0] = atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
    e[1] = asin(fmax(-1.0, fmin(1.0, 2*(q[3]*q[1]-q[0]*q[2]))));
    e[2] = atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
}

/* Static pipeline solutions via the flight TRIAD (attitude_init.h). */
static bool staticSolution(const double R[3][3], const double map[3][3],
                           const double accRefZ, const double B0ref[3], double qout[4]) {
    double e3[3] = {0,0,1}, a_e[3], m_e[3];
    matVec(R, e3, a_e);
    double a_b[3], m_b[3];
    matVec(map, a_e, a_b);
    matVec(R, B_E, m_e);
    matVec(map, m_e, m_b);
    const float acc[3]  = {(float)a_b[0], (float)a_b[1], (float)a_b[2]};
    const float mag[3]  = {(float)m_b[0], (float)m_b[1], (float)m_b[2]};
    const float aref[3] = {0.f, 0.f, (float)accRefZ};
    const float mref[3] = {(float)B0ref[0], (float)B0ref[1], (float)B0ref[2]};
    float qf[4];
    if (!bTriadAttitudeInit(acc, mag, aref, mref, qf)) return false;
    for (int i = 0; i < 4; i++) qout[i] = qf[i];
    return true;
}

static int g_fail = 0;
static unsigned g_seed = 1357;
static double frand(double lo, double hi) {
    g_seed = g_seed*1103515245u + 12345u;
    return lo + (hi - lo) * (((g_seed >> 16) & 0x7fff) / 32767.0);
}

/* ================= Test A: static output equivalence ================= */
static void test_static_equivalence() {
    std::printf("[test_static_equivalence] old vs new pipeline outputs, random attitudes\n");
    double worst = 0.0;
    int tested = 0;
    for (int t = 0; t < 4000; t++) {
        double q[4] = { frand(-1,1), frand(-1,1), frand(-1,1), frand(-1,1) };
        double n = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        if (n < 0.1) continue;
        for (int i = 0; i < 4; i++) q[i] /= n;
        double R[3][3];
        MofQ(q, R);   /* truth: earth -> IMU frame */

        double qOld[4], qNew[4];
        if (!staticSolution(R, S_MAP, +1.0, B0_ZUP, qOld)) continue;
        if (!staticSolution(R, C_MAP, -1.0, B0_NED, qNew)) continue;
        double eo[3], en[3];
        eulerOld(qOld, eo);
        eulerNew(qNew, en);
        if (fabs(sin(eo[1])) > 0.98) continue;   /* skip gimbal-lock ill-conditioning */
        ++tested;
        for (int i = 0; i < 3; i++) {
            double d = fabs(wrapPi(eo[i] - en[i]));
            if (d > worst) worst = d;
            if (d > 3.5e-3) {   /* ~0.2 deg: two float32 TRIAD pipelines back-to-back */
                g_fail++;
                std::printf("  FAIL trial %d axis %d: old=%.5f new=%.5f rad\n", t, i, eo[i], en[i]);
            }
        }
    }
    std::printf("  ok   %d attitudes, worst |output difference| = %.2e rad (%.4f deg)\n",
                tested, worst, worst*180/M_PI);
}

/* ================= Test B: dynamic consistency through the real EKF ================= */

/* firmware model callbacks (exact replicas of Main.ino's, dt = DT) */
static double gZ0 = -1.0;
static const double* gB0 = B0_NED;

static bool cbX(Matrix& Xn, const Matrix& X, const Matrix& U) {
    double q[4] = {X[0][0], X[1][0], X[2][0], X[3][0]};
    double w[3] = {U[0][0]-X[4][0], U[1][0]-X[5][0], U[2][0]-X[6][0]};
    double qd[4];
    qdot(q, w, qd);
    double qn[4] = {q[0]+qd[0]*DT, q[1]+qd[1]*DT, q[2]+qd[2]*DT, q[3]+qd[3]*DT};
    qnormalize(qn);
    for (int i = 0; i < 4; i++) Xn[i][0] = qn[i];
    for (int i = 4; i < 7; i++) Xn[i][0] = X[i][0];
    return true;
}
static bool cbY(Matrix& Y, const Matrix& X, const Matrix& /*U*/) {
    double q[4] = {X[0][0], X[1][0], X[2][0], X[3][0]};
    double M[3][3];
    MofQ(q, M);
    double aref[3] = {0, 0, gZ0};
    double a[3], m[3];
    matVec(M, aref, a);
    matVec(M, gB0, m);
    for (int i = 0; i < 3; i++) { Y[i][0] = a[i]; Y[3+i][0] = m[i]; }
    return true;
}
static bool cbF(Matrix& F, const Matrix& X, const Matrix& U) {
    double q0=X[0][0], q1=X[1][0], q2=X[2][0], q3=X[3][0];
    double p=U[0][0]-X[4][0], q=U[1][0]-X[5][0], r=U[2][0]-X[6][0];
    F.vSetToZero();
    F[0][0]=1;              F[0][1]=-0.5*p*DT; F[0][2]=-0.5*q*DT; F[0][3]=-0.5*r*DT;
    F[1][0]=0.5*p*DT;       F[1][1]=1;         F[1][2]= 0.5*r*DT; F[1][3]=-0.5*q*DT;
    F[2][0]=0.5*q*DT;       F[2][1]=-0.5*r*DT; F[2][2]=1;         F[2][3]= 0.5*p*DT;
    F[3][0]=0.5*r*DT;       F[3][1]= 0.5*q*DT; F[3][2]=-0.5*p*DT; F[3][3]=1;
    F[0][4]= 0.5*q1*DT; F[1][4]=-0.5*q0*DT; F[2][4]=-0.5*q3*DT; F[3][4]= 0.5*q2*DT;
    F[0][5]= 0.5*q2*DT; F[1][5]= 0.5*q3*DT; F[2][5]=-0.5*q0*DT; F[3][5]=-0.5*q1*DT;
    F[0][6]= 0.5*q3*DT; F[1][6]=-0.5*q2*DT; F[2][6]= 0.5*q1*DT; F[3][6]=-0.5*q0*DT;
    F[4][4]=1; F[5][5]=1; F[6][6]=1;
    return true;
}
static bool cbH(Matrix& H, const Matrix& X, const Matrix& /*U*/) {
    double q0=X[0][0], q1=X[1][0], q2=X[2][0], q3=X[3][0];
    H.vSetToZero();
    H[0][0]=-2*q2*gZ0; H[0][1]= 2*q3*gZ0; H[0][2]=-2*q0*gZ0; H[0][3]= 2*q1*gZ0;
    H[1][0]= 2*q1*gZ0; H[1][1]= 2*q0*gZ0; H[1][2]= 2*q3*gZ0; H[1][3]= 2*q2*gZ0;
    H[2][0]= 2*q0*gZ0; H[2][1]=-2*q1*gZ0; H[2][2]=-2*q2*gZ0; H[2][3]= 2*q3*gZ0;
    const double Bx=gB0[0], By=gB0[1], Bz=gB0[2];
    H[3][0]= 2*q0*Bx+2*q3*By-2*q2*Bz; H[3][1]= 2*q1*Bx+2*q2*By+2*q3*Bz;
    H[3][2]=-2*q2*Bx+2*q1*By-2*q0*Bz; H[3][3]=-2*q3*Bx+2*q0*By+2*q1*Bz;
    H[4][0]=-2*q3*Bx+2*q0*By+2*q1*Bz; H[4][1]= 2*q2*Bx-2*q1*By+2*q0*Bz;
    H[4][2]= 2*q1*Bx+2*q2*By+2*q3*Bz; H[4][3]=-2*q0*Bx-2*q3*By+2*q2*Bz;
    H[5][0]= 2*q2*Bx-2*q1*By+2*q0*Bz; H[5][1]= 2*q3*Bx-2*q0*By-2*q1*Bz;
    H[5][2]= 2*q0*Bx+2*q3*By-2*q2*Bz; H[5][3]= 2*q1*Bx+2*q2*By+2*q3*Bz;
    return true;
}
static bool cbNorm(Matrix& X) {
    double n = sqrt(X[0][0]*X[0][0]+X[1][0]*X[1][0]+X[2][0]*X[2][0]+X[3][0]*X[3][0]);
    if (n < 1e-12) return false;
    for (int i = 0; i < 4; i++) X[i][0] /= n;
    return true;
}

struct RunStats { double meanInnovTumble; double maxOutErrTumble[3]; };

/* Simulate truth (rest 1.5 s, tumble 4 s, rest 1 s) and run the real EKF with
 * the given sensor mapping and references. Output error is measured against
 * the certified static map (the old pipeline's converged output at the
 * instantaneous truth attitude -- what the operator sees as correct). */
static RunStats run_dynamic(const double map[3][3], double Z0, const double B0ref[3],
                            void (*eulerOut)(const double[4], double[3])) {
    gZ0 = Z0;
    gB0 = B0ref;

    Matrix Xi(SS_X_LEN, 1); Xi.vSetToZero(); Xi[0][0] = 1.0;
    Matrix P(SS_X_LEN, SS_X_LEN); P.vSetToZero();
    for (int i = 0; i < 4; i++) P[i][i] = 10.0;
    for (int i = 4; i < 7; i++) P[i][i] = 0.02;
    Matrix Q(SS_X_LEN, SS_X_LEN); Q.vSetToZero();
    for (int i = 0; i < 4; i++) Q[i][i] = 1e-6;
    for (int i = 4; i < 7; i++) Q[i][i] = 1e-8;
    Matrix R(SS_Z_LEN, SS_Z_LEN); R.vSetToZero();
    for (int i = 0; i < 6; i++) R[i][i] = 0.00015;
    EKF filter(Xi, P, Q, R, cbX, cbY, cbF, cbH, cbNorm);

    double qt[4] = {1, 0, 0, 0};
    const int steps = (int)(6.5/DT);
    RunStats st = {0, {0, 0, 0}};
    int innovN = 0;
    Matrix Y(SS_Z_LEN, 1), U(SS_U_LEN, 1);
    for (int k = 0; k < steps; k++) {
        const double t = k*DT;
        const bool tumbling = (t >= 1.5 && t < 5.5);
        double w[3] = {0, 0, 0};
        if (tumbling) { w[0] = 12*M_PI/180; w[1] = 8*M_PI/180; w[2] = -10*M_PI/180; }

        /* sensors at the current truth */
        double Rt[3][3];
        MofQ(qt, Rt);
        double e3[3] = {0,0,1}, a_e[3], m_e[3], a_b[3], m_b[3], w_b[3];
        matVec(Rt, e3, a_e);
        matVec(Rt, B_E, m_e);
        matVec(map, a_e, a_b);
        matVec(map, m_e, m_b);
        matVec(map, w, w_b);
        for (int i = 0; i < 3; i++) { Y[i][0] = a_b[i]; Y[3+i][0] = m_b[i]; U[i][0] = w_b[i]; }

        bool ok = filter.bUpdate(Y, U);
        if (!ok) { g_fail++; std::printf("  FAIL bUpdate returned false at k=%d\n", k); return st; }

        if (tumbling && t > 2.0) {
            Matrix E = filter.GetErr();
            double in = 0;
            for (int i = 0; i < 6; i++) in += E[i][0]*E[i][0];
            st.meanInnovTumble += sqrt(in);
            ++innovN;
            /* certified reference output at this instant */
            double qRef[4], eRef[3], eEst[3];
            if (staticSolution(Rt, S_MAP, +1.0, B0_ZUP, qRef) && fabs(2*(qRef[0]*qRef[2]-qRef[3]*qRef[1])) < 0.98) {
                eulerOld(qRef, eRef);
                Matrix Xf = filter.GetX();
                double qe[4] = {Xf[0][0], Xf[1][0], Xf[2][0], Xf[3][0]};
                eulerOut(qe, eEst);
                for (int i = 0; i < 3; i++) {
                    double d = fabs(wrapPi(eEst[i] - eRef[i]));
                    if (d > st.maxOutErrTumble[i]) st.maxOutErrTumble[i] = d;
                }
            }
        }

        /* propagate truth (10 substeps for accuracy) */
        for (int s = 0; s < 10; s++) {
            double qd[4];
            qdot(qt, w, qd);
            for (int i = 0; i < 4; i++) qt[i] += qd[i]*(DT/10);
            qnormalize(qt);
        }
    }
    if (innovN > 0) st.meanInnovTumble /= innovN;
    return st;
}

static void test_dynamic_consistency() {
    std::printf("[test_dynamic_consistency] real EKF, tumbling truth (12,8,-10 deg/s)\n");
    RunStats neu = run_dynamic(C_MAP, -1.0, B0_NED, eulerNew);
    RunStats old = run_dynamic(S_MAP, +1.0, B0_ZUP, eulerOld);
    const double r2d = 180/M_PI;
    std::printf("  info NEW map: mean|innov|=%.5f  max out err r/p/y = %.3f/%.3f/%.3f deg\n",
                neu.meanInnovTumble, neu.maxOutErrTumble[0]*r2d, neu.maxOutErrTumble[1]*r2d, neu.maxOutErrTumble[2]*r2d);
    std::printf("  info OLD map: mean|innov|=%.5f  max out err r/p/y = %.3f/%.3f/%.3f deg\n",
                old.meanInnovTumble, old.maxOutErrTumble[0]*r2d, old.maxOutErrTumble[1]*r2d, old.maxOutErrTumble[2]*r2d);

    if (!(neu.meanInnovTumble < 1e-3)) {
        g_fail++; std::printf("  FAIL new map: innovation did not vanish (fight still present)\n");
    } else std::printf("  ok   new map: prediction agrees with correction (innovation ~ 0)\n");
    double neuWorst = fmax(neu.maxOutErrTumble[0], fmax(neu.maxOutErrTumble[1], neu.maxOutErrTumble[2]));
    if (!(neuWorst < 0.5*M_PI/180)) {
        g_fail++; std::printf("  FAIL new map: output deviates from the certified attitude mid-rotation\n");
    } else std::printf("  ok   new map: output tracks the certified attitude through the tumble\n");
    if (!(old.meanInnovTumble > 20*neu.meanInnovTumble)) {
        g_fail++; std::printf("  FAIL expected the old map to show the prediction/correction fight\n");
    } else std::printf("  ok   old map shows the fight (innovation %.0fx larger), documenting the bug\n",
                       old.meanInnovTumble / neu.meanInnovTumble);
}

int main() {
    std::printf("frame maps: det(S)=-1 (old, reflection)  det(C)=+1 (new, rotation)\n\n");
    test_static_equivalence();
    test_dynamic_consistency();
    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
