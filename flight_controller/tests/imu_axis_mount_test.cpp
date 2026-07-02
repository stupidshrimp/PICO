/*************************************************************************************************************
 * Host-side guard for the IMU -> aircraft/EKF axis alignment (imu_axis_mount.h).
 *
 * The attitude EKF is a right-handed quaternion filter, so the transform that
 * rotates the raw IMU axes into the aircraft frame MUST be a proper rotation
 * (determinant +1). A determinant -1 transform is a reflection: it swaps roll and
 * pitch, offsets and reverses the reported heading, and makes the gyro prediction
 * fight the accel/mag correction (drift). The firmware used to apply a bare X/Y
 * swap, (x,y,z) -> (y,x,z), which is exactly such a reflection. This test rebuilds
 * the transform matrix from imuAxesToBody() and proves:
 *   1. determinant == +1  (a proper rotation, NOT the old reflection),
 *   2. the columns are orthonormal (it is a rotation, not a shear/scale),
 *   3. the Z (yaw) axis is preserved (a pure yaw-plane mount),
 *   4. it is a genuine 90-degree rotation in the X/Y plane,
 * for BOTH values of FC_IMU_MOUNT_YAW_ROTATION.
 *
 * Build & run (no Arduino toolchain needed):
 *   c++ -std=c++17 -I.. -O2 -o /tmp/imu_axis_mount_test imu_axis_mount_test.cpp && /tmp/imu_axis_mount_test
 ************************************************************************************************************/

/* Satisfy konfig.h's include guard with a host (PC) configuration so the firmware
 * header compiles without the Arduino toolchain (mirrors ekf_decouple_mag_test.cpp). */
#define KONFIG_H
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#define float_prec double
#include "../imu_axis_mount.h"

#include <cstdio>
#include <cmath>

static int g_fail = 0;
static void check(bool ok, const char* name) {
    if (!ok) { g_fail++; std::printf("  FAIL %s\n", name); }
    else       std::printf("  ok   %s\n", name);
}

/* Build the 3x3 transform matrix M (columns are images of the basis vectors)
 * by probing imuAxesToBody() with e_x, e_y, e_z. */
static void build(double M[3][3]) {
    double bx, by, bz;
    imuAxesToBody(1, 0, 0, bx, by, bz); M[0][0]=bx; M[1][0]=by; M[2][0]=bz;
    imuAxesToBody(0, 1, 0, bx, by, bz); M[0][1]=bx; M[1][1]=by; M[2][1]=bz;
    imuAxesToBody(0, 0, 1, bx, by, bz); M[0][2]=bx; M[1][2]=by; M[2][2]=bz;
}

static double det3(const double M[3][3]) {
    return M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
         - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
         + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
}

int main() {
    double M[3][3];
    build(M);

    std::printf("Transform matrix (FC_IMU_MOUNT_YAW_ROTATION=%d):\n", FC_IMU_MOUNT_YAW_ROTATION);
    for (int i = 0; i < 3; i++)
        std::printf("  [% .0f % .0f % .0f]\n", M[i][0], M[i][1], M[i][2]);

    /* 1. Proper rotation, not the old det=-1 reflection. */
    double d = det3(M);
    check(std::fabs(d - 1.0) < 1e-12, "determinant == +1 (proper rotation, not a reflection)");

    /* 2. Orthonormal columns: M^T M == I. */
    bool ortho = true;
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++) {
            double dot = M[0][a]*M[0][b] + M[1][a]*M[1][b] + M[2][a]*M[2][b];
            double want = (a == b) ? 1.0 : 0.0;
            if (std::fabs(dot - want) > 1e-12) ortho = false;
        }
    check(ortho, "columns are orthonormal (rotation, not shear/scale)");

    /* 3. Z (yaw) axis preserved: e_z -> e_z. */
    double bx, by, bz;
    imuAxesToBody(0, 0, 1, bx, by, bz);
    check(std::fabs(bx) < 1e-12 && std::fabs(by) < 1e-12 && std::fabs(bz - 1) < 1e-12,
          "Z axis preserved (pure yaw-plane mount)");

    /* 4. Genuine 90-degree rotation: X -> +/-Y, Y -> -/+X (no X or Y component survives on its own axis). */
    imuAxesToBody(1, 0, 0, bx, by, bz);
    bool xTo90 = std::fabs(bx) < 1e-12 && std::fabs(std::fabs(by) - 1) < 1e-12 && std::fabs(bz) < 1e-12;
    check(xTo90, "X axis rotates 90 deg into +/-Y");

    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
