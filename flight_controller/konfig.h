/*************************************************************************************************************
 * This file contains configuration parameters
 * 
 * 
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#ifndef KONFIG_H
#define KONFIG_H

#include <stdlib.h>
#include <stdint.h>
#include <math.h>



/* Decouple the magnetometer from roll & pitch.
 *   0 (default) = legacy 3-axis magnetometer fusion. The full body-frame field
 *                 is a measurement, so its Jacobian couples into every attitude
 *                 DOF and magnetic disturbances (hard/soft-iron residual, motor
 *                 current, local anomalies, a wrong inclination constant) bleed
 *                 into roll & pitch.
 *   1           = the magnetometer feeds ONLY a tilt-compensated heading
 *                 measurement (a scalar yaw), so roll & pitch come purely from
 *                 the accelerometer + gyro and are immune to magnetic error. The
 *                 measurement vector shrinks from accel(3)+mag(3) to
 *                 accel(3)+yaw(1); see Main.ino for the model and tests/
 *                 ekf_decouple_mag_test.cpp for the host-side proof.
 *
 * NOT YET BENCH-VERIFIED: the decoupled path has not been compiled with the
 * Arduino toolchain or flight-tested, and R_INIT_YAW / the heading gate still
 * need flight tuning. Default OFF; the legacy path is bit-for-bit unchanged. */
#ifndef FC_EKF_DECOUPLE_MAG
#define FC_EKF_DECOUPLE_MAG 0
#endif

/* State Space dimension */
#define SS_X_LEN    (7)
#if FC_EKF_DECOUPLE_MAG
#define SS_Z_LEN    (4)     /* accel(3) + tilt-compensated heading(1) */
#else
#define SS_Z_LEN    (6)     /* accel(3) + magnetometer(3) */
#endif
#define SS_U_LEN    (3)
#define SS_DT_MILIS (8)                             /* 8 ms */
#define SS_DT       float_prec(SS_DT_MILIS/1000.)   /* Sampling time */


/* High-rate gyro prediction for the attitude EKF.
 *   0 (default) = the proven single-rate predict+correct cycle at 125 Hz.
 *   1           = run the cheap gyro PREDICTION at EKF_PREDICT_PERIOD_US (lower
 *                 output latency and a smaller integration step) while the
 *                 noisier accel/mag CORRECTION still runs at the original 125 Hz,
 *                 on the latest sample and through the identical gates.
 *
 * Unlike the reverted two-rate change (PR #582), this deliberately does NOT
 * average accel/mag across the prediction window (averaging body-frame vectors
 * smears and shrinks them while rotating, which both lags the estimate and trips
 * the magnitude/innovation gates -> the correction gets rejected mid-rotation and
 * the filter periodically snaps back when it re-acquires). It also does NOT move
 * the correction off 125 Hz and does NOT change the IMU DLPF bandwidth, so every
 * correction-side behavior (gates, innovation-gate warmup, failure handling)
 * stays identical to the proven filter.
 *
 * Default ON. Set to 0 for a one-line rollback to the proven single-rate filter;
 * that path is bit-for-bit identical to the previous behavior.
 *
 * Before flying, confirm CPU/I2C headroom at the prediction rate (~2x the IMU
 * reads/predicts), that the IWDG watchdog stays happy, and that attitude
 * tracks correctly (no lag, no periodic snap) while rotating. */
#ifndef FC_EKF_FAST_PREDICT
#define FC_EKF_FAST_PREDICT 1
#endif


/* Change this size based on the biggest matrix you will use */
#define MATRIX_MAXIMUM_SIZE     (7)

/* Define this to enable matrix bound checking */
#define MATRIX_USE_BOUNDS_CHECKING

/* Set this define to choose math precision of the system */
#define PRECISION_SINGLE    1
#define PRECISION_DOUBLE    2
#define FPU_PRECISION       (PRECISION_SINGLE)

#if (FPU_PRECISION == PRECISION_SINGLE)
    #define float_prec          float
    #define float_prec_ZERO     (1e-7)
    #define float_prec_ZERO_ECO (1e-5)      /* 'Economical' zero, for noisy calculation where 'somewhat zero' is good enough */
#elif (FPU_PRECISION == PRECISION_DOUBLE)
    #define float_prec          double
    #define float_prec_ZERO     (1e-13)
    #define float_prec_ZERO_ECO (1e-8)      /* 'Economical' zero, for noisy calculation where 'somewhat zero' is good enough */
#else
    #error("FPU_PRECISION has not been defined!");
#endif



/* Set this define to choose system implementation (mainly used to define how you print the matrix via the Matrix::vCetak() function) */
#define SYSTEM_IMPLEMENTATION_PC                    1
#define SYSTEM_IMPLEMENTATION_EMBEDDED_CUSTOM       2
#define SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO      3

#define SYSTEM_IMPLEMENTATION                       (SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)


/* Flight-build diagnostics
 *
 * Keep verbose control-loop serial diagnostics off by default for flight builds
 * so USB/Serial formatting cannot add periodic timing jitter. Define this as 1
 * in a local build flag or bench-test configuration when investigating RC,
 * servo, or telemetry timing.
 */
#ifndef FC_CONTROL_DEBUG_SERIAL_OUTPUT
#define FC_CONTROL_DEBUG_SERIAL_OUTPUT 0
#endif



/* ASSERT is evaluated locally (without function call) to lower the computation cost */
void SPEW_THE_ERROR(char const * str);
#define ASSERT(truth, str) { if (!(truth)) SPEW_THE_ERROR(str); }


#endif // KONFIG_H
