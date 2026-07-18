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
 *                 into roll & pitch. This is the flight-proven, tuned path.
 *   1           = the magnetometer feeds ONLY a tilt-compensated heading
 *                 measurement (a scalar yaw), so roll & pitch come purely from
 *                 the accelerometer + gyro and are immune to magnetic error.
 *                 This is how mainstream autopilots (PX4/ArduPilot-style AHRS)
 *                 fuse a compass. The measurement vector shrinks from
 *                 accel(3)+mag(3) to accel(3)+yaw(1); see Main.ino for the
 *                 model and tests/ekf_decouple_mag_test.cpp for the host-side
 *                 proof (yaw Jacobian vs finite difference, innovation sign,
 *                 and the decoupling property through the real EKF class).
 *
 * DISABLED after the first flight attempt went unstable ("haywire").
 * ROOT-CAUSED since, by replaying the full correction pipeline (R ramp, both
 * innovation gates, warmup, sub-stepped predicts, single precision) against a
 * simulated takeoff + coordinated-turn flight with realistic sensor noise --
 * see the flight divergence test in tests/ekf_decouple_mag_test.cpp. The
 * measurement model itself is correct; two compounding integration defects
 * caused the instability, and both are now fixed in the decoupled path:
 *
 *   1. The tilt-compensated heading was over-trusted during dynamics. Its
 *      error is ~tan(inclination) (~2.4x at this site's 67 deg) times the
 *      roll/pitch error, and is CORRELATED with the state error rather than
 *      the white noise R assumes. Fusing it at the quiet-air R_INIT_YAW while
 *      sustained acceleration (takeoff roll, banked turn) corrupted the accel
 *      -- the ONLY roll/pitch reference in this build -- closed an unstable
 *      feedback loop: tilt error -> amplified heading innovation -> tight
 *      yaw/gyro-bias corrections bleeding back into roll/pitch through the P
 *      cross-covariances. FIX: the heading measurement noise is slaved to the
 *      accel trust ratio, so the heading fades exactly when the tilt it was
 *      compensated with becomes untrustworthy (Main.ino, decoupled block).
 *   2. The innovation gates had no escape from self-lockout. Once the
 *      estimate's own error exceeded a gate, every clean sample was rejected
 *      against the broken estimate forever: the accel gate (~38 deg) locks
 *      out roll/pitch (a permanent gyro coast on corrupted bias states pinned
 *      at the clamp), and the heading gate (~34 deg) locks out yaw after
 *      coasting through a maneuver. The legacy path self-recovers from both
 *      via the 3-axis mag, which is why it never needed an escape. FIX: a
 *      sustained streak of otherwise-valid but innovation-rejected samples
 *      suspends THAT row's gate for a short re-acquire window
 *      (EKF_GATE_REACQUIRE_REJECT_STREAK); the heading's streak additionally
 *      requires the accel row to be trusted, so yaw re-acquisition waits for
 *      the tilt reference it depends on to be healthy again rather than
 *      fusing a tilt-corrupted heading mid-maneuver, plus COAST EVIDENCE (a
 *      sustained accel-untrusted stretch since the heading last fused near
 *      base trust) so a persistent compass fault in steady flight -- which
 *      produces the same reject streak without any coast -- stays gated
 *      indefinitely and yaw rides the gyro instead of adopting the fault.
 *
 * In that simulation the unfixed decoupled build diverges to ~180 deg on
 * every run and never recovers; the fixed build stays bounded through the
 * maneuvers and re-converges to a few degrees in straight flight, and with
 * FC_ACCEL_CENTRIPETAL_COMPENSATION additionally enabled it outperforms the
 * legacy fusion through turns.
 *
 * Default OFF; the legacy 3-axis path is bit-for-bit unchanged. Before
 * re-enabling for flight: bench/flight-validate the fixed path, consider
 * enabling FC_ACCEL_CENTRIPETAL_COMPENSATION once pitot/GPS/baro are trusted
 * (it restores the accel reference in exactly the turns that stress this
 * build), and expect to tune R_INIT_YAW / MAG_YAW_INNOVATION_GATE. */
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
 *   0           = the proven single-rate predict+correct cycle at 125 Hz.
 *   1 (default) = run the cheap gyro PREDICTION at EKF_PREDICT_PERIOD_US (lower
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
 * that path is bit-for-bit identical to the previous behavior. */
#ifndef FC_EKF_FAST_PREDICT
#define FC_EKF_FAST_PREDICT 1
#endif


/* Change this size based on the biggest matrix you will use */
#define MATRIX_MAXIMUM_SIZE     (7)

/* Define this to enable matrix bound checking. Off by default for flight
 * builds: the EKF matrices are fixed-size so this should never legitimately
 * fire, it costs a branch on every hot-loop matrix access, and its failure
 * path (SPEW_THE_ERROR -> while(1)) freezes the board for ~100 ms until the
 * IWDG force-resets it. Enable locally when bench-testing matrix code; the
 * host-side test in tests/ekf_decouple_mag_test.cpp defines it independently. */
// #define MATRIX_USE_BOUNDS_CHECKING

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
