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



/* State Space dimension */
#define SS_X_LEN    (7)
#define SS_Z_LEN    (6)
#define SS_U_LEN    (3)
#define SS_DT_MILIS (8)                             /* 8 ms */
#define SS_DT       float_prec(SS_DT_MILIS/1000.)   /* Sampling time */


/* Two-rate attitude EKF master switch. 1 = high-rate gyro prediction with a
 * decimated/averaged accel/mag correction (and the matching lower IMU DLPF
 * bandwidth); 0 = original single-rate 125 Hz predict+correct cycle with the
 * original IMU DLPF bandwidth. Defined here (rather than in Main.ino) so the IMU
 * driver can select a matching DLPF and the rollback path is truly the original
 * behavior. */
#ifndef FC_EKF_TWO_RATE
#define FC_EKF_TWO_RATE 1
#endif


/* Continuous (adaptive) measurement-noise scaling for the EKF correction.
 * 1 = inflate per-axis measurement variance R smoothly as a measurement deviates
 * from the gyro-propagated prediction (a "soft gate"), so a vibrating or
 * maneuvering accelerometer / disturbed magnetometer is de-weighted gradually
 * instead of being switched fully on/off (the binary gate causes a visible
 * attitude "pop" when it toggles); 0 = original hard accept/reject gate. The
 * hard outlier rejection at the gate boundary is retained in both modes.
 *
 * Defaults to FC_EKF_TWO_RATE so that the master switch keeps its one-line
 * rollback contract: setting FC_EKF_TWO_RATE 0 restores the original single-rate
 * 125 Hz predict+correct behavior (including the original binary gate) without
 * needing to know about this second macro. Define FC_EKF_ADAPTIVE_R explicitly
 * to mix the two features (e.g. single-rate with the adaptive gate) for A/B
 * testing. */
#ifndef FC_EKF_ADAPTIVE_R
#define FC_EKF_ADAPTIVE_R FC_EKF_TWO_RATE
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
