
/*************************************************************************************************************
 *  
 * Feather Flight Program
 * 
 * This sketch reads data from an MPU9250, MS5611, and MS4525D0 sensors while updating GPS data from an
 * M8N module. IMU/EKF work and attitude telemetry cache updates run at ~125 Hz, while GPS
 * telemetry, GPS UART draining, barometer, and airspeed work run on independent lower-rate timers.
 * The printed output includes roll, pitch, yaw, altitude (ft), airspeed (mph), longitude, latitude, and
 * EKF computation time.
 * 
 ************************************************************************************************************/

#include <Wire.h>
#include <elapsedMillis.h>
#include <IWatchdog.h>

// Arduino IDE sketch-local debug toggles must be defined before including
// konfig.h because konfig.h supplies guarded defaults. Keep this bench-build
// value enabled so FCDBG lines are emitted to the USB Serial Monitor. Set this
// to 0 before flight if the extra serial formatting jitter is not acceptable.
#ifndef FC_CONTROL_DEBUG_SERIAL_OUTPUT
#define FC_CONTROL_DEBUG_SERIAL_OUTPUT 1
#endif

#include "konfig.h"
#include "matrix.h"
#include "ekf.h"
#include "simple_mpu9250.h"
#include <Arduino.h>
#include <Servo.h>
#include "ms5611.h" 
#include "ms4525d0.h" 
#include "m8n.h"
#include "control_mode.h"
#include <CRSFforArduino.hpp>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Define additional hardware serial ports if the core does not provide them.
// These mappings correspond to the STM32F405 feather board where
// USART3 is on PB11 (RX) / PB10 (TX) and USART6 is on PC7 (RX) / PC6 (TX).
// The core already defines Serial3 when the underlying hardware exposes USART3;
// avoid redefining it to prevent link errors.
#if !defined(USART3)
HardwareSerial Serial3(PB11, PB10);
#endif

// Serial port mapping for the STM32F405 Feather flight controller:
//   USART3 -> CRSF receiver/telemetry on PB11 (RX) / PB10 (TX)
//   USART6 -> M8N GPS               on PC7  (RX) / PC6  (TX)
//
// IMPORTANT: `USART3`/`USART6` are CMSIS peripheral macros that are ALWAYS
// defined on the STM32F405 (the chip physically has those peripherals),
// regardless of which Arduino HardwareSerial objects exist or which pins they
// use. The previous `#if !defined(USART6)` guard was therefore always false, so
// the explicit PC7/PC6 pin mapping was silently compiled out and the GPS UART
// fell back to whatever the core's default `Serial6` pins are -- which are NOT
// PC7/PC6 on this board, so the M8N transmitted into pins nothing was reading
// and the GPS appeared dead.
//
// CRSF keeps using the core-provided `Serial3` (its default USART3 pins already
// match PB11/PB10 here). The GPS gets a dedicated, uniquely named HardwareSerial
// constructed with explicit pins so it is ALWAYS bound to USART6 on PC7/PC6,
// independent of the core's defaults and with no risk of clashing with a
// core-provided `Serial6`.
HardwareSerial gpsSerial(PC7, PC6);  // RX = PC7, TX = PC6 (USART6)

// ----- IMU & EKF Variables -----
#define IMU_ACC_Z0  (1)
// Default magnetic reference for central Illinois (~40.0 N, 89.0 W),
// evaluated for 2026-06-11. Override these at build time for a specific
// flying site; declination is positive east and inclination is positive down.
#ifndef FC_MAG_DECLINATION_RAD
#define FC_MAG_DECLINATION_RAD (-0.05640509f)  // -3.2318 deg
#endif
#ifndef FC_MAG_INCLINATION_RAD
#define FC_MAG_INCLINATION_RAD (1.17209583f)   // +67.1561 deg
#endif
// Set to 1 for a bench-only magnetometer calibration run. The helper blocks
// normal flight startup, asks the user to rotate the fully assembled aircraft,
// then prints hard-iron and diagonal soft-iron constants to the debug serial
// port for copying into the defaults below.
#ifndef FC_MAG_CALIBRATION_MODE
#define FC_MAG_CALIBRATION_MODE 0
#endif
#ifndef FC_MAG_CALIBRATION_DURATION_MS
#define FC_MAG_CALIBRATION_DURATION_MS 60000UL
#endif
#ifndef FC_MAG_CALIBRATION_START_DELAY_MS
#define FC_MAG_CALIBRATION_START_DELAY_MS 10000UL
#endif
#ifndef FC_MAG_CALIBRATION_SAMPLE_PERIOD_MS
#define FC_MAG_CALIBRATION_SAMPLE_PERIOD_MS 20UL
#endif
#ifndef FC_MAG_CALIBRATION_MIN_AXIS_SPAN_UT
#define FC_MAG_CALIBRATION_MIN_AXIS_SPAN_UT 20.0f
#endif

// Set to 1 for a bench-only GPS wiring/parser diagnostic run. This helper
// blocks normal flight startup after USART6 is initialized, pings the M8N,
// echoes raw GPS traffic, and prints parsed latitude/longitude from GGA/RMC.
#ifndef FC_GPS_DIAGNOSTIC_MODE
#define FC_GPS_DIAGNOSTIC_MODE 0
#endif
#ifndef FC_GPS_DIAGNOSTIC_BAUD
#define FC_GPS_DIAGNOSTIC_BAUD 9600UL
#endif
#ifndef FC_GPS_DIAGNOSTIC_PING_PERIOD_MS
#define FC_GPS_DIAGNOSTIC_PING_PERIOD_MS 5000UL
#endif
#ifndef FC_GPS_DIAGNOSTIC_STATUS_PERIOD_MS
#define FC_GPS_DIAGNOSTIC_STATUS_PERIOD_MS 1000UL
#endif
#ifndef FC_GPS_DIAGNOSTIC_NO_DATA_WARNING_MS
#define FC_GPS_DIAGNOSTIC_NO_DATA_WARNING_MS 3000UL
#endif
float_prec IMU_MAG_B0_data[3] = {
  cos(FC_MAG_INCLINATION_RAD)*cos(FC_MAG_DECLINATION_RAD),
  cos(FC_MAG_INCLINATION_RAD)*sin(FC_MAG_DECLINATION_RAD),
  -sin(FC_MAG_INCLINATION_RAD)  // Convert geomagnetic down-positive inclination to EKF +Z-up/specific-force convention.
};
Matrix IMU_MAG_B0(3, 1, IMU_MAG_B0_data);
float_prec HARD_IRON_BIAS_data[3] = {
  -33.941257f, -10.753434f, -2.073374f
};
Matrix HARD_IRON_BIAS(3, 1, HARD_IRON_BIAS_data);
float_prec SOFT_IRON_MATRIX_data[9] = {
  0.803654f, 0.0f, 0.0f,
  0.0f, 0.953951f, 0.0f,
  0.0f, 0.0f, 1.413603f
};
Matrix SOFT_IRON_MATRIX(3, 3, SOFT_IRON_MATRIX_data);

// EKF initialization constants and matrices (values defined in konfig.h)
#define P_INIT_QUAT      (10.)
#define P_INIT_GYRO_BIAS (0.02)
#define Q_INIT_QUAT      (1e-6)
#define Q_INIT_GYRO_BIAS (1e-8)
#define R_INIT_ACC       (0.0015/10.)
#define R_INIT_MAG       (0.0015/10.)
#define R_REJECTED       (1.0e3)
#define GRAVITY_NOMINAL_MSS (9.80665f)
#define ACCEL_NORM_GATE_FRACTION (0.35f)
// Reject normalized vector measurements whose direction disagrees with the
// gyro-propagated attitude by more than these Euclidean innovation gates.
// For unit vectors, 0.65 is roughly a 38-degree direction error and 0.55 is
// roughly a 32-degree direction error.  This prevents transient acceleration
// and magnetic disturbances from pulling the attitude solution toward a
// physically unlikely orientation after the short startup convergence window.
#define ACCEL_INNOVATION_GATE (0.65f)
#define MAG_INNOVATION_GATE   (0.55f)
#define EKF_INNOVATION_GATE_WARMUP_UPDATES (250U)
#define EKF_MAX_CONSECUTIVE_FAILURES (25)
// Threshold to protect against division by zero when normalizing sensor vectors
const float NORM_EPSILON = 1e-6f;
float_prec gEkfRuntimeDt = SS_DT;
uint8_t ekfConsecutiveFailures = 0;
uint16_t ekfInnovationGateWarmupUpdates = 0;
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {
  P_INIT_QUAT, 0, 0, 0, 0, 0, 0,
  0, P_INIT_QUAT, 0, 0, 0, 0, 0,
  0, 0, P_INIT_QUAT, 0, 0, 0, 0,
  0, 0, 0, P_INIT_QUAT, 0, 0, 0,
  0, 0, 0, 0, P_INIT_GYRO_BIAS, 0, 0,
  0, 0, 0, 0, 0, P_INIT_GYRO_BIAS, 0,
  0, 0, 0, 0, 0, 0, P_INIT_GYRO_BIAS
};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {
  Q_INIT_QUAT, 0, 0, 0, 0, 0, 0,
  0, Q_INIT_QUAT, 0, 0, 0, 0, 0,
  0, 0, Q_INIT_QUAT, 0, 0, 0, 0,
  0, 0, 0, Q_INIT_QUAT, 0, 0, 0,
  0, 0, 0, 0, Q_INIT_GYRO_BIAS, 0, 0,
  0, 0, 0, 0, 0, Q_INIT_GYRO_BIAS, 0,
  0, 0, 0, 0, 0, 0, Q_INIT_GYRO_BIAS
};
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
float_prec EKF_RINIT_data[SS_Z_LEN*SS_Z_LEN] = {
  R_INIT_ACC, 0, 0, 0, 0, 0,
  0, R_INIT_ACC, 0, 0, 0, 0,
  0, 0, R_INIT_ACC, 0, 0, 0,
  0, 0, 0, R_INIT_MAG, 0, 0,
  0, 0, 0, 0, R_INIT_MAG, 0,
  0, 0, 0, 0, 0, R_INIT_MAG
};
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);
float_prec EKF_RACTIVE_data[SS_Z_LEN*SS_Z_LEN];
Matrix EKF_RACTIVE(SS_Z_LEN, SS_Z_LEN, EKF_RACTIVE_data);

// Nonlinear update and Jacobian functions (assumed implemented)
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);
bool Main_bNormalizeState(Matrix& X);

// EKF state variables
Matrix quaternionData(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
EKF EKF_IMU(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT,
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY,
            Main_bCalcJacobianF, Main_bCalcJacobianH, Main_bNormalizeState);

// ----- Auxiliary Variables -----
elapsedMicros timerEKF;
uint64_t u64compuTime;
char bufferTxSer[100];
char cmd;

#ifndef FC_TIMING_INSTRUMENTATION
#define FC_TIMING_INSTRUMENTATION 1
#endif

#ifndef FC_TIMING_SERIAL_OUTPUT
#define FC_TIMING_SERIAL_OUTPUT 0
#endif

constexpr uint32_t EKF_PERIOD_US = SS_DT_MILIS * 1000UL;
// Independent hardware watchdog (IWDG) timeout. The main loop services many
// tasks per attitude period (8 ms) and the longest blocking call is now bounded
// by the 25 ms I2C timeout, so 100 ms leaves ample margin against false resets
// while still recovering quickly if the loop ever hangs (wedged bus, math
// assert, etc.). The IWDG runs off the independent LSI clock, so it fires even
// if the main clock or loop is stuck.
//
// This is kept flat (not tied to FC_CONTROL_DEBUG_SERIAL_OUTPUT) so the tight
// flight-safe window is the default for every build -- the default Arduino
// sketch forces the debug macro to 1, so coupling the timeout to it would
// silently relax the watchdog for ordinary builds. The once-per-second FCDBG
// diagnostic line is the only long blocking write, and loop() reloads the
// watchdog immediately before emitting it, so a normal (host-reading) print
// starts with a full 100 ms window and cannot trip the IWDG.
constexpr uint32_t WATCHDOG_TIMEOUT_US = 100000UL;
constexpr uint16_t SERVO_UPDATE_HYSTERESIS_US = 3;
constexpr uint32_t SERVO_FORCE_REFRESH_PERIOD_US = 100000UL;
constexpr uint32_t RC_FAILSAFE_TIMEOUT_US = 250000UL;
// CRSF parser stalls in the field have shown up as short gaps where raw bytes
// still arrive but RC_CHANNELS_PACKED frames do not decode. Keep only a brief
// blend window after RC goes stale so flight builds do not preserve stale
// high-deflection surface commands for multiple seconds.
constexpr uint32_t RC_SERVO_HOLD_TIMEOUT_US = 500000UL;
constexpr uint32_t CRSF_BYTE_ACTIVITY_TIMEOUT_US = RC_FAILSAFE_TIMEOUT_US;
constexpr uint32_t BAROMETER_TEMPERATURE_PERIOD_US = 500000UL;

struct TimingCounter {
  uint32_t lastUs;
  uint32_t maxUs;
  uint32_t count;
};

#if FC_TIMING_INSTRUMENTATION
TimingCounter timingEkf = {0, 0, 0};
TimingCounter timingBarometer = {0, 0, 0};
TimingCounter timingAirspeed = {0, 0, 0};
TimingCounter timingGpsParse = {0, 0, 0};
TimingCounter timingCrsfUpdate = {0, 0, 0};
TimingCounter timingLoop = {0, 0, 0};
elapsedMillis timingPrintTimer;

void recordTiming(TimingCounter& counter, uint32_t startUs) {
  uint32_t elapsedUs = micros() - startUs;
  counter.lastUs = elapsedUs;
  if (elapsedUs > counter.maxUs) {
    counter.maxUs = elapsedUs;
  }
  ++counter.count;
}

void printTimingCounter(const char *label, const TimingCounter& counter) {
  Serial.print(label);
  Serial.print(" last/max/count=");
  Serial.print(counter.lastUs);
  Serial.print('/');
  Serial.print(counter.maxUs);
  Serial.print('/');
  Serial.print(counter.count);
  Serial.print(" us ");
}

void maybePrintTimingStats() {
#if FC_TIMING_SERIAL_OUTPUT
  if (timingPrintTimer >= 1000) {
    timingPrintTimer = 0;
    printTimingCounter("EKF", timingEkf);
    printTimingCounter("Baro", timingBarometer);
    printTimingCounter("Airspeed", timingAirspeed);
    printTimingCounter("GPS", timingGpsParse);
    printTimingCounter("CRSF", timingCrsfUpdate);
    printTimingCounter("Loop", timingLoop);
    Serial.println();
  }
#endif
}
#endif

// ----- I2C -----
// Create an alternate I2C instance on PB9 (SDA) and PB8 (SCL)
TwoWire I2C_Alternate(PB9, PB8);

// ----- Sensors -----
// Airspeed Sensor (MS4525D0)
MS4525D0 airspeedSensor(I2C_Alternate, 0x28);
// Barometer (MS5611)
MS5611 barometer(&I2C_Alternate, 0x77);
// IMU (MPU9250)
SimpleMPU9250 IMU(I2C_Alternate, 0x68);

// ----- Servo Outputs -----
// Roll     (channel 1) -> A1
// Pitch    (channel 2) -> A2
// Throttle (channel 3) -> A4
// Yaw      (channel 4) -> A3
Servo servoRoll;
Servo servoPitch;
Servo servoYaw;
Servo servoThrottle;

// Cache the last commanded servo pulse widths so we only update hardware
// when values change. This reduces Servo library ISR load and helps keep
// telemetry timing stable.
uint16_t lastRollCommandUs = 0;
uint16_t lastPitchCommandUs = 0;
uint16_t lastYawCommandUs = 0;
uint16_t lastThrottleCommandUs = 0;
uint32_t lastRollWriteUs = 0;
uint32_t lastPitchWriteUs = 0;
uint32_t lastYawWriteUs = 0;
uint32_t lastThrottleWriteUs = 0;
uint32_t lastControlUpdateUs = 0;
uint32_t lastRcPacketUs = 0;
uint32_t lastCrsfByteUs = 0;
bool rcReceiverFailsafeActive = true;
bool rcFailsafeActive = true;
bool rcServoHoldBlendActive = false;
uint16_t rcServoHoldStartRollUs = 1500;
uint16_t rcServoHoldStartPitchUs = 1500;
uint16_t rcServoHoldStartYawUs = 1500;

struct ControlDebugCounters {
  uint32_t rcPackets;
  uint32_t rcFailsafePackets;
  uint32_t ekfUpdates;
  uint32_t servoLoopFresh;
  uint32_t servoLoopStale;
  uint32_t servoLoopHold;
  uint32_t airspeedInvalidReads;
  uint32_t rollServoWrites;
  uint32_t pitchServoWrites;
  uint32_t yawServoWrites;
  uint32_t throttleServoWrites;
  uint32_t attitudeTelemetryWrites;
  uint32_t gpsTelemetryWrites;
  uint32_t crsfTelemetryUartFrames;
  uint32_t crsfTelemetryAttitudeUartFrames;
  uint32_t crsfTelemetryGpsUartFrames;
  uint32_t crsfTelemetryOtherUartFrames;
  uint32_t crsfRxBytes;
  uint32_t crsfCompleteFrames;
  uint32_t crsfValidFrames;
  uint32_t crsfCrcErrors;
  uint32_t crsfRcFrames;
  uint32_t crsfRcWrongAddressFrames;
  uint32_t crsfOtherValidFrames;
  uint32_t crsfFrameTimeoutResets;
  uint8_t crsfLastFrameType;
  uint8_t crsfLastFrameAddress;
  uint8_t crsfLastFrameLength;
  uint8_t crsfLastTelemetryFrameType;
  uint32_t loopIterations;
  uint32_t crsfServiceCalls;
  uint32_t maxRcAgeUs;
};

ControlDebugCounters controlDebugCounters = {0};
elapsedMillis controlDebugPrintTimer;

void setEkfMeasurementNoise(float_prec accVariance, float_prec magVariance) {
  EKF_RACTIVE.vSetToZero();
  EKF_RACTIVE[0][0] = accVariance;
  EKF_RACTIVE[1][1] = accVariance;
  EKF_RACTIVE[2][2] = accVariance;
  EKF_RACTIVE[3][3] = magVariance;
  EKF_RACTIVE[4][4] = magVariance;
  EKF_RACTIVE[5][5] = magVariance;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float vectorInnovationNorm(const Matrix& measurement, const Matrix& prediction, uint8_t startIndex) {
  const float dx = measurement[startIndex][0] - prediction[startIndex][0];
  const float dy = measurement[startIndex + 1][0] - prediction[startIndex + 1][0];
  const float dz = measurement[startIndex + 2][0] - prediction[startIndex + 2][0];
  return sqrtf(dx*dx + dy*dy + dz*dz);
}

void resetControlDebugCounters() {
  controlDebugCounters.rcPackets = 0;
  controlDebugCounters.rcFailsafePackets = 0;
  controlDebugCounters.ekfUpdates = 0;
  controlDebugCounters.servoLoopFresh = 0;
  controlDebugCounters.servoLoopStale = 0;
  controlDebugCounters.servoLoopHold = 0;
  controlDebugCounters.airspeedInvalidReads = 0;
  controlDebugCounters.rollServoWrites = 0;
  controlDebugCounters.pitchServoWrites = 0;
  controlDebugCounters.yawServoWrites = 0;
  controlDebugCounters.throttleServoWrites = 0;
  controlDebugCounters.attitudeTelemetryWrites = 0;
  controlDebugCounters.gpsTelemetryWrites = 0;
  controlDebugCounters.crsfTelemetryUartFrames = 0;
  controlDebugCounters.crsfTelemetryAttitudeUartFrames = 0;
  controlDebugCounters.crsfTelemetryGpsUartFrames = 0;
  controlDebugCounters.crsfTelemetryOtherUartFrames = 0;
  controlDebugCounters.crsfRxBytes = 0;
  controlDebugCounters.crsfCompleteFrames = 0;
  controlDebugCounters.crsfValidFrames = 0;
  controlDebugCounters.crsfCrcErrors = 0;
  controlDebugCounters.crsfRcFrames = 0;
  controlDebugCounters.crsfRcWrongAddressFrames = 0;
  controlDebugCounters.crsfOtherValidFrames = 0;
  controlDebugCounters.crsfFrameTimeoutResets = 0;
  controlDebugCounters.crsfLastFrameType = 0;
  controlDebugCounters.crsfLastFrameAddress = 0;
  controlDebugCounters.crsfLastFrameLength = 0;
  controlDebugCounters.crsfLastTelemetryFrameType = 0;
  controlDebugCounters.loopIterations = 0;
  controlDebugCounters.crsfServiceCalls = 0;
  controlDebugCounters.maxRcAgeUs = 0;
}

// Create a CRSFforArduino instance using Serial3.
CRSFforArduino crsf(&Serial3);
serialReceiverLayer::serialReceiverDiagnostics_t lastCrsfDiagnostics = {};

// Store the latest received RC channel data.
serialReceiverLayer::rcChannels_t latestRcChannels;

ControlMode controlMode = CONTROL_MODE_MANUAL;
ThrottleMode throttleMode = THROTTLE_MODE_MANUAL;

const uint16_t RC_INPUT_MIN = 172;
const uint16_t RC_INPUT_MAX = 1811;
const uint16_t RC_INPUT_CENTER = (RC_INPUT_MIN + RC_INPUT_MAX) / 2;

// Mode channel targets from the ground station (channel 6) and a guard band to avoid chatter.
const uint16_t CONTROL_MODE_FLY_BY_WIRE_TARGET = 1700;
const uint16_t CONTROL_MODE_SWITCH_DEADBAND = 150;
const uint16_t CONTROL_MODE_FLY_BY_WIRE_MIN = CONTROL_MODE_FLY_BY_WIRE_TARGET - CONTROL_MODE_SWITCH_DEADBAND;

// Throttle mode is carried on CH7/AUX3 so CH5/AUX1 can stay dedicated to ELRS
// arming and CH6/AUX2 can carry Manual/Fly-By-Wire mode.
const uint16_t THROTTLE_MODE_AUTO_TARGET = 1700;
const uint16_t THROTTLE_MODE_SWITCH_DEADBAND = 150;
const uint16_t THROTTLE_MODE_AUTO_MIN = THROTTLE_MODE_AUTO_TARGET - THROTTLE_MODE_SWITCH_DEADBAND;

const float AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH = 100.0f;
const float AUTO_THROTTLE_DEFAULT_TARGET_MPH = 20.0f;
const uint32_t AIRSPEED_FAILSAFE_TIMEOUT_US = 100000UL;
const float AUTO_THROTTLE_STALE_DECAY_PERCENT_PER_S = 50.0f;

const uint16_t SERVO_MIN_US = 1000;
const uint16_t SERVO_MAX_US = 2000;
const uint16_t SERVO_CENTER_US = 1500;
const uint16_t THROTTLE_MIN_US = 1000;
const uint16_t THROTTLE_MAX_US = 2000;
const uint16_t THROTTLE_CUT_US = THROTTLE_MIN_US;
const uint16_t SERVO_HALF_TRAVEL_US = (SERVO_MAX_US - SERVO_MIN_US) / 2;
const uint16_t SERVO_CALIBRATION_ACTIVE_US = SERVO_CENTER_US + ((SERVO_HALF_TRAVEL_US * 9) / 10);
const uint16_t SERVO_INDICATOR_HOLD_MS = 350;

// Fly-by-wire tuning constants.
const float FBW_MAX_ROLL_ANGLE_DEG = 80.0f;
const float FBW_MAX_PITCH_ANGLE_DEG = 80.0f;
const float FBW_PID_OUTPUT_LIMIT_US = 400.0f;
const float FBW_PID_INTEGRAL_LIMIT = 100.0f;
const float FBW_ATTITUDE_FILTER_CUTOFF_HZ = 5.0f;
const float FBW_PID_ERROR_DEADBAND_DEG = 0.5f;

// PID gains (servo microseconds per degree / degree-second) tuned for the Aeroscout airframe.
const float FBW_ROLL_KP = 5.0f;
const float FBW_ROLL_KI = 0.25f;
const float FBW_ROLL_KD = 0.9f;

const float FBW_PITCH_KP = 6.0f;
const float FBW_PITCH_KI = 0.30f;
const float FBW_PITCH_KD = 1.1f;

// Airspeed-hold throttle PID. Output is interpreted as percent-per-second and
// integrated into the current auto-throttle command at the control-loop rate.
const float AUTO_THROTTLE_KP = 0.8f;
const float AUTO_THROTTLE_KI = 0.04f;
const float AUTO_THROTTLE_KD = 0.15f;
const float AUTO_THROTTLE_OUTPUT_LIMIT_PERCENT_PER_S = 100.0f;
const float AUTO_THROTTLE_INTEGRAL_LIMIT = 100.0f;
const float AUTO_THROTTLE_ERROR_DEADBAND_MPH = 0.2f;

struct LowPassFilter {
  float cutoffHz;
  float alpha;
  float state;
  bool hasState;

  LowPassFilter(float cutoffHz, float dt)
    : cutoffHz(cutoffHz), alpha(computeAlpha(cutoffHz, dt)), state(0.0f), hasState(false) {}

  static float computeAlpha(float cutoffHz, float dt) {
    if (cutoffHz <= 0.0f || dt <= 0.0f) {
      return 1.0f;
    }
    float rc = 1.0f / (2.0f * M_PI * cutoffHz);
    float alpha = dt / (rc + dt);
    if (alpha < 0.0f) {
      alpha = 0.0f;
    } else if (alpha > 1.0f) {
      alpha = 1.0f;
    }
    return alpha;
  }

  float update(float input, float dt) {
    alpha = computeAlpha(cutoffHz, dt);
    if (!hasState) {
      state = input;
      hasState = true;
      return state;
    }
    state += alpha * (input - state);
    return state;
  }

  void reset() {
    hasState = false;
  }
};

struct PIDController {
  float kp;
  float ki;
  float kd;
  float integrator;
  float prevMeasurement;
  bool hasPrevMeasurement;
  float outputMin;
  float outputMax;
  float integratorMin;
  float integratorMax;
  float errorDeadband;

  PIDController(float p, float i, float d,
                float outMin, float outMax,
                float integMin, float integMax,
                float deadband)
    : kp(p), ki(i), kd(d), integrator(0.0f), prevMeasurement(0.0f), hasPrevMeasurement(false),
      outputMin(outMin), outputMax(outMax), integratorMin(integMin), integratorMax(integMax),
      errorDeadband(deadband) {}

  void reset() {
    integrator = 0.0f;
    prevMeasurement = 0.0f;
    hasPrevMeasurement = false;
  }

  float update(float target, float measurement, float dt) {
    float error = target - measurement;
    if (fabsf(error) < errorDeadband) {
      error = 0.0f;
    }
    float dMeas = 0.0f;
    if (hasPrevMeasurement && dt > 0.0f) {
      dMeas = (measurement - prevMeasurement) / dt;
    }

    prevMeasurement = measurement;
    hasPrevMeasurement = true;

    float integratorIncrement = error * dt;
    float pTerm = kp * error;
    float iTerm = ki * integrator;
    float dTerm = -kd * dMeas;

    float unclampedOutput = pTerm + iTerm + dTerm;
    float clampedOutput = constrain(unclampedOutput, outputMin, outputMax);

    float integratorEffect = ki * integratorIncrement;
    bool pushingUpperSaturation = (unclampedOutput >= outputMax) && (integratorEffect > 0.0f);
    bool pushingLowerSaturation = (unclampedOutput <= outputMin) && (integratorEffect < 0.0f);

    if (!(pushingUpperSaturation || pushingLowerSaturation)) {
      integrator += integratorIncrement;
      integrator = constrain(integrator, integratorMin, integratorMax);
      iTerm = ki * integrator;
      unclampedOutput = pTerm + iTerm + dTerm;
      clampedOutput = constrain(unclampedOutput, outputMin, outputMax);
    }

    return clampedOutput;
  }
};

PIDController rollPid(FBW_ROLL_KP, FBW_ROLL_KI, FBW_ROLL_KD,
                      -FBW_PID_OUTPUT_LIMIT_US, FBW_PID_OUTPUT_LIMIT_US,
                      -FBW_PID_INTEGRAL_LIMIT, FBW_PID_INTEGRAL_LIMIT,
                      FBW_PID_ERROR_DEADBAND_DEG);

PIDController pitchPid(FBW_PITCH_KP, FBW_PITCH_KI, FBW_PITCH_KD,
                       -FBW_PID_OUTPUT_LIMIT_US, FBW_PID_OUTPUT_LIMIT_US,
                       -FBW_PID_INTEGRAL_LIMIT, FBW_PID_INTEGRAL_LIMIT,
                       FBW_PID_ERROR_DEADBAND_DEG);

PIDController throttlePid(AUTO_THROTTLE_KP, AUTO_THROTTLE_KI, AUTO_THROTTLE_KD,
                          -AUTO_THROTTLE_OUTPUT_LIMIT_PERCENT_PER_S,
                          AUTO_THROTTLE_OUTPUT_LIMIT_PERCENT_PER_S,
                          -AUTO_THROTTLE_INTEGRAL_LIMIT,
                          AUTO_THROTTLE_INTEGRAL_LIMIT,
                          AUTO_THROTTLE_ERROR_DEADBAND_MPH);

float autoThrottlePercent = 0.0f;
float latestAutoThrottleTargetMph = AUTO_THROTTLE_DEFAULT_TARGET_MPH;
uint32_t lastAirspeedUpdateUs = 0;
bool latestAirspeedValid = false;

LowPassFilter rollAngleFilter(FBW_ATTITUDE_FILTER_CUTOFF_HZ, static_cast<float>(SS_DT));
LowPassFilter pitchAngleFilter(FBW_ATTITUDE_FILTER_CUTOFF_HZ, static_cast<float>(SS_DT));

// Callback to capture incoming RC channel packets.
void rcChannelsCallback(serialReceiverLayer::rcChannels_t *channels) {
  if (channels == nullptr) {
    rcReceiverFailsafeActive = true;
    ++controlDebugCounters.rcFailsafePackets;
    return;
  }

  // CRSFforArduino derives channels->failsafe from CRSF link-statistics
  // frames.  The ground station's direct USB/serial control link sends
  // RC_CHANNELS_PACKED frames but does not send receiver link-statistics, so
  // the library reports failsafe even while fresh RC frames are arriving.
  // Accept the decoded channel frame and let rcInputFresh() enforce our real
  // failsafe from packet age instead of the missing link-statistics flag.
  rcReceiverFailsafeActive = channels->failsafe;
  if (channels->failsafe) {
    ++controlDebugCounters.rcFailsafePackets;
  }

  latestRcChannels = *channels;
  lastRcPacketUs = micros();
  ++controlDebugCounters.rcPackets;
}

uint16_t mapRcToUs(uint16_t value) {
  const uint16_t outMin = SERVO_MIN_US;
  const uint16_t outMax = SERVO_MAX_US;
  if (value < RC_INPUT_MIN) value = RC_INPUT_MIN;
  if (value > RC_INPUT_MAX) value = RC_INPUT_MAX;
  return (uint16_t)(((uint32_t)(value - RC_INPUT_MIN) * (outMax - outMin)) /
                    (RC_INPUT_MAX - RC_INPUT_MIN) + outMin);
}

float mapRcToPercent(uint16_t value) {
  if (value < RC_INPUT_MIN) value = RC_INPUT_MIN;
  if (value > RC_INPUT_MAX) value = RC_INPUT_MAX;
  return (static_cast<float>(value - RC_INPUT_MIN) * 100.0f) /
         static_cast<float>(RC_INPUT_MAX - RC_INPUT_MIN);
}

uint16_t mapPercentToThrottleUs(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  return static_cast<uint16_t>(roundf(
      THROTTLE_MIN_US + (percent / 100.0f) * (THROTTLE_MAX_US - THROTTLE_MIN_US)));
}

float mapRcToAutoThrottleTargetMph(uint16_t value) {
  return (mapRcToPercent(value) / 100.0f) * AUTO_THROTTLE_SPEED_CHANNEL_MAX_MPH;
}


bool shouldUpdateServo(uint16_t newCommandUs, uint16_t lastCommandUs, uint32_t lastWriteUs, uint32_t nowUs) {
  return abs(static_cast<int>(newCommandUs) - static_cast<int>(lastCommandUs)) >= SERVO_UPDATE_HYSTERESIS_US ||
         (uint32_t)(nowUs - lastWriteUs) >= SERVO_FORCE_REFRESH_PERIOD_US;
}

void writeRollPitchIndicator(uint16_t commandUs) {
  servoRoll.writeMicroseconds(commandUs);
  servoPitch.writeMicroseconds(commandUs);
  lastRollCommandUs = commandUs;
  lastPitchCommandUs = commandUs;
  lastRollWriteUs = micros();
  lastPitchWriteUs = lastRollWriteUs;
}

void centerAllServos() {
  writeRollPitchIndicator(SERVO_CENTER_US);
  servoYaw.writeMicroseconds(SERVO_CENTER_US);
  servoThrottle.writeMicroseconds(THROTTLE_CUT_US);
  lastYawCommandUs = SERVO_CENTER_US;
  lastThrottleCommandUs = THROTTLE_CUT_US;
  lastYawWriteUs = lastRollWriteUs;
  lastThrottleWriteUs = lastRollWriteUs;
}

void initializeServoOutputs() {
  servoRoll.attach(A1);
  servoPitch.attach(A2);
  servoYaw.attach(A3);
  servoThrottle.attach(A4);

  centerAllServos();
}

void haltStartupWithNeutralServos() {
  centerAllServos();
  while (1) { ; }
}

void signalCalibrationActive() {
  writeRollPitchIndicator(SERVO_CALIBRATION_ACTIVE_US);
  delay(SERVO_INDICATOR_HOLD_MS);
  centerAllServos();
}

void signalCalibrationComplete() {
  writeRollPitchIndicator(SERVO_MIN_US);
  delay(SERVO_INDICATOR_HOLD_MS);
  writeRollPitchIndicator(SERVO_MAX_US);
  delay(SERVO_INDICATOR_HOLD_MS);
  writeRollPitchIndicator(SERVO_CENTER_US);
}

uint32_t rcInputAgeUs(uint32_t nowUs) {
  return lastRcPacketUs == 0 ? UINT32_MAX : static_cast<uint32_t>(nowUs - lastRcPacketUs);
}

bool rcInputFresh(uint32_t nowUs) {
  return rcInputAgeUs(nowUs) <= RC_FAILSAFE_TIMEOUT_US;
}

uint32_t crsfByteAgeUs(uint32_t nowUs) {
  return lastCrsfByteUs == 0 ? UINT32_MAX : static_cast<uint32_t>(nowUs - lastCrsfByteUs);
}

bool crsfBytesActive(uint32_t nowUs) {
  return crsfByteAgeUs(nowUs) <= CRSF_BYTE_ACTIVITY_TIMEOUT_US;
}

bool rcInputWithinServoHold(uint32_t nowUs) {
  return rcInputAgeUs(nowUs) <= RC_SERVO_HOLD_TIMEOUT_US && crsfBytesActive(nowUs);
}

uint16_t blendServoTowardNeutral(uint16_t startUs, uint32_t nowUs) {
  const uint32_t rcAgeUs = rcInputAgeUs(nowUs);
  const uint32_t holdBlendDurationUs =
      (RC_SERVO_HOLD_TIMEOUT_US > RC_FAILSAFE_TIMEOUT_US)
          ? (RC_SERVO_HOLD_TIMEOUT_US - RC_FAILSAFE_TIMEOUT_US)
          : 0UL;
  if (holdBlendDurationUs == 0 || rcAgeUs >= RC_SERVO_HOLD_TIMEOUT_US) {
    return SERVO_CENTER_US;
  }

  const uint32_t blendAgeUs =
      rcAgeUs > RC_FAILSAFE_TIMEOUT_US ? rcAgeUs - RC_FAILSAFE_TIMEOUT_US : 0;
  const float progress = constrain(
      static_cast<float>(blendAgeUs) / static_cast<float>(holdBlendDurationUs),
      0.0f,
      1.0f);
  const float commandUs = static_cast<float>(startUs) +
      (static_cast<float>(SERVO_CENTER_US) - static_cast<float>(startUs)) * progress;
  return static_cast<uint16_t>(roundf(commandUs));
}

bool airspeedInputFresh(uint32_t nowUs) {
  return latestAirspeedValid &&
         lastAirspeedUpdateUs != 0 &&
         (uint32_t)(nowUs - lastAirspeedUpdateUs) <= AIRSPEED_FAILSAFE_TIMEOUT_US;
}

float mapRcToNormalized(uint16_t value) {
  const float inMin = static_cast<float>(RC_INPUT_MIN);
  const float inMax = static_cast<float>(RC_INPUT_MAX);
  float clamped = constrain(static_cast<float>(value), inMin, inMax);
  float halfRange = (inMax - inMin) * 0.5f;
  if (halfRange <= 0.0f) {
    return 0.0f;
  }
  float center = inMin + halfRange;
  float normalized = (clamped - center) / halfRange;
  return constrain(normalized, -1.0f, 1.0f);
}

void setControlMode(ControlMode newMode) {
  if (controlMode != newMode) {
    controlMode = newMode;
    rollPid.reset();
    pitchPid.reset();
    rollAngleFilter.reset();
    pitchAngleFilter.reset();
  }
}

void setThrottleMode(ThrottleMode newMode) {
  if (throttleMode != newMode) {
    throttleMode = newMode;
    throttlePid.reset();
    if (newMode == THROTTLE_MODE_MANUAL) {
      autoThrottlePercent = 0.0f;
    }
  }
}

void updateControlMode() {
  // Manual/Fly-By-Wire mode is carried on CH6/AUX2 so CH5/AUX1 can remain
  // dedicated to the ELRS arm state.  CRSF channel arrays are zero-indexed.
  const size_t modeChannelIndex = 5;
  const size_t channelCount = sizeof(latestRcChannels.value) / sizeof(latestRcChannels.value[0]);
  if (modeChannelIndex >= channelCount) {
    setControlMode(CONTROL_MODE_MANUAL);
    return;
  }

  const uint16_t modeValue = latestRcChannels.value[modeChannelIndex];
  if (modeValue >= CONTROL_MODE_FLY_BY_WIRE_MIN) {
    setControlMode(CONTROL_MODE_FLY_BY_WIRE);
  } else {
    // Treat every non-high value as Manual.  Leaving the previous FBW state
    // latched while AUX2 is centered or transient made the roll/pitch PID stay
    // active after the ground station requested Manual.  Manual is the safe
    // default unless the mode channel is explicitly driven high.
    setControlMode(CONTROL_MODE_MANUAL);
  }
}

void updateThrottleMode() {
  // Auto throttle mode is carried on CH7/AUX3.  CH5/AUX1 remains the ELRS arm
  // state and CH6/AUX2 remains Manual/Fly-By-Wire mode.
  const size_t throttleModeChannelIndex = 6;
  const size_t channelCount = sizeof(latestRcChannels.value) / sizeof(latestRcChannels.value[0]);
  if (throttleModeChannelIndex >= channelCount) {
    setThrottleMode(THROTTLE_MODE_MANUAL);
    return;
  }
  uint16_t modeValue = latestRcChannels.value[throttleModeChannelIndex];
  if (modeValue >= THROTTLE_MODE_AUTO_MIN) {
    setThrottleMode(THROTTLE_MODE_AUTO);
  } else {
    // Match the control-mode fail-safe behavior: require an explicit high AUX3
    // command before enabling the throttle PID, otherwise pass throttle through.
    setThrottleMode(THROTTLE_MODE_MANUAL);
  }
}

void serviceCrsfLink() {
#if FC_TIMING_INSTRUMENTATION
  uint32_t timingStartUs = micros();
#endif
  crsf.update();
  const serialReceiverLayer::serialReceiverDiagnostics_t crsfDiagnostics = crsf.getDiagnostics();
  if (crsfDiagnostics.parser.bytesReceived != lastCrsfDiagnostics.parser.bytesReceived) {
    lastCrsfByteUs = micros();
  }
  controlDebugCounters.crsfTelemetryUartFrames +=
      static_cast<uint32_t>(crsfDiagnostics.telemetryFramesSent - lastCrsfDiagnostics.telemetryFramesSent);
  controlDebugCounters.crsfTelemetryAttitudeUartFrames +=
      static_cast<uint32_t>(crsfDiagnostics.telemetryAttitudeFramesSent - lastCrsfDiagnostics.telemetryAttitudeFramesSent);
  controlDebugCounters.crsfTelemetryGpsUartFrames +=
      static_cast<uint32_t>(crsfDiagnostics.telemetryGpsFramesSent - lastCrsfDiagnostics.telemetryGpsFramesSent);
  controlDebugCounters.crsfTelemetryOtherUartFrames +=
      static_cast<uint32_t>(crsfDiagnostics.telemetryOtherFramesSent - lastCrsfDiagnostics.telemetryOtherFramesSent);
  controlDebugCounters.crsfRxBytes +=
      static_cast<uint32_t>(crsfDiagnostics.parser.bytesReceived - lastCrsfDiagnostics.parser.bytesReceived);
  controlDebugCounters.crsfCompleteFrames +=
      static_cast<uint32_t>(crsfDiagnostics.parser.completeFrames - lastCrsfDiagnostics.parser.completeFrames);
  controlDebugCounters.crsfValidFrames +=
      static_cast<uint32_t>(crsfDiagnostics.parser.validFrames - lastCrsfDiagnostics.parser.validFrames);
  controlDebugCounters.crsfCrcErrors +=
      static_cast<uint32_t>(crsfDiagnostics.parser.crcErrors - lastCrsfDiagnostics.parser.crcErrors);
  controlDebugCounters.crsfRcFrames +=
      static_cast<uint32_t>(crsfDiagnostics.parser.rcFrames - lastCrsfDiagnostics.parser.rcFrames);
  controlDebugCounters.crsfRcWrongAddressFrames +=
      static_cast<uint32_t>(crsfDiagnostics.parser.rcWrongAddressFrames - lastCrsfDiagnostics.parser.rcWrongAddressFrames);
  controlDebugCounters.crsfOtherValidFrames +=
      static_cast<uint32_t>(crsfDiagnostics.parser.otherValidFrames - lastCrsfDiagnostics.parser.otherValidFrames);
  controlDebugCounters.crsfFrameTimeoutResets +=
      static_cast<uint32_t>(crsfDiagnostics.parser.frameTimeoutResets - lastCrsfDiagnostics.parser.frameTimeoutResets);
  controlDebugCounters.crsfLastFrameType = crsfDiagnostics.parser.lastFrameType;
  controlDebugCounters.crsfLastFrameAddress = crsfDiagnostics.parser.lastDeviceAddress;
  controlDebugCounters.crsfLastFrameLength = crsfDiagnostics.parser.lastFrameLength;
  controlDebugCounters.crsfLastTelemetryFrameType = crsfDiagnostics.lastTelemetryFrameType;
  lastCrsfDiagnostics = crsfDiagnostics;
  ++controlDebugCounters.crsfServiceCalls;
#if FC_TIMING_INSTRUMENTATION
  recordTiming(timingCrsfUpdate, timingStartUs);
#endif
  updateControlMode();
  updateThrottleMode();
}

// ----- GPS -----
// Instantiate the GPS object on gpsSerial
M8N gps(gpsSerial);

// Global variables to store the latest GPS data
double latestLatitude  = 0;
double latestLongitude = 0;
uint8_t satsInUse      = 0;       // GPS satellites currently in use
double latestGpsCourse = 0.0;

// Telemetry values prepared for CRSF GPS frame. The GPS CRSF frame uses the
// latest cached GPS coordinates plus separately sampled airspeed/barometer data.
float airSpeedCms      = 0.0f; // Airspeed from sensor in centimeters per second
float sensorAltitudeCm = 0.0f; // Altitude from barometer in centimeters
float latestAirspeedMph = 0.0f;
float latestAltitudeFeet = 0.0f;

// ----- Sensor and telemetry timing -----
elapsedMicros attitudeTelemetryTimer;
elapsedMicros gpsTelemetryTimer;
elapsedMicros gpsDrainTimer;
elapsedMicros barometerTimer;
elapsedMicros airspeedTimer;
constexpr uint32_t ATTITUDE_TELEMETRY_PERIOD_US = 8000;  // 125 Hz
constexpr uint32_t GPS_TELEMETRY_PERIOD_US = 50000;      // 20 Hz, reduced to lower radio link load
constexpr uint32_t GPS_DRAIN_PERIOD_US = 100000;         // 10 Hz UART drain/cache refresh
constexpr uint32_t BAROMETER_PERIOD_US = 16667;          // ~60 Hz hardware read/cache refresh
constexpr uint32_t AIRSPEED_PERIOD_US = 16667;           // ~60 Hz hardware read/cache refresh

enum BarometerReadState {
  BAROMETER_IDLE = 0,
  BAROMETER_WAIT_PRESSURE,
  BAROMETER_WAIT_TEMPERATURE
};

BarometerReadState barometerReadState = BAROMETER_IDLE;
uint32_t barometerConversionStartUs = 0;
uint32_t barometerRawPressure = 0;
uint32_t barometerRawTemperature = 0;
uint32_t lastBarometerTemperatureUs = 0;
bool barometerTemperatureValid = false;

int16_t latestAttitudeRoll = 0;
int16_t latestAttitudePitch = 0;
int16_t latestAttitudeYaw = 0;
bool attitudeSampleValid = false;

void updateGpsCache() {
#if FC_TIMING_INSTRUMENTATION
  uint32_t timingStartUs = micros();
#endif
  gps.gatherData();
#if FC_TIMING_INSTRUMENTATION
  recordTiming(timingGpsParse, timingStartUs);
#endif
  satsInUse = gps.satellites_in_use;
  if (gps.has_valid_fix) {
    latestLatitude = gps.latitude;
    latestLongitude = gps.longitude;
    latestGpsCourse = gps.course;
  } else {
    latestLatitude = 0.0;
    latestLongitude = 0.0;
    latestGpsCourse = 0.0;
  }
}

void applyBarometerPressure(float baroPressure) {
  if (!isfinite(baroPressure) || baroPressure <= 0.0f) {
    return;
  }
  const float altitudeMeters = barometer.getAltitude(baroPressure, barometer.getSeaLevelPressure());
  if (!isfinite(altitudeMeters)) {
    return;
  }
  sensorAltitudeCm = altitudeMeters * 100.0f;
  latestAltitudeFeet = altitudeMeters * 3.28084f;
}

void updateBarometerCacheBlocking() {
#if FC_TIMING_INSTRUMENTATION
  uint32_t timingStartUs = micros();
#endif
  applyBarometerPressure(barometer.readPressure());
#if FC_TIMING_INSTRUMENTATION
  recordTiming(timingBarometer, timingStartUs);
#endif
}

void serviceBarometerCache() {
#if FC_TIMING_INSTRUMENTATION
  uint32_t timingStartUs = micros();
#endif
  bool barometerDidWork = false;
  const uint32_t nowUs = micros();
  const uint32_t conversionWaitUs = static_cast<uint32_t>(barometer.getConversionTimeMs()) * 1000UL;

  switch (barometerReadState) {
    case BAROMETER_IDLE:
      if (barometerTimer >= BAROMETER_PERIOD_US) {
        const bool temperatureDue = !barometerTemperatureValid ||
                                    (uint32_t)(nowUs - lastBarometerTemperatureUs) >= BAROMETER_TEMPERATURE_PERIOD_US;
        barometerTimer = 0;
        barometerConversionStartUs = nowUs;
        if (temperatureDue) {
          barometer.startRawTemperatureConversion();
          barometerReadState = BAROMETER_WAIT_TEMPERATURE;
        } else {
          barometer.startRawPressureConversion();
          barometerReadState = BAROMETER_WAIT_PRESSURE;
        }
        barometerDidWork = true;
      }
      break;

    case BAROMETER_WAIT_PRESSURE:
      if ((uint32_t)(nowUs - barometerConversionStartUs) >= conversionWaitUs) {
        uint32_t rawPressure = 0;
        if (barometer.readAdc(rawPressure)) {
          barometerRawPressure = rawPressure;
          if (barometerTemperatureValid) {
            applyBarometerPressure(barometer.calculatePressure(barometerRawPressure, barometerRawTemperature));
          }
        }
        barometerReadState = BAROMETER_IDLE;
        barometerDidWork = true;
      }
      break;

    case BAROMETER_WAIT_TEMPERATURE:
      if ((uint32_t)(nowUs - barometerConversionStartUs) >= conversionWaitUs) {
        uint32_t rawTemperature = 0;
        if (barometer.readAdc(rawTemperature)) {
          barometerRawTemperature = rawTemperature;
          lastBarometerTemperatureUs = micros();
          barometerTemperatureValid = true;
        }
        barometerReadState = BAROMETER_IDLE;
        barometerDidWork = true;
      }
      break;
  }
#if FC_TIMING_INSTRUMENTATION
  if (barometerDidWork) {
    recordTiming(timingBarometer, timingStartUs);
  }
#endif
}


void updateAirspeedCache() {
#if FC_TIMING_INSTRUMENTATION
  uint32_t timingStartUs = micros();
#endif
  float airspeedMph = airspeedSensor.getAirspeed();
  if (isnan(airspeedMph)) {
    // Serial.println("Airspeed sensor error");
    ++controlDebugCounters.airspeedInvalidReads;
    airspeedMph = 0.0f;
    latestAirspeedValid = false;
  } else {
    latestAirspeedValid = true;
  }
  latestAirspeedMph = airspeedMph;
  airSpeedCms = airspeedMph * 44.704f;   // mph to cm/s
  lastAirspeedUpdateUs = micros();
#if FC_TIMING_INSTRUMENTATION
  recordTiming(timingAirspeed, timingStartUs);
#endif
}

void resetPeriodicTimers() {
  // elapsedMicros/elapsedMillis start counting at construction, so long setup
  // tasks such as calibration and cache priming can otherwise create a large
  // backlog that replays periodic work every loop immediately after boot.
  attitudeTelemetryTimer = 0;
  gpsTelemetryTimer = 0;
  gpsDrainTimer = 0;
  barometerTimer = 0;
  airspeedTimer = 0;
  timerEKF = 0;
  barometerReadState = BAROMETER_IDLE;
  barometerTemperatureValid = false;
  lastBarometerTemperatureUs = 0;
  lastControlUpdateUs = micros();
  controlDebugPrintTimer = 0;
  resetControlDebugCounters();
}


#if FC_MAG_CALIBRATION_MODE
void printMagCalibrationConstantSet(float hardX, float hardY, float hardZ,
                                    float softX, float softY, float softZ) {
  Serial.println("MAGCAL copy these constants into Main.ino after verifying the fit:");
  Serial.println("float_prec HARD_IRON_BIAS_data[3] = {");
  Serial.print("  "); Serial.print(hardX, 6); Serial.print("f, ");
  Serial.print(hardY, 6); Serial.print("f, ");
  Serial.print(hardZ, 6); Serial.println("f");
  Serial.println("};");
  Serial.println("float_prec SOFT_IRON_MATRIX_data[9] = {");
  Serial.print("  "); Serial.print(softX, 6); Serial.println("f, 0.0f, 0.0f,");
  Serial.print("  0.0f, "); Serial.print(softY, 6); Serial.println("f, 0.0f,");
  Serial.print("  0.0f, 0.0f, "); Serial.print(softZ, 6); Serial.println("f");
  Serial.println("};");
}

void runMagnetometerCalibrationDebug() {
  Serial.println();
  Serial.println("MAGCAL mode is ENABLED. This is a bench-only helper; do not fly with FC_MAG_CALIBRATION_MODE=1.");
  Serial.println("MAGCAL set FC_MAG_CALIBRATION_MODE to 0 and reflash/reset to skip calibration.");
  Serial.print("MAGCAL calibration starts in ");
  Serial.print(FC_MAG_CALIBRATION_START_DELAY_MS / 1000UL);
  Serial.println(" seconds. Rotate the fully assembled aircraft through every orientation when sampling starts.");
  Serial.println("MAGCAL If you do not want to rotate/calibrate now, power down or reset before sampling starts.");

  uint32_t countdownStartMs = millis();
  uint32_t nextCountdownPrintMs = countdownStartMs;
  while ((uint32_t)(millis() - countdownStartMs) < FC_MAG_CALIBRATION_START_DELAY_MS) {
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - nextCountdownPrintMs) >= 1000UL) {
      nextCountdownPrintMs += 1000UL;
      uint32_t elapsedMs = nowMs - countdownStartMs;
      uint32_t remainingMs = (elapsedMs >= FC_MAG_CALIBRATION_START_DELAY_MS)
                               ? 0UL
                               : (FC_MAG_CALIBRATION_START_DELAY_MS - elapsedMs);
      Serial.print("MAGCAL starting in ");
      Serial.print((remainingMs + 999UL) / 1000UL);
      Serial.println(" s");
    }
    delay(10);
  }

  Serial.println("MAGCAL sampling started. Keep rotating slowly: nose up/down, left/right wing down, inverted, and yaw sweeps.");

  float minX = 0.0f;
  float minY = 0.0f;
  float minZ = 0.0f;
  float maxX = 0.0f;
  float maxY = 0.0f;
  float maxZ = 0.0f;
  bool haveSample = false;
  uint32_t sampleCount = 0;
  uint32_t rejectedCount = 0;
  uint32_t sampleStartMs = millis();
  uint32_t lastSampleMs = sampleStartMs;
  uint32_t lastStatusMs = sampleStartMs;

  while ((uint32_t)(millis() - sampleStartMs) < FC_MAG_CALIBRATION_DURATION_MS) {
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastSampleMs) >= FC_MAG_CALIBRATION_SAMPLE_PERIOD_MS) {
      lastSampleMs = nowMs;
      if (IMU.readSensor() > 0) {
        // Use the same aircraft-frame magnetometer axes as the EKF update path.
        float x = IMU.getMagY_uT();
        float y = IMU.getMagX_uT();
        float z = IMU.getMagZ_uT();
        float norm = sqrt(x*x + y*y + z*z);
        if (norm > NORM_EPSILON) {
          if (!haveSample) {
            minX = maxX = x;
            minY = maxY = y;
            minZ = maxZ = z;
            haveSample = true;
          } else {
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
          }
          ++sampleCount;
        } else {
          ++rejectedCount;
        }
      } else {
        ++rejectedCount;
      }
    }

    if ((uint32_t)(nowMs - lastStatusMs) >= 5000UL) {
      lastStatusMs = nowMs;
      uint32_t elapsedMs = nowMs - sampleStartMs;
      uint32_t remainingMs = (elapsedMs >= FC_MAG_CALIBRATION_DURATION_MS)
                               ? 0UL
                               : (FC_MAG_CALIBRATION_DURATION_MS - elapsedMs);
      Serial.print("MAGCAL samples="); Serial.print(sampleCount);
      Serial.print(" rejected="); Serial.print(rejectedCount);
      Serial.print(" remaining_s="); Serial.println((remainingMs + 999UL) / 1000UL);
    }
    delay(1);
  }

  Serial.println("MAGCAL sampling complete.");
  if (!haveSample || sampleCount < 50) {
    Serial.println("MAGCAL failed: not enough valid magnetometer samples. Check IMU wiring and rerun calibration.");
    return;
  }

  float hardX = (maxX + minX) * 0.5f;
  float hardY = (maxY + minY) * 0.5f;
  float hardZ = (maxZ + minZ) * 0.5f;
  float spanX = maxX - minX;
  float spanY = maxY - minY;
  float spanZ = maxZ - minZ;
  float radiusX = spanX * 0.5f;
  float radiusY = spanY * 0.5f;
  float radiusZ = spanZ * 0.5f;

  Serial.print("MAGCAL raw_min_uT="); Serial.print(minX, 3); Serial.print(','); Serial.print(minY, 3); Serial.print(','); Serial.println(minZ, 3);
  Serial.print("MAGCAL raw_max_uT="); Serial.print(maxX, 3); Serial.print(','); Serial.print(maxY, 3); Serial.print(','); Serial.println(maxZ, 3);
  Serial.print("MAGCAL hard_iron_uT="); Serial.print(hardX, 6); Serial.print(','); Serial.print(hardY, 6); Serial.print(','); Serial.println(hardZ, 6);
  Serial.print("MAGCAL span_uT="); Serial.print(spanX, 6); Serial.print(','); Serial.print(spanY, 6); Serial.print(','); Serial.println(spanZ, 6);
  Serial.print("MAGCAL radii_uT="); Serial.print(radiusX, 6); Serial.print(','); Serial.print(radiusY, 6); Serial.print(','); Serial.println(radiusZ, 6);

  if (spanX < FC_MAG_CALIBRATION_MIN_AXIS_SPAN_UT ||
      spanY < FC_MAG_CALIBRATION_MIN_AXIS_SPAN_UT ||
      spanZ < FC_MAG_CALIBRATION_MIN_AXIS_SPAN_UT) {
    Serial.print("MAGCAL failed: each axis must span at least ");
    Serial.print(FC_MAG_CALIBRATION_MIN_AXIS_SPAN_UT, 1);
    Serial.println(" uT. Rerun and rotate through all orientations.");
    return;
  }

  float averageRadius = (radiusX + radiusY + radiusZ) / 3.0f;
  float softX = averageRadius / radiusX;
  float softY = averageRadius / radiusY;
  float softZ = averageRadius / radiusZ;
  Serial.println("MAGCAL note: this helper computes hard-iron plus diagonal soft-iron from min/max coverage.");
  Serial.println("MAGCAL note: for off-diagonal soft-iron terms, export raw samples and run a full ellipsoid fit offboard.");
  printMagCalibrationConstantSet(hardX, hardY, hardZ, softX, softY, softZ);
}
#endif


#if FC_GPS_DIAGNOSTIC_MODE
constexpr size_t GPS_DIAG_NMEA_BUFFER_SIZE = 121;
constexpr size_t GPS_DIAG_MAX_FIELDS = 20;
constexpr uint32_t GPS_DIAG_POST_PING_WINDOW_MS = 1500UL;

char gpsDiagNmeaBuffer[GPS_DIAG_NMEA_BUFFER_SIZE];
size_t gpsDiagNmeaBufferIndex = 0;
bool gpsDiagReceivingNmea = false;
bool gpsDiagDiscardingNmea = false;
uint32_t gpsDiagByteCount = 0;
uint32_t gpsDiagSentenceCount = 0;
uint32_t gpsDiagChecksumOkCount = 0;
uint32_t gpsDiagChecksumFailCount = 0;
uint32_t gpsDiagOverlengthCount = 0;
uint32_t gpsDiagPingCount = 0;
uint32_t gpsDiagBytesAtLastPing = 0;
uint32_t gpsDiagLastPingMs = 0;
uint32_t gpsDiagLastByteMs = 0;
uint32_t gpsDiagLastStatusMs = 0;

double gpsDiagConvertNmeaCoordinate(const char *rawValue, char direction) {
  if (rawValue == nullptr || rawValue[0] == '\0') {
    return 0.0;
  }

  const int degreeDigits = (direction == 'N' || direction == 'S') ? 2 :
                           (direction == 'E' || direction == 'W') ? 3 : 0;
  if (degreeDigits == 0 || strlen(rawValue) < static_cast<size_t>(degreeDigits)) {
    return 0.0;
  }

  char degreeBuffer[4] = {0};
  memcpy(degreeBuffer, rawValue, degreeDigits);
  double decimal = static_cast<double>(atoi(degreeBuffer)) + (atof(rawValue + degreeDigits) / 60.0);
  if (direction == 'S' || direction == 'W') {
    decimal = -decimal;
  }
  return decimal;
}

bool gpsDiagValidateAndStripChecksum(char *sentence) {
  if (sentence == nullptr || sentence[0] != '$') {
    return false;
  }

  char *checksumMarker = strchr(sentence, '*');
  if (checksumMarker == nullptr || checksumMarker[1] == '\0' || checksumMarker[2] == '\0') {
    return false;
  }

  uint8_t calculated = 0;
  for (char *cursor = sentence + 1; cursor < checksumMarker; ++cursor) {
    calculated ^= static_cast<uint8_t>(*cursor);
  }

  char checksumText[3] = {checksumMarker[1], checksumMarker[2], '\0'};
  char *end = nullptr;
  const unsigned long expected = strtoul(checksumText, &end, 16);
  if (end == checksumText || *end != '\0' || expected > 0xFFUL) {
    return false;
  }

  if (calculated != static_cast<uint8_t>(expected)) {
    return false;
  }

  *checksumMarker = '\0';
  return true;
}

size_t gpsDiagSplitFields(char *sentence, char *fields[], size_t maxFields) {
  size_t fieldCount = 0;
  char *fieldStart = sentence;
  while (fieldCount < maxFields) {
    fields[fieldCount++] = fieldStart;
    char *comma = strchr(fieldStart, ',');
    if (comma == nullptr) {
      break;
    }
    *comma = '\0';
    fieldStart = comma + 1;
  }
  return fieldCount;
}

void gpsDiagPrintParsedSentence(char *sentence) {
  Serial.print("GPSDIAG RAW: ");
  Serial.println(sentence);
  ++gpsDiagSentenceCount;

  if (!gpsDiagValidateAndStripChecksum(sentence)) {
    ++gpsDiagChecksumFailCount;
    Serial.println("GPSDIAG PARSED: checksum failed/missing; check baud, noise, or wiring.");
    return;
  }

  ++gpsDiagChecksumOkCount;
  char *fields[GPS_DIAG_MAX_FIELDS] = {nullptr};
  const size_t fieldCount = gpsDiagSplitFields(sentence, fields, GPS_DIAG_MAX_FIELDS);

  if ((strncmp(fields[0], "$GNGGA", 6) == 0 || strncmp(fields[0], "$GPGGA", 6) == 0) && fieldCount > 9) {
    const double parsedLat = gpsDiagConvertNmeaCoordinate(fields[2], fields[3][0]);
    const double parsedLon = gpsDiagConvertNmeaCoordinate(fields[4], fields[5][0]);
    Serial.print("GPSDIAG GGA: fix="); Serial.print(atoi(fields[6]));
    Serial.print(" sats="); Serial.print(atoi(fields[7]));
    Serial.print(" lat="); Serial.print(parsedLat, 8);
    Serial.print(" lon="); Serial.print(parsedLon, 8);
    Serial.print(" alt_m="); Serial.println(strtod(fields[9], nullptr), 2);
  } else if ((strncmp(fields[0], "$GNRMC", 6) == 0 || strncmp(fields[0], "$GPRMC", 6) == 0) && fieldCount > 9) {
    const double parsedLat = gpsDiagConvertNmeaCoordinate(fields[3], fields[4][0]);
    const double parsedLon = gpsDiagConvertNmeaCoordinate(fields[5], fields[6][0]);
    Serial.print("GPSDIAG RMC: status="); Serial.print(fields[2]);
    Serial.print(" lat="); Serial.print(parsedLat, 8);
    Serial.print(" lon="); Serial.print(parsedLon, 8);
    Serial.print(" speed_knots="); Serial.print(fields[7]);
    Serial.print(" course_deg="); Serial.println(fields[8]);
  } else {
    Serial.print("GPSDIAG PARSED: valid ");
    Serial.print(fields[0]);
    Serial.println(" sentence");
  }
}

void gpsDiagSendPing() {
  static const uint8_t ubxMonVerPoll[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
  static const char nmeaPubxPositionPoll[] = "$PUBX,00*33\r\n";
  gpsSerial.write(ubxMonVerPoll, sizeof(ubxMonVerPoll));
  gpsSerial.print(nmeaPubxPositionPoll);
  gpsSerial.flush();

  ++gpsDiagPingCount;
  gpsDiagBytesAtLastPing = gpsDiagByteCount;
  gpsDiagLastPingMs = millis();
  Serial.print("GPSDIAG PING: sent UBX-MON-VER and PUBX position poll #");
  Serial.println(gpsDiagPingCount);
}

void gpsDiagPrintStatus() {
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - gpsDiagLastStatusMs) < FC_GPS_DIAGNOSTIC_STATUS_PERIOD_MS) {
    return;
  }
  gpsDiagLastStatusMs = nowMs;

  Serial.print("GPSDIAG STATUS: bytes="); Serial.print(gpsDiagByteCount);
  Serial.print(" sentences="); Serial.print(gpsDiagSentenceCount);
  Serial.print(" checksum_ok="); Serial.print(gpsDiagChecksumOkCount);
  Serial.print(" checksum_fail="); Serial.print(gpsDiagChecksumFailCount);
  Serial.print(" overlength="); Serial.print(gpsDiagOverlengthCount);
  Serial.print(" ms_since_last_byte="); Serial.println(nowMs - gpsDiagLastByteMs);

  if (gpsDiagLastPingMs != 0 &&
      (uint32_t)(nowMs - gpsDiagLastPingMs) >= GPS_DIAG_POST_PING_WINDOW_MS &&
      (uint32_t)(nowMs - gpsDiagLastPingMs) < GPS_DIAG_POST_PING_WINDOW_MS + FC_GPS_DIAGNOSTIC_STATUS_PERIOD_MS) {
    Serial.print("GPSDIAG PING RESULT: bytes_after_last_ping=");
    Serial.print(gpsDiagByteCount - gpsDiagBytesAtLastPing);
    Serial.println((gpsDiagByteCount > gpsDiagBytesAtLastPing) ? " (PC7 RX saw activity)" : " (no reply/activity seen)");
  }

  if (gpsDiagByteCount == 0 || (uint32_t)(nowMs - gpsDiagLastByteMs) > FC_GPS_DIAGNOSTIC_NO_DATA_WARNING_MS) {
    Serial.println("GPSDIAG WARNING: no recent GPS bytes. Check M8N power, ground, GPS TX -> PC7/USART6 RX, GPS RX <- PC6/USART6 TX, and baud.");
  }
}

void runGpsDiagnosticDebug() {
  Serial.println();
  Serial.println("GPSDIAG mode is ENABLED. This is a bench-only helper; do not fly with FC_GPS_DIAGNOSTIC_MODE=1.");
  Serial.println("GPSDIAG set FC_GPS_DIAGNOSTIC_MODE to 0 and reflash/reset to skip GPS diagnostics.");
  Serial.println("GPSDIAG wiring: GPS TX -> board PC7/USART6 RX, GPS RX <- board PC6/USART6 TX, common ground, GPS powered.");
  pinMode(PC7, INPUT);
  const int rxIdleLevel = digitalRead(PC7);
  Serial.print("GPSDIAG PC7/USART6 RX idle level before UART start: ");
  Serial.println(rxIdleLevel == HIGH ? "HIGH (normal UART idle if GPS TX is connected/powered)" : "LOW (possible short, swapped wire, or unpowered GPS)");

  gpsSerial.begin(FC_GPS_DIAGNOSTIC_BAUD);
  gpsDiagLastByteMs = millis();
  gpsDiagLastStatusMs = millis();
  gpsDiagSendPing();

  while (true) {
    while (gpsSerial.available() > 0) {
      const int rawByte = gpsSerial.read();
      if (rawByte < 0) {
        continue;
      }
      const char c = static_cast<char>(rawByte & 0xFF);
      ++gpsDiagByteCount;
      gpsDiagLastByteMs = millis();

      if (c == '$') {
        gpsDiagNmeaBufferIndex = 0;
        gpsDiagReceivingNmea = true;
        gpsDiagDiscardingNmea = false;
      } else if (!gpsDiagReceivingNmea) {
        continue;
      }

      if (c == '\r') {
        continue;
      }

      if (c == '\n') {
        if (!gpsDiagDiscardingNmea) {
          gpsDiagNmeaBuffer[gpsDiagNmeaBufferIndex] = '\0';
          if (gpsDiagNmeaBufferIndex > 0) {
            gpsDiagPrintParsedSentence(gpsDiagNmeaBuffer);
          }
        }
        gpsDiagNmeaBufferIndex = 0;
        gpsDiagNmeaBuffer[0] = '\0';
        gpsDiagReceivingNmea = false;
        gpsDiagDiscardingNmea = false;
        continue;
      }

      if (!gpsDiagDiscardingNmea) {
        if (gpsDiagNmeaBufferIndex < GPS_DIAG_NMEA_BUFFER_SIZE - 1) {
          gpsDiagNmeaBuffer[gpsDiagNmeaBufferIndex++] = c;
        } else {
          ++gpsDiagOverlengthCount;
          gpsDiagNmeaBufferIndex = 0;
          gpsDiagNmeaBuffer[0] = '\0';
          gpsDiagReceivingNmea = false;
          gpsDiagDiscardingNmea = true;
        }
      }
    }

    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - gpsDiagLastPingMs) >= FC_GPS_DIAGNOSTIC_PING_PERIOD_MS) {
      gpsDiagSendPing();
    }
    gpsDiagPrintStatus();
    delay(1);
  }
}
#endif

void maybePrintControlDebugStats() {
#if FC_CONTROL_DEBUG_SERIAL_OUTPUT
  if (controlDebugPrintTimer < 1000) {
    return;
  }

  const uint32_t elapsedMs = controlDebugPrintTimer;
  controlDebugPrintTimer = 0;
  const float scale = elapsedMs > 0 ? (1000.0f / static_cast<float>(elapsedMs)) : 0.0f;
  const uint32_t nowUs = micros();
  const uint32_t currentRcAgeUs = lastRcPacketUs == 0 ? 0 : static_cast<uint32_t>(nowUs - lastRcPacketUs);
  const uint32_t maxRcAgeUs = max(controlDebugCounters.maxRcAgeUs, currentRcAgeUs);
  const int16_t telemetryPitchDdeg = static_cast<int16_t>(-latestAttitudePitch);
  const float telemetryLatitude = static_cast<float>(latestLatitude);
  const float telemetryLongitude = static_cast<float>(latestLongitude);

  Serial.print("FCDBG ");
  Serial.print("rc_hz="); Serial.print(controlDebugCounters.rcPackets * scale, 1);
  Serial.print(" rc_failsafe_hz="); Serial.print(controlDebugCounters.rcFailsafePackets * scale, 1);
  Serial.print(" ekf_hz="); Serial.print(controlDebugCounters.ekfUpdates * scale, 1);
  Serial.print(" att_tx_hz="); Serial.print(controlDebugCounters.attitudeTelemetryWrites * scale, 1);
  Serial.print(" gps_tx_hz="); Serial.print(controlDebugCounters.gpsTelemetryWrites * scale, 1);
  // Report the telemetry values as they are represented on the CRSF uplink.
  // Attitude pitch is sign-inverted by the CRSF telemetry encoder, latitude and
  // longitude are rounded to float by telemetryWriteGPS(), and fractional
  // altitude/speed centimeters are preserved so FCDBG can be compared directly
  // with the values transmitted to the RX module.
  Serial.print(" tlm_roll_ddeg="); Serial.print(latestAttitudeRoll);
  Serial.print(" tlm_pitch_ddeg="); Serial.print(telemetryPitchDdeg);
  Serial.print(" tlm_yaw_ddeg="); Serial.print(latestAttitudeYaw);
  Serial.print(" tlm_roll_deg="); Serial.print(latestAttitudeRoll / 10.0f, 1);
  Serial.print(" tlm_pitch_deg="); Serial.print(telemetryPitchDdeg / 10.0f, 1);
  Serial.print(" tlm_yaw_deg="); Serial.print(latestAttitudeYaw / 10.0f, 1);
  Serial.print(" tlm_lat="); Serial.print(telemetryLatitude, 7);
  Serial.print(" tlm_lon="); Serial.print(telemetryLongitude, 7);
  Serial.print(" gps_raw_lat="); Serial.print(gps.latitude, 8);
  Serial.print(" gps_raw_lon="); Serial.print(gps.longitude, 8);
  Serial.print(" gps_raw_fix_quality="); Serial.print(gps.fix_quality);
  Serial.print(" gps_raw_sats="); Serial.print(gps.satellites_in_use);
  Serial.print(" tlm_alt_cm="); Serial.print(sensorAltitudeCm, 2);
  Serial.print(" tlm_alt_ft="); Serial.print(latestAltitudeFeet, 1);
  Serial.print(" tlm_speed_cms="); Serial.print(airSpeedCms, 2);
  Serial.print(" tlm_speed_mph="); Serial.print(latestAirspeedMph, 1);
  Serial.print(" airspeed_invalid_hz="); Serial.print(controlDebugCounters.airspeedInvalidReads * scale, 1);
  Serial.print(" tlm_course="); Serial.print(latestGpsCourse, 1);
  Serial.print(" tlm_sats="); Serial.print(satsInUse);
  Serial.print(" tlm_att_valid="); Serial.print(attitudeSampleValid ? 1 : 0);
  Serial.print(" tlm_gps_fix="); Serial.print(gps.has_valid_fix ? 1 : 0);
  Serial.print(" tlm_uart_hz="); Serial.print(controlDebugCounters.crsfTelemetryUartFrames * scale, 1);
  Serial.print('/'); Serial.print(controlDebugCounters.crsfTelemetryAttitudeUartFrames * scale, 1);
  Serial.print('/'); Serial.print(controlDebugCounters.crsfTelemetryGpsUartFrames * scale, 1);
  Serial.print('/'); Serial.print(controlDebugCounters.crsfTelemetryOtherUartFrames * scale, 1);
  Serial.print(" crsf_rx_bytes_s="); Serial.print(controlDebugCounters.crsfRxBytes * scale, 1);
  Serial.print(" crsf_frame_hz="); Serial.print(controlDebugCounters.crsfCompleteFrames * scale, 1);
  Serial.print('/'); Serial.print(controlDebugCounters.crsfValidFrames * scale, 1);
  Serial.print('/'); Serial.print(controlDebugCounters.crsfCrcErrors * scale, 1);
  Serial.print('/'); Serial.print(controlDebugCounters.crsfFrameTimeoutResets * scale, 1);
  Serial.print(" crsf_rc_frame_hz="); Serial.print(controlDebugCounters.crsfRcFrames * scale, 1);
  Serial.print(" crsf_rc_wrong_addr_hz="); Serial.print(controlDebugCounters.crsfRcWrongAddressFrames * scale, 1);
  Serial.print(" crsf_other_frame_hz="); Serial.print(controlDebugCounters.crsfOtherValidFrames * scale, 1);
  Serial.print(" crsf_last=0x"); Serial.print(controlDebugCounters.crsfLastFrameType, HEX);
  Serial.print("@0x"); Serial.print(controlDebugCounters.crsfLastFrameAddress, HEX);
  Serial.print('/'); Serial.print(controlDebugCounters.crsfLastFrameLength);
  Serial.print(" tlm_last=0x"); Serial.print(controlDebugCounters.crsfLastTelemetryFrameType, HEX);
  Serial.print(" servo_loop_fresh_hz="); Serial.print(controlDebugCounters.servoLoopFresh * scale, 1);
  Serial.print(" servo_loop_stale_hz="); Serial.print(controlDebugCounters.servoLoopStale * scale, 1);
  Serial.print(" servo_loop_hold_hz="); Serial.print(controlDebugCounters.servoLoopHold * scale, 1);
  Serial.print(" servo_writes_hz=");
  Serial.print(controlDebugCounters.rollServoWrites * scale, 1); Serial.print('/');
  Serial.print(controlDebugCounters.pitchServoWrites * scale, 1); Serial.print('/');
  Serial.print(controlDebugCounters.yawServoWrites * scale, 1); Serial.print('/');
  Serial.print(controlDebugCounters.throttleServoWrites * scale, 1);
  Serial.print(" crsf_service_hz="); Serial.print(controlDebugCounters.crsfServiceCalls * scale, 1);
  Serial.print(" loop_hz="); Serial.print(controlDebugCounters.loopIterations * scale, 1);
  Serial.print(" rc_age_ms="); Serial.print(currentRcAgeUs / 1000.0f, 1);
  Serial.print(" rc_max_age_ms="); Serial.print(maxRcAgeUs / 1000.0f, 1);
  Serial.print(" rc_fresh="); Serial.print(rcInputFresh(nowUs) ? 1 : 0);
  Serial.print(" rx_failsafe="); Serial.print(rcReceiverFailsafeActive ? 1 : 0);
  Serial.print(" mode="); Serial.print(controlMode == CONTROL_MODE_FLY_BY_WIRE ? "FBW" : "MANUAL");
  Serial.print(" mode_ch="); Serial.print(latestRcChannels.value[5]);
  Serial.print(" throttle_mode="); Serial.print(throttleMode == THROTTLE_MODE_AUTO ? "AUTO" : "MANUAL");
  Serial.print(" throttle_mode_ch="); Serial.print(latestRcChannels.value[6]);
  Serial.print(" throttle_target_mph="); Serial.print(latestAutoThrottleTargetMph, 1);
  Serial.print(" auto_throttle_pct="); Serial.println(autoThrottlePercent, 1);

  lastCrsfDiagnostics = crsf.getDiagnostics();
  resetControlDebugCounters();
#endif
}



void setup() {
  // ----- Initialize Debug Serial -----
  Serial.begin(115200);
  // Allow time for a serial connection, but don't block startup
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart < 3000)) {
    delay(10);
  }
#if FC_CONTROL_DEBUG_SERIAL_OUTPUT
  Serial.println("FCDBG serial output enabled; emitting control stats once per second.");
#else
  Serial.println("FCDBG serial output disabled; define FC_CONTROL_DEBUG_SERIAL_OUTPUT before konfig.h to enable.");
#endif

  // Run GPSDIAG before any non-GPS sensor startup can halt the bench test.
#if FC_GPS_DIAGNOSTIC_MODE
  runGpsDiagnosticDebug();
#endif

  // ----- Initialize Servo Outputs -----
  initializeServoOutputs();

  // Briefly move the ailerons and elevator before sensor calibration begins,
  // then return to neutral before any blocking I2C sensor calls. This gives the
  // pilot a visible startup-calibration indication without holding the surfaces
  // near an end stop if a disconnected sensor stalls initialization.
  signalCalibrationActive();

  // ----- Initialize I2C -----
  I2C_Alternate.begin();
  I2C_Alternate.setClock(400000);
  // Bound every blocking I2C transaction (IMU/barometer/airspeed all share this
  // bus) so a stuck SDA/SCL line cannot block readSensor() in the 125 Hz loop
  // forever and freeze the control surfaces.
  //
  // The Arduino Wire timeout API (setWireTimeout) is only present on cores that
  // advertise WIRE_HAS_TIMEOUT -- e.g. the AVR core. The STM32duino TwoWire used
  // for flight builds does NOT expose it (its HAL bounds each transfer with its
  // own internal timeout instead), so the call is guarded to keep the firmware
  // compiling on both. On STM32duino the HAL timeout plus the hardware watchdog
  // below provide the wedged-bus protection; where the Arduino API is available
  // we additionally release the peripheral on timeout (reset_with_timeout=true).
#if defined(WIRE_HAS_TIMEOUT)
  I2C_Alternate.setWireTimeout(25000 /* us */, true /* reset_with_timeout */);
#endif

  // ----- Calibrate Barometer -----
  if (!barometer.begin()) {
    Serial.println("MS5611 initialization unsuccessful");
    Serial.println("Check barometer wiring or try cycling power");
    haltStartupWithNeutralServos();
  }
  // Keep conversion latency low so the 60 Hz barometer cache does not starve
  // the 125 Hz IMU/EKF loop. LOW_POWER uses shorter conversion delays than
  // HIGH_RES at the cost of some pressure resolution.
  barometer.setOversampling("LOW_POWER");
  barometer.calibrate();

  // ----- Calibrate Airspeed Sensor -----
  airspeedSensor.calibrate();

  // ----- Initialize IMU -----
  Serial.println("Calibrating IMU bias...");
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    haltStartupWithNeutralServos();
  }
  Serial.println("IMU Calibration complete...");
#if FC_MAG_CALIBRATION_MODE
  runMagnetometerCalibrationDebug();
  Serial.println("MAGCAL complete. Halting startup so calibration mode cannot be used for flight.");
  haltStartupWithNeutralServos();
#endif

  // ----- Initialize EKF -----
  quaternionData.vSetToZero();
  quaternionData[0][0] = 1.0;
  EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
  snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "Adafruit STM32F405 Feather Express (%s)\r\n",
           (FPU_PRECISION == PRECISION_SINGLE) ? "Float32" : "Double64");
  Serial.print(bufferTxSer);

  for (size_t i = 0; i < (sizeof(latestRcChannels.value) / sizeof(latestRcChannels.value[0])); ++i) {
    latestRcChannels.value[i] = RC_INPUT_CENTER;
  }

  // ----- Initialize GPS (gpsSerial) -----
  gpsSerial.begin(9600);
  delay(1000);
  Serial.println("GPS module initialized on USART6.");

  // Prime slow-sensor caches so the first GPS telemetry frames do not carry
  // default airspeed/altitude values while waiting for their first timers.
  updateBarometerCacheBlocking();
  updateAirspeedCache();
  updateGpsCache();

  // ----- Initialize CRSF Telemetry -----
  // Use a baud rate of 921600 as required.
  if (!crsf.begin(921600)) {
    Serial.println("CRSF for Arduino initialization failed!");
    haltStartupWithNeutralServos();
  }
  crsf.setRcChannelsCallback(rcChannelsCallback);

  // Sweep the ailerons and elevator through full travel once after all startup
  // initialization is complete, then return them to neutral for normal servo
  // operation.
  signalCalibrationComplete();

  resetPeriodicTimers();

  // Start the hardware watchdog only after all blocking startup work and
  // halt-on-failure sensor checks have completed. Starting it here preserves
  // the existing "halt with neutral servos" behavior for startup sensor faults
  // (those paths intentionally never reach this line) while protecting the
  // flight loop: if any iteration stalls longer than WATCHDOG_TIMEOUT_US the
  // board resets instead of holding stale servo commands indefinitely.
  IWatchdog.begin(WATCHDOG_TIMEOUT_US);

  Serial.println("CRSF Telemetry Ready");
}


void loop() {
  ++controlDebugCounters.loopIterations;
  // Kick the watchdog once per iteration. Placed at the top so a hang anywhere
  // in the body (wedged I2C, CRSF service, EKF math) lets the IWDG expire and
  // reset the board rather than freezing the control surfaces.
  IWatchdog.reload();
#if FC_TIMING_INSTRUMENTATION
  uint32_t loopStartUs = micros();
#endif
  serviceCrsfLink();

  bool attitudeTelemetrySentThisLoop = false;
  bool gpsTelemetrySentThisLoop = false;

  serviceBarometerCache();
  serviceCrsfLink();

  if (airspeedTimer >= AIRSPEED_PERIOD_US) {
    airspeedTimer = 0;
    updateAirspeedCache();
    serviceCrsfLink();
  }

  if (gpsDrainTimer >= GPS_DRAIN_PERIOD_US) {
    gpsDrainTimer = 0;
    // Drain the 9600-baud GPS UART at the old 50 Hz cadence to avoid RX
    // buffer overflow; telemetry below only reuses the latest parsed cache.
    updateGpsCache();
    serviceCrsfLink();
  }

  // ----- Sensor Fusion, EKF, and Control Update (125 Hz) -----
  if (timerEKF >= EKF_PERIOD_US) {
    timerEKF -= EKF_PERIOD_US;
    ++controlDebugCounters.ekfUpdates;
    const uint32_t controlUpdateUs = micros();
    float controlDt = (lastControlUpdateUs == 0)
                        ? static_cast<float>(SS_DT)
                        : static_cast<float>(controlUpdateUs - lastControlUpdateUs) * 1.0e-6f;
    if (controlDt < 0.001f || controlDt > 0.050f) {
      controlDt = static_cast<float>(SS_DT);
    }
    lastControlUpdateUs = controlUpdateUs;
    
    // Read sensor data from the IMU
    IMU.readSensor();
    // Swap X/Y axes to align IMU frame with aircraft frame
    float Ax = IMU.getAccelY_mss();
    float Ay = IMU.getAccelX_mss();
    float Az = IMU.getAccelZ_mss();
    float Bx = IMU.getMagY_uT();
    float By = IMU.getMagX_uT();
    float Bz = IMU.getMagZ_uT();
    float p  = IMU.getGyroY_rads();
    float q  = IMU.getGyroX_rads();
    float r  = IMU.getGyroZ_rads();
    
    // Populate matrices for EKF update
    U[0][0] = p;  U[1][0] = q;  U[2][0] = r;
    Y[0][0] = Ax; Y[1][0] = Ay; Y[2][0] = Az;
    Y[3][0] = Bx; Y[4][0] = By; Y[5][0] = Bz;

    setEkfMeasurementNoise(R_INIT_ACC, R_INIT_MAG);
    gEkfRuntimeDt = static_cast<float_prec>(controlDt);
    Matrix predictedX = EKF_IMU.GetX();
    Matrix predictedY(SS_Z_LEN, 1);
    if (Main_bUpdateNonlinearX(predictedX, predictedX, U)) {
      Main_bUpdateNonlinearY(predictedY, predictedX, U);
    } else {
      Main_bUpdateNonlinearY(predictedY, EKF_IMU.GetX(), U);
    }

    // Compensate for hard-iron and soft-iron magnetometer calibration without changing aircraft axes.
    float magBiasX = Y[3][0] - HARD_IRON_BIAS[0][0];
    float magBiasY = Y[4][0] - HARD_IRON_BIAS[1][0];
    float magBiasZ = Y[5][0] - HARD_IRON_BIAS[2][0];
    Y[3][0] = SOFT_IRON_MATRIX[0][0]*magBiasX + SOFT_IRON_MATRIX[0][1]*magBiasY + SOFT_IRON_MATRIX[0][2]*magBiasZ;
    Y[4][0] = SOFT_IRON_MATRIX[1][0]*magBiasX + SOFT_IRON_MATRIX[1][1]*magBiasY + SOFT_IRON_MATRIX[1][2]*magBiasZ;
    Y[5][0] = SOFT_IRON_MATRIX[2][0]*magBiasX + SOFT_IRON_MATRIX[2][1]*magBiasY + SOFT_IRON_MATRIX[2][2]*magBiasZ;

    // Normalize accelerometer vector, but reject it when magnitude indicates non-gravity acceleration.
    float normG = sqrt(Y[0][0]*Y[0][0] + Y[1][0]*Y[1][0] + Y[2][0]*Y[2][0]);
    bool accelRejected = (normG <= NORM_EPSILON) ||
                         (fabs(normG - GRAVITY_NOMINAL_MSS) > (GRAVITY_NOMINAL_MSS * ACCEL_NORM_GATE_FRACTION));
    if (!accelRejected) {
      Y[0][0] /= normG; Y[1][0] /= normG; Y[2][0] /= normG;
      if (ekfInnovationGateWarmupUpdates >= EKF_INNOVATION_GATE_WARMUP_UPDATES) {
        accelRejected = vectorInnovationNorm(Y, predictedY, 0) > ACCEL_INNOVATION_GATE;
      }
    }
    if (accelRejected) {
      Y[0][0] = predictedY[0][0];
      Y[1][0] = predictedY[1][0];
      Y[2][0] = predictedY[2][0];
      EKF_RACTIVE[0][0] = R_REJECTED;
      EKF_RACTIVE[1][1] = R_REJECTED;
      EKF_RACTIVE[2][2] = R_REJECTED;
    }

    // Normalize magnetometer vector, but reject invalid fields instead of faking a nominal field.
    float normM = sqrt(Y[3][0]*Y[3][0] + Y[4][0]*Y[4][0] + Y[5][0]*Y[5][0]);
    bool magRejected = (normM <= NORM_EPSILON);
    if (!magRejected) {
      Y[3][0] /= normM; Y[4][0] /= normM; Y[5][0] /= normM;
      if (ekfInnovationGateWarmupUpdates >= EKF_INNOVATION_GATE_WARMUP_UPDATES) {
        magRejected = vectorInnovationNorm(Y, predictedY, 3) > MAG_INNOVATION_GATE;
      }
    }
    if (magRejected) {
      Y[3][0] = predictedY[3][0];
      Y[4][0] = predictedY[4][0];
      Y[5][0] = predictedY[5][0];
      EKF_RACTIVE[3][3] = R_REJECTED;
      EKF_RACTIVE[4][4] = R_REJECTED;
      EKF_RACTIVE[5][5] = R_REJECTED;
    }

    // Update the EKF and measure computation time
    Matrix ekfPreviousX = EKF_IMU.GetX();
    Matrix ekfPreviousP = EKF_IMU.GetP();
    EKF_IMU.vSetMeasurementNoise(EKF_RACTIVE);
    u64compuTime = micros();
    if (!EKF_IMU.bUpdate(Y, U)) {
      ++ekfConsecutiveFailures;
      if (ekfConsecutiveFailures >= EKF_MAX_CONSECUTIVE_FAILURES) {
        quaternionData.vSetToZero();
        quaternionData[0][0] = 1.0;
        EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
        ekfConsecutiveFailures = 0;
        ekfInnovationGateWarmupUpdates = 0;
      } else {
        EKF_IMU.vReset(ekfPreviousX, ekfPreviousP, EKF_QINIT, EKF_RINIT);
      }
      // Serial.println("Whoop ");
    } else {
      ekfConsecutiveFailures = 0;
      if (ekfInnovationGateWarmupUpdates < EKF_INNOVATION_GATE_WARMUP_UPDATES) {
        ++ekfInnovationGateWarmupUpdates;
      }
    }
#if FC_TIMING_INSTRUMENTATION
    recordTiming(timingEkf, static_cast<uint32_t>(u64compuTime));
#endif
    u64compuTime = micros() - u64compuTime;
    
    // Convert quaternion to Euler angles
    quaternionData = EKF_IMU.GetX();
    Main_bNormalizeState(quaternionData);
    float q0 = quaternionData[0][0];
    float q1 = quaternionData[1][0];
    float q2 = quaternionData[2][0];
    float q3 = quaternionData[3][0];
    
    // Invert roll sign so right rolls are negative and left rolls are positive
    float roll  = -atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)) * (180.0 / M_PI);
    float pitchArg = clampFloat(2.0*(q0*q2 - q3*q1), -1.0f, 1.0f);
    float pitch = asin(pitchArg) * (180.0 / M_PI);
    float yaw   = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3)) * (180.0 / M_PI);
    // Previously applied calibration offsets have been removed so that
    // raw EKF-derived roll and pitch values are reported directly.
    
    // Cache the most recent attitude in decidegrees so telemetry can be
    // emitted independently of the EKF work.
    latestAttitudeRoll = static_cast<int16_t>(roundf(roll * 10.0f));
    latestAttitudePitch = static_cast<int16_t>(roundf(pitch * 10.0f));
    latestAttitudeYaw = static_cast<int16_t>(roundf(yaw * 10.0f));
    attitudeSampleValid = true;

    serviceCrsfLink();

    const size_t channelCount = sizeof(latestRcChannels.value) / sizeof(latestRcChannels.value[0]);
    const uint32_t servoUpdateUs = micros();
    const bool rcFresh = rcInputFresh(servoUpdateUs);
    const bool rcServoHold = !rcFresh && rcInputWithinServoHold(servoUpdateUs);
    if (lastRcPacketUs != 0) {
      const uint32_t rcAgeUs = rcInputAgeUs(servoUpdateUs);
      if (rcAgeUs > controlDebugCounters.maxRcAgeUs) {
        controlDebugCounters.maxRcAgeUs = rcAgeUs;
      }
    }
    if (rcFresh) {
      ++controlDebugCounters.servoLoopFresh;
    } else {
      ++controlDebugCounters.servoLoopStale;
      if (rcServoHold) {
        ++controlDebugCounters.servoLoopHold;
      }
    }
    if (!rcFresh) {
      if (!rcFailsafeActive) {
        rollPid.reset();
        pitchPid.reset();
        throttlePid.reset();
        rollAngleFilter.reset();
        pitchAngleFilter.reset();
      }
      autoThrottlePercent = 0.0f;
      rcFailsafeActive = true;
      setControlMode(CONTROL_MODE_MANUAL);
      setThrottleMode(THROTTLE_MODE_MANUAL);
    } else {
      rcFailsafeActive = false;
      rcServoHoldBlendActive = false;
    }

    uint16_t rcRollRaw = (channelCount > 0) ? latestRcChannels.value[0] : RC_INPUT_CENTER;
    uint16_t rcPitchRaw = (channelCount > 1) ? latestRcChannels.value[1] : RC_INPUT_CENTER;
    uint16_t rcThrottleRaw = (channelCount > 2) ? latestRcChannels.value[2] : RC_INPUT_MIN;
    uint16_t rcYawRaw = (channelCount > 3) ? latestRcChannels.value[3] : RC_INPUT_CENTER;

    uint16_t rollCommandUs = SERVO_CENTER_US;
    uint16_t pitchCommandUs = SERVO_CENTER_US;
    uint16_t yawCommandUs = rcFresh ? mapRcToUs(rcYawRaw) : SERVO_CENTER_US;
    uint16_t throttleCommandUs = THROTTLE_CUT_US;

    if (!rcFresh) {
      if (rcServoHold) {
        if (!rcServoHoldBlendActive) {
          rcServoHoldStartRollUs = lastRollCommandUs;
          rcServoHoldStartPitchUs = lastPitchCommandUs;
          rcServoHoldStartYawUs = lastYawCommandUs;
          rcServoHoldBlendActive = true;
        }
        rollCommandUs = blendServoTowardNeutral(rcServoHoldStartRollUs, servoUpdateUs);
        pitchCommandUs = blendServoTowardNeutral(rcServoHoldStartPitchUs, servoUpdateUs);
        yawCommandUs = blendServoTowardNeutral(rcServoHoldStartYawUs, servoUpdateUs);
      } else {
        rcServoHoldBlendActive = false;
        rollCommandUs = SERVO_CENTER_US;
        pitchCommandUs = SERVO_CENTER_US;
        yawCommandUs = SERVO_CENTER_US;
      }
    } else if (controlMode == CONTROL_MODE_FLY_BY_WIRE) {
      const float filteredRoll = rollAngleFilter.update(roll, controlDt);
      const float filteredPitch = pitchAngleFilter.update(pitch, controlDt);
      const float rollCommandNorm = mapRcToNormalized(rcRollRaw);
      const float pitchCommandNorm = mapRcToNormalized(rcPitchRaw);

      const float desiredRoll = rollCommandNorm * FBW_MAX_ROLL_ANGLE_DEG;
      const float desiredPitch = pitchCommandNorm * FBW_MAX_PITCH_ANGLE_DEG;

      const float rollPidOutput = rollPid.update(desiredRoll, filteredRoll, controlDt);
      const float pitchPidOutput = pitchPid.update(desiredPitch, filteredPitch, controlDt);

      rollCommandUs = static_cast<uint16_t>(constrain(SERVO_CENTER_US + rollPidOutput,
                                                      static_cast<float>(SERVO_MIN_US),
                                                      static_cast<float>(SERVO_MAX_US)));
      pitchCommandUs = static_cast<uint16_t>(constrain(SERVO_CENTER_US + pitchPidOutput,
                                                       static_cast<float>(SERVO_MIN_US),
                                                       static_cast<float>(SERVO_MAX_US)));
    } else {
      // Manual mode must be a direct RC-to-servo pass-through. Keep the FBW
      // state cleared while Manual is active so attitude-error correction can
      // never bleed into the commanded aileron/elevator outputs.
      rollPid.reset();
      pitchPid.reset();
      rollAngleFilter.reset();
      pitchAngleFilter.reset();
      rollCommandUs = mapRcToUs(rcRollRaw);
      pitchCommandUs = mapRcToUs(rcPitchRaw);
    }

    if (!rcFresh) {
      throttleCommandUs = THROTTLE_CUT_US;
    } else if (throttleMode == THROTTLE_MODE_AUTO) {
      latestAutoThrottleTargetMph = mapRcToAutoThrottleTargetMph(rcThrottleRaw);
      if (!airspeedInputFresh(servoUpdateUs)) {
        throttlePid.reset();
        autoThrottlePercent = max(
            0.0f,
            autoThrottlePercent - (AUTO_THROTTLE_STALE_DECAY_PERCENT_PER_S * controlDt));
      } else {
        float throttleAdjustment = throttlePid.update(
            latestAutoThrottleTargetMph, latestAirspeedMph, controlDt) * controlDt;
        autoThrottlePercent = constrain(autoThrottlePercent + throttleAdjustment, 0.0f, 100.0f);
      }
      throttleCommandUs = mapPercentToThrottleUs(autoThrottlePercent);
    } else {
      throttlePid.reset();
      autoThrottlePercent = mapRcToPercent(rcThrottleRaw);
      throttleCommandUs = mapPercentToThrottleUs(autoThrottlePercent);
    }

    if (shouldUpdateServo(rollCommandUs, lastRollCommandUs, lastRollWriteUs, servoUpdateUs)) {
      servoRoll.writeMicroseconds(rollCommandUs);
      lastRollCommandUs = rollCommandUs;
      lastRollWriteUs = servoUpdateUs;
      ++controlDebugCounters.rollServoWrites;
    }

    if (shouldUpdateServo(pitchCommandUs, lastPitchCommandUs, lastPitchWriteUs, servoUpdateUs)) {
      servoPitch.writeMicroseconds(pitchCommandUs);
      lastPitchCommandUs = pitchCommandUs;
      lastPitchWriteUs = servoUpdateUs;
      ++controlDebugCounters.pitchServoWrites;
    }

    if (shouldUpdateServo(yawCommandUs, lastYawCommandUs, lastYawWriteUs, servoUpdateUs)) {
      servoYaw.writeMicroseconds(yawCommandUs);
      lastYawCommandUs = yawCommandUs;
      lastYawWriteUs = servoUpdateUs;
      ++controlDebugCounters.yawServoWrites;
    }

    if (shouldUpdateServo(throttleCommandUs, lastThrottleCommandUs, lastThrottleWriteUs, servoUpdateUs)) {
      servoThrottle.writeMicroseconds(throttleCommandUs);
      lastThrottleCommandUs = throttleCommandUs;
      lastThrottleWriteUs = servoUpdateUs;
      ++controlDebugCounters.throttleServoWrites;
    }

    // Give CRSF a chance to run immediately after any servo updates in case
    // PWM ISRs added latency.
    serviceCrsfLink();

    uint16_t rc1 = rcRollRaw;
    uint16_t rc2 = rcPitchRaw;
    uint16_t rc3 = (channelCount > 2) ? latestRcChannels.value[2] : RC_INPUT_CENTER;
    uint16_t rc4 = rcYawRaw;
    #if 0 // Temporarily disable detailed debug prints
    // ----- Print all values in one line -----
    Serial.print("Roll: "); Serial.print(roll, 2);
    Serial.print(" | Pitch: "); Serial.print(pitch, 2);
    Serial.print(" | Yaw: "); Serial.print(yaw, 2);
    Serial.print(" | Alt: "); Serial.print(latestAltitudeFeet, 2); Serial.print(" ft");
    Serial.print(" | Airspeed: "); Serial.print(latestAirspeedMph, 2); Serial.print(" mph");
    Serial.print(" | Lon: "); Serial.print(latestLongitude, 6);
    Serial.print(" | Lat: "); Serial.print(latestLatitude, 6);
    Serial.print(" | RC1: "); Serial.print(rc1);
    Serial.print(" RC2: "); Serial.print(rc2);
    Serial.print(" RC3: "); Serial.print(rc3);
    Serial.print(" RC4: "); Serial.print(rc4);
    Serial.print(" | Comp Time: "); Serial.print((float)u64compuTime);
    Serial.print(" µs");
    Serial.print(" | TLM Sent: ");
    if (attitudeTelemetrySentThisLoop) {
      Serial.print("Att");
    }
    if (gpsTelemetrySentThisLoop) {
      if (attitudeTelemetrySentThisLoop) {
        Serial.print("+");
      }
      Serial.print("GPS");
    }
    if (!attitudeTelemetrySentThisLoop && !gpsTelemetrySentThisLoop) {
      Serial.print("None");
    }
    Serial.println();
    #endif
  }

  if (attitudeSampleValid && attitudeTelemetryTimer >= ATTITUDE_TELEMETRY_PERIOD_US) {
    attitudeTelemetryTimer = 0;
    crsf.telemetryWriteAttitude(
        latestAttitudeRoll,
        latestAttitudePitch,
        latestAttitudeYaw);
    serviceCrsfLink();
    attitudeTelemetrySentThisLoop = true;
    ++controlDebugCounters.attitudeTelemetryWrites;
  }

  if (gpsTelemetryTimer >= GPS_TELEMETRY_PERIOD_US) {
    gpsTelemetryTimer = 0;
    // Send GPS Telemetry in CRSF order using the latest cached values:
    // latitude, longitude, altitude, speed, course, satellites
    crsf.telemetryWriteGPS(latestLatitude, latestLongitude, sensorAltitudeCm,
                           airSpeedCms, latestGpsCourse, satsInUse);
    serviceCrsfLink();
    gpsTelemetrySentThisLoop = true;
    ++controlDebugCounters.gpsTelemetryWrites;
  }

  serviceCrsfLink();
  // The once-per-second FCDBG line is long; a backpressured USB serial write
  // can take much longer than a control iteration. Reload right before it so the
  // print always starts with a full watchdog window and normal diagnostic
  // logging cannot trigger a false reset. (No-op cost when logging is disabled.)
  IWatchdog.reload();
  maybePrintControlDebugStats();

#if FC_TIMING_INSTRUMENTATION
  recordTiming(timingLoop, loopStartUs);
  maybePrintTimingStats();
#endif

  (void)attitudeTelemetrySentThisLoop;
  (void)gpsTelemetrySentThisLoop;
}




bool Main_bNormalizeState(Matrix& X)
{
    float_prec quatNorm = sqrt(X[0][0]*X[0][0] + X[1][0]*X[1][0] + X[2][0]*X[2][0] + X[3][0]*X[3][0]);
    if (quatNorm < float_prec(float_prec_ZERO)) {
        return false;
    }
    X[0][0] /= quatNorm;
    X[1][0] /= quatNorm;
    X[2][0] /= quatNorm;
    X[3][0] /= quatNorm;
    return true;
}

bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* State is [quaternion, gyro_bias]. Bias-corrected gyro rates drive
     * quaternion integration; bias is modeled as a random walk and kept
     * constant in the deterministic prediction.
     */
    float_prec q0 = X[0][0];
    float_prec q1 = X[1][0];
    float_prec q2 = X[2][0];
    float_prec q3 = X[3][0];
    float_prec bp = X[4][0];
    float_prec bq = X[5][0];
    float_prec br = X[6][0];

    float_prec p = U[0][0] - bp;
    float_prec q = U[1][0] - bq;
    float_prec r = U[2][0] - br;

    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*gEkfRuntimeDt + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*gEkfRuntimeDt + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*gEkfRuntimeDt + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*gEkfRuntimeDt + q3;
    X_Next[4][0] = bp;
    X_Next[5][0] = bq;
    X_Next[6][0] = br;

    return Main_bNormalizeState(X_Next);
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    float_prec q0 = X[0][0];
    float_prec q1 = X[1][0];
    float_prec q2 = X[2][0];
    float_prec q3 = X[3][0];

    float_prec q0_2 = q0 * q0;
    float_prec q1_2 = q1 * q1;
    float_prec q2_2 = q2 * q2;
    float_prec q3_2 = q3 * q3;

    Y[0][0] = (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;
    Y[1][0] = (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;
    Y[2][0] = (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;

    Y[3][0] = (+(q0_2)+(q1_2)-(q2_2)-(q3_2)) * IMU_MAG_B0[0][0]
             +(2*(q1*q2+q0*q3)) * IMU_MAG_B0[1][0]
             +(2*(q1*q3-q0*q2)) * IMU_MAG_B0[2][0];

    Y[4][0] = (2*(q1*q2-q0*q3)) * IMU_MAG_B0[0][0]
             +(+(q0_2)-(q1_2)+(q2_2)-(q3_2)) * IMU_MAG_B0[1][0]
             +(2*(q2*q3+q0*q1)) * IMU_MAG_B0[2][0];

    Y[5][0] = (2*(q1*q3+q0*q2)) * IMU_MAG_B0[0][0]
             +(2*(q2*q3-q0*q1)) * IMU_MAG_B0[1][0]
             +(+(q0_2)-(q1_2)-(q2_2)+(q3_2)) * IMU_MAG_B0[2][0];

    return true;
}

bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U)
{
    float_prec q0 = X[0][0];
    float_prec q1 = X[1][0];
    float_prec q2 = X[2][0];
    float_prec q3 = X[3][0];
    float_prec p = U[0][0] - X[4][0];
    float_prec q = U[1][0] - X[5][0];
    float_prec r = U[2][0] - X[6][0];

    F.vSetToZero();

    F[0][0] =  1.000;
    F[1][0] =  0.5*p * gEkfRuntimeDt;
    F[2][0] =  0.5*q * gEkfRuntimeDt;
    F[3][0] =  0.5*r * gEkfRuntimeDt;

    F[0][1] = -0.5*p * gEkfRuntimeDt;
    F[1][1] =  1.000;
    F[2][1] = -0.5*r * gEkfRuntimeDt;
    F[3][1] =  0.5*q * gEkfRuntimeDt;

    F[0][2] = -0.5*q * gEkfRuntimeDt;
    F[1][2] =  0.5*r * gEkfRuntimeDt;
    F[2][2] =  1.000;
    F[3][2] = -0.5*p * gEkfRuntimeDt;

    F[0][3] = -0.5*r * gEkfRuntimeDt;
    F[1][3] = -0.5*q * gEkfRuntimeDt;
    F[2][3] =  0.5*p * gEkfRuntimeDt;
    F[3][3] =  1.000;

    F[0][4] =  0.5*q1 * gEkfRuntimeDt;
    F[1][4] = -0.5*q0 * gEkfRuntimeDt;
    F[2][4] = -0.5*q3 * gEkfRuntimeDt;
    F[3][4] =  0.5*q2 * gEkfRuntimeDt;

    F[0][5] =  0.5*q2 * gEkfRuntimeDt;
    F[1][5] =  0.5*q3 * gEkfRuntimeDt;
    F[2][5] = -0.5*q0 * gEkfRuntimeDt;
    F[3][5] = -0.5*q1 * gEkfRuntimeDt;

    F[0][6] =  0.5*q3 * gEkfRuntimeDt;
    F[1][6] = -0.5*q2 * gEkfRuntimeDt;
    F[2][6] =  0.5*q1 * gEkfRuntimeDt;
    F[3][6] = -0.5*q0 * gEkfRuntimeDt;

    F[4][4] = 1.000;
    F[5][5] = 1.000;
    F[6][6] = 1.000;

    return true;
}

bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
    float_prec q0 = X[0][0];
    float_prec q1 = X[1][0];
    float_prec q2 = X[2][0];
    float_prec q3 = X[3][0];

    H.vSetToZero();

    H[0][0] = -2*q2 * IMU_ACC_Z0;
    H[1][0] = +2*q1 * IMU_ACC_Z0;
    H[2][0] = +2*q0 * IMU_ACC_Z0;
    H[3][0] =  2*q0*IMU_MAG_B0[0][0] + 2*q3*IMU_MAG_B0[1][0] - 2*q2*IMU_MAG_B0[2][0];
    H[4][0] = -2*q3*IMU_MAG_B0[0][0] + 2*q0*IMU_MAG_B0[1][0] + 2*q1*IMU_MAG_B0[2][0];
    H[5][0] =  2*q2*IMU_MAG_B0[0][0] - 2*q1*IMU_MAG_B0[1][0] + 2*q0*IMU_MAG_B0[2][0];

    H[0][1] = +2*q3 * IMU_ACC_Z0;
    H[1][1] = +2*q0 * IMU_ACC_Z0;
    H[2][1] = -2*q1 * IMU_ACC_Z0;
    H[3][1] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];
    H[4][1] =  2*q2*IMU_MAG_B0[0][0]-2*q1*IMU_MAG_B0[1][0] + 2*q0*IMU_MAG_B0[2][0];
    H[5][1] =  2*q3*IMU_MAG_B0[0][0]-2*q0*IMU_MAG_B0[1][0] - 2*q1*IMU_MAG_B0[2][0];

    H[0][2] = -2*q0 * IMU_ACC_Z0;
    H[1][2] = +2*q3 * IMU_ACC_Z0;
    H[2][2] = -2*q2 * IMU_ACC_Z0;
    H[3][2] = -2*q2*IMU_MAG_B0[0][0]+2*q1*IMU_MAG_B0[1][0] - 2*q0*IMU_MAG_B0[2][0];
    H[4][2] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];
    H[5][2] =  2*q0*IMU_MAG_B0[0][0]+2*q3*IMU_MAG_B0[1][0] - 2*q2*IMU_MAG_B0[2][0];

    H[0][3] = +2*q1 * IMU_ACC_Z0;
    H[1][3] = +2*q2 * IMU_ACC_Z0;
    H[2][3] = +2*q3 * IMU_ACC_Z0;
    H[3][3] = -2*q3*IMU_MAG_B0[0][0]+2*q0*IMU_MAG_B0[1][0] + 2*q1*IMU_MAG_B0[2][0];
    H[4][3] = -2*q0*IMU_MAG_B0[0][0]-2*q3*IMU_MAG_B0[1][0] + 2*q2*IMU_MAG_B0[2][0];
    H[5][3] =  2*q1*IMU_MAG_B0[0][0]+2*q2*IMU_MAG_B0[1][0] + 2*q3*IMU_MAG_B0[2][0];

    return true;
}


void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
//        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}
