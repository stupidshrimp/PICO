
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

// Define additional hardware serial ports if the core does not provide them.
// These mappings correspond to the STM32F405 feather board where
// USART3 is on PB11 (RX) / PB10 (TX) and USART6 is on PC7 (RX) / PC6 (TX).
// The core already defines Serial3/Serial6 when the underlying hardware
// exposes USART3/USART6; avoid redefining them to prevent link errors.
#if !defined(USART3)
HardwareSerial Serial3(PB11, PB10);
#endif

#if !defined(USART6)
HardwareSerial Serial6(PC7, PC6);
#endif

// ----- IMU & EKF Variables -----
#define IMU_ACC_Z0  (1)
#ifndef FC_MAG_DECLINATION_RAD
#define FC_MAG_DECLINATION_RAD (0.0f)
#endif
#ifndef FC_MAG_INCLINATION_RAD
#define FC_MAG_INCLINATION_RAD (0.0f)
#endif
float_prec IMU_MAG_B0_data[3] = { cos(FC_MAG_INCLINATION_RAD)*cos(FC_MAG_DECLINATION_RAD), cos(FC_MAG_INCLINATION_RAD)*sin(FC_MAG_DECLINATION_RAD), sin(FC_MAG_INCLINATION_RAD) };
Matrix IMU_MAG_B0(3, 1, IMU_MAG_B0_data);
float_prec HARD_IRON_BIAS_data[3] = { 8.832973, 7.243323, 23.95714 };
Matrix HARD_IRON_BIAS(3, 1, HARD_IRON_BIAS_data);
float_prec SOFT_IRON_MATRIX_data[9] = {
  1.0, 0.0, 0.0,
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0
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
#define ACCEL_INNOVATION_SOFT_NORM (0.25f)
#define ACCEL_INNOVATION_FULL_SCALE_NORM (0.70f)
#define ACCEL_DYNAMIC_R_MAX_SCALE (50.0f)
#define MAG_FIELD_NORM_MIN_UT (15.0f)
#define MAG_FIELD_NORM_MAX_UT (80.0f)
#define MAG_INNOVATION_SOFT_NORM (0.20f)
#define MAG_INNOVATION_FULL_SCALE_NORM (0.60f)
#define MAG_DYNAMIC_R_MAX_SCALE (100.0f)
#define EKF_MAX_CONSECUTIVE_FAILURES (25)
// Threshold to protect against division by zero when normalizing sensor vectors
const float NORM_EPSILON = 1e-6f;
float_prec gEkfRuntimeDt = SS_DT;
uint8_t ekfConsecutiveFailures = 0;
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
constexpr uint16_t SERVO_UPDATE_HYSTERESIS_US = 3;
constexpr uint32_t SERVO_FORCE_REFRESH_PERIOD_US = 100000UL;
constexpr uint32_t RC_FAILSAFE_TIMEOUT_US = 250000UL;
// CRSF parser stalls in the field have shown up as 1-3 second gaps where raw
// bytes still arrive but RC_CHANNELS_PACKED frames do not decode.  Keep the
// last servo pulse during those short dropouts so surfaces do not twitch to
// neutral, but still enter hard failsafe if the link stays quiet.
constexpr uint32_t RC_SERVO_HOLD_TIMEOUT_US = 3000000UL;
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

#ifndef FC_CONTROL_DEBUG_SERIAL_OUTPUT
#define FC_CONTROL_DEBUG_SERIAL_OUTPUT 0
#endif

struct ControlDebugCounters {
  uint32_t rcPackets;
  uint32_t rcFailsafePackets;
  uint32_t ekfUpdates;
  uint32_t servoLoopFresh;
  uint32_t servoLoopStale;
  uint32_t servoLoopHold;
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

float vector3ResidualNorm(const Matrix& a, const Matrix& b, uint8_t startIndex) {
  const float dx = a[startIndex][0] - b[startIndex][0];
  const float dy = a[startIndex + 1][0] - b[startIndex + 1][0];
  const float dz = a[startIndex + 2][0] - b[startIndex + 2][0];
  return sqrtf(dx*dx + dy*dy + dz*dz);
}

float dynamicVarianceScale(float residualNorm, float softNorm, float rejectNorm, float maxScale) {
  if (residualNorm <= softNorm || rejectNorm <= softNorm) {
    return 1.0f;
  }
  const float normalizedResidual = clampFloat((residualNorm - softNorm) / (rejectNorm - softNorm),
                                              0.0f,
                                              1.0f);
  return 1.0f + (maxScale - 1.0f) * normalizedResidual * normalizedResidual;
}

void setMeasurementVarianceTriplet(Matrix& R, uint8_t startIndex, float variance) {
  R[startIndex][startIndex] = variance;
  R[startIndex + 1][startIndex + 1] = variance;
  R[startIndex + 2][startIndex + 2] = variance;
}

void resetControlDebugCounters() {
  controlDebugCounters.rcPackets = 0;
  controlDebugCounters.rcFailsafePackets = 0;
  controlDebugCounters.ekfUpdates = 0;
  controlDebugCounters.servoLoopFresh = 0;
  controlDebugCounters.servoLoopStale = 0;
  controlDebugCounters.servoLoopHold = 0;
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
  const uint32_t holdBlendDurationUs = RC_SERVO_HOLD_TIMEOUT_US - RC_FAILSAFE_TIMEOUT_US;
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
// Instantiate the GPS object on Serial6
M8N gps(Serial6);

constexpr uint32_t GPS_BAUD_CANDIDATES[] = {9600UL, 38400UL, 57600UL, 115200UL, 4800UL};
constexpr uint8_t GPS_BAUD_CANDIDATE_COUNT =
    sizeof(GPS_BAUD_CANDIDATES) / sizeof(GPS_BAUD_CANDIDATES[0]);
constexpr uint32_t GPS_BAUD_DETECT_TIMEOUT_MS = 2500UL;
uint8_t gpsBaudCandidateIndex = 0;
uint32_t gpsBaudLastSwitchMs = 0;
uint32_t gpsValidSentencesAtLastBaudCheck = 0;
uint32_t gpsUbxPvtAtLastBaudCheck = 0;
bool gpsBaudLocked = false;

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
constexpr uint32_t GPS_TELEMETRY_PERIOD_US = 20000;      // 50 Hz, aligned with GPS cache refresh
constexpr uint32_t GPS_DRAIN_PERIOD_US = 20000;          // 50 Hz UART drain/cache refresh
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

void configureGpsBaud(uint8_t candidateIndex, bool announce) {
  gpsBaudCandidateIndex = candidateIndex % GPS_BAUD_CANDIDATE_COUNT;
  const uint32_t baud = GPS_BAUD_CANDIDATES[gpsBaudCandidateIndex];
  Serial6.end();
  delay(20);
  Serial6.begin(baud);
  gps.resetParser(true);
  gpsValidSentencesAtLastBaudCheck = 0;
  gpsUbxPvtAtLastBaudCheck = 0;
  gpsBaudLastSwitchMs = millis();
  gpsBaudLocked = false;
  if (announce) {
    Serial.print("GPS baud probe: ");
    Serial.println(baud);
  }
}

void serviceGpsBaudAutodetect() {
  if (gpsBaudLocked) {
    return;
  }

  if (gps.valid_sentence_count > gpsValidSentencesAtLastBaudCheck ||
      gps.ubx_nav_pvt_count > gpsUbxPvtAtLastBaudCheck) {
    gpsBaudLocked = true;
    Serial.print("GPS data detected at ");
    Serial.print(GPS_BAUD_CANDIDATES[gpsBaudCandidateIndex]);
    Serial.print(" baud via ");
    Serial.println((gps.ubx_nav_pvt_count > gpsUbxPvtAtLastBaudCheck) ? "UBX" : "NMEA");
    return;
  }

  if ((uint32_t)(millis() - gpsBaudLastSwitchMs) < GPS_BAUD_DETECT_TIMEOUT_MS) {
    return;
  }

  configureGpsBaud((gpsBaudCandidateIndex + 1) % GPS_BAUD_CANDIDATE_COUNT, true);
}

void updateGpsCache() {
#if FC_TIMING_INSTRUMENTATION
  uint32_t timingStartUs = micros();
#endif
  gps.gatherData();
  serviceGpsBaudAutodetect();
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

  Serial.print("FCDBG ");
  Serial.print("rc_hz="); Serial.print(controlDebugCounters.rcPackets * scale, 1);
  Serial.print(" rc_failsafe_hz="); Serial.print(controlDebugCounters.rcFailsafePackets * scale, 1);
  Serial.print(" ekf_hz="); Serial.print(controlDebugCounters.ekfUpdates * scale, 1);
  Serial.print(" att_tx_hz="); Serial.print(controlDebugCounters.attitudeTelemetryWrites * scale, 1);
  Serial.print(" gps_tx_hz="); Serial.print(controlDebugCounters.gpsTelemetryWrites * scale, 1);
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
  Serial.print(" gps_baud="); Serial.print(GPS_BAUD_CANDIDATES[gpsBaudCandidateIndex]);
  Serial.print(" gps_lock="); Serial.print(gpsBaudLocked ? 1 : 0);
  Serial.print(" gps_valid="); Serial.print(gps.valid_sentence_count);
  Serial.print(" gps_bad="); Serial.print(gps.checksum_error_count);
  Serial.print(" ubx_pvt="); Serial.print(gps.ubx_nav_pvt_count);
  Serial.print(" ubx_bad="); Serial.print(gps.ubx_checksum_error_count);
  Serial.print(" gps_fix="); Serial.print(gps.has_valid_fix ? 1 : 0);
  Serial.print(" gps_sats="); Serial.print(satsInUse);
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

  // ----- Initialize GPS (Serial6) -----
  Serial6.begin(GPS_BAUD_CANDIDATES[gpsBaudCandidateIndex]);
  gpsBaudLastSwitchMs = millis();
  delay(1000);
  Serial.print("GPS module initialized on USART6 at ");
  Serial.print(GPS_BAUD_CANDIDATES[gpsBaudCandidateIndex]);
  Serial.println(" baud (auto-detect enabled).");

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
  Serial.println("CRSF Telemetry Ready");
}


void loop() {
  ++controlDebugCounters.loopIterations;
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

    // Normalize accelerometer vector. Reject impossible magnitudes, and
    // progressively de-weight plausible-but-surprising gravity directions so
    // maneuver acceleration does not yank the attitude estimate.
    float normG = sqrt(Y[0][0]*Y[0][0] + Y[1][0]*Y[1][0] + Y[2][0]*Y[2][0]);
    const float accelMagnitudeFraction = fabs(normG - GRAVITY_NOMINAL_MSS) /
                                         (GRAVITY_NOMINAL_MSS * ACCEL_NORM_GATE_FRACTION);
    bool accelRejected = !isfinite(normG) ||
                         (normG <= NORM_EPSILON) ||
                         (accelMagnitudeFraction > 1.0f);
    float accelInnovationNorm = 0.0f;
    if (!accelRejected) {
      Y[0][0] /= normG; Y[1][0] /= normG; Y[2][0] /= normG;
      accelInnovationNorm = vector3ResidualNorm(Y, predictedY, 0);
      const float innovationScale = dynamicVarianceScale(accelInnovationNorm,
                                                         ACCEL_INNOVATION_SOFT_NORM,
                                                         ACCEL_INNOVATION_FULL_SCALE_NORM,
                                                         ACCEL_DYNAMIC_R_MAX_SCALE);
      const float magnitudeScale = 1.0f + (ACCEL_DYNAMIC_R_MAX_SCALE - 1.0f) *
                                   clampFloat(accelMagnitudeFraction, 0.0f, 1.0f) *
                                   clampFloat(accelMagnitudeFraction, 0.0f, 1.0f);
      setMeasurementVarianceTriplet(EKF_RACTIVE, 0, R_INIT_ACC * fmaxf(innovationScale, magnitudeScale));
    }
    if (accelRejected) {
      Y[0][0] = predictedY[0][0];
      Y[1][0] = predictedY[1][0];
      Y[2][0] = predictedY[2][0];
      setMeasurementVarianceTriplet(EKF_RACTIVE, 0, R_REJECTED);
    }

    // Normalize magnetometer vector. Reject impossible field magnitudes and
    // dynamically de-weight magnetic innovations before hard rejection by magnitude.
    float normM = sqrt(Y[3][0]*Y[3][0] + Y[4][0]*Y[4][0] + Y[5][0]*Y[5][0]);
    bool magRejected = !isfinite(normM) ||
                       (normM <= NORM_EPSILON) ||
                       (normM < MAG_FIELD_NORM_MIN_UT) ||
                       (normM > MAG_FIELD_NORM_MAX_UT);
    float magInnovationNorm = 0.0f;
    if (!magRejected) {
      Y[3][0] /= normM; Y[4][0] /= normM; Y[5][0] /= normM;
      magInnovationNorm = vector3ResidualNorm(Y, predictedY, 3);
      const float innovationScale = dynamicVarianceScale(magInnovationNorm,
                                                         MAG_INNOVATION_SOFT_NORM,
                                                         MAG_INNOVATION_FULL_SCALE_NORM,
                                                         MAG_DYNAMIC_R_MAX_SCALE);
      setMeasurementVarianceTriplet(EKF_RACTIVE, 3, R_INIT_MAG * innovationScale);
    }
    if (magRejected) {
      Y[3][0] = predictedY[3][0];
      Y[4][0] = predictedY[4][0];
      Y[5][0] = predictedY[5][0];
      setMeasurementVarianceTriplet(EKF_RACTIVE, 3, R_REJECTED);
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
      } else {
        EKF_IMU.vReset(ekfPreviousX, ekfPreviousP, EKF_QINIT, EKF_RINIT);
      }
      // Serial.println("Whoop ");
    } else {
      ekfConsecutiveFailures = 0;
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
