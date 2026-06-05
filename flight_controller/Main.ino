
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
float_prec IMU_MAG_B0_data[3] = { cos(0), sin(0), 0.0 };
Matrix IMU_MAG_B0(3, 1, IMU_MAG_B0_data);
float_prec HARD_IRON_BIAS_data[3] = { 8.832973, 7.243323, 23.95714 };
Matrix HARD_IRON_BIAS(3, 1, HARD_IRON_BIAS_data);

// EKF initialization constants and matrices (values defined in konfig.h)
#define P_INIT      (10.)
#define Q_INIT      (1e-6)
#define R_INIT_ACC  (0.0015/10.)
#define R_INIT_MAG  (0.0015/10.)
// Threshold to protect against division by zero when normalizing sensor vectors
const float NORM_EPSILON = 1e-6f;
float_prec gEkfRuntimeDt = SS_DT;
float_prec EKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {
  P_INIT, 0, 0, 0,
  0, P_INIT, 0, 0,
  0, 0, P_INIT, 0,
  0, 0, 0, P_INIT
};
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
float_prec EKF_QINIT_data[SS_X_LEN*SS_X_LEN] = {
  Q_INIT, 0, 0, 0,
  0, Q_INIT, 0, 0,
  0, 0, Q_INIT, 0,
  0, 0, 0, Q_INIT
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

// Nonlinear update and Jacobian functions (assumed implemented)
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);

// EKF state variables
Matrix quaternionData(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
EKF EKF_IMU(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT,
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY,
            Main_bCalcJacobianF, Main_bCalcJacobianH);

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
// Roll  (channel 1) -> A1
// Pitch (channel 2) -> A2
// Yaw   (channel 4) -> A3
Servo servoRoll;
Servo servoPitch;
Servo servoYaw;

// Cache the last commanded servo pulse widths so we only update hardware
// when values change. This reduces Servo library ISR load and helps keep
// telemetry timing stable.
uint16_t lastRollCommandUs = 0;
uint16_t lastPitchCommandUs = 0;
uint16_t lastYawCommandUs = 0;
uint32_t lastRollWriteUs = 0;
uint32_t lastPitchWriteUs = 0;
uint32_t lastYawWriteUs = 0;
uint32_t lastControlUpdateUs = 0;
uint32_t lastRcPacketUs = 0;
bool rcReceiverFailsafeActive = true;
bool rcFailsafeActive = true;

#ifndef FC_CONTROL_DEBUG_SERIAL_OUTPUT
#define FC_CONTROL_DEBUG_SERIAL_OUTPUT 1
#endif

struct ControlDebugCounters {
  uint32_t rcPackets;
  uint32_t rcFailsafePackets;
  uint32_t ekfUpdates;
  uint32_t servoLoopFresh;
  uint32_t servoLoopStale;
  uint32_t rollServoWrites;
  uint32_t pitchServoWrites;
  uint32_t yawServoWrites;
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

void resetControlDebugCounters() {
  controlDebugCounters.rcPackets = 0;
  controlDebugCounters.rcFailsafePackets = 0;
  controlDebugCounters.ekfUpdates = 0;
  controlDebugCounters.servoLoopFresh = 0;
  controlDebugCounters.servoLoopStale = 0;
  controlDebugCounters.rollServoWrites = 0;
  controlDebugCounters.pitchServoWrites = 0;
  controlDebugCounters.yawServoWrites = 0;
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

const uint16_t RC_INPUT_MIN = 172;
const uint16_t RC_INPUT_MAX = 1811;
const uint16_t RC_INPUT_CENTER = (RC_INPUT_MIN + RC_INPUT_MAX) / 2;

// Mode channel targets from the ground station (channel 5) and a guard band to avoid chatter.
const uint16_t CONTROL_MODE_MANUAL_TARGET = 400;
const uint16_t CONTROL_MODE_FLY_BY_WIRE_TARGET = 1700;
const uint16_t CONTROL_MODE_SWITCH_DEADBAND = 150;

const uint16_t CONTROL_MODE_MANUAL_MAX = CONTROL_MODE_MANUAL_TARGET + CONTROL_MODE_SWITCH_DEADBAND;
const uint16_t CONTROL_MODE_FLY_BY_WIRE_MIN = CONTROL_MODE_FLY_BY_WIRE_TARGET - CONTROL_MODE_SWITCH_DEADBAND;

const uint16_t SERVO_MIN_US = 1000;
const uint16_t SERVO_MAX_US = 2000;
const uint16_t SERVO_CENTER_US = 1500;
const uint16_t SERVO_HALF_TRAVEL_US = (SERVO_MAX_US - SERVO_MIN_US) / 2;
const uint16_t SERVO_CALIBRATION_ACTIVE_US = SERVO_CENTER_US + ((SERVO_HALF_TRAVEL_US * 9) / 10);
const uint16_t SERVO_INDICATOR_HOLD_MS = 350;

// Fly-by-wire tuning constants.
const float FBW_MAX_ROLL_ANGLE_DEG = 45.0f;
const float FBW_MAX_PITCH_ANGLE_DEG = 30.0f;
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

void initializeServoOutputs() {
  servoRoll.attach(A1);
  servoPitch.attach(A2);
  servoYaw.attach(A3);

  writeRollPitchIndicator(SERVO_CENTER_US);
  servoYaw.writeMicroseconds(SERVO_CENTER_US);
  lastYawCommandUs = SERVO_CENTER_US;
  lastYawWriteUs = lastRollWriteUs;
}

void signalCalibrationActive() {
  writeRollPitchIndicator(SERVO_CALIBRATION_ACTIVE_US);
}

void signalCalibrationComplete() {
  writeRollPitchIndicator(SERVO_MIN_US);
  delay(SERVO_INDICATOR_HOLD_MS);
  writeRollPitchIndicator(SERVO_MAX_US);
  delay(SERVO_INDICATOR_HOLD_MS);
  writeRollPitchIndicator(SERVO_CENTER_US);
}

bool rcInputFresh(uint32_t nowUs) {
  return lastRcPacketUs != 0 &&
         (uint32_t)(nowUs - lastRcPacketUs) <= RC_FAILSAFE_TIMEOUT_US;
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

void updateControlMode() {
  const size_t modeChannelIndex = 4;
  const size_t channelCount = sizeof(latestRcChannels.value) / sizeof(latestRcChannels.value[0]);
  if (modeChannelIndex >= channelCount) {
    return;
  }
  uint16_t modeValue = latestRcChannels.value[modeChannelIndex];
  if (modeValue <= CONTROL_MODE_MANUAL_MAX) {
    setControlMode(CONTROL_MODE_MANUAL);
  } else if (modeValue >= CONTROL_MODE_FLY_BY_WIRE_MIN) {
    setControlMode(CONTROL_MODE_FLY_BY_WIRE);
  }
}

void serviceCrsfLink() {
#if FC_TIMING_INSTRUMENTATION
  uint32_t timingStartUs = micros();
#endif
  crsf.update();
  const serialReceiverLayer::serialReceiverDiagnostics_t crsfDiagnostics = crsf.getDiagnostics();
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
}

// ----- GPS -----
// Instantiate the GPS object on Serial6
M8N gps(Serial6);

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
  const float altitudeMeters = barometer.getAltitude(baroPressure, barometer.getSeaLevelPressure());
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
        barometerRawPressure = barometer.readAdc();
        if (barometerTemperatureValid) {
          applyBarometerPressure(barometer.calculatePressure(barometerRawPressure, barometerRawTemperature));
        }
        barometerReadState = BAROMETER_IDLE;
        barometerDidWork = true;
      }
      break;

    case BAROMETER_WAIT_TEMPERATURE:
      if ((uint32_t)(nowUs - barometerConversionStartUs) >= conversionWaitUs) {
        barometerRawTemperature = barometer.readAdc();
        lastBarometerTemperatureUs = micros();
        barometerTemperatureValid = true;
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
  }
  latestAirspeedMph = airspeedMph;
  airSpeedCms = airspeedMph * 44.704f;   // mph to cm/s
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
  Serial.print(" servo_writes_hz=");
  Serial.print(controlDebugCounters.rollServoWrites * scale, 1); Serial.print('/');
  Serial.print(controlDebugCounters.pitchServoWrites * scale, 1); Serial.print('/');
  Serial.print(controlDebugCounters.yawServoWrites * scale, 1);
  Serial.print(" crsf_service_hz="); Serial.print(controlDebugCounters.crsfServiceCalls * scale, 1);
  Serial.print(" loop_hz="); Serial.print(controlDebugCounters.loopIterations * scale, 1);
  Serial.print(" rc_age_ms="); Serial.print(currentRcAgeUs / 1000.0f, 1);
  Serial.print(" rc_max_age_ms="); Serial.print(maxRcAgeUs / 1000.0f, 1);
  Serial.print(" rc_fresh="); Serial.print(rcInputFresh(nowUs) ? 1 : 0);
  Serial.print(" rx_failsafe="); Serial.print(rcReceiverFailsafeActive ? 1 : 0);
  Serial.print(" mode="); Serial.println(controlMode == CONTROL_MODE_FLY_BY_WIRE ? "FBW" : "MANUAL");

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

  // Move the ailerons and elevator before sensor calibration begins so the
  // pilot has a visible indication that startup calibration is in progress.
  signalCalibrationActive();

  // ----- Initialize I2C -----
  I2C_Alternate.begin();
  I2C_Alternate.setClock(400000);

  // ----- Calibrate Barometer -----
  barometer.begin();
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
    while (1) {}
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
  Serial6.begin(9600);
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
    while (1) { ; }
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
    
    // Compensate for magnetometer hard-iron bias
    Y[3][0] -= HARD_IRON_BIAS[0][0];
    Y[4][0] -= HARD_IRON_BIAS[1][0];
    Y[5][0] -= HARD_IRON_BIAS[2][0];
    
    // Normalize accelerometer vector, guard against near-zero norms
    float normG = sqrt(Y[0][0]*Y[0][0] + Y[1][0]*Y[1][0] + Y[2][0]*Y[2][0]);
    if (normG > NORM_EPSILON) {
      Y[0][0] /= normG; Y[1][0] /= normG; Y[2][0] /= normG;
    } else {
      // Serial.println("Warning: Accelerometer norm below threshold; using default vector");
      Y[0][0] = 0.0f; Y[1][0] = 0.0f; Y[2][0] = 1.0f;
    }
    // Normalize magnetometer vector, guard against near-zero norms
    float normM = sqrt(Y[3][0]*Y[3][0] + Y[4][0]*Y[4][0] + Y[5][0]*Y[5][0]);
    if (normM > NORM_EPSILON) {
      Y[3][0] /= normM; Y[4][0] /= normM; Y[5][0] /= normM;
    } else {
      // Serial.println("Warning: Magnetometer norm below threshold; using default vector");
      Y[3][0] = IMU_MAG_B0[0][0];
      Y[4][0] = IMU_MAG_B0[1][0];
      Y[5][0] = IMU_MAG_B0[2][0];
    }
    
    // Update the EKF and measure computation time
    gEkfRuntimeDt = static_cast<float_prec>(controlDt);
    u64compuTime = micros();
    if (!EKF_IMU.bUpdate(Y, U)) {
      quaternionData.vSetToZero();
      quaternionData[0][0] = 1.0;
      EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
      // Serial.println("Whoop ");
    }
#if FC_TIMING_INSTRUMENTATION
    recordTiming(timingEkf, static_cast<uint32_t>(u64compuTime));
#endif
    u64compuTime = micros() - u64compuTime;
    
    // Convert quaternion to Euler angles
    quaternionData = EKF_IMU.GetX();
    float q0 = quaternionData[0][0];
    float q1 = quaternionData[1][0];
    float q2 = quaternionData[2][0];
    float q3 = quaternionData[3][0];
    
    // Invert roll sign so right rolls are negative and left rolls are positive
    float roll  = -atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)) * (180.0 / M_PI);
    float pitch = asin(2.0*(q0*q2 - q3*q1)) * (180.0 / M_PI);
    float yaw   = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3)) * (180.0 / M_PI);
    float filteredRoll = rollAngleFilter.update(roll, controlDt);
    float filteredPitch = pitchAngleFilter.update(pitch, controlDt);
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
    if (lastRcPacketUs != 0) {
      const uint32_t rcAgeUs = static_cast<uint32_t>(servoUpdateUs - lastRcPacketUs);
      if (rcAgeUs > controlDebugCounters.maxRcAgeUs) {
        controlDebugCounters.maxRcAgeUs = rcAgeUs;
      }
    }
    if (rcFresh) {
      ++controlDebugCounters.servoLoopFresh;
    } else {
      ++controlDebugCounters.servoLoopStale;
    }
    if (!rcFresh) {
      if (!rcFailsafeActive) {
        rollPid.reset();
        pitchPid.reset();
        rollAngleFilter.reset();
        pitchAngleFilter.reset();
      }
      rcFailsafeActive = true;
      setControlMode(CONTROL_MODE_MANUAL);
    } else {
      rcFailsafeActive = false;
    }

    uint16_t rcRollRaw = (channelCount > 0) ? latestRcChannels.value[0] : RC_INPUT_CENTER;
    uint16_t rcPitchRaw = (channelCount > 1) ? latestRcChannels.value[1] : RC_INPUT_CENTER;
    uint16_t rcYawRaw = (channelCount > 3) ? latestRcChannels.value[3] : RC_INPUT_CENTER;

    uint16_t rollCommandUs = SERVO_CENTER_US;
    uint16_t pitchCommandUs = SERVO_CENTER_US;
    uint16_t yawCommandUs = rcFresh ? mapRcToUs(rcYawRaw) : SERVO_CENTER_US;

    if (!rcFresh) {
      rollCommandUs = SERVO_CENTER_US;
      pitchCommandUs = SERVO_CENTER_US;
    } else if (controlMode == CONTROL_MODE_FLY_BY_WIRE) {
      float rollCommandNorm = mapRcToNormalized(rcRollRaw);
      float pitchCommandNorm = mapRcToNormalized(rcPitchRaw);

      float desiredRoll = rollCommandNorm * FBW_MAX_ROLL_ANGLE_DEG;
      float desiredPitch = pitchCommandNorm * FBW_MAX_PITCH_ANGLE_DEG;

      float rollPidOutput = rollPid.update(desiredRoll, filteredRoll, controlDt);
      float pitchPidOutput = pitchPid.update(desiredPitch, filteredPitch, controlDt);

      rollCommandUs = static_cast<uint16_t>(constrain(SERVO_CENTER_US + rollPidOutput,
                                                      static_cast<float>(SERVO_MIN_US),
                                                      static_cast<float>(SERVO_MAX_US)));
      pitchCommandUs = static_cast<uint16_t>(constrain(SERVO_CENTER_US + pitchPidOutput,
                                                       static_cast<float>(SERVO_MIN_US),
                                                       static_cast<float>(SERVO_MAX_US)));
    } else {
      rollCommandUs = mapRcToUs(rcRollRaw);
      pitchCommandUs = mapRcToUs(rcPitchRaw);
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




bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     * The quaternion update function:
     *  q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
     *  q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
     *  q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
     *  q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )
     * 
     * Euler method for integration:
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec q0, q1, q2, q3;
    float_prec p, q, r;
    
    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
    p = U[0][0];
    q = U[1][0];
    r = U[2][0];
    
    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*gEkfRuntimeDt + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*gEkfRuntimeDt + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*gEkfRuntimeDt + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*gEkfRuntimeDt + q3;
    
    
    /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
    if (!X_Next.bNormVector()) {
        /* System error, return false so we can reset the UKF */
        return false;
    }
    
    return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;
    float_prec q0_2, q1_2, q2_2, q3_2;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];

    q0_2 = q0 * q0;
    q1_2 = q1 * q1;
    q2_2 = q2 * q2;
    q3_2 = q3 * q3;
    
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
    /* In Main_bUpdateNonlinearX():
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec p, q, r;

    p = U[0][0];
    q = U[1][0];
    r = U[2][0];

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
    
    return true;
}

bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
    /* In Main_bUpdateNonlinearY():
     * 
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 -g]            --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];
    
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
