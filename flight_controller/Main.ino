
/*************************************************************************************************************
 *  
 * Feather Flight Program
 * 
 * This sketch reads data from an MPU9250, MS5611, and MS4525D0 sensors while updating GPS data from an
 * M8N module. IMU/EKF work and CRSF telemetry cache updates run at ~125 Hz, while slower GPS,
 * barometer, and airspeed sensors are sampled on independent lower-rate timers.
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
elapsedMillis timerEKF;
uint64_t u64compuTime;
char bufferTxSer[100];
char cmd;

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

// Create a CRSFforArduino instance using Serial3.
CRSFforArduino crsf(&Serial3);

// Store the latest received RC channel data.
serialReceiverLayer::rcChannels_t latestRcChannels;

enum ControlMode {
  CONTROL_MODE_MANUAL = 0,
  CONTROL_MODE_FLY_BY_WIRE
};

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
  float alpha;
  float state;
  bool hasState;

  LowPassFilter(float cutoffHz, float dt)
    : alpha(computeAlpha(cutoffHz, dt)), state(0.0f), hasState(false) {}

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

  float update(float input) {
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
  latestRcChannels = *channels;
}

uint16_t mapRcToUs(uint16_t value) {
  const uint16_t outMin = SERVO_MIN_US;
  const uint16_t outMax = SERVO_MAX_US;
  if (value < RC_INPUT_MIN) value = RC_INPUT_MIN;
  if (value > RC_INPUT_MAX) value = RC_INPUT_MAX;
  return (uint16_t)(((uint32_t)(value - RC_INPUT_MIN) * (outMax - outMin)) /
                    (RC_INPUT_MAX - RC_INPUT_MIN) + outMin);
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
  crsf.update();
  updateControlMode();
}

// ----- GPS -----
// Instantiate the GPS object on Serial6
M8N gps(Serial6);

// Global variables to store the latest GPS data
double latestLatitude  = 0;
double latestLongitude = 0;
uint8_t satsInUse      = 0;       // GPS satellites currently in use

// Telemetry values prepared for CRSF GPS frame. The GPS CRSF frame uses the
// latest cached GPS coordinates plus separately sampled airspeed/barometer data.
float airSpeedCms      = 0.0f; // Airspeed from sensor in centimeters per second
float sensorAltitudeCm = 0.0f; // Altitude from barometer in centimeters
float latestAirspeedMph = 0.0f;
float latestAltitudeFeet = 0.0f;

// ----- Sensor and telemetry timing -----
elapsedMicros attitudeTelemetryTimer;
elapsedMicros gpsTelemetryTimer;
elapsedMicros gpsReadTimer;
elapsedMicros barometerTimer;
elapsedMicros airspeedTimer;
constexpr uint32_t ATTITUDE_TELEMETRY_PERIOD_US = 8000;  // 125 Hz
constexpr uint32_t GPS_TELEMETRY_PERIOD_US = 8000;       // 125 Hz, using cached GPS/airdata values
constexpr uint32_t GPS_READ_PERIOD_US = 200000;          // 5 Hz hardware read/cache refresh
constexpr uint32_t BAROMETER_PERIOD_US = 16667;          // ~60 Hz hardware read/cache refresh
constexpr uint32_t AIRSPEED_PERIOD_US = 16667;           // ~60 Hz hardware read/cache refresh

struct AttitudeTelemetryFrame {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
};

AttitudeTelemetryFrame latestAttitudeFrame = {0, 0, 0};
bool attitudeSampleValid = false;

void updateGpsCache() {
  gps.gatherData();
  latestLatitude = gps.latitude;
  latestLongitude = gps.longitude;
  satsInUse = gps.satellites_in_use;
}

void updateBarometerCache() {
  const float baroPressure = barometer.readPressure();
  const float altitudeMeters = barometer.getAltitude(baroPressure, barometer.getSeaLevelPressure());
  sensorAltitudeCm = altitudeMeters * 100.0f;
  latestAltitudeFeet = altitudeMeters * 3.28084f;
}

void updateAirspeedCache() {
  float airspeedMph = airspeedSensor.getAirspeed();
  if (isnan(airspeedMph)) {
    // Serial.println("Airspeed sensor error");
    airspeedMph = 0.0f;
  }
  latestAirspeedMph = airspeedMph;
  airSpeedCms = airspeedMph * 44.704f;   // mph to cm/s
}



void setup() {
  // ----- Initialize Debug Serial -----
  Serial.begin(115200);
  // Allow time for a serial connection, but don't block startup
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart < 3000)) {
    delay(10);
  }

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

  // ----- Initialize Servo Outputs -----
  servoRoll.attach(A1);
  servoPitch.attach(A2);
  servoYaw.attach(A3);
  servoRoll.writeMicroseconds(SERVO_CENTER_US);
  servoPitch.writeMicroseconds(SERVO_CENTER_US);
  servoYaw.writeMicroseconds(SERVO_CENTER_US);
  lastRollCommandUs = SERVO_CENTER_US;
  lastPitchCommandUs = SERVO_CENTER_US;
  lastYawCommandUs = SERVO_CENTER_US;

  for (size_t i = 0; i < (sizeof(latestRcChannels.value) / sizeof(latestRcChannels.value[0])); ++i) {
    latestRcChannels.value[i] = RC_INPUT_CENTER;
  }

  // ----- Initialize GPS (Serial6) -----
  Serial6.begin(9600);
  delay(1000);
  Serial.println("GPS module initialized on USART6.");

  // Prime slow-sensor caches so the first GPS telemetry frames do not carry
  // default airspeed/altitude values while waiting for their first timers.
  updateBarometerCache();
  updateAirspeedCache();
  updateGpsCache();

  // ----- Initialize CRSF Telemetry -----
  // Use a baud rate of 921600 as required.
  if (!crsf.begin(921600)) {
    Serial.println("CRSF for Arduino initialization failed!");
    while (1) { ; }
  }
  crsf.setRcChannelsCallback(rcChannelsCallback);
  Serial.println("CRSF Telemetry Ready");
}


void loop() {
  serviceCrsfLink();

  bool attitudeTelemetrySentThisLoop = false;
  bool gpsTelemetrySentThisLoop = false;

  if (barometerTimer >= BAROMETER_PERIOD_US) {
    barometerTimer -= BAROMETER_PERIOD_US;
    updateBarometerCache();
    serviceCrsfLink();
  }

  if (airspeedTimer >= AIRSPEED_PERIOD_US) {
    airspeedTimer -= AIRSPEED_PERIOD_US;
    updateAirspeedCache();
    serviceCrsfLink();
  }

  if (gpsReadTimer >= GPS_READ_PERIOD_US) {
    gpsReadTimer -= GPS_READ_PERIOD_US;
    updateGpsCache();
    serviceCrsfLink();
  }

  if (attitudeSampleValid && attitudeTelemetryTimer >= ATTITUDE_TELEMETRY_PERIOD_US) {
    attitudeTelemetryTimer -= ATTITUDE_TELEMETRY_PERIOD_US;
    crsf.telemetryWriteAttitude(
        latestAttitudeFrame.roll,
        latestAttitudeFrame.pitch,
        latestAttitudeFrame.yaw);
    serviceCrsfLink();
    attitudeTelemetrySentThisLoop = true;
  }

  if (gpsTelemetryTimer >= GPS_TELEMETRY_PERIOD_US) {
    gpsTelemetryTimer -= GPS_TELEMETRY_PERIOD_US;
    // Send GPS Telemetry in CRSF order using the latest cached values:
    // latitude, longitude, altitude, speed, course, satellites
    crsf.telemetryWriteGPS(latestLatitude, latestLongitude, sensorAltitudeCm,
                           airSpeedCms, gps.course, satsInUse);
    serviceCrsfLink();
    gpsTelemetrySentThisLoop = true;
  }

  // ----- Sensor Fusion, EKF, and Control Update (125 Hz) -----
  if (timerEKF >= SS_DT_MILIS) {
    timerEKF -= SS_DT_MILIS;
    
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
    u64compuTime = micros();
    if (!EKF_IMU.bUpdate(Y, U)) {
      quaternionData.vSetToZero();
      quaternionData[0][0] = 1.0;
      EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
      // Serial.println("Whoop ");
    }
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
    float filteredRoll = rollAngleFilter.update(roll);
    float filteredPitch = pitchAngleFilter.update(pitch);
    // Previously applied calibration offsets have been removed so that
    // raw EKF-derived roll and pitch values are reported directly.
    
    // Cache the most recent attitude in decidegrees so telemetry can be
    // emitted independently of the EKF work.
    latestAttitudeFrame.roll = static_cast<int16_t>(roundf(roll * 10.0f));
    latestAttitudeFrame.pitch = static_cast<int16_t>(roundf(pitch * 10.0f));
    latestAttitudeFrame.yaw = static_cast<int16_t>(roundf(yaw * 10.0f));
    attitudeSampleValid = true;

    serviceCrsfLink();

    const size_t channelCount = sizeof(latestRcChannels.value) / sizeof(latestRcChannels.value[0]);
    uint16_t rcRollRaw = (channelCount > 0) ? latestRcChannels.value[0] : RC_INPUT_CENTER;
    uint16_t rcPitchRaw = (channelCount > 1) ? latestRcChannels.value[1] : RC_INPUT_CENTER;
    uint16_t rcYawRaw = (channelCount > 3) ? latestRcChannels.value[3] : RC_INPUT_CENTER;

    uint16_t rollCommandUs = SERVO_CENTER_US;
    uint16_t pitchCommandUs = SERVO_CENTER_US;
    uint16_t yawCommandUs = mapRcToUs(rcYawRaw);

    if (controlMode == CONTROL_MODE_FLY_BY_WIRE) {
      float rollCommandNorm = mapRcToNormalized(rcRollRaw);
      float pitchCommandNorm = mapRcToNormalized(rcPitchRaw);

      float desiredRoll = rollCommandNorm * FBW_MAX_ROLL_ANGLE_DEG;
      float desiredPitch = pitchCommandNorm * FBW_MAX_PITCH_ANGLE_DEG;

      float rollPidOutput = rollPid.update(desiredRoll, filteredRoll, SS_DT);
      float pitchPidOutput = pitchPid.update(desiredPitch, filteredPitch, SS_DT);

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

    if (rollCommandUs != lastRollCommandUs) {
      servoRoll.writeMicroseconds(rollCommandUs);
      lastRollCommandUs = rollCommandUs;
    }

    if (pitchCommandUs != lastPitchCommandUs) {
      servoPitch.writeMicroseconds(pitchCommandUs);
      lastPitchCommandUs = pitchCommandUs;
    }

    if (yawCommandUs != lastYawCommandUs) {
      servoYaw.writeMicroseconds(yawCommandUs);
      lastYawCommandUs = yawCommandUs;
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

  serviceCrsfLink();

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
    
    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*SS_DT + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*SS_DT + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*SS_DT + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*SS_DT + q3;
    
    
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
    F[1][0] =  0.5*p * SS_DT;
    F[2][0] =  0.5*q * SS_DT;
    F[3][0] =  0.5*r * SS_DT;

    F[0][1] = -0.5*p * SS_DT;
    F[1][1] =  1.000;
    F[2][1] = -0.5*r * SS_DT;
    F[3][1] =  0.5*q * SS_DT;

    F[0][2] = -0.5*q * SS_DT;
    F[1][2] =  0.5*r * SS_DT;
    F[2][2] =  1.000;
    F[3][2] = -0.5*p * SS_DT;

    F[0][3] = -0.5*r * SS_DT;
    F[1][3] = -0.5*q * SS_DT;
    F[2][3] =  0.5*p * SS_DT;
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
