#include "ms5611.h"

MS5611::MS5611(TwoWire* i2c, uint8_t address)
  : _i2c(i2c), _address(address), _ct(1), _uosr(0), sea_level_pressure(1013.25) {
  // Note: The sensor will be reset and PROM read when begin() is called.
}

void MS5611::begin() {
  reset();
  readPROM();
}

void MS5611::reset() {
  _i2c->beginTransmission(_address);
  _i2c->write(0x1E);  // Reset command
  _i2c->endTransmission();
  delay(100);         // Wait for reset to complete
}

void MS5611::readPROM() {
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t reg = 0xA2 + (i * 2);
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    _i2c->endTransmission();
    delay(10);
    _i2c->requestFrom(_address, (uint8_t)2);
    while (_i2c->available() < 2) { }
    uint8_t msb = _i2c->read();
    uint8_t lsb = _i2c->read();
    fc[i] = (msb << 8) | lsb;
  }
}

void MS5611::setOversampling(const String& osr) {
  if (osr == "ULTRA_LOW_POWER") {
    _uosr = 0;
    _ct = 1;
  } else if (osr == "LOW_POWER") {
    _uosr = 2;
    _ct = 2;
  } else if (osr == "STANDARD") {
    _uosr = 4;
    _ct = 3;
  } else if (osr == "HIGH_RES") {
    _uosr = 6;
    _ct = 5;
  } else if (osr == "ULTRA_HIGH_RES") {
    _uosr = 8;
    _ct = 10;
  } else {
    _uosr = 0;
    _ct = 1;
  }
}

void MS5611::setSeaLevelPressure(float p) {
  sea_level_pressure = p;
}

void MS5611::calibrate() {
//  Serial.println("Calibrating barometer bias... Please keep the sensor idle.");
  unsigned long startTime = millis();
  float pressureSum = 0.0;
  uint16_t sampleCount = 0;
  
  // 5-second calibration at 100 Hz (delay 10 ms each)
  while (millis() - startTime < 5000) {
    float p = readPressure(); // call this instance's readPressure method
    pressureSum += p;
    sampleCount++;
    delay(10);
  }
  
  float avgPressure = pressureSum / sampleCount;
  setSeaLevelPressure(avgPressure);
//  Serial.println("Calibration complete. Relative altitude set to 0 ft.");
}

float MS5611::getSeaLevelPressure() {
  return sea_level_pressure;
}

float MS5611::calculateRawPressureMbar(uint32_t raw_pressure) {
  uint32_t D2 = readRawTemperature();
  int32_t dT = (int32_t)D2 - ((uint32_t)fc[4] * 256);
  int64_t OFF = (int64_t)fc[1] * 65536 + ((int64_t)fc[3] * dT) / 128;
  int64_t SENS = (int64_t)fc[0] * 32768 + ((int64_t)fc[2] * dT) / 256;
  // The formula below gives pressure in Pa; divide by 100 to get mbar.
  int32_t P = (((int64_t)raw_pressure * SENS) / 2097152 - OFF) / 32768;
  return P / 100.0;
}

uint32_t MS5611::readRawTemperature() {
  // Send temperature conversion command: 0x50 + _uosr
  _i2c->beginTransmission(_address);
  _i2c->write(0x50 + _uosr);
  _i2c->endTransmission();
  delay(_ct);
  return readRegister24(0x00);
}

uint32_t MS5611::readRawPressure() {
  // Send pressure conversion command: 0x40 + _uosr
  _i2c->beginTransmission(_address);
  _i2c->write(0x40 + _uosr);
  _i2c->endTransmission();
  delay(_ct);
  return readRegister24(0x00);
}

float MS5611::readPressure(bool compensation) {
  uint32_t D1 = readRawPressure();
  uint32_t D2 = readRawTemperature();
  int32_t dT = (int32_t)D2 - ((uint32_t)fc[4] * 256);
  int64_t OFF = (int64_t)fc[1] * 65536 + ((int64_t)fc[3] * dT) / 128;
  int64_t SENS = (int64_t)fc[0] * 32768 + ((int64_t)fc[2] * dT) / 256;

  if (compensation) {
    int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    if (TEMP < 2000) {
      OFF2 = 5 * ((int64_t)(TEMP - 2000) * (TEMP - 2000)) / 2;
      SENS2 = 5 * ((int64_t)(TEMP - 2000) * (TEMP - 2000)) / 4;
      if (TEMP < -1500) {
        OFF2 += 7 * ((int64_t)(TEMP + 1500) * (TEMP + 1500));
        SENS2 += (11 * ((int64_t)(TEMP + 1500) * (TEMP + 1500))) / 2;
      }
      OFF -= OFF2;
      SENS -= SENS2;
    }
  }
  int32_t P = (((int64_t)D1 * SENS) / 2097152 - OFF) / 32768;
  return P / 100.0;
}

float MS5611::readTemperature(bool compensation) {
  uint32_t D2 = readRawTemperature();
  int32_t dT = (int32_t)D2 - ((uint32_t)fc[4] * 256);
  int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;
  if (compensation && TEMP < 2000) {
    TEMP -= ((int64_t)dT * dT) / (1LL << 31);
  }
  return TEMP / 100.0;
}

float MS5611::calibrateSeaLevel(uint16_t duration, float interval) {
//  Serial.println("Starting sea level pressure calibration...");
  float pressureSum = 0;
  uint16_t count = 0;
  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)duration * 1000UL) {
    float pressure = readPressure();
    pressureSum += pressure;
    count++;
//    Serial.print("Reading Pressure: ");
//    Serial.print(pressure, 2);
//    Serial.println(" mbar");
    delay((int)(interval * 1000));
  }
  float avg_pressure = pressureSum / count;
//  Serial.print("Average Pressure: ");
//  Serial.print(avg_pressure, 2);
//  Serial.println(" mbar");
  sea_level_pressure = avg_pressure;
//  Serial.print("Calibration complete. Sea Level Pressure set to ");
//  Serial.print(sea_level_pressure, 2);
//  Serial.println(" mbar");
  return sea_level_pressure;
}

float MS5611::calibrateRelativeAltitude(uint16_t duration, float interval) {
  // Same as calibrateSeaLevel since it sets current reading as sea level.
  return calibrateSeaLevel(duration, interval);
}

float MS5611::getAltitude(float pressure, float sea_level_pressure) {
  return 44330.0 * (1 - pow((pressure / sea_level_pressure), 0.1903));
}

uint32_t MS5611::readRegister24(uint8_t reg) {
  // The ADC conversion result is read from register 0x00.
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  _i2c->endTransmission();
  _i2c->requestFrom(_address, (uint8_t)3);
  while (_i2c->available() < 3) { }
  uint32_t value = ((uint32_t)_i2c->read() << 16) |
                   ((uint32_t)_i2c->read() << 8) |
                   ((uint32_t)_i2c->read());
  return value;
}
