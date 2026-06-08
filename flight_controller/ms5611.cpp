#include "ms5611.h"

namespace {
constexpr uint32_t MS5611_I2C_READ_TIMEOUT_US = 2000UL;
}

MS5611::MS5611(TwoWire* i2c, uint8_t address)
  : _i2c(i2c), _address(address), _ct(1), _uosr(0), sea_level_pressure(1013.25) {
  // Note: The sensor will be reset and PROM read when begin() is called.
}

bool MS5611::begin() {
  reset();
  return readPROM();
}

void MS5611::reset() {
  _i2c->beginTransmission(_address);
  _i2c->write(0x1E);  // Reset command
  _i2c->endTransmission();
  delay(100);         // Wait for reset to complete
}

bool MS5611::readPROM() {
  uint16_t prom[8] = {0};

  for (uint8_t i = 0; i < 8; i++) {
    const uint8_t reg = 0xA0 + (i * 2);
    if (!readPromWord(reg, prom[i])) {
      return false;
    }
  }

  if (!validatePromCrc(prom)) {
    return false;
  }

  for (uint8_t i = 0; i < 6; i++) {
    fc[i] = prom[i + 1];
    if (fc[i] == 0) {
      return false;
    }
  }

  return true;
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

uint8_t MS5611::getConversionTimeMs() const {
  return _ct;
}

void MS5611::startRawTemperatureConversion() {
  _i2c->beginTransmission(_address);
  _i2c->write(0x50 + _uosr);
  _i2c->endTransmission();
}

void MS5611::startRawPressureConversion() {
  _i2c->beginTransmission(_address);
  _i2c->write(0x40 + _uosr);
  _i2c->endTransmission();
}

uint32_t MS5611::readAdc() {
  return readRegister24(0x00);
}

bool MS5611::readAdc(uint32_t& value) {
  return readRegister24(0x00, value);
}

void MS5611::calibrate() {
//  Serial.println("Calibrating barometer bias... Please keep the sensor idle.");
  unsigned long startTime = millis();
  float pressureSum = 0.0;
  uint16_t sampleCount = 0;
  
  // 5-second calibration at 100 Hz (delay 10 ms each)
  while (millis() - startTime < 5000) {
    float p = readPressure(); // call this instance's readPressure method
    if (isfinite(p)) {
      pressureSum += p;
      sampleCount++;
    }
    delay(10);
  }

  if (sampleCount == 0) {
    return;
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
  startRawTemperatureConversion();
  delay(_ct);
  uint32_t rawTemperature = 0;
  return readAdc(rawTemperature) ? rawTemperature : 0;
}

uint32_t MS5611::readRawPressure() {
  startRawPressureConversion();
  delay(_ct);
  uint32_t rawPressure = 0;
  return readAdc(rawPressure) ? rawPressure : 0;
}

float MS5611::calculatePressure(uint32_t raw_pressure, uint32_t raw_temperature, bool compensation) {
  int32_t dT = (int32_t)raw_temperature - ((uint32_t)fc[4] * 256);
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
  int32_t P = (((int64_t)raw_pressure * SENS) / 2097152 - OFF) / 32768;
  return P / 100.0;
}

float MS5611::readPressure(bool compensation) {
  startRawPressureConversion();
  delay(_ct);
  uint32_t D1 = 0;
  if (!readAdc(D1)) {
    return NAN;
  }

  startRawTemperatureConversion();
  delay(_ct);
  uint32_t D2 = 0;
  if (!readAdc(D2)) {
    return NAN;
  }

  return calculatePressure(D1, D2, compensation);
}

float MS5611::readTemperature(bool compensation) {
  startRawTemperatureConversion();
  delay(_ct);
  uint32_t D2 = 0;
  if (!readAdc(D2)) {
    return NAN;
  }
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
    if (isfinite(pressure)) {
      pressureSum += pressure;
      count++;
    }
//    Serial.print("Reading Pressure: ");
//    Serial.print(pressure, 2);
//    Serial.println(" mbar");
    delay((int)(interval * 1000));
  }
  if (count == 0) {
    return sea_level_pressure;
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


bool MS5611::readPromWord(uint8_t reg, uint16_t& value) {
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  if (_i2c->endTransmission() != 0) {
    value = 0;
    return false;
  }

  _i2c->requestFrom(_address, (uint8_t)2);
  const uint32_t startUs = micros();
  while (_i2c->available() < 2) {
    if ((uint32_t)(micros() - startUs) >= MS5611_I2C_READ_TIMEOUT_US) {
      while (_i2c->available() > 0) {
        (void)_i2c->read();
      }
      value = 0;
      return false;
    }
  }

  value = ((uint16_t)_i2c->read() << 8) | (uint16_t)_i2c->read();
  return true;
}

bool MS5611::validatePromCrc(const uint16_t prom[8]) const {
  uint16_t promCopy[8];
  for (uint8_t i = 0; i < 8; i++) {
    promCopy[i] = prom[i];
  }

  const uint8_t expectedCrc = promCopy[7] & 0x0F;
  promCopy[7] &= 0xFF00;

  uint16_t remainder = 0;
  for (uint8_t byteIndex = 0; byteIndex < 16; byteIndex++) {
    if (byteIndex & 1) {
      remainder ^= promCopy[byteIndex >> 1] & 0x00FF;
    } else {
      remainder ^= promCopy[byteIndex >> 1] >> 8;
    }

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (remainder & 0x8000) {
        remainder = (remainder << 1) ^ 0x3000;
      } else {
        remainder <<= 1;
      }
    }
  }

  const uint8_t calculatedCrc = (remainder >> 12) & 0x0F;
  return calculatedCrc == expectedCrc;
}

uint32_t MS5611::readRegister24(uint8_t reg) {
  uint32_t value = 0;
  return readRegister24(reg, value) ? value : 0;
}

bool MS5611::readRegister24(uint8_t reg, uint32_t& value) {
  // The ADC conversion result is read from register 0x00. Bound the wait so
  // a missing sensor or wedged I2C bus cannot stall the flight-control loop
  // indefinitely while servos hold their previous command.
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  _i2c->endTransmission();
  _i2c->requestFrom(_address, (uint8_t)3);

  const uint32_t startUs = micros();
  while (_i2c->available() < 3) {
    if ((uint32_t)(micros() - startUs) >= MS5611_I2C_READ_TIMEOUT_US) {
      while (_i2c->available() > 0) {
        (void)_i2c->read();
      }
      value = 0;
      return false;
    }
  }

  value = ((uint32_t)_i2c->read() << 16) |
          ((uint32_t)_i2c->read() << 8) |
          ((uint32_t)_i2c->read());
  return true;
}
