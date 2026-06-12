#include "ms4525d0.h"
#include <math.h>

namespace {
constexpr uint32_t MS4525D0_I2C_READ_TIMEOUT_US = 2000UL;
}

MS4525D0::MS4525D0(TwoWire &wirePort, uint8_t address)
    : _i2c(wirePort), _address(address), _baselinePressure(0.0f)
{
    // Constructor body (if additional initialization is needed)
}

void MS4525D0::calibrate() {
//    Serial.println("Calibrating airspeed sensor... Please keep the sensor idle.");
    const int numReadings = 10;
    float readings[numReadings];
    int validCount = 0;

    for (int i = 0; i < numReadings; i++) {
        uint16_t raw = 0;
        if (readRawData(raw)) {
            float pressure = convertToPressure(raw);
            if (!isnan(pressure)) {
                readings[validCount++] = pressure;
            }
        }
        delay(100);  // 100 ms delay between readings
    }

    if (validCount > 0) {
        float sum = 0.0f;
        for (int i = 0; i < validCount; i++) {
            sum += readings[i];
        }
        _baselinePressure = sum / validCount;
//        Serial.print("Calibration complete. Baseline pressure: ");
//        Serial.print(_baselinePressure, 4);
//        Serial.println(" Pa");
    } else {
//        Serial.println("Calibration failed. No valid readings.");
    }
}

float MS4525D0::getAirspeed(float ambientPressure) {
    uint16_t raw = 0;
    if (!readRawData(raw)) {
//        Serial.println("MS4525D0: raw read timeout");
        return NAN;
    }

    float pressure = convertToPressure(raw);
    if (isnan(pressure)) {
//        Serial.println("MS4525D0: invalid pressure reading");
        return NAN;
    }
    // Adjust the measured pressure using the calibrated baseline
    float correctedPressure = pressure - _baselinePressure;

    // Use default air density of 1.225 kg/m³ if ambient pressure is not provided
    float airDensity = (ambientPressure > 0.0f) ? calculateAirDensity(ambientPressure) : 1.225f;

    if (correctedPressure > 0.0f) {
        // Calculate airspeed (m/s) from differential pressure (using Bernoulli's principle)
        float airspeedMPS = sqrt(2.0f * correctedPressure / airDensity);
        return airspeedMPS * MPS_TO_MPH;
    } else {
        return 0.0f;
    }
}

bool MS4525D0::readRawData(uint16_t &raw) {
    raw = 0;

    // Bound the wait so a missing sensor or wedged I2C bus cannot stall the
    // flight-control loop indefinitely while servos hold their previous command.
    _i2c.requestFrom(_address, (uint8_t)2);

    const uint32_t startUs = micros();
    while (_i2c.available() < 2) {
        if ((uint32_t)(micros() - startUs) >= MS4525D0_I2C_READ_TIMEOUT_US) {
            while (_i2c.available() > 0) {
                (void)_i2c.read();
            }
//            Serial.println("Error reading raw data");
            return false;
        }
    }

    uint8_t highByte = _i2c.read();
    uint8_t lowByte = _i2c.read();
    raw = ((uint16_t)highByte << 8) | lowByte;
    return true;
}

float MS4525D0::convertToPressure(uint16_t raw) {
    // Check if the raw value is within the valid range
    if (raw < OUTPUT_MIN || raw > OUTPUT_MAX) {
        return NAN;
    }
    // Map raw reading to differential pressure in PSI, then convert to Pascals.
    float pressurePsi = ((float)(raw - OUTPUT_MIN) / (OUTPUT_MAX - OUTPUT_MIN)) *
                        (PRESSURE_MAX - PRESSURE_MIN) + PRESSURE_MIN;
    float pressurePascal = pressurePsi * PSI_TO_PASCAL;
    return pressurePascal;
}

float MS4525D0::calculateAirDensity(float ambientPressure) {
    const float R = 287.05f;      // Specific gas constant for dry air (J/(kg·K))
    const float T0 = 288.15f;     // Standard temperature at sea level (K)
    const float L = 0.0065f;      // Temperature lapse rate (K/m)
    const float P0 = 101325.0f;   // Standard atmospheric pressure at sea level (Pa)

    // Estimate altitude (m) from ambient pressure:
    float altitude = (pow(P0 / ambientPressure, 1.0f / 5.257f) - 1.0f) * (T0 / L);
    // Calculate temperature at this altitude:
    float T = T0 - L * altitude;
    // Compute air density:
    float airDensity = ambientPressure / (R * T);
    return airDensity;
}
