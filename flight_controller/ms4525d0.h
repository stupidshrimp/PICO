#ifndef MS4525D0_H
#define MS4525D0_H

#include <Arduino.h>
#include <Wire.h>

class MS4525D0 {
public:
    // Conversion factors and sensor constants
    static constexpr float PSI_TO_PASCAL = 6894.76f; // PSI to Pascal conversion factor
    static constexpr uint16_t OUTPUT_MIN = 1638;       // Minimum raw output value (from datasheet)
    static constexpr uint16_t OUTPUT_MAX = 14745;      // Maximum raw output value (from datasheet)
    static constexpr float PRESSURE_MIN = -1.0f;         // Minimum differential pressure (PSI)
    static constexpr float PRESSURE_MAX = 1.0f;          // Maximum differential pressure (PSI)
    static constexpr float MPS_TO_MPH = 2.23694f;        // Conversion factor from m/s to mph

    /**
     * Constructor.
     * @param wirePort Reference to a TwoWire object (I2C bus).
     * @param address I2C address of the sensor (default: 0x28).
     */
    MS4525D0(TwoWire &wirePort, uint8_t address = 0x28);

    /**
     * Calibrate the sensor to establish a baseline pressure.
     * Takes 10 readings (with 100 ms delay between each) and averages them.
     */
    void calibrate();

    /**
     * Get the airspeed in mph.
     * Optionally, provide an ambient static pressure (in Pa) to calculate air density.
     * If ambientPressure is zero, a default air density of 1.225 kg/m³ is used.
     *
     * @param ambientPressure Ambient static pressure in Pascals (optional).
     * @return Airspeed in mph, or NAN if the pressure reading is invalid.
     */
    float getAirspeed(float ambientPressure = 0.0f);

private:
    TwoWire &_i2c;
    uint8_t _address;
    float _baselinePressure;  // Baseline pressure from calibration (Pa)

    /**
     * Read 2 bytes of raw sensor data.
     * @param raw Receives the 16-bit sensor value when the read succeeds.
     * @return true when 2 bytes were read before the timeout, false otherwise.
     */
    bool readRawData(uint16_t &raw);

    /**
     * Convert a raw reading to differential pressure in Pascals.
     * @param raw Raw 16-bit sensor value.
     * @return Differential pressure in Pascals, or NAN if out of range.
     */
    float convertToPressure(uint16_t raw);

    /**
     * Calculate air density using ambient pressure.
     * Uses standard atmospheric constants.
     *
     * @param ambientPressure Ambient static pressure in Pascals.
     * @return Air density in kg/m³.
     */
    float calculateAirDensity(float ambientPressure);
};

#endif  // MS4525D0_H
