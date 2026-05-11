#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MS5611 {
public:
  /**
   * Constructor.
   * @param i2c Pointer to a TwoWire instance.
   * @param address I2C address of the sensor (default: 0x76).
   */
  MS5611(TwoWire* i2c, uint8_t address = 0x76);

  /**
   * Initialize the sensor by performing a reset and reading the PROM.
   * (Call this after initializing the I2C bus.)
   */
  void begin();

  /**
   * Reset the sensor.
   */
  void reset();

  /**
   * Read the calibration coefficients from PROM.
   */
  void readPROM();
  
  /**
   * Set the sea level pressure used for altitude calculations.
   * @param p Sea level pressure in mbar.
   */
  void setSeaLevelPressure(float p);

  /**
   * calibrate.
   */
  void calibrate();

  /**
   * Get the current sea level pressure used for altitude calculations.
   * @return Sea level pressure in mbar.
   */
  float getSeaLevelPressure();

  /**
   * Set oversampling mode.
   * Valid keys are: "ULTRA_LOW_POWER", "LOW_POWER", "STANDARD", "HIGH_RES", "ULTRA_HIGH_RES".
   * Defaults to ULTRA_LOW_POWER if an unknown key is provided.
   */
  void setOversampling(const String& osr);

  /**
   * Convert a raw pressure reading to mbar (without additional compensation).
   * @param raw_pressure Raw ADC pressure (D1) value.
   * @return Pressure in mbar.
   */
  float calculateRawPressureMbar(uint32_t raw_pressure);

  /**
   * Read raw temperature (D2) from the sensor.
   * @return 24-bit raw temperature value.
   */
  uint32_t readRawTemperature();

  /**
   * Read raw pressure (D1) from the sensor.
   * @return 24-bit raw pressure value.
   */
  uint32_t readRawPressure();

  /**
   * Calculate and return the pressure in mbar.
   * @param compensation Enable second order compensation (default true).
   * @return Pressure in mbar.
   */
  float readPressure(bool compensation = true);

  /**
   * Calculate and return the temperature in Celsius.
   * @param compensation Enable second order compensation (default true).
   * @return Temperature in °C.
   */
  float readTemperature(bool compensation = true);

  /**
   * Calibrate sea level pressure so that altitude reads 0 m.
   * @param duration Duration (in seconds) over which to average readings.
   * @param interval Time (in seconds) between consecutive readings.
   * @return Calculated sea level pressure in mbar.
   */
  float calibrateSeaLevel(uint16_t duration = 10, float interval = 0.1);

  /**
   * Calibrate relative altitude (set current altitude as 0).
   * @param duration Duration (in seconds) over which to average readings.
   * @param interval Time (in seconds) between consecutive readings.
   * @return Calculated sea level pressure in mbar.
   */
  float calibrateRelativeAltitude(uint16_t duration = 10, float interval = 0.1);

  /**
   * Calculate altitude from a measured pressure.
   * @param pressure Measured pressure in mbar.
   * @param sea_level_pressure Reference sea level pressure in mbar (default 1013.25).
   * @return Altitude in meters.
   */
  float getAltitude(float pressure, float sea_level_pressure = 1013.25);

private:
  TwoWire* _i2c;
  uint8_t _address;
  uint8_t _ct;     // Conversion time in ms (depends on oversampling)
  uint8_t _uosr;   // Command offset for oversampling
  uint16_t fc[6];  // Calibration coefficients (C1 to C6)
  float sea_level_pressure;

  /**
   * Read a 24-bit value from the ADC (register 0x00).
   * @return 24-bit result.
   */
  uint32_t readRegister24(uint8_t reg);
};

#endif // MS5611_H
