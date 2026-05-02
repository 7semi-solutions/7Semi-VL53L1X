#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "7Semi_I2C_Interface.h"
#include "BusIO_7Semi.h"

#define REG_CHIP_ID                          0x010F
#define REG_CHIP_ID_EXPECTED                 0xEACC  

#define REG_SYSTEM_START                     0x0087 
#define REG_GPIO_TIO_HV_STATUS               0x0031

#define REG_RESULT_DISTANCE                  0x0096  
#define REG_RESULT_AMBIENT                   0x0090  
#define REG_RESULT_STATUS                    0x0089 
#define RESULT_RANGE_STATUS                  0x0089 

#define REG_GPIO_CLEAR                       0x0086  
#define REG_I2C_SLAVE_DEVICE_ADDR            0x0001 

#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI    0x005E
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI    0x0061

#define PHASECAL_CONFIG_TIMEOUT_MACROP       0x004B
#define RANGE_CONFIG_VCSEL_PERIOD_A          0x0060
#define RANGE_CONFIG_VCSEL_PERIOD_B          0x0063
#define RANGE_CONFIG_VALID_PHASE_HIGH        0x0069
#define SD_CONFIG_WOI_SD0                    0x0078
#define SD_CONFIG_INITIAL_PHASE_SD0          0x007A

#define SOFT_RESET                           0x001E

/**
 * Distance mode selection
 * - SHORT: better close range performance
 * - MEDIUM: balanced
 * - LONG: best max range
 */
enum Mode : uint8_t
{
  SHORT = 0,
  MEDIUM = 1,
  LONG = 2,
};

/**S
 * Read status returned by read functions
 * - STATUS_OK: sample read and cache updated
 * - STATUS_TIMEOUT: sample not ready within timeout
 * - STATUS_ERROR: I2C/register access error
 */
enum Status
{
  STATUS_OK = 0,
  STATUS_TIMEOUT,
  STATUS_I2C_ERROR,
  STATUS_NOT_READY,
  STATUS_INVALID_PARAM,
  STATUS_HARDWARE_FAIL
};
/**
 * VL53L1X Range Status
 *
 * - Decoded from RESULT__RANGE_STATUS (0x0089)
 * - Lower 5 bits represent measurement quality
 */
enum RangeStatus : uint8_t
{
  RANGE_VALID            = 0, // Perfect measurement

  RANGE_SIGMA_FAIL       = 1, // Sigma (noise) too high
  RANGE_SIGNAL_FAIL      = 2, // Signal too weak
  RANGE_MIN_RANGE_FAIL   = 3, // Below minimum range 
  RANGE_PHASE_FAIL       = 4, // Phase error
  RANGE_HARDWARE_FAIL    = 5, // Internal error

  RANGE_WRAP_TARGET_FAIL = 7, // Wrapped signal (too far)

  RANGE_VALID_WEAK       = 12,// Valid but degraded (use with caution)
  MIN_RANGE_CLIPPED      = 13,// too close / saturated
  RANGE_UNKNOWN          = 255 
};

class VL53L1X_7Semi
{
public:

  VL53L1X_7Semi();

  ~VL53L1X_7Semi();

  /**
   * Sensor initialization
   *
   * - Initializes I2C interface with given address and clock
   * - Recreates BusIO layer (ensures clean state on re-init)
   * - Performs sensor soft reset sequence
   * - Waits for sensor boot completion using GPIO status
   * - Reads and verifies chip ID (optional strict check)
   * - Loads ST default configuration block
   *
   * Notes
   * - Safe to call multiple times (previous bus instance is deleted)
   * - Soft reset ensures sensor starts from known state
   * - Boot wait uses GPIO__TIO_HV_STATUS bit[1]
   * - Default config is mandatory for proper operation
   *
   * Returns
   * - true  : Sensor initialized successfully
   * - false : I2C error, boot timeout, or config failure
   */
  bool begin(uint8_t i2cAddress = 0x29,
             TwoWire &i2cPort = Wire,
             uint32_t i2cClock = 400000);

  /**
   * Set timeout (ms)
   * - Defines maximum wait time for blocking operations
   * - Used in data-ready polling loops
   */
  void setTimeout(uint32_t timeoutMs);

  /**
   * Get timeout (ms)
   * - Returns currently configured timeout value
   */
  uint32_t getTimeout() const;

  /**
   * Set measurement range mode
   * - Configures sensor for SHORT / MEDIUM / LONG distance
   * - Applies ST recommended register presets
   * - Stops ongoing measurement before applying changes
   */
  bool setMeasurementRange(Mode mode);

  /**
   * Set timing budget (ms)
   * - Controls measurement duration and accuracy
   * - Higher values improve accuracy but reduce speed
   * - Converts ms into sensor macro timing registers
   */
  bool setMeasurementTime(uint16_t timingBudgetMs);

  /**
   * Get timing budget (ms)
   *
   * - Reads encoded timeout values from sensor registers
   * - Decodes macro timing values back to milliseconds
   * - Returns approximate measurement timing budget
   */
  uint16_t getMeasurementTime();

  /**
   * Start continuous measurement
   *
   * - Enables sensor ranging engine
   * - Updates internal running state
   */
  bool startMeasurement();

  /**
   * Stop continuous measurement
   *
   * - Disables sensor ranging engine
   * - Updates internal running state
   */
  bool stopMeasurement();

  /**
   * Blocking read
   *
   * - Ensures measurement is running
   * - Waits until data is ready or timeout occurs
   * - Reads one sample and clears interrupt
   */
  Status readBlocking();

  /**
   * Continuous-read convenience
   * - Compatibility alias for older sketches
   * - Same behavior as readDistance()
   */
  uint16_t readContinous() { return readDistance(); }

  /**
   * Non-blocking read
   *
   * - Checks if data is ready
   * - Reads distance, ambient, and raw status
   * - Updates internal cached values
   */
  Status readOnce();

  /**
   * Read distance (mm)
   *
   * - Performs blocking read internally
   * - Returns last measured distance
   */
  uint16_t readDistance();

  /**
   * Get ambient value
   *
   * - Returns last measured ambient light value
   */
  uint16_t getAmbient() const;

  /**
   * Get range status
   *
   * - Converts raw sensor status into user-friendly enum
   * - Uses ST recommended mapping
   */
  uint8_t getStatus() const;

  /**
   * Change I2C address
   *
   * - Writes new 7-bit address into sensor register
   * - Updates internal address variable
   */
  bool changeI2CAddress(uint8_t newAddress);

  /**
   * Read chip ID
   *
   * - Reads 16-bit IDENTIFICATION__MODEL_ID register
   * - Used to verify correct sensor device
   */
  bool readChipID(uint16_t &chipId);

  /**
   * Check data-ready status
   *
   * - Reads interrupt bits from RESULT__RANGE_STATUS
   * - Indicates if a new measurement is available
   */
  bool isDataReady(bool &ready);

  /**
   * Clear interrupt
   *
   * - Clears data-ready interrupt flag
   * - Required before next measurement cycle
   */
  bool clearInterrupt();

  /**
   * Set interrupt polarity
   *
   * - Configures GPIO interrupt polarity (active high/low)
   */
  bool setInterruptPolarity(bool activeHigh);

  /**
   * Get interrupt polarity
   *
   * - Reads current GPIO interrupt polarity setting
   */
  bool getInterruptPolarity(bool &activeHigh);

  /**
   * Set offset calibration (mm)
   *
   * - Compensates systematic distance error
   * - Stored internally as (offset_mm * 4)
   */
  bool setOffset(int16_t offset_mm);

  /**
   * Get offset calibration (mm)
   *
   * - Reads offset register and converts back to mm
   */
  bool getOffset(int16_t &offset_mm);

private:
  I2C_Interface i2c;
  BusIO_7Semi<I2C_Interface> *bus;

  uint8_t i2c_address = 0x29;
  uint32_t timeout_ms = 100;

  uint16_t distance_mm;
  uint16_t ambient;
  uint8_t range_status;

  bool is_ranging;
  uint8_t distance_range;

  bool waitDataReady(uint32_t timeoutMs);

  bool applyDefaultConfiguration();
};
