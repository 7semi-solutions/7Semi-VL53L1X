#include "7Semi_VL53L1X.h"


VL53L1X_7Semi::VL53L1X_7Semi() : bus(nullptr) {
  
}

VL53L1X_7Semi::~VL53L1X_7Semi()
{
    delete bus;
}

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
bool VL53L1X_7Semi::begin(uint8_t i2cAddress,
                         TwoWire &i2cPort,
                         uint32_t i2cClock)
{
  // Cleanup previous instance to avoid memory leak on re-init
  if (bus)
  {
    delete bus;
    bus = nullptr;
  }

  // Initialize I2C interface with address and clock
  i2c.beginI2C(i2cAddress, i2cPort, i2cClock);

  // Create BusIO wrapper for register access
  bus = new BusIO_7Semi<I2C_Interface>(i2c);
  if (!bus)
    return false;

  // Perform soft reset sequence (reset + release)
  bus->write(SOFT_RESET, (uint8_t)0x00);
  delay(2);
  bus->write(SOFT_RESET, (uint8_t)0x01);
  delay(2);

  // Wait for sensor boot completion using GPIO status bit[1]
  uint8_t status = 0;
  uint32_t start = millis();
  do
  {
    // Read boot status register
    if (!bus->read(REG_GPIO_TIO_HV_STATUS, status))
      return false;

    // Timeout after 1 second if sensor not ready
    if ((millis() - start) > 1000UL)
      break;

  } while (status & 0x02); // Bit[1] = 1 → still booting

  // Read chip ID to verify device presence
  uint16_t chipId = 0;
  if (!readChipID(chipId))
    return false;

  // Optional strict check for correct sensor (disabled by default)
  if (chipId != REG_CHIP_ID_EXPECTED)
  {
    // return false;
  }

  // Apply ST default configuration block
  if (!applyDefaultConfiguration())
    return false;

  // Initialization successful
  return true;
}

/**
 * Set timeout (ms)
 * - Defines maximum wait time for blocking operations
 * - Used in data-ready polling loops
 */
void VL53L1X_7Semi::setTimeout(uint32_t timeoutMs)
{
  timeout_ms = timeoutMs;
}

/**
 * Get timeout (ms)
 * - Returns currently configured timeout value
 */
uint32_t VL53L1X_7Semi::getTimeout() const
{
  return timeout_ms;
}

/**
 * Set measurement range mode
 * - Configures sensor for SHORT / MEDIUM / LONG distance
 * - Applies ST recommended register presets
 * - Stops ongoing measurement before applying changes
 */
bool VL53L1X_7Semi::setMeasurementRange(Mode mode)
{
  stopMeasurement();

  switch (mode)
  {
  case SHORT:
    bus->write(PHASECAL_CONFIG_TIMEOUT_MACROP, (uint8_t)0x14); /* ST tuning preset */
    bus->write(RANGE_CONFIG_VCSEL_PERIOD_A, (uint8_t)0x07);    /* VCSEL period A for short */
    bus->write(RANGE_CONFIG_VCSEL_PERIOD_B, (uint8_t)0x05);    /* VCSEL period B for short */
    bus->write(RANGE_CONFIG_VALID_PHASE_HIGH, (uint8_t)0x38);
    bus->write(SD_CONFIG_WOI_SD0, (uint16_t)0x0705);
    bus->write(SD_CONFIG_INITIAL_PHASE_SD0, (uint16_t)0x0606);
    distance_range = 0;
    break;

  case MEDIUM:
    bus->write(PHASECAL_CONFIG_TIMEOUT_MACROP, (uint8_t)0x0B); /* ST tuning preset */
    bus->write(RANGE_CONFIG_VCSEL_PERIOD_A, (uint8_t)0x0F);    /* VCSEL period A for medium/long */
    bus->write(RANGE_CONFIG_VCSEL_PERIOD_B, (uint8_t)0x0D);    /* VCSEL period B for medium/long */
    bus->write(RANGE_CONFIG_VALID_PHASE_HIGH, (uint8_t)0x68);
    bus->write(SD_CONFIG_WOI_SD0, (uint16_t)0x0F0D);
    bus->write(SD_CONFIG_INITIAL_PHASE_SD0,(uint16_t) 0x0D0D);
    distance_range = 1;
    break;

  case LONG:
    bus->write(PHASECAL_CONFIG_TIMEOUT_MACROP, (uint8_t)0x0A); /* ST tuning preset */
    bus->write(RANGE_CONFIG_VCSEL_PERIOD_A, (uint8_t)0x0F);
    bus->write(RANGE_CONFIG_VCSEL_PERIOD_B, (uint8_t)0x0D);
    bus->write(RANGE_CONFIG_VALID_PHASE_HIGH, (uint8_t)0xB8);
    bus->write(SD_CONFIG_WOI_SD0, (uint16_t)0x0F0D);
    bus->write(SD_CONFIG_INITIAL_PHASE_SD0, (uint16_t)0x0E0E);
    distance_range = 2;
    break;
  }

  return true;
}

/**
 * Set timing budget (ms)
 * - Controls measurement duration and accuracy
 * - Higher values improve accuracy but reduce speed
 * - Converts ms into sensor macro timing registers
 */
bool VL53L1X_7Semi::setMeasurementTime(uint16_t timingBudgetMs)
{
  if (timingBudgetMs < 15)
    timingBudgetMs = 15;
  if (timingBudgetMs > 500)
    timingBudgetMs = 500;

  uint8_t vcselPeriodA = 0;
  uint8_t vcselPeriodB = 0;

  if (distance_range)
  {
    vcselPeriodA = 0x0F;
    vcselPeriodB = 0x0D;
  }
  else
  {
    vcselPeriodA = 0x07;
    vcselPeriodB = 0x05;
  }

  auto calcMacroPeriod = [](uint8_t vcsel_period_pclks) -> uint32_t
  {
    return ((uint32_t)2304 * (uint32_t)vcsel_period_pclks * 1655 + 500) * 2.17 / 1000;
  };

  uint32_t macroA_ns = calcMacroPeriod(vcselPeriodA);
  uint32_t macroB_ns = calcMacroPeriod(vcselPeriodB);

  const uint32_t overheadA_ns = 1320;
  const uint32_t overheadB_ns = 960;

  uint32_t timeoutMacroA = ((uint32_t)timingBudgetMs * 1000000UL - overheadA_ns) / macroA_ns;
  uint32_t timeoutMacroB = ((uint32_t)timingBudgetMs * 1000000UL - overheadB_ns) / macroB_ns;

  auto encodeTimeout = [](uint32_t timeoutMclks) -> uint16_t
  {
    if (timeoutMclks == 0)
      return 0;
    uint32_t lsByte = timeoutMclks - 1; /* Stored as (timeout - 1) */
    uint16_t msByte = 0;
    while (lsByte > 0xFF)
    {
      lsByte >>= 1; /* Normalize mantissa */
      msByte++;     /* Exponent increments */
    }
    return (uint16_t)((msByte << 8) | (lsByte & 0xFF)); /* {exponent, mantissa} */
  };

  if (!bus->write(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, encodeTimeout(timeoutMacroA)))
    return false;
  if (!bus->write(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, encodeTimeout(timeoutMacroB)))
    return false;

  return true;
}

/**
 * Get timing budget (ms)
 *
 * - Reads encoded timeout values from sensor registers
 * - Decodes macro timing values back to milliseconds
 * - Returns approximate measurement timing budget
 */
uint16_t VL53L1X_7Semi::getMeasurementTime()
{
  uint16_t regA = 0;
  uint16_t regB = 0;

  // Read timeout register A from sensor
  if (!bus->read(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, regA))
    return 0;

  // Read timeout register B from sensor
  if (!bus->read(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, regB))
    return 0;

  // Decode exponent + mantissa format into macro clock cycles
  auto decodeTimeout = [](uint16_t regValue) -> uint32_t
  {
    uint16_t msByte = (regValue >> 8) & 0xFF;
    uint16_t lsByte = regValue & 0xFF;

    // Convert from {exponent, mantissa} to raw clock cycles
    return ((uint32_t)(lsByte) + 1) << msByte;
  };

  // Decode both timeout registers
  uint32_t timeoutMacroA = decodeTimeout(regA);
  uint32_t timeoutMacroB = decodeTimeout(regB);

  uint8_t vcselPeriodA = 0;
  uint8_t vcselPeriodB = 0;

  // Select VCSEL timing based on current range mode
  if (distance_range)
  {
    vcselPeriodA = 0x0F;
    vcselPeriodB = 0x0D;
  }
  else
  {
    vcselPeriodA = 0x07;
    vcselPeriodB = 0x05;
  }

  // Calculate macro period in nanoseconds
  auto calcMacroPeriod = [](uint8_t vcsel_period_pclks) -> uint32_t
  {
    return ((uint32_t)2304 * vcsel_period_pclks * 1655 + 500) * 2.17 / 1000;
  };

  uint32_t macroA_ns = calcMacroPeriod(vcselPeriodA);
  uint32_t macroB_ns = calcMacroPeriod(vcselPeriodB);

  // Fixed overhead values from ST reference
  const uint32_t overheadA_ns = 1320;
  const uint32_t overheadB_ns = 960;

  // Convert macro timing into milliseconds
  uint32_t timingA_ms = (timeoutMacroA * macroA_ns + overheadA_ns) / 1000000UL;
  uint32_t timingB_ms = (timeoutMacroB * macroB_ns + overheadB_ns) / 1000000UL;

  // Return the larger timing value as effective budget
  return (uint16_t)((timingA_ms > timingB_ms) ? timingA_ms : timingB_ms);
}

/**
 * Start continuous measurement
 *
 * - Enables sensor ranging engine
 * - Updates internal running state
 */
bool VL53L1X_7Semi::startMeasurement()
{
  // Send start command to sensor
  if (!bus->write(REG_SYSTEM_START, (uint8_t)0x40))
  {
    return false;
  }

  // Mark measurement as active
  is_ranging = true;

  // Start successful
  return true;
}

/**
 * Stop continuous measurement
 *
 * - Disables sensor ranging engine
 * - Updates internal running state
 */
bool VL53L1X_7Semi::stopMeasurement()
{
  // Send stop command to sensor
  if (!bus->write(REG_SYSTEM_START, (uint8_t)0x00))
  {
    return false;
  }

  // Mark measurement as stopped
  is_ranging = false;

  // Stop successful
  return true;
}

/**
 * Blocking read
 *
 * - Ensures measurement is running
 * - Waits until data is ready or timeout occurs
 * - Reads one sample and clears interrupt
 */
Status VL53L1X_7Semi::readBlocking()
{
  // Start measurement if not already running
  if (!is_ranging)
  {
    if (!startMeasurement())
    {
      return STATUS_I2C_ERROR;
    }
  }

  // Wait for data-ready signal within timeout
  if (!waitDataReady(timeout_ms))
  {
    return STATUS_TIMEOUT;
  }

  // Perform single measurement read
  Status s = readOnce();
  if (s != STATUS_OK)
  {
    return s;
  }

  // Clear interrupt after reading data
  clearInterrupt();

  // Return success
  return STATUS_OK;
}

/**
 * Non-blocking read
 *
 * - Checks if data is ready
 * - Reads distance, ambient, and raw status
 * - Updates internal cached values
 */
Status VL53L1X_7Semi::readOnce()
{
  bool ready = false;

  // Check if measurement data is available
  if (!isDataReady(ready))
  {
    return STATUS_I2C_ERROR;
  }

  // Exit if data not ready
  if (!ready)
  {
    return STATUS_TIMEOUT;
  }

  uint16_t distance = 0;
  uint16_t current_ambient = 0;
  uint8_t status = 0;

  // Read distance value
  if (!bus->read(REG_RESULT_DISTANCE, distance))
    return STATUS_I2C_ERROR;

  // Read ambient light value
  if (!bus->read(REG_RESULT_AMBIENT, current_ambient))
    return STATUS_I2C_ERROR;

  // Read raw status register
  if (!bus->read(REG_RESULT_STATUS, status))
    return STATUS_I2C_ERROR;

  // Store results internally
  distance_mm = distance;
  ambient = current_ambient;

  // Extract valid status bits (lower 5 bits)
  range_status = (status & 0x1F);

  // Read successful
  return STATUS_OK;
}

/**
 * Read distance (mm)
 *
 * - Performs blocking read internally
 * - Returns last measured distance
 */
uint16_t VL53L1X_7Semi::readDistance()
{
  // Trigger blocking measurement
  (void)readBlocking();

  // Return cached distance value
  return distance_mm;
}

/**
 * Get ambient value
 *
 * - Returns last measured ambient light value
 */
uint16_t VL53L1X_7Semi::getAmbient() const
{
  // Return cached ambient value
  return ambient;
}

/**
 * Get range status
 *
 * - Converts raw sensor status into user-friendly enum
 * - Uses ST recommended mapping
 */
uint8_t VL53L1X_7Semi::getStatus() const
{
  // Decode raw status value using ST mapping
  switch (range_status & 0x1F)
  {
    case 9:   return RANGE_VALID;              // Valid measurement
    case 6:   return RANGE_SIGMA_FAIL;         // Noise too high
    case 4:   return RANGE_SIGNAL_FAIL;        // Weak signal
    case 8:   return RANGE_MIN_RANGE_FAIL;     // Object too close
    case 5:   return RANGE_PHASE_FAIL;         // Phase error
    case 3:   return RANGE_HARDWARE_FAIL;      // Internal error
    case 7:   return RANGE_WRAP_TARGET_FAIL;   // Target too far

    case 12:  return RANGE_VALID_WEAK;         // Valid but degraded
    case 13:  return MIN_RANGE_CLIPPED;        // Saturated / clipped

    default:  return RANGE_UNKNOWN;            // Undefined status
  }
}

/**
 * Change I2C address
 *
 * - Writes new 7-bit address into sensor register
 * - Updates internal address variable
 */
bool VL53L1X_7Semi::changeI2CAddress(uint8_t newAddress)
{
  // Ensure address is limited to 7-bit
  uint8_t raw = (uint8_t)(newAddress & 0x7F);

  // Write new address to sensor
  if (!bus->write(REG_I2C_SLAVE_DEVICE_ADDR, raw))
  {
    return false;
  }

  // Update stored address
  i2c_address = newAddress;

  // Address change successful
  return true;
}

/**
 * Read chip ID
 *
 * - Reads 16-bit IDENTIFICATION__MODEL_ID register
 * - Used to verify correct sensor device
 */
bool VL53L1X_7Semi::readChipID(uint16_t &chipId)
{
  uint8_t d = 0;
  chipId = 0;

  // Read high byte of chip ID
  if (!bus->read(REG_CHIP_ID, d))
    return false;
  chipId = ((uint16_t)d << 8);

  // Read low byte of chip ID
  if (!bus->read(REG_CHIP_ID + 1, d))
    return false;
  chipId |= d;

  // Chip ID read successful
  return true;
}
/**
 * Wait for data ready
 *
 * - Polls sensor until new measurement is available
 * - Exits on timeout or I2C error
 */
bool VL53L1X_7Semi::waitDataReady(uint32_t timeoutMs)
{
  // Store start time for timeout tracking
  uint32_t start = millis();
  bool ready = false;

  // Poll until timeout expires
  while ((millis() - start) < timeoutMs)
  {
    // Check data-ready status
    if (!isDataReady(ready))
    {
      return false;
    }

    // Exit when data becomes available
    if (ready)
    {
      return true;
    }

    // Small delay to avoid tight polling loop
    delay(2);
  }

  // Timeout occurred
  return false;
}

/**
 * Check data-ready status
 *
 * - Reads interrupt bits from RESULT__RANGE_STATUS
 * - Indicates if a new measurement is available
 */
bool VL53L1X_7Semi::isDataReady(bool &ready)
{
  uint8_t intStatus = 0;

  // Read interrupt/status register
  if (!bus->read(RESULT_RANGE_STATUS, intStatus))
  {
    return false;
  }

  // Check lower 3 bits for data-ready flag
  ready = ((intStatus & 0x07) != 0);

  return true;
}

/**
 * Clear interrupt
 *
 * - Clears data-ready interrupt flag
 * - Required before next measurement cycle
 */
bool VL53L1X_7Semi::clearInterrupt()
{
  // Write clear command to interrupt register
  return bus->write(REG_GPIO_CLEAR, (uint8_t)0x01);
}

/**
 * Apply default configuration
 *
 * - Loads ST reference configuration block
 * - Initializes sensor internal settings
 */
bool VL53L1X_7Semi::applyDefaultConfiguration()
{
  // Default configuration array from ST reference
  static const uint8_t DEFAULT_CFG[] = {
      0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
      0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x05, 0x00,
      0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
      0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
      0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00, 0x00, 0x02, 0xc7, 0xff,
      0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

  // Write configuration block to sensor registers
  for (uint16_t addr = 0x2D; addr <= 0x87; addr++)
  {
    uint8_t value = DEFAULT_CFG[addr - 0x2D];

    // Write each register value
    if (!bus->write(addr, value))
    {
      return false;
    }
  }

  // Configuration applied successfully
  return true;
}

/**
 * Set interrupt polarity
 *
 * - Configures GPIO interrupt polarity (active high/low)
 */
bool VL53L1X_7Semi::setInterruptPolarity(bool activeHigh)
{
  // Write polarity bit (bit[4] of register 0x30)
  if (!bus->writeBit(0x30, 4, (uint8_t)activeHigh))
    return false;

  return true;
}

/**
 * Get interrupt polarity
 *
 * - Reads current GPIO interrupt polarity setting
 */
bool VL53L1X_7Semi::getInterruptPolarity(bool &activeHigh)
{
  uint8_t v = 0;

  // Read polarity bit from register
  if (!bus->readBit(0x30, 4, v))
    return false;

  // Convert bit value to boolean
  activeHigh = (v == 0x01);

  return true;
}

/**
 * Set offset calibration (mm)
 *
 * - Compensates systematic distance error
 * - Stored internally as (offset_mm * 4)
 */
bool VL53L1X_7Semi::setOffset(int16_t offset_mm)
{
  // Scale offset to sensor format
  int16_t scaled = offset_mm * 4;

  // Write scaled value to offset register
  return bus->write(0x001E, (uint16_t)scaled);
}

/**
 * Get offset calibration (mm)
 *
 * - Reads offset register and converts back to mm
 */
bool VL53L1X_7Semi::getOffset(int16_t &offset_mm)
{
  uint16_t raw = 0;

  // Read raw offset value
  if (!bus->read(0x001E, raw))
    return false;

  // Convert to signed value and scale back to mm
  int16_t signed_val = (int16_t)raw;
  offset_mm = signed_val / 4;

  return true;
}