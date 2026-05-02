/**
 * VL53L1X 7Semi Library Example
 *
 * - Demonstrates basic distance measurement using VL53L1X ToF sensor
 * - Uses blocking read to get distance, ambient, and status
 *
 * Hardware Connection (I2C)
 * - VIN  -> 3.3V / 5V (depending on module)
 * - GND  -> GND
 * - SDA  -> SDA (e.g. Arduino A4 / ESP32 GPIO21)
 * - SCL  -> SCL (e.g. Arduino A5 / ESP32 GPIO22)
 * - XSHUT (optional) -> GPIO (for hardware shutdown)
 * - GPIO1 (optional) -> GPIO (interrupt output)
 */

#include <7Semi_VL53L1X.h>

VL53L1X_7Semi sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize sensor
  if (!sensor.begin()) {
    Serial.println("Failed to initialize VL53L1X!");
    while (1);
  }

  // Configure measurement settings
  sensor.setMeasurementRange(LONG);   // LONG / MEDIUM / SHORT
  sensor.setMeasurementTime(50);      // Timing budget in ms

  // Ensure measurement is stopped before loop starts
  if (!sensor.stopMeasurement()) {
    Serial.println(F("VL53L1X stop failed"));
    while (1) { delay(100); }
  }

  Serial.println("VL53L1X started successfully!");
}

void loop() {
  // Read and print measurement data
  Serial.print(F("Distance(mm): "));
  Serial.print(sensor.readDistance());

  Serial.print(F("  Ambient: "));
  Serial.print(sensor.getAmbient());

  Serial.print(F("  Status: "));
  Serial.println(sensor.getStatus());

  delay(100);
}