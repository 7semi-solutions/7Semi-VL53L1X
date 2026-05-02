/* ===============================
 * File: examples/BasicRead/BasicRead.ino
 * =============================== */
#include <Wire.h>
#include "7Semi_VL53L1X.h"

VL53L1X_7Semi tof(Wire, 0x29);

void setup() {
  Serial.begin(115200);

  if (!tof.begin(true, 500)) {
    Serial.println("VL53L1X init failed");
    while (1) {}
  }

  tof.setMeasurementRange(LONG);
  tof.setMeasurementTime(50);
}

void loop() {
  if (tof.readBlocking() == STATUS_OK) {
    Serial.print("Distance(mm): ");
    Serial.print(tof.readDistance());
    Serial.print("  Status: ");
    Serial.println(tof.getStatus());
  } else {
    Serial.println("Read timeout/error");
  }

  delay(100);
}
