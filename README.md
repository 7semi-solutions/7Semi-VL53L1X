# 7Semi VL53L1X Arduino Library

Arduino driver for the STMicroelectronics VL53L1X time-of-flight (ToF) distance sensor.

The VL53L1X provides accurate distance measurement using laser time-of-flight technology, supporting long-range detection, configurable timing budgets, and multiple range modes for flexible applications.

---

## Features

- High-accuracy distance measurement
  - Up to 4 meters range
  - mm-level resolution

- Multiple range modes
  - SHORT (high ambient immunity)
  - MEDIUM (balanced)
  - LONG (maximum distance)

- Configurable timing budget
  - 15 ms to 500 ms
  - Trade-off between speed and accuracy

- Blocking and non-blocking read APIs

- Ambient light measurement

- I²C address change support

- Status monitoring with detailed error decoding

---

## Connections / Wiring

The VL53L1X uses **I²C communication**.

---

## I²C Connection

| VL53L1X Pin | MCU Pin         | Notes                     |
| ----------- | --------------- | ------------------------- |
| VIN         | 3.3V / 5V*      | Check module specs        |
| GND         | GND             | Common ground             |
| SDA         | SDA             | I²C data                  |
| SCL         | SCL             | I²C clock                 |
| XSHUT       | GPIO (optional) | Hardware shutdown control |
| GPIO1       | GPIO (optional) | Interrupt output          |

---

## I²C Notes

- Default I²C address: 0x29
- Supported bus speeds:
  - 100 kHz  
  - 400 kHz (recommended)  

---

## Installation

### Arduino Library Manager

1. Open Arduino IDE  
2. Go to Library Manager  
3. Search for **7Semi VL53L1X**  
4. Click Install  

---

### Manual Installation

1. Download repository as ZIP  
2. Arduino IDE → Sketch → Include Library → Add .ZIP Library  

---

## Library Overview

---

### Initialize Sensor

```cpp
 if (!sensor.begin())
  {
    Serial.println("Sensor init failed");
    while (1);
  }
```

### Reading Distance

```cpp
uint16_t distance = sensor.readDistance();
```

- Returns distance in millimeters

## Reading Ambient Light

```cpp
uint16_t ambient = sensor.getAmbient();
```

## Setting Range Mode

```cpp
sensor.setMeasurementRange(LONG);
```

- SHORT → best for close range, high ambient immunity
- MEDIUM → balanced performance
- LONG → maximum distance

## Setting Timing Budget

```cpp
sensor.setMeasurementTime(100);
```

- Range: 15 ms – 500 ms
- Higher value → better accuracy, slower updates

## Blocking Read

```cpp
sensor.readBlocking();
```

- Waits until measurement is ready
- Uses timeout internally

## Non-Blocking Read

```cpp
Status status = sensor.readOnce();
```

- Returns immediately
- Check status before using data

### Checking Status

```cpp
uint8_t status = sensor.getStatus();
```

- RANGE_VALID → valid measurement
- RANGE_VALID_WEAK → usable but degraded
- Other values → error conditions

### Offset Calibration

```cpp
sensor.setOffset(-10);
```

- Adjusts measurement error (in mm)

### Changing I²C Address

```cpp
sensor.changeI2CAddress(0x30);
```

### Interrupt Handling (Optional)

```cpp
sensor.setInterruptPolarity(true); // active HIGH
```

## Applications

- Robotics (obstacle detection)
- Drones (altitude sensing)
- Smart devices (presence detection)
- Industrial distance measurement
- Gesture sensing

## License

- MIT License
