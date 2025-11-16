# VL53L0X Time-of-Flight Sensor Calibration Guide

## Overview

This guide explains how to calibrate the VL53L0X Time-of-Flight (ToF) distance sensor(s) on the Wheelie robot. Calibration ensures accurate distance measurements and optimal sensor performance, especially when using multiple sensors.

---

## Why Calibrate?

- **Factory offsets**: Each VL53L0X sensor may have slight manufacturing differences.
- **Mounting position**: Sensor angle and height affect readings.
- **Environmental factors**: Lighting, reflectivity, and temperature can impact accuracy.

---

## Prerequisites

- Wheelie robot assembled with VL53L0X sensor(s) connected and powered.
- Firmware flashed and serial monitor available (via PlatformIO or Arduino IDE).
- A flat, matte, light-colored calibration target (e.g., white card or paper).

---

## Calibration Steps

### 1. Prepare the Robot

- Place the robot on a flat surface.
- Ensure the VL53L0X sensor(s) are unobstructed and facing the calibration target.
- Measure and record the exact distance from the sensor face to the target (e.g., 200mm).

### 2. Start the Calibration Routine

- Power on the robot and connect to the serial monitor (baud rate: 115200).
- If your firmware includes a calibration mode, enter it (e.g., send `calibrate_vl53` via serial or use the CLI menu).
- If not, modify your code to print raw VL53L0X readings continuously.

### 3. Record Raw Readings

- Observe the distance values reported by the sensor.
- Compare the reported value to the actual measured distance.
- Repeat for each sensor if using multiple VL53L0X units.

### 4. Calculate Offset

- **Offset = Actual Distance - Reported Distance**
- Example: If actual = 200mm, reported = 190mm, then offset = +10mm.
- Repeat for several distances (e.g., 100mm, 200mm, 300mm) and average the offsets.

### 5. Update Firmware

- In your code, add the calculated offset to the raw sensor reading:

```cpp
// Example: Apply offset in your read function
int raw_distance = vl53.readRangeSingleMillimeters();
int calibrated_distance = raw_distance + VL53_OFFSET_MM; // Define VL53_OFFSET_MM per sensor
```

- For multiple sensors, use separate offsets for each (e.g., `VL53_FRONT_OFFSET_MM`, `VL53_REAR_OFFSET_MM`).

### 6. Validate Calibration

- Re-test at known distances to confirm accuracy.
- Adjust offsets if necessary.

---

## Multi-Sensor Notes

- Calibrate each VL53L0X sensor individually.
- Label each sensor and record its offset.
- Store offsets in firmware or EEPROM for persistent calibration.

---

## Troubleshooting

- **Inconsistent readings**: Check wiring, power supply, and ensure no reflective surfaces nearby.
- **Large offsets**: Verify sensor mounting and target distance.
- **No response**: Confirm I2C address and bus configuration.

---

## References

- [VL53L0X Datasheet](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
- [Pololu VL53L0X Arduino Library](https://github.com/pololu/vl53l0x-arduino)
- [Wheelie Robot Documentation](../)

---
