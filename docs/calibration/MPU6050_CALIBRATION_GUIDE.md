# MPU6050 IMU Calibration Guide

## Overview

This guide explains how to calibrate the MPU6050 6-axis IMU (accelerometer and gyroscope) on the Wheelie robot. Proper calibration ensures accurate motion sensing, stable orientation, and reliable sensor fusion.

---

## Why Calibrate?

- **Factory offsets**: Each MPU6050 has unique bias and offset values.
- **Mounting orientation**: Small misalignments affect readings.
- **Environmental factors**: Temperature and vibration can introduce drift.

---

## Prerequisites

- Wheelie robot assembled with MPU6050 connected and powered.
- Firmware flashed and serial monitor available (via PlatformIO or Arduino IDE).
- Robot placed on a stable, level surface.

---

## Calibration Steps

### 1. Prepare the Robot

- Place the robot on a flat, stable surface.
- Ensure the robot is completely still during calibration.

### 2. Start the Calibration Routine

- Power on the robot and connect to the serial monitor (baud rate: 115200).
- If your firmware includes a calibration mode, enter it (e.g., send `calibrate_mpu` via serial or use the CLI menu).
- If not, modify your code to print raw accelerometer and gyroscope readings continuously.

### 3. Record Raw Readings

- Observe the raw values for each axis (Accel X/Y/Z, Gyro X/Y/Z).
- For a stationary, level robot:
  - Accelerometer Z should read close to +1g (typically ~16384 for ±2g setting).
  - Accelerometer X and Y should be near zero.
  - All gyroscope axes should be near zero.

### 4. Calculate Offsets

- **Accel Offset = 0 (X/Y), +1g (Z) - Raw Value**
- **Gyro Offset = 0 - Raw Value**
- Average readings over several seconds for best results.

### 5. Update Firmware

- In your code, subtract the calculated offsets from each raw reading:

```cpp
// Example: Apply offsets in your read function
int16_t ax, ay, az, gx, gy, gz;
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
ax -= ACCEL_X_OFFSET;
ay -= ACCEL_Y_OFFSET;
az -= ACCEL_Z_OFFSET;
gx -= GYRO_X_OFFSET;
gy -= GYRO_Y_OFFSET;
gz -= GYRO_Z_OFFSET;
```

- Store offsets in firmware or EEPROM for persistent calibration.

### 6. Validate Calibration

- Re-test with the robot stationary and moving.
- Confirm that stationary readings are close to expected values.
- Adjust offsets if necessary.

---

## Advanced: Using Calibration Libraries

- Libraries like [MPU6050_light](https://github.com/rfetick/MPU6050_light) or [Jeff Rowberg's MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) include built-in calibration routines.
- Follow library instructions for automated offset calculation and storage.

---

## Troubleshooting

- **Drifting values**: Re-run calibration, check for vibration, or use digital low-pass filter.
- **Large offsets**: Verify sensor orientation and mounting.
- **No response**: Check I2C wiring and address.

---

## References

- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU6050 Arduino Library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
- [Wheelie Robot Documentation](../)

---
