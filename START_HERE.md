# START HERE: Wheelie Robot Project Guide

Welcome to the Wheelie robot project! This guide will help you get started, understand the project structure, and point you to the most important documentation for building, wiring, programming, calibrating, and operating your robot.

---

## 1. Project Overview

- **Wheelie** is an open-source, modular robot platform based on the ESP32 microcontroller.
- Features include: motor control, multiple sensors (ToF, IMU, encoders, sound, ultrasonic), WiFi, OTA updates, and more.

---

## 2. Quick Start Checklist

1. **Read the [Assembly Guide](docs/assembly/SETUP_GUIDE.md)**
2. **Wire your hardware using the [Wiring Guide](docs/assembly/WIRING.md)**
3. **Flash the firmware using PlatformIO**
4. **Calibrate your sensors ([Calibration Guides](docs/calibration/))**
5. **Test and operate your robot!**

---

## 3. Directory Structure

- `/docs/` — All documentation
  - `/assembly/` — Hardware setup, wiring, shopping list
  - `/calibration/` — Sensor calibration guides
  - `/components/` — Guides for each hardware module
  - `/sensors/` — Sensor integration and usage
  - `/power/` — Power system documentation
- `/src/` — Main firmware source code
- `/include/` — Header files
- `/tests/` — Test programs
- `/data/` — Web server files

---

## 4. Essential Documentation

- **Assembly:** [`SETUP_GUIDE.md`](docs/assembly/SETUP_GUIDE.md), [`SHOPPING_LIST.md`](docs/assembly/SHOPPING_LIST.md)
- **Wiring:** [`WIRING.md`](docs/assembly/WIRING.md), [`HARDWARE_PIN_MAPPING.md`](docs/assembly/HARDWARE_PIN_MAPPING.md)
- **Power:** [`POWER_MANAGEMENT_SYSTEM.md`](docs/power/POWER_MANAGEMENT_SYSTEM.md), [`XL4015_POWER_GUIDE.md`](docs/power/XL4015_POWER_GUIDE.md)
- **Motors:** [`MOSFET_MOTOR_DRIVER_GUIDE.md`](docs/components/MOSFET_MOTOR_DRIVER_GUIDE.md)
- **Sensors:**
  - ToF: [`VL53L0X_GY_VL53L0XV2_GUIDE.md`](docs/sensors/VL53L0X_GY_VL53L0XV2_GUIDE.md), [`VL53L0X_CALIBRATION_GUIDE.md`](docs/calibration/VL53L0X_CALIBRATION_GUIDE.md)
  - IMU: [`MPU6050_GY521_GUIDE.md`](docs/sensors/MPU6050_GY521_GUIDE.md), [`MPU6050_CALIBRATION_GUIDE.md`](docs/calibration/MPU6050_CALIBRATION_GUIDE.md)
  - Encoders: [`LM393_H2010_ENCODER_GUIDE.md`](docs/sensors/LM393_H2010_ENCODER_GUIDE.md)
  - Sound: [`H-1-0332_SOUND_SENSOR_GUIDE.md`](docs/sensors/H-1-0332_SOUND_SENSOR_GUIDE.md)
- **WiFi/OTA:** [`WIFI_CONNECTIVITY_GUIDE.md`](docs/components/WIFI_CONNECTIVITY_GUIDE.md), [`OTA_GUIDE.md`](docs/OTA_GUIDE.md)

---

## 5. Building and Flashing Firmware

- Install [PlatformIO](https://platformio.org/) in VS Code.
- Open the project folder.
- Edit `platformio.ini` if needed for your board.
- Use the PlatformIO sidebar to **Build** and **Upload** the firmware.

---

## 6. Calibration

- Calibrate all sensors for best performance:
  - [VL53L0X ToF Calibration](docs/calibration/VL53L0X_CALIBRATION_GUIDE.md)
  - [MPU6050 IMU Calibration](docs/calibration/MPU6050_CALIBRATION_GUIDE.md)

---

## 7. Troubleshooting & Support

- Check the relevant guide in `/docs/` for your issue.
- Review the serial monitor output for errors.
- Visit the project repository for updates and community support.

---

## 8. Contributing

- See [`CONTRIBUTING.md`](CONTRIBUTING.md) for guidelines.

---

*Happy building!*
