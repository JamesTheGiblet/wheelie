# changelog_week_2025-11-23

## Changelog: RobotForge Firmware - The "Sensor Resilience" Update

This milestone update dramatically improves the robot's reliability and real-world performance by introducing a robust sensor handling architecture. The focus has been on making the robot resilient to the electrical noise and communication failures that are common during motion, ensuring stable navigation and control.

## ✨ New Features

* **️ Robust Sensor Reader (`RobustSensorReader.h`)**: Introduced a new, dedicated module to manage all sensor interactions. This class acts as a crucial middleware layer between the low-level hardware and the high-level navigation logic.
  * **Error Handling**: The reader intelligently detects and handles I2C communication failures (e.g., `Error -1`, `Error 263`). Instead of passing invalid data (like `0`), it provides the last known good reading, preventing the robot from making erratic decisions.
  * **Data Filtering**: Implemented a configurable moving average filter for all sensors (ToF, Ultrasonic, and IMU). This smooths out noisy readings, providing a much more stable data stream to the navigation system.
  * **Outlier Rejection**: The ultrasonic sensor logic now includes basic outlier rejection to discard erroneous readings that are outside a sane physical range.

* **🧠 Enhanced Learning Navigator (`LearningNavigator.h`)**: The navigator has been upgraded to directly integrate with the new sensor architecture.
  * **Direct Sensor Integration**: The `update()` method now accepts raw sensor distances, allowing the navigator to internally calculate repulsive forces and manage its own `ObstacleMemory`.
  * **Automatic Memory**: Obstacles detected by the real-time sensors are now automatically added to the `ObstacleMemory`, giving the robot short-term spatial awareness and helping it navigate around objects even if a sensor misses a reading.

## 🚀 Improvements

* **🏗️ Architectural Refinement (C++)**:
  * **Separation of Declaration and Implementation**: The `RobustSensorReader` logic was split into a header (`.h`) and an implementation file (`.cpp`). This resolves "incomplete type" compilation errors, breaks dependency chains, and is a standard C++ best practice.
  * **Template Programming Fixes**: Corrected several template-related errors in `RobustSensorReader.h` to make it truly generic. This involved using `T()` for default construction and fixing operator overloads for the `IMUData` struct.

* **🔧 Build System & Dependencies**:
  * **`platformio.ini` Corrected**: Fixed a critical issue where `lib_deps` in specific environments (like `[env:debug]`) were overwriting the common dependencies instead of extending them. All libraries are now correctly defined in each environment, resolving numerous "file not found" compilation errors.
  * **Cleaned Up Old Libraries**: Removed includes and references to the old `MPU6050_light.h` library from the project, finalizing the transition to the `Adafruit MPU6050` library.

## 🛠️ Code Refactoring

* **`WheelieHAL.cpp`**: Commented out and removed old, direct sensor reading logic (`mpu.update()`, `tofSensor.read...`) that has now been superseded by the `RobustSensorReader`. The HAL is now cleaner and delegates sensor handling correctly.
* **`IMUData` Struct**: Overloaded the `+` and `/` operators to allow the `IMUData` struct to work seamlessly with the generic moving average filter template in `SensorState`.
* **Calibration System**: Overhauled the entire calibration process, moving from hardcoded values to a dynamic, user-friendly setup.
  * **Web-Based Interface**: Implemented a new calibration mode accessible via a web page, allowing for real-time tuning of sensor offsets and PID parameters without needing to re-flash the firmware.
  * **EEPROM Storage**: Calibration values (e.g., IMU gyro/accelerometer offsets) are now automatically saved to and loaded from EEPROM, ensuring they persist between reboots.
  * **Guided Process**: The new setup includes a guided step-by-step process for calibrating the IMU, which improves the accuracy of the balance point calculation.
  * **Simplified Workflow**: The overall process is now significantly faster and less error-prone, making it easier to get the robot operational.
