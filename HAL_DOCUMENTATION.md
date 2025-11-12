# Hardware Abstraction Layer (HAL) Architecture

This document outlines the design and purpose of the Hardware Abstraction Layer (HAL) in the Wheelie robot's firmware. The HAL is a critical component of the software architecture, designed to create a clean, modular, and portable codebase.

## 1. What is the HAL?

A Hardware Abstraction Layer is a layer of software that acts as a bridge between the high-level application logic and the low-level hardware components of the robot.

In this project, the HAL consists of a set of modules that provide a simple and consistent interface to the robot's hardware, such as motors, sensors, and indicators. It hides the complex, hardware-specific details (like register manipulation, I2C communication, and specific library calls) behind easy-to-use functions.

For example, instead of the main navigation code needing to know how to write PWM signals to specific motor driver pins, it can simply call a function like `setMotorPWM(-150, 150)` to make a turn. The HAL takes care of the rest.

## 2. Why Did We Choose a HAL Architecture?

Implementing a HAL was a deliberate architectural choice to gain several key benefits:

* **Modularity & Readability**: The code is organized into logical, self-contained modules. The `robot.cpp` and `navigation.cpp` files focus on high-level logic (what the robot should do), while the HAL modules handle the low-level implementation (how to do it). This makes the code easier to read, understand, and maintain.

* **Portability & Maintainability**: If a hardware component is changed (e.g., swapping the MPU6050 for a different IMU), we only need to update the relevant HAL file (e.g., `sensors.cpp`). The high-level application code, which calls generic functions like `readIMUData()`, remains completely unchanged. This saves significant time and reduces the risk of introducing bugs.

* **Separation of Concerns**: Each module has a single, well-defined responsibility.
  * `sensors.cpp` is only responsible for reading data.
  * `motors.cpp` is only responsible for driving the motors.
  * `navigation.cpp` is only responsible for making movement decisions.
    This principle prevents the creation of a monolithic, unmanageable `main.cpp` file where all logic is intertwined.

* **Testability**: With a HAL, individual hardware components can be tested in isolation. We can write a simple test sketch that includes only `motors.h` to verify motor functionality without needing the entire navigation system.

## 3. How is the HAL Structured?

The HAL in this project is composed of several key files, primarily located in the `include/` and `src/` directories.

### Core HAL Modules

* #### `motors.h` / `motors.cpp`

  * **Purpose**: To control the robot's motors.
  * **Abstracts**: Low-level details of the ESP32's LEDC (PWM) controller and the specific pinout of the motor driver.
  * **Key Functions**: `setupMotors()`, `setMotorPWM()`, `allStop()`, `stopWithBrake()`.

* #### `sensors.h` / `sensors.cpp`

  * **Purpose**: To initialize and read data from all on-board sensors.
  * **Abstracts**: I2C communication, sensor-specific libraries (`VL53L0X`, `MPU6050_light`), interrupt handling, and data processing.
  * **Key Functions**: `initializeSensors()`, `updateAllSensors()`, `readToFDistance()`, `readIMUData()`.

* #### `indicators.h` / `indicators.cpp`

  * **Purpose**: To manage user feedback through the RGB LED and the buzzer.
  * **Abstracts**: `digitalWrite` calls for the LED pins and PWM generation for the buzzer.
  * **Key Functions**: `setupIndicators()`, `setLEDColor()`, `playTone()`, `indicators_update()`.

* #### `power_manager.h` / `power_manager.cpp`

  * **Purpose**: To monitor battery voltage and manage power-saving modes.
  * **Abstracts**: Analog pin reading for the battery and the logic for transitioning between power states.
  * **Key Functions**: `initializePowerManagement()`, `monitorPower()`, `getBatteryVoltage()`.

### Supporting Configuration Files

* #### `pins.h`

  * **Purpose**: Centralizes all GPIO pin assignments for the ESP32. If a sensor is moved to a different pin, this is the only file that needs to be changed.

* #### `config.h`

  * **Purpose**: Stores system-wide constants and tuning parameters, such as PWM frequencies, sensor thresholds, and communication intervals. This keeps magic numbers out of the main code.

* #### `types.h`

  * **Purpose**: Defines all major data structures (`SystemStatus`, `SensorData`) and enumerations (`RobotStateEnum`) used across the entire application, ensuring type consistency.

By adhering to this HAL structure, the project is more robust, easier to debug, and well-prepared for future expansion and hardware changes.
