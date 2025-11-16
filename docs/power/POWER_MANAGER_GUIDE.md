# Power Manager Module Guide (`power_manager.cpp`)

## 1. Overview

The `power_manager.cpp` module is responsible for monitoring the robot's battery and managing its power consumption. It continuously reads the battery voltage, determines the current power mode (e.g., `NORMAL`, `ECONOMY`, `CRITICAL`), and provides this information to other systems.

This module is the core of the robot's intelligent power management system, enabling it to gracefully degrade performance to extend its operational time and to automatically shut down to protect the LiPo battery from damage.

---

## 2. Hardware Integration

The power manager relies on a **voltage divider circuit** to safely measure the battery voltage with the ESP32's Analog-to-Digital Converter (ADC), which has a 3.3V limit.

### Voltage Divider Circuit

For the 2S (7.4V nominal, 8.4V max) LiPo battery pack, a voltage divider reduces the voltage to a safe level for the ADC.

```txt
Battery + (8.4V max) ----[R1: 20kΩ]----+----[R2: 10kΩ]---- GND
                                     |
                                 ESP32 GPIO 34 (ADC)
```

- **Pin**: The output of the voltage divider is connected to `GPIO 34`.
- **Scaling**: This circuit scales the maximum 8.4V from the battery down to ~2.8V, which is safely within the ESP32's 0-3.3V ADC range.

---

## 3. Core Logic & Data Structures

The module's state is managed through a central struct and an enum.

### `BatteryMonitor_t` Struct

This struct, typically named `battery`, holds all real-time information about the power system.

```cpp
typedef struct {
    float voltage;
    float percentage;
    PowerMode_t mode;
    uint32_t lastUpdate;
    bool isCharging; // Future use
    uint32_t lowVoltageStartTime;
} BatteryMonitor_t;
```

### `PowerMode_t` Enum

This enum defines the 5 distinct power modes the robot can operate in.

```cpp
typedef enum {
    POWER_NORMAL,    // > 7.8V
    POWER_ECONOMY,   // 7.2V - 7.8V
    POWER_LOW,       // 6.8V - 7.2V
    POWER_CRITICAL,  // 6.4V - 6.8V
    POWER_SHUTDOWN   // < 6.4V
} PowerMode_t;
```

---

## 4. Key Functions

### `void initializePowerManagement()`

- **Purpose**: Sets up the ADC pin for reading the battery voltage.
- **Called From**: `WheelieHAL::init()`.
- **Action**: Configures `BATTERY_VOLTAGE_PIN` (`GPIO 34`) as an analog input.

### `void monitorPower()`

- **Purpose**: This is the main update function for the power manager. It reads the voltage, updates the power mode, and triggers actions based on the mode.
- **Called From**: `WheelieHAL::update()` in the main loop.
- **Logic**:
    1. Reads the raw ADC value from `GPIO 34`.
    2. Converts the ADC value to a voltage using the voltage divider ratio.
    3. Applies a moving average filter to smooth out the voltage reading and prevent noise from causing rapid mode changes.
    4. Determines the corresponding battery percentage.
    5. Compares the stable voltage against the thresholds for each power mode (`POWER_NORMAL`, `POWER_ECONOMY`, etc.).
    6. Updates the global `battery.mode` state. It uses hysteresis (a time delay) to ensure the voltage is consistently in a new range before switching modes.

### `float getBatteryVoltage()`

- **Purpose**: A simple getter function that returns the current, filtered battery voltage.
- **Called From**: Any module that needs to know the battery voltage, such as the HAL or data logger.

### `PowerMode_t getPowerMode()`

- **Purpose**: A getter function that returns the current `PowerMode_t`.
- **Called From**: The HAL or Brain, to decide whether to scale back performance (e.g., reduce motor speed or sensor polling frequency).

---

## 5. Integration with the HAL

The power manager is a low-level driver (Layer 0) that is exclusively controlled by the HAL (Layer 1).

- During initialization, `WheelieHAL::init()` calls `initializePowerManagement()`.
- In every loop cycle, `WheelieHAL::update()` calls `monitorPower()` to keep the battery status up-to-date.
- When the Brain requests a velocity (`hal.setVelocity()`), the HAL can check the power mode via `getPowerMode()` and decide to scale down the requested motor PWM values if in `ECONOMY` or `LOW` power mode.

This design ensures that the Brain remains unaware of power management details. It simply requests a velocity, and the Body (HAL) intelligently executes that command based on its available power.

---

## 6. Configuration & Calibration

The accuracy of the power manager depends on a few key constants, typically defined in a configuration file.

```cpp
// Calibration constants for the voltage divider
#define BATTERY_VOLTAGE_DIVIDER_RATIO 3.0f // (20k + 10k) / 10k
#define BATTERY_ADC_REFERENCE 3.3f

// Voltage thresholds for power modes
#define VOLTAGE_NORMAL    7.8f
#define VOLTAGE_ECONOMY   7.2f
#define VOLTAGE_LOW       6.8f
#define VOLTAGE_CRITICAL  6.4f
```

If voltage readings are inaccurate, you can use a multimeter to measure the actual battery voltage and compare it to the value reported in the serial monitor. If they differ, you can adjust the `BATTERY_VOLTAGE_DIVIDER_RATIO` constant slightly to calibrate the readings.
