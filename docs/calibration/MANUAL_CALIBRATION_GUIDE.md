# 🛠️ Manual Robot Calibration Guide

## 1. Overview

- **Note:** The right encoder is connected to GPIO 5, the left encoder to GPIO 33. The VL53L0X ToF sensor is used for front obstacle detection and calibration. The HC-SR04 ultrasonic sensor is for rear/backup only and not used in calibration.

This guide is for developers and advanced users who need to manually calibrate the robot. Manual calibration is useful when:

- The autonomous calibration repeatedly fails due to environmental constraints.
- You need to fine-tune performance beyond the automated setup.
- You are debugging hardware issues with motors, encoders, or the IMU.

**Prerequisite:** You should be comfortable editing firmware in PlatformIO/VS Code and using the Serial Monitor.

---

## ❗ Important Note on OTA Updates

**Symptom:** Over-the-Air (OTA) updates may fail to connect (e.g., `No response from the ESP`) if the robot has successfully completed its autonomous calibration.

**Cause & Workaround:** This happens because a successful calibration may cause the robot to enter a state where the OTA background service does not run correctly. If you need to perform an OTA update, intentionally causing the calibration to fail (e.g., by lifting the robot off the ground during startup) will often allow the OTA update to proceed normally. This is a known issue being investigated.

---

## 2. When to Use Manual Calibration

| Scenario | Recommendation |
| :--- | :--- |
| **Standard Use** | Use the **[Autonomous Calibration](./CALIBRATION_QUICK_START.md)**. It's faster and highly accurate. |
| **Fine-Tuning** | First, run the autonomous calibration. Then, use this guide to manually adjust the saved values. |
| **Hardware Debugging** | This guide provides isolated tests for motors, encoders, and sensors to help pinpoint issues. |
| **Auto-Calib Fails** | If autonomous calibration fails, this guide helps you generate usable values to get the robot moving. |

---

## 3. Step-by-Step Manual Calibration

### Step 1: Prepare a Test Sketch

Create a new, simple `.cpp` file in your `src/` directory (e.g., `manual_test.cpp`) or modify your `main.cpp` to perform isolated tests. You will need to include the relevant headers (`motors.h`, `sensors.h`, etc.) and call their setup functions.

**Example Test `setup()`:**

```cpp
#include "robot.h"

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("--- Manual Calibration Test ---");

    setupMotors();
    initializeSensors(); // For MPU and encoders (Right: GPIO 5, Left: GPIO 33)
    resetEncoders();
}

void loop() {
    // Your test code will go here.
    delay(5000); // Run once every 5 seconds
}
```

### Step 2: Motor Direction Mapping

The goal is to find which motor commands (`M1_FORWARD`, `M2_REVERSE`, etc.) make the robot turn left/right and move forward.

1. Place the robot on the ground.
2. Upload and run code that tests one motor combination at a time.
3. Observe the robot's movement and note the combination for each action.

**Test Code:**

```cpp
// Place this in your test sketch's loop()

Serial.println("Testing: M1 FORWARD, M2 FORWARD");
setMotorSpeeds(150, 150); // M1_FORWARD, M2_FORWARD
delay(1000);
allStop();
delay(2000);

Serial.println("Testing: M1 REVERSE, M2 FORWARD");
setMotorSpeeds(-150, 150); // M1_REVERSE, M2_FORWARD
delay(1000);
allStop();
delay(2000);

// ...add other combinations (FWD/REV, REV/REV)
```

**Record your findings:**

- **Forward:** `M1_FORWARD`, `M2_FORWARD`
- **Left Turn:** `M1_REVERSE`, `M2_FORWARD`
- **Right Turn:** `M1_FORWARD`, `M2_REVERSE`

### Step 3: Encoder Calibration

#### Ticks per 90-Degree Turn (`ticksPer90Degrees`)

**Encoders:** Right encoder = GPIO 5, Left encoder = GPIO 33

1. Use the motor commands you found for a **right turn**.
2. Run the motors for a fixed number of encoder ticks and stop.
3. Use a protractor or a corner to measure the angle the robot turned.
4. Adjust the `TARGET_TICKS` value until the robot turns exactly 90 degrees.

**Test Code:**

```cpp
// Place this in your test sketch's loop()

long TARGET_TICKS = 1450; // Adjust this value!

resetEncoders();
Serial.printf("Testing turn with %ld ticks...\n", TARGET_TICKS);

// Use your combination for a right turn
calibratedTurnRight(150); // Assumes calibratedTurnRight uses the correct directions

while(getAverageEncoderCount() < TARGET_TICKS) {
    delay(10);
}
allStop();

Serial.println("Turn complete. Measure the angle.");
delay(10000); // Wait 10s for you to measure
```

The final `TARGET_TICKS` value is your `ticksPer90Degrees`.

#### Ticks per Millimeter (`ticksPerMillimeter`)

**Encoders:** Right encoder = GPIO 5, Left encoder = GPIO 33

1. Use the motor commands for **forward** movement.
2. Run the motors for a fixed number of ticks (e.g., 5000).
3. Measure the actual distance traveled in millimeters with a ruler.
4. Calculate the ratio: `ticksPerMillimeter = total_ticks / distance_mm`.

**Test Code:**

```cpp
// Place this in your test sketch's loop()

long TARGET_TICKS = 5000;

resetEncoders();
Serial.printf("Moving forward for %ld ticks...\n", TARGET_TICKS);

calibratedMoveForward(150); // Assumes this uses the correct directions

while(getAverageEncoderCount() < TARGET_TICKS) {
    delay(10);
}
allStop();

Serial.println("Movement complete. Measure the distance traveled.");
delay(10000); // Wait 10s
```

**Calculation:** If the robot moved 480mm, your `ticksPerMillimeter` is `5000 / 480 = 10.42`.

### Step 4: MPU6050 Offset Calibration

The IMU (gyroscope/accelerometer) needs to be calibrated to remove sensor bias. The `MPU6050_light` library has a built-in, blocking function for this.

1. Place the robot on a **perfectly level and stable surface**.
2. Run the code below. The process will take a few seconds.
3. The code will print the calculated offsets. **Copy these values.**

**Test Code:**

```cpp
// Place this in your test sketch's setup() after initializing the MPU

if (sysStatus.mpuAvailable) {
    Serial.println("Calculating MPU offsets... Do not move the robot!");
    mpu.calcOffsets(); // This is a blocking function
    Serial.println("MPU calibration complete.");
    Serial.println("--- COPY THESE VALUES ---");
    Serial.printf("AccX: %f\n", mpu.getAccXoffset());
    Serial.printf("AccY: %f\n", mpu.getAccYoffset());
    Serial.printf("AccZ: %f\n", mpu.getAccZoffset());
    Serial.printf("GyroX: %f\n", mpu.getGyroXoffset());
    Serial.printf("GyroY: %f\n", mpu.getGyroYoffset());
    Serial.printf("GyroZ: %f\n", mpu.getGyroZoffset());
    Serial.println("-------------------------");
}
```

### Step 5: Applying Manual Values

Once you have your manual values, you can hard-code them into the firmware. This is useful if you want to bypass the EEPROM and autonomous calibration entirely.

In your code (e.g., `calibration.cpp`), you can create a default `CalibrationData` struct and apply the MPU offsets.

```cpp
#include "calibration.h"

void loadManualCalibration() {
    isCalibrated = true;
    calibData.magic = 0x7B;
    calibData.version = 1;

    // Step 2 Results
    calibData.motorDirs.left_M1 = REVERSE;
    calibData.motorDirs.left_M2 = FORWARD;
    // ... and so on for all directions

    // Step 3 Results
    calibData.ticksPer90Degrees = 1450.0f;
    calibData.ticksPerMillimeter = 10.42f;

    // Step 4 Results
    mpu.setAccOffsets(-1.23, 4.56, -7.89); // Use your copied values
    mpu.setGyroOffsets(0.12, -0.34, 0.56); // Use your copied values

    Serial.println("Loaded HARD-CODED manual calibration data.");
}
```

You would then call `loadManualCalibration()` in your `setup()` instead of the EEPROM-based logic.
