# 🤖 Autonomous Calibration System Guide

## The "Zero-Tuning" System for Precision Navigation

---

## 1. Overview

The Autonomous Calibration System is one of the project's most powerful features. It is a **one-time, self-performing routine** that the robot runs on its very first boot to learn the unique physical properties of its own body.

This system completely eliminates the need for manual tuning of motor speeds, turn rates, or sensor offsets. It ensures that high-level commands like "move forward 1 meter" or "turn right 90 degrees" are executed with precision, regardless of minor variations in motors, battery voltage, or wheel assembly.

**The Goal**: To create a professional-grade robot that works perfectly out of the box, without any manual configuration.

---

## 2. The Problem It Solves

No two robots are identical. Even when built from the same kit, there are always small differences:

- One motor might be slightly faster than the other.
- The wheels might have a slightly different diameter.
- The IMU sensor might have a small inherent drift.

Without calibration, commanding the robot to "go straight" might result in it veering to one side. A "90-degree turn" could be 85 or 95 degrees. Autonomous calibration solves this by measuring these real-world characteristics and saving them as correction factors.

---

## 3. The Calibration Sequence

When the robot is powered on for the first time, it will automatically enter the `ROBOT_CALIBRATING` state and perform the following sequence.

**Visual/Audio Feedback**: During calibration, the robot's LED will be solid **yellow**, and it will play tones to indicate the start and end of each phase.

### Phase 0: MPU6050 IMU Calibration (Static)

**Action**: The robot remains perfectly still.
**Goal**: To measure the "zero" point for the accelerometer and gyroscope.

1. The system takes thousands of readings from the IMU while the robot is at rest.
2. It calculates the average error (offset) for each of the 6 axes (3 accelerometer, 3 gyroscope). For example, if the gyro reports a 0.5°/s rotation on the Z-axis while standing still, it knows it has a +0.5°/s drift.
3. These offsets are stored and will be automatically subtracted from all future IMU readings, ensuring that a stationary robot reports zero movement.

### Phase 1: Motor Direction Mapping (Dynamic)

**Action**: The robot will briefly spin each wheel forward and backward.
**Goal**: To determine which PWM signal combination makes the robot move forward, backward, left, and right.

1. The system sends a positive PWM signal to the left motor and observes the encoder and IMU.
2. It determines if this action resulted in forward or backward motion.
3. It repeats this for all motors and directions, building a "map" of motor commands.
4. The result is a set of multipliers (e.g., `left_motor_fwd = 1`, `right_motor_fwd = -1`) that abstracts away how the motors were wired.

### Phase 2: Turn Calibration (Dynamic)

**Action**: The robot will attempt to turn 90 degrees to the right.
**Goal**: To calculate `ticksPerDegree`, or how many encoder ticks are needed for one degree of rotation.

1. The robot resets its IMU heading to 0.
2. It begins turning right, counting the encoder ticks from both wheels.
3. It continuously checks its IMU heading until it reaches 90 degrees.
4. It stops and calculates: `ticksPerDegree = total_encoder_ticks / 90.0`.

With this value, the robot can now perform precise, IMU-verified turns of any angle.

### Phase 3: Distance Calibration (Dynamic)

**Action**: The robot will drive forward a fixed distance (e.g., 50cm).
**Goal**: To calculate `ticksPerMillimeter`, or how many encoder ticks are needed to travel one millimeter.

1. The robot resets its encoder counts.
2. It drives forward, counting the encoder ticks.
3. It uses its front-facing Time-of-Flight (ToF) sensor to measure the distance to a wall or obstacle it started facing.
4. It stops once it has traveled the target distance.
5. It calculates: `ticksPerMillimeter = total_encoder_ticks / 500.0`.

The robot can now travel precise linear distances.

---

## 4. Data Persistence (The "Run-Once" Magic)

The calibration data is too valuable to lose on reboot.

- **EEPROM Storage**: After the calibration sequence completes successfully, all calculated values (`MPU offsets`, `ticksPerDegree`, `ticksPerMillimeter`, etc.) are saved to the ESP32's onboard EEPROM (a form of non-volatile memory).
- **Calibration Flag**: A special flag (e.g., `isCalibrated = true`) is also written to the EEPROM.
- **On Next Boot**: During the `hal.init()` sequence, the system first checks for this flag in EEPROM.
  - If the flag is `true`, it skips the entire calibration sequence and loads the saved values directly into the active configuration.
  - If the flag is `false` or the data is corrupt, it triggers the full calibration sequence again.

---

## 5. Forcing a Recalibration

In some cases, you may need to force the robot to recalibrate (e.g., after changing the wheels or motors).

The firmware includes a `shouldForceRecalibration()` function. While the exact trigger can vary, it is typically implemented to check for one of the following:

- **A CLI Command**: A command like `recalibrate` sent via the serial monitor.
- **A Physical Button**: Holding down a specific button on the robot during boot.
- **A Web Interface Trigger**: A button on the robot's web dashboard.

When triggered, this function will cause the system to ignore the EEPROM flag and run the full calibration sequence on the next boot.

---

## 6. Integration with the HAL

The entire calibration process is managed within the robot's specific HAL implementation (e.g., `WheelieHAL.cpp`).

- The main `hal.init()` function contains the logic to check the calibration flag.
- The `runFullCalibrationSequence()` function orchestrates the different phases.
- The calibration results are stored in the `CalibrationData` struct, which is used by the HAL's `setVelocity()` and `updateOdometry()` functions to perform accurate, corrected movements.

This tight integration ensures that the "Brain" can issue a generic command like `setVelocity(Vector2D(100, 0))`, and the "Body" (HAL) will use its unique calibration data to translate that into the precise motor signals needed to move forward at 100mm/s.
