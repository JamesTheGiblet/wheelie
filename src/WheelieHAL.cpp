#include "WheelieHAL.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// --- Include all the necessary HAL component headers ---
#include "indicators.h"
#include "power_manager.h"
#include "motors.h"
#include "wifi_manager.h"
#include "logger.h"
#include "calibration.h"

// --- Global System State (accessed by HAL) ---
extern SystemStatus sysStatus;
extern SensorData sensors;
extern CalibrationData calibData;
extern SensorHealth_t sensorHealth;
extern bool isCalibrated;

// --- Internal HAL Configuration & State ---
#include "config.h"

// Ultrasonic constants now defined in config.h

// State variables for filtering and non-blocking reads
float ultrasonicReadings[ULTRASONIC_FILTER_SIZE] = {0};
int ultrasonicReadingIndex = 0;
bool ultrasonicFilterPrimed = false;
enum UltrasonicState { US_IDLE, US_TRIGGERED, US_ECHO_IN_PROGRESS };
UltrasonicState ultrasonicState = US_IDLE;
unsigned long ultrasonicTriggerTime = 0;
unsigned long ultrasonicEchoStartTime = 0;
unsigned long nextUltrasonicReadTime = 0; // Added missing variable

// --- WheelieHAL Constructor ---
WheelieHAL::WheelieHAL() {
    currentPose.position = Vector2D(0, 0);
    currentPose.heading = 0.0;
}

// ═══════════════════════════════════════════════════════════════════════════
// HAL LIFECYCLE IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

bool WheelieHAL::init() {
    Serial.begin(115200);
    delay(100);
    setRobotState(ROBOT_BOOTING);
    setupIndicators();
    setupMotors();
    initializeLogging();
    this->initializeSensors();

    // Minimal calibration for reliable startup
    runMinimalCalibration();

    updateAllSensors();
    lastLeftEncoder = sensors.leftEncoderCount;
    lastRightEncoder = sensors.rightEncoderCount;
    lastHeading = sensors.headingAngle;
    currentPose.heading = lastHeading;

    Serial.println("✅ WheelieHAL Initialized (Minimal Mode).");
    return true;
}

void WheelieHAL::update() {
    updateAllSensors();

    // --- Run PID Heading Controller (at a fixed rate, e.g., 50Hz) ---
    unsigned long currentTime = millis();
    if (currentTime - lastPidTime > 20) { // 50Hz
        float deltaTime = (currentTime - lastPidTime) / 1000.0f;
        lastPidTime = currentTime;

        // --- FIX: Bypass PID controller if target velocity is zero ---
        if (targetVelocity.magnitude() < 0.1f) {
            // If we're supposed to be stopped, command motors to stop directly.
            setMotorPWM(0, 0);
            // Reset PID integral to prevent wind-up from previous movements.
            integral = 0.0f;       // Reset integral term
            previous_error = 0.0f; // Reset derivative term
            return; // Skip the rest of the PID calculation
        }

        // 1. Calculate Error
        // The target heading is the angle of the desired velocity vector.
        float targetHeading = targetVelocity.angle() * 180.0f / M_PI;
        float currentHeading = getPose().heading;

        // Find the shortest angle between target and current heading
        float error = targetHeading - currentHeading;
        while (error > 180.0f)  error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        // 2. Calculate PID terms
        // Proportional term
        float p_term = pid_p * error;

        // Integral term (with anti-windup)
        integral += error * deltaTime;
        integral = constrain(integral, -50, 50); // Anti-windup
        float i_term = pid_i * integral;

        // Derivative term
        float derivative = (error - previous_error) / deltaTime;
        float d_term = pid_d * derivative;
        previous_error = error;

        // 3. Calculate total correction and apply to motors
        float turnCorrection = p_term + i_term + d_term;
        setMotorsFromVector(targetVelocity, turnCorrection);
    }
    updateOdometry();
    indicators_update();
}

// ═══════════════════════════════════════════════════════════════════════════
// HAL SENSING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Normalizes an angle to the range [-180, 180].
 * The MPU6050_light library's getAngleZ() function returns a continuously
 * accumulating angle. This function wraps it to a standard range.
 * @param angle The angle to normalize.
 * @return The normalized angle.
 */
float normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void WheelieHAL::updateAllSensors() {
    // --- Read Front Time-of-Flight (ToF) Sensor ---
    if (sysStatus.tofAvailable) {
        // The robust reader handles filtering and error checking.
        if (sensorReader.readTofSensor(tofSensor)) {
            sensors.frontDistanceCm = sensorReader.tof.currentValue / 10.0f; // Convert mm to cm
        } else {
            sensors.frontDistanceCm = sensorReader.tof.lastGoodValue / 10.0f; // Use last good value on failure
        }
    }

    // --- Read Rear Ultrasonic Sensor ---
    if (sysStatus.ultrasonicAvailable) {
        // The robust reader handles basic outlier rejection.
        if (sensorReader.readUltrasonicSensor(FRONT_ULTRASONIC_TRIG_PIN, FRONT_ULTRASONIC_ECHO_PIN)) {
            sensors.rearDistanceCm = sensorReader.ultrasonic.currentValue;
        } else {
            sensors.rearDistanceCm = sensorReader.ultrasonic.lastGoodValue; // Use last good value on failure
        }
    }

    if (sysStatus.mpuAvailable) {
        // The robust reader is a good pattern, but for now, we'll read directly.
        if (!sensorReader.readMPUSensor(mpu)) {
            // On failure, use the last known good value to prevent system from using zeros.
            // This makes the system more robust to intermittent sensor failures.
            sensorHealth.mpuHealthy = false;
            sensors.tiltX = sensorReader.mpu.lastGoodValue.accX;
            sensors.tiltY = sensorReader.mpu.lastGoodValue.accY;
            sensors.gyroZ = sensorReader.mpu.lastGoodValue.gyroZ;
            sensors.temperature = sensorReader.mpu.lastGoodValue.temp;
        } else {
            // On successful read, get the latest filtered values.
            sensorHealth.mpuHealthy = true;
            sensors.tiltX = sensorReader.mpu.currentValue.accX;
            sensors.tiltY = sensorReader.mpu.currentValue.accY;
            sensors.gyroZ = sensorReader.mpu.currentValue.gyroZ;
            sensors.temperature = sensorReader.mpu.currentValue.temp;
        }
    }
    sensors.leftEncoderCount = getLeftEncoderCount();
    sensors.rightEncoderCount = getRightEncoderCount();
}

void WheelieHAL::updateOdometry() {
    // This logic is moved from the old `main.cpp`
    // It assumes the MPU6050 is mounted at the robot's center of rotation (axle line).
    // This allows us to use the gyro's heading directly without compensating for
    // its position during a turn.
    // USER CONFIRMED: MPU is mounted at the center-front of the robot.

    long currentLeft = sensors.leftEncoderCount;
    long currentRight = sensors.rightEncoderCount;
    currentPose.heading = sensors.headingAngle;
    
    long deltaLeft = currentLeft - lastLeftEncoder;
    long deltaRight = currentRight - lastRightEncoder;
    
    float ticksPerMm = (isCalibrated && calibData.ticksPerMillimeter > 0) ? calibData.ticksPerMillimeter : 1.0;
    float deltaDistance = ((float)(deltaLeft + deltaRight) / 2.0) / ticksPerMm;
    
    float avgHeading = (currentPose.heading + lastHeading) / 2.0;
    if (abs(currentPose.heading - lastHeading) > 180.0) {
        avgHeading += 180.0;
    }
    // Convert heading to radians for trig functions. Standard math assumes 0 degrees is along the +X axis (East).
    float avgHeadingRad = avgHeading * M_PI / 180.0; 
    
    // Update world-frame position
    currentPose.position.x += deltaDistance * cos(avgHeadingRad);
    currentPose.position.y += deltaDistance * sin(avgHeadingRad);
    
    lastLeftEncoder = currentLeft;
    lastRightEncoder = currentRight;
    lastHeading = currentPose.heading;
}

Vector2D WheelieHAL::getObstacleRepulsion() {
    // This is the "Translator" for Wheelie's specific hardware.
    Vector2D totalRepulsionForce(0, 0);

    // --- 1. Calculate force from front ToF sensor ---
    if (sysStatus.tofAvailable) {
        float frontDistanceCm = sensors.frontDistanceCm; // This is correct
        const float FRONT_INFLUENCE_RADIUS = 40.0f; // 40cm
        const float FRONT_REPULSION_STRENGTH = 30.0f;

        if (frontDistanceCm < FRONT_INFLUENCE_RADIUS) {
            // ToF is at the front (HAL standard: X+).
            // Repulsion force is backward (HAL standard: X-).
            float strength = FRONT_REPULSION_STRENGTH * (1.0f - (frontDistanceCm / FRONT_INFLUENCE_RADIUS));
            totalRepulsionForce += Vector2D(-strength, 0.0f);
        }
    }

    // --- 2. Calculate force from rear Ultrasonic sensor ---
    if (sysStatus.ultrasonicAvailable) {
        float rearDistanceCm = sensors.rearDistanceCm; // This is now being updated
        const float REAR_INFLUENCE_RADIUS = 30.0f; // 30cm
        const float REAR_REPULSION_STRENGTH = 25.0f;

        if (rearDistanceCm < REAR_INFLUENCE_RADIUS) {
            // Ultrasonic is at the rear (HAL standard: X-).
            // Repulsion force is forward (HAL standard: X+).
            float strength = REAR_REPULSION_STRENGTH * (1.0f - (rearDistanceCm / REAR_INFLUENCE_RADIUS));
            totalRepulsionForce += Vector2D(strength, 0.0f); // Correct
        }
    }

    return totalRepulsionForce;
}

float WheelieHAL::scaleVelocityToPWM(float velocity) {
    // Use the calibrated minimum PWM value if available, otherwise use a safe default.
    const float minPwmValue = isCalibrated ? (float)calibData.minMotorSpeedPWM : 180.0f;
    const float MAX_NAVIGATOR_VELOCITY_MM_S = 40.0f; // Maximum expected velocity from navigator
    const float MAX_PWM = 255.0f;          // Maximum PWM

    // Scale magnitude from [0, MAX_NAVIGATOR_VELOCITY_MM_S] to [minPwmValue, MAX_PWM]
    float scaledMagnitude = 0.0f;
    if (velocity > 0.1f) {  // Deadzone
      // Linear scaling with offset for minimum motor speed
      scaledMagnitude = minPwmValue + (velocity / MAX_NAVIGATOR_VELOCITY_MM_S) * (MAX_PWM - minPwmValue);
      scaledMagnitude = constrain(scaledMagnitude, minPwmValue, MAX_PWM);
    }
    return scaledMagnitude;
}

/**
 * @brief Applies motor PWM based on a target velocity and a turn correction.
 * This is called by the PID controller.
 * @param velocity The target velocity vector (magnitude and direction).
 * @param turnCorrection The corrective turning force from the PID controller.
 */
void WheelieHAL::setMotorsFromVector(const Vector2D& velocity, float turnCorrection) {
    float basePwm = 0;
    float magnitude = velocity.magnitude();

    if (magnitude > 0.1) {
        // Scale the PWM based on the magnitude of the velocity
        basePwm = scaleVelocityToPWM(magnitude);

        // CRITICAL FIX: Check the direction of the velocity vector.
        // If the x component is negative, the robot should move backward.
        if (velocity.x < 0) {
            basePwm = -basePwm;
        }
    }
    setMotorPWM(basePwm - turnCorrection, basePwm + turnCorrection);
}

RobotPose WheelieHAL::getPose() {
    // Return the internally-tracked pose
    return currentPose;
}

void WheelieHAL::autoDetectSensors() {
    Serial.println("🔎 HAL: Scanning I2C Bus...");
    // Wire.begin(I2C_SDA, I2C_SCL); // Moved to main.cpp setup()
    delay(100); // Allow I2C bus to settle (matches test sketch)
    
    byte count = 0;
    sysStatus.sensorsActive = 0;

    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("   ✓ Found I2C Device at 0x%02X", address);
            switch (address) {
                case 0x29: Serial.println(" -> ToF Sensor (VL53L0X)"); sysStatus.tofAvailable = true; break;
                case 0x68: Serial.println(" -> IMU (MPU6050)"); sysStatus.mpuAvailable = true; break;
                default: Serial.println(" -> Unknown Device"); break;
            }
            count++;
        }
    }
    
    if (count == 0) Serial.println("⚠️ No I2C devices found.");
    else Serial.printf("   📊 Scan Complete. Found %d I2C devices.\n", count);
    
    // Setup digital pins
    pinMode(SOUND_SENSOR_PIN, INPUT);
    pinMode(EDGE_SENSOR_PIN, INPUT_PULLUP);
    pinMode(PIR_SENSOR_PIN, INPUT);
    
    // Encoders are set up in calibration.cpp's setupEncoders()
    setupEncoders();

    // --- Non-I2C Sensor Availability ---
    // Test for Ultrasonic Sensor (HC-SR04) by sending a test ping.
    Serial.print("   🔧 Test Ultrasonic... ");
    pinMode(FRONT_ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ULTRASONIC_ECHO_PIN, INPUT);

    // Send a short pulse to trigger the sensor
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);

    // Wait for the echo pulse. pulseIn() will return 0 on timeout.
    // A timeout of 50ms (50000 us) is more than enough for any reasonable range.
    unsigned long duration = pulseIn(FRONT_ULTRASONIC_ECHO_PIN, HIGH, 50000);

    if (duration > 0) {
        sysStatus.ultrasonicAvailable = true;
        Serial.println("✅ Found Ultrasonic Sensor (HC-SR04)");
    } else {
        sysStatus.ultrasonicAvailable = false;
        Serial.println("❌ Ultrasonic Sensor not responding.");
    }
}

void WheelieHAL::initializeSensors() {
    autoDetectSensors();

    // Add a small delay to allow I2C devices to settle after the bus scan.
    delay(250);

    if (sysStatus.tofAvailable) {
        Serial.print("   🔧 Init ToF... ");
        this->tofSensor.setTimeout(500); // Use longer timeout for init, as in test sketch
        if (this->tofSensor.init()) {
            // Set timing budget for better accuracy and reliability in continuous mode
            this->tofSensor.setMeasurementTimingBudget(TOF_TIMING_BUDGET);

            Serial.println("✅ ToF sensor initialized successfully.");
            this->tofSensor.setTimeout(1); // Reduce timeout for normal operation
            this->tofSensor.startContinuous(0);
            sysStatus.sensorsActive++;
        } else {
            Serial.println("❌ ToF sensor initialization failed! Check wiring and power.");
            sysStatus.tofAvailable = false;
        }
    }

    if (sysStatus.mpuAvailable) {
        Serial.print("   🔧 Init IMU... ");
        // The 'false' parameter tells the library NOT to call Wire.begin() again,
        // as we have already initialized it in main.cpp. This prevents the
        // "Bus already started in Master Mode" warning and the resulting init failure.
        if (mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire, false)) {
            // Set a higher gyroscope range to prevent saturation during fast turns. The MPU6050_light
            // library uses integer values for this: 0=±250, 1=±500, 2=±1000, 3=±2000 dps.
            // The previous setting (2) was saturating. Set to max range to prevent this.
            // mpu.setGyroConfig(3); // Set gyro range from ±1000°/s to ±2000°/s

            // --- Set MPU6050 Configuration ---
            // The mpu.begin() function already wakes the sensor up from sleep mode.
            // Set the gyroscope range to its maximum to prevent saturation during fast turns.
            mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
            Serial.println("   - Gyro range set to ±2000°/s");
            
            if (isCalibrated && calibData.valid) {
                Serial.println("✅ Ready (Using Saved Calibration)");
                // mpu.setAccOffsets(calibData.mpuOffsets.accelX, calibData.mpuOffsets.accelY, calibData.mpuOffsets.accelZ);
                // mpu.setGyroOffsets(calibData.mpuOffsets.gyroX, calibData.mpuOffsets.gyroY, calibData.mpuOffsets.gyroZ);
            } else {
                 Serial.println("⚠️ Uncalibrated. Running automatic calibration...");
                 // Run the MPU calibration routine now. This is blocking but only runs once.
                 calibrateMPU(); 
            }
            sysStatus.sensorsActive++;
        } else {
            Serial.println("❌ Init Failed");
            sysStatus.mpuAvailable = false;
        }
    }

    if (sysStatus.ultrasonicAvailable) {
        Serial.print("   🔧 Init Ultrasonic... ");
        // Set pin modes for manual operation
        pinMode(FRONT_ULTRASONIC_TRIG_PIN, OUTPUT);
        pinMode(FRONT_ULTRASONIC_ECHO_PIN, INPUT);
        Serial.println("✅ Ready");
        sysStatus.sensorsActive++;
    }
    
    Serial.printf("   📊 %d active sensors initialized.\n", sysStatus.sensorsActive);
}

CalibrationResult WheelieHAL::calibrateMPU() {
    Serial.println("\n🔄 PHASE: MPU6050 Software Offset Calibration");
    Serial.println("──────────────────────────────────────────");
    
    if (!sysStatus.mpuAvailable) {
         Serial.println("❌ MPU not available for calibration.");
         return CALIB_ERR_SENSOR_INVALID;
    }

    Serial.println("📊 DO NOT MOVE ROBOT. Averaging sensor readings...");
    const int numSamples = 1000;
    // Use individual floats since Vector3D is not defined
    float accSumX = 0, accSumY = 0, accSumZ = 0;
    float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;

    for (int i = 0; i < numSamples; ++i) {
        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp)) {
            accSumX += a.acceleration.x;
            accSumY += a.acceleration.y;
            accSumZ += a.acceleration.z;
            gyroSumX += g.gyro.x;
            gyroSumY += g.gyro.y;
            gyroSumZ += g.gyro.z;
        }
        if (i % 100 == 0) {
            Serial.print(".");
        }
        delay(2); // Small delay between readings
    }
    Serial.println("\n✅ Averaging complete.");

    // Calculate the average offsets
    calibData.mpuOffsets.accelX = accSumX / numSamples;
    calibData.mpuOffsets.accelY = accSumY / numSamples;
    // The Z-axis should read ~9.8 m/s^2 (1g) when level. The offset is the difference.
    calibData.mpuOffsets.accelZ = (accSumZ / numSamples) - 9.81; 
    calibData.mpuOffsets.gyroX = gyroSumX / numSamples;
    calibData.mpuOffsets.gyroY = gyroSumY / numSamples;
    calibData.mpuOffsets.gyroZ = gyroSumZ / numSamples;

    Serial.println("📊 MPU Offsets Saved to Calibration Data:");
    Serial.printf("   Acc (X,Y,Z): %.3f, %.3f, %.3f\n", calibData.mpuOffsets.accelX, calibData.mpuOffsets.accelY, calibData.mpuOffsets.accelZ);
    Serial.printf("   Gyro(X,Y,Z): %.3f, %.3f, %.3f\n", calibData.mpuOffsets.gyroX, calibData.mpuOffsets.gyroY, calibData.mpuOffsets.gyroZ);
    
    // CRITICAL: Set the global flag to true so the offsets are applied.
    isCalibrated = true;

    // Now that the sensor is calibrated, establish the "zero" baseline for the current surface.
    calibrateSensorBaselines();

    return CALIB_SUCCESS;
}

void WheelieHAL::emergencyStop() {
    // A simple passthrough to the motor driver's emergency stop.
    // This can be expanded later if needed.
    ::stopWithBrake();
    setRobotState(ROBOT_ERROR); // Or a more specific safety state
    Serial.println("🚨 HAL: Emergency Stop Activated!");
}

// ═══════════════════════════════════════════════════════════════════════════
// HAL ACTUATION IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

void WheelieHAL::setVelocity(const Vector2D& velocity) {
    // Instead of directly setting motors, we just store the desired velocity.
    // The PID controller in the update() loop will handle the rest.
    targetVelocity = velocity;
}

Vector2D WheelieHAL::getVelocity() const {
    return targetVelocity;
}

void WheelieHAL::setMaxSpeed(float speedRatio) {
    // This is a conceptual function. The actual implementation
    // depends on how the navigator uses it. For now, it's a placeholder.
    Serial.printf("HAL: Max speed conceptually set to %.1f%%\n", speedRatio * 100);
}

void WheelieHAL::setLEDBrightness(int brightness) {
    // This would pass through to an LED driver if one existed.
    Serial.printf("HAL: LED brightness conceptually set to %d\n", brightness);
}

// ═══════════════════════════════════════════════════════════════════════════
// HAL UTILITY IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

void WheelieHAL::setStatusLED(const LEDColor& color) {
    setLEDColor(color); // Passthrough
}

void WheelieHAL::playTone(int frequency, int duration) {
    ::playTone(frequency, duration); // Passthrough
}

float WheelieHAL::getBatteryVoltage() {
    return battery.voltage; // Hook into power_manager
}