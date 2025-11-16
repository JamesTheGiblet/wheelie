#include "WheelieHAL.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// --- Include all the necessary HAL component headers ---
#include "indicators.h"
#include "power_manager.h"
#include "motors.h"
#include "wifi_manager.h"
// OTA removed
// #include "espnow_manager.h" // REMOVED: This is now handled by SwarmCommunicator
#include "logger.h"
#include "calibration.h"
#include "main.h" // For setRobotState
#include <MPU6050_light.h>

// --- Global Hardware Objects (now owned by the HAL) ---
VL53L0X tofSensor;
MPU6050 mpu(Wire);
// OTA removed

// --- Global System State (accessed by HAL) ---
extern SystemStatus sysStatus;
extern SensorData sensors;
extern CalibrationData calibData;
extern SensorHealth_t sensorHealth;
extern bool isCalibrated;

// --- Internal State for Ultrasonic Filtering ---
const int ULTRASONIC_FILTER_SIZE = 5;
float ultrasonicReadings[ULTRASONIC_FILTER_SIZE] = {0};
int ultrasonicReadingIndex = 0;
bool ultrasonicFilterPrimed = false;

// --- Internal State for Non-Blocking Ultrasonic ---
enum UltrasonicState { US_IDLE, US_TRIGGERED, US_ECHO_IN_PROGRESS };
UltrasonicState ultrasonicState = US_IDLE;
unsigned long ultrasonicTriggerTime = 0;
unsigned long ultrasonicEchoStartTime = 0;
const unsigned long ULTRASONIC_TIMEOUT_US = 38000; // 38ms, matches sensor's max range
const unsigned long ULTRASONIC_READ_INTERVAL = 50; // Read every 50ms
unsigned long nextUltrasonicReadTime = 0;

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
    // Non-blocking wait for Serial to initialize
    unsigned long serialStartTime = millis();
    while (!Serial && (millis() - serialStartTime < 1000)) {
        // Wait for serial connection or timeout
    }

    // --- Core System Initialization ---
    setRobotState(ROBOT_BOOTING);
    setupIndicators(); // Correct
    startupAnimation(); // This is now non-blocking
    // Manually update indicators to show the first step of the animation
    indicators_update(); 

    initializePowerManagement(); // Was setupPowerManager
    setupMotors(); // Correct
    initializeWiFi(); // Was setupWiFi
    // OTA removed
    // initializeESPNow(); // REMOVED: SwarmCommunicator handles this in main::setup()
    initializeLogging(); // Was setupLogger
    
    // --- Sensor Auto-Discovery & Init ---
    this->initializeSensors(); // This now runs autoDetectSensors()

    // --- Calibration ---
    CalibrationResult loadResult = loadCalibrationData();
    if (shouldForceRecalibration() || loadResult != CALIB_SUCCESS) {
        Serial.println("WARN: No calibration data found. Running auto-calibration.");
        setRobotState(ROBOT_CALIBRATING);

        // MPU calibration must happen before the main sequence
        if (calibrateMPU() != CALIB_SUCCESS) {
            Serial.println("❌ CRITICAL: MPU calibration failed. Robot halted.");
            setRobotState(ROBOT_ERROR);
            return false; // Init failed
        }

        // IMPORTANT: Wait for the MPU's DMP to stabilize after calibration
        Serial.println("   ⏳ Waiting for MPU to stabilize...");
        unsigned long mpuStabilizeTime = millis();
        while (millis() - mpuStabilizeTime < 1000) {
            // Allow background tasks to run during stabilization
            indicators_update();
        }
        Serial.println("   ✅ MPU stable.");

        // After MPU calibration, establish the "zero" angle baselines
        Serial.println("📊 Establishing zero-angle baseline for IMU...");
        Serial.println("   [DEBUG] Before mpu.update()");
        // This is a two-step process to avoid a NaN result.
        // 1. Get a raw reading first to establish the baseline.
        mpu.update();
        Serial.println("   [DEBUG] After mpu.update()");
        Serial.println("   [DEBUG] Before getAngleX()");
        calibData.mpuOffsets.baselineTiltX = mpu.getAngleX();
        Serial.println("   [DEBUG] After getAngleX()");
        Serial.println("   [DEBUG] Before getAngleY()");
        calibData.mpuOffsets.baselineTiltY = mpu.getAngleY();
        Serial.println("   [DEBUG] After getAngleY()");

        // 2. Now, subsequent calls to updateAllSensors() will correctly subtract this valid baseline.
        Serial.println("   [DEBUG] Before this->updateAllSensors()");
        this->updateAllSensors();
        Serial.println("   [DEBUG] After this->updateAllSensors()");
        Serial.printf("   ✅ Baseline established. Tilt X: %.2f, Tilt Y: %.2f\n", calibData.mpuOffsets.baselineTiltX, calibData.mpuOffsets.baselineTiltY);

        if (runFullCalibrationSequence() == CALIB_SUCCESS) { // This now saves automatically
            Serial.println("✅ Calibration successful. Proceeding to normal operation...");
            // No reboot needed. The init function will now continue.
        } else {
            Serial.println("❌ CRITICAL: Calibration failed. Robot halted.");
            setRobotState(ROBOT_ERROR);
            return false; // Init failed
        }
    }

    // --- Odometry Baseline ---
    updateAllSensors(); // Get initial sensor readings
    lastLeftEncoder = sensors.leftEncoderCount;
    lastRightEncoder = sensors.rightEncoderCount;
    lastHeading = sensors.headingAngle;
    currentPose.heading = lastHeading;

    Serial.println("✅ WheelieHAL Initialized.");
    return true; // Init successful
}

void WheelieHAL::update() {
    // --- Poll Hardware ---
    updateAllSensors(); // Read ToF, IMU, Digital, Encoders
    updateOdometry();   // Update internal pose (x, y, heading) from sensor data

    // --- Run Background System Tasks ---
    monitorPower();       // Was updatePowerManager
    // OTA removed
    // performESPNowMaintenance(); // REMOVED: SwarmCommunicator handles this in main::loop()
    indicators_update();  // Was updateIndicators
    // periodicDataLogging(); // This is now handled by loggerTask on Core 0
}

// ═══════════════════════════════════════════════════════════════════════════
// HAL SENSING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

void WheelieHAL::updateAllSensors() {
    // This is the logic moved from the old `sensors.cpp`
    if (sysStatus.tofAvailable) {
        // This function will now only block for a very short time (1ms)
        // because of the timeout set in initializeSensors().
        int frontDistanceMm = tofSensor.readRangeContinuousMillimeters();
        if (tofSensor.timeoutOccurred()) {
            sensors.frontDistanceCm = 819.0f; // Use a standard "out of range" value on timeout
        } else {
            sensors.frontDistanceCm = frontDistanceMm / 10.0f; // Convert mm to cm
        }
    }

    if (sysStatus.mpuAvailable) {
        mpu.update();
        sensors.tiltX = mpu.getAngleX() - calibData.mpuOffsets.baselineTiltX;
        sensors.tiltY = mpu.getAngleY() - calibData.mpuOffsets.baselineTiltY;
        sensors.headingAngle = mpu.getAngleZ();
        sensors.gyroZ = mpu.getGyroZ(); // Populate the new gyroZ field
    }

    if (sysStatus.ultrasonicAvailable) {
        unsigned long currentTime = millis();
        unsigned long currentMicros = micros();

        // State 1: Time to start a new reading
        if (ultrasonicState == US_IDLE && currentTime >= nextUltrasonicReadTime) {
            digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, HIGH);
            ultrasonicTriggerTime = currentMicros;
            ultrasonicState = US_TRIGGERED;
        }

        // State 2: End the trigger pulse after 10us
        if (ultrasonicState == US_TRIGGERED && currentMicros - ultrasonicTriggerTime >= 10) {
            digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);
            ultrasonicState = US_ECHO_IN_PROGRESS;
            ultrasonicEchoStartTime = currentMicros; // Start timeout timer
        }

        // State 3: Wait for the echo pulse to finish
        if (ultrasonicState == US_ECHO_IN_PROGRESS) {
            // If echo pin is high, we are measuring. If it goes low, the pulse is over.
            if (digitalRead(FRONT_ULTRASONIC_ECHO_PIN) == LOW) {
                long duration_us = currentMicros - ultrasonicEchoStartTime;
                
                // The first few micros after trigger are noise, so ignore very short pulses.
                if (duration_us > 100) { 
                    float newReading = (duration_us * 0.0343 / 2.0);
                    if (newReading > 2.0) { // Ignore readings closer than 2cm
                        ultrasonicReadings[ultrasonicReadingIndex] = newReading;
                        ultrasonicReadingIndex = (ultrasonicReadingIndex + 1) % ULTRASONIC_FILTER_SIZE;
                        if (!ultrasonicFilterPrimed && ultrasonicReadingIndex == 0) {
                            ultrasonicFilterPrimed = true;
                        }
                    }
                }
                ultrasonicState = US_IDLE; // Reading complete, go back to idle
                nextUltrasonicReadTime = currentTime + ULTRASONIC_READ_INTERVAL;
            } 
            // Timeout: If we wait too long for the echo, abort the reading.
            else if (currentMicros - ultrasonicEchoStartTime > ULTRASONIC_TIMEOUT_US) {
                ultrasonicState = US_IDLE; // Abort and go back to idle
                nextUltrasonicReadTime = currentTime + ULTRASONIC_READ_INTERVAL;
            }
        }

        // This part remains the same: always calculate the average from the buffer.
        float total = 0;
        int numReadings = ultrasonicFilterPrimed ? ULTRASONIC_FILTER_SIZE : ultrasonicReadingIndex;
        if (numReadings > 0) {
            for (int i = 0; i < numReadings; i++) {
                total += ultrasonicReadings[i];
            }
            sensors.rearDistanceCm = total / numReadings;
        }
    }
    
    // Encoder counts are updated by ISRs, just read them.
    sensors.leftEncoderCount = getLeftEncoderCount();
    sensors.rightEncoderCount = getRightEncoderCount();

    sensors.soundDetected = (digitalRead(SOUND_SENSOR_PIN) == HIGH);
    sensors.edgeDetected = (digitalRead(EDGE_SENSOR_PIN) == HIGH);
    sensors.motionDetected = (digitalRead(PIR_SENSOR_PIN) == HIGH);
}

void WheelieHAL::updateOdometry() {
    // This logic is moved from the old `main.cpp`
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
        float frontDistanceCm = sensors.frontDistanceCm;
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
        float rearDistanceCm = sensors.rearDistanceCm;
        const float REAR_INFLUENCE_RADIUS = 30.0f; // 30cm
        const float REAR_REPULSION_STRENGTH = 25.0f;

        if (rearDistanceCm < REAR_INFLUENCE_RADIUS) {
            // Ultrasonic is at the rear (HAL standard: X-).
            // Repulsion force is forward (HAL standard: X+).
            float strength = REAR_REPULSION_STRENGTH * (1.0f - (rearDistanceCm / REAR_INFLUENCE_RADIUS));
            totalRepulsionForce += Vector2D(strength, 0.0f);
        }
    }

    return totalRepulsionForce;
}

RobotPose WheelieHAL::getPose() {
    // Return the internally-tracked pose
    return currentPose;
}

void WheelieHAL::autoDetectSensors() {
    Serial.println("🔎 HAL: Scanning I2C Bus...");
    Wire.begin(I2C_SDA, I2C_SCL);
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
    // We assume the ultrasonic sensor is present if wired.
    sysStatus.ultrasonicAvailable = true;
    Serial.println("   ✓ Found Ultrasonic Sensor (HC-SR04)");
}

void WheelieHAL::initializeSensors() {
    autoDetectSensors();

    if (sysStatus.tofAvailable) {
        Serial.print("   🔧 Init ToF... ");
        tofSensor.setTimeout(500); // Use longer timeout for init, as in test sketch
        if (tofSensor.init()) {
            Serial.println("✅ ToF sensor initialized successfully.");
            tofSensor.setTimeout(1); // Reduce timeout for normal operation
            tofSensor.startContinuous(0);
            sysStatus.sensorsActive++;
        } else {
            Serial.println("❌ ToF sensor initialization failed! Check wiring and power.");
            sysStatus.tofAvailable = false;
        }
    }

    if (sysStatus.mpuAvailable) {
        Serial.print("   🔧 Init IMU... ");
        if (mpu.begin() == 0) {
            // Set a higher gyroscope range to prevent saturation during fast turns. The MPU6050_light
            // library uses integer values for this: 0=250, 1=500, 2=1000, 3=2000 dps.
            // 1000 dps is a robust choice for this robot.
            mpu.setGyroConfig(2); // Set gyro range to ±1000°/s

            if (isCalibrated && calibData.valid) {
                Serial.println("✅ Ready (Applying Saved Calibration)");
                mpu.setAccOffsets(calibData.mpuOffsets.accelX, calibData.mpuOffsets.accelY, calibData.mpuOffsets.accelZ);
                mpu.setGyroOffsets(calibData.mpuOffsets.gyroX, calibData.mpuOffsets.gyroY, calibData.mpuOffsets.gyroZ);
            } else {
                 Serial.println("⚠️ Ready (Uncalibrated, using defaults)");
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
    Serial.println("\n🔄 PHASE: MPU6050 Offset Calibration");
    Serial.println("──────────────────────────────────────────");
    
    if (!sysStatus.mpuAvailable) {
         Serial.println("❌ MPU not available for calibration.");
         return CALIB_ERR_SENSOR_INVALID;
    }
    
    Serial.println("📊 DO NOT MOVE ROBOT. Calibrating MPU (Acc & Gyro)...");
    Serial.println("   This will take about a minute...");
    
    // CRITICAL: Set MPU to a known, zeroed state before calibration
    mpu.setAccOffsets(0, 0, 0);
    mpu.setGyroOffsets(0, 0, 0);
    // Non-blocking delay to allow settings to apply
    unsigned long applyTime = millis();
    while(millis() - applyTime < 100) { indicators_update(); }
    
    // This function from the MPU6050_light library does all the work.
    // NOTE: This is an inherently blocking call, which is acceptable for a one-time setup routine.
    mpu.calcOffsets(true, true); // true, true = print debug info
    
    Serial.println("\n✅ MPU offset calculation complete.");

    // Retrieve the calculated offsets and store them in our struct
    calibData.mpuOffsets.accelX = mpu.getAccXoffset();
    calibData.mpuOffsets.accelY = mpu.getAccYoffset();
    calibData.mpuOffsets.accelZ = mpu.getAccZoffset();
    calibData.mpuOffsets.gyroX = mpu.getGyroXoffset();
    calibData.mpuOffsets.gyroY = mpu.getGyroYoffset();
    calibData.mpuOffsets.gyroZ = mpu.getGyroZoffset();

    // CRITICAL: Re-apply these offsets to the sensor immediately to ensure they are active.
    mpu.setAccOffsets(calibData.mpuOffsets.accelX, calibData.mpuOffsets.accelY, calibData.mpuOffsets.accelZ);
    mpu.setGyroOffsets(calibData.mpuOffsets.gyroX, calibData.mpuOffsets.gyroY, calibData.mpuOffsets.gyroZ);

    // Add a longer delay here to allow the MPU's internal DMP (Digital Motion Processor)
    // to fully stabilize with the new offsets before we start taking readings.
    unsigned long stabilizeTime = millis();
    while(millis() - stabilizeTime < 1000) {
        indicators_update(); // Keep indicators active during wait
    }
    
    Serial.println("📊 MPU Offsets Saved to Calibration Data:");
    Serial.printf("   Acc: X=%d, Y=%d, Z=%d\n", calibData.mpuOffsets.accelX, calibData.mpuOffsets.accelY, calibData.mpuOffsets.accelZ);
    Serial.printf("   Gyro: X=%d, Y=%d, Z=%d\n", calibData.mpuOffsets.gyroX, calibData.mpuOffsets.gyroY, calibData.mpuOffsets.gyroZ);
    
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
    // This is the "Translator" for Wheelie's differential drive.
    // It converts the Brain's desired HAL-standard vector (X=fwd, Y=left)
    // into Left/Right PWM signals.

    // 1. Decompose the vector
    float forward = velocity.x; // X+ is Forward
    float turn = velocity.y;    // Y+ is Left

    // 2. Mix for differential drive
    // To turn Left (Y+), we need Left wheel slower, Right wheel faster.
    int pwmLeft = (int)(forward - turn);
    int pwmRight = (int)(forward + turn);
    
    // 3. Send to low-level motor driver
    setMotorPWM(pwmLeft, pwmRight);
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