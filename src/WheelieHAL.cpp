#include "WheelieHAL.h"
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <VL53L0X.h>
#include <MPU6050_light.h>

// --- Include all the necessary HAL component headers ---
#include "indicators.h"
#include "power_manager.h"
#include "motors.h"
#include "wifi_manager.h"
#include "ota_manager.h"
#include "espnow_manager.h"
#include "logger.h"
#include "calibration.h"
#include "main.h" // For setRobotState

// --- Global Hardware Objects (now owned by the HAL) ---
VL53L0X tofSensor;
MPU6050 mpu(Wire);
// ArduinoOTA does not require instantiation; use its static methods directly.

// --- Global System State (accessed by HAL) ---
extern SystemStatus sysStatus;
extern SensorData sensors;
extern CalibrationData calibData;
extern SensorHealth_t sensorHealth;
extern bool isCalibrated;

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
    delay(1000);

    // --- Core System Initialization ---
    setRobotState(ROBOT_BOOTING);
    setupIndicators(); // Correct
    startupAnimation(); // Was playBootSequence
    initializePowerManagement(); // Was setupPowerManager
    setupMotors(); // Correct
    initializeWiFi(); // Was setupWiFi
    initializeOTA(); // Was setupOTA
    initializeESPNow(); // Was setupESPNOW
    initializeLogging(); // Was setupLogger
    
    // --- Sensor Auto-Discovery & Init ---
    this->initializeSensors(); // This now runs autoDetectSensors()

    // --- Calibration ---
    CalibrationResult loadResult = loadCalibrationData();
    if (loadResult != CALIB_SUCCESS || shouldForceRecalibration()) {
        Serial.println("WARN: No calibration data found. Running auto-calibration.");
        setRobotState(ROBOT_CALIBRATING);

        // MPU calibration must happen before the main sequence
        if (calibrateMPU() != CALIB_SUCCESS) {
            Serial.println("❌ CRITICAL: MPU calibration failed. Robot halted.");
            setRobotState(ROBOT_ERROR);
            return false; // Init failed
        }

        // After MPU calibration, establish the "zero" angle baselines
        Serial.println("📊 Establishing zero-angle baseline for IMU...");
        this->updateAllSensors(); // Get a fresh reading
        delay(100);
        this->updateAllSensors(); // Get another to be sure
        calibData.mpuOffsets.baselineTiltX = sensors.tiltX;
        calibData.mpuOffsets.baselineTiltY = sensors.tiltY;
        Serial.printf("   ✅ Baseline established. Tilt X: %.2f, Tilt Y: %.2f\n", calibData.mpuOffsets.baselineTiltX, calibData.mpuOffsets.baselineTiltY);

        if (runFullCalibrationSequence() == CALIB_SUCCESS) {
            saveCalibrationData();
            Serial.println("✅ Calibration successful. Rebooting...");
            delay(1000);
            ESP.restart();
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
    ArduinoOTA.handle();
    performESPNowMaintenance(); // Was updateESPNOW
    indicators_update();  // Was updateIndicators
    periodicDataLogging(); // Was logData
}

// ═══════════════════════════════════════════════════════════════════════════
// HAL SENSING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

void WheelieHAL::updateAllSensors() {
    // This is the logic moved from the old `sensors.cpp`
    if (sysStatus.tofAvailable) {
        sensors.distance = tofSensor.readRangeContinuousMillimeters();
        if (tofSensor.timeoutOccurred()) {
            sensors.distance = 8190;
        }
    }

    if (sysStatus.mpuAvailable) {
        mpu.update();
        sensors.tiltX = mpu.getAngleX() - calibData.mpuOffsets.baselineTiltX;
        sensors.tiltY = mpu.getAngleY() - calibData.mpuOffsets.baselineTiltY;
        sensors.headingAngle = mpu.getAngleZ();
        sensors.gyroZ = mpu.getGyroZ(); // Populate the new gyroZ field
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
    
    // 1. Get Wheelie's single ToF sensor reading
    float distance = (float)sensors.distance; // in mm
    
    // 2. Define parameters
    const float INFLUENCE_RADIUS = 500.0f; // 50cm
    const float REPULSION_STRENGTH = 20.0f; // Tunable
    
    // 3. Calculate force
    if (distance < INFLUENCE_RADIUS) {
        // Sensor is forward (HAL standard: X+).
        // Repulsion force is backward (HAL standard: X-).
        float strength = REPULSION_STRENGTH * (1.0f - (distance / INFLUENCE_RADIUS));
        return Vector2D(-strength, 0.0f);
    }

    return Vector2D(0, 0); // No obstacle
}

RobotPose WheelieHAL::getPose() {
    // Return the internally-tracked pose
    return currentPose;
}

void WheelieHAL::autoDetectSensors() {
    Serial.println("🔎 HAL: Scanning I2C Bus...");
    Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK);
    
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
}

void WheelieHAL::initializeSensors() {
    autoDetectSensors();

    if (sysStatus.tofAvailable) {
        Serial.print("   🔧 Init ToF... ");
        tofSensor.setTimeout(500);
        if (tofSensor.init()) {
            tofSensor.startContinuous();
            tofSensor.readRangeContinuousMillimeters(); 
            if(tofSensor.timeoutOccurred()) {
                 Serial.println("⚠️ Timeout (Warning)");
            } else {
                 Serial.println("✅ Ready");
                 sysStatus.sensorsActive++;
            }
        } else {
            Serial.println("❌ Init Failed");
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
    
    // Set MPU to a known state before calibration
    mpu.setAccOffsets(0, 0, 0);
    mpu.setGyroOffsets(0, 0, 0);
    
    // This function from the MPU6050_light library does all the work.
    mpu.calcOffsets(true, true); // true, true = print debug info
    
    Serial.println("\n✅ MPU offset calculation complete.");

    // Retrieve the calculated offsets and store them in our struct
    calibData.mpuOffsets.accelX = mpu.getAccXoffset();
    calibData.mpuOffsets.accelY = mpu.getAccYoffset();
    calibData.mpuOffsets.accelZ = mpu.getAccZoffset();
    calibData.mpuOffsets.gyroX = mpu.getGyroXoffset();
    calibData.mpuOffsets.gyroY = mpu.getGyroYoffset();
    calibData.mpuOffsets.gyroZ = mpu.getGyroZoffset();

    // Apply these offsets to the sensor immediately
    mpu.setAccOffsets(calibData.mpuOffsets.accelX, calibData.mpuOffsets.accelY, calibData.mpuOffsets.accelZ);
    mpu.setGyroOffsets(calibData.mpuOffsets.gyroX, calibData.mpuOffsets.gyroY, calibData.mpuOffsets.gyroZ);
    
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