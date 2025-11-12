#include "WheelieHAL.h"
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoOTA.h>

// --- Include all the necessary HAL component headers ---
#include "indicators.h"
#include "power_manager.h"
#include "motors.h"
#include "wifi_manager.h"
#include "ota_manager.h"
#include "espnow_manager.h"
#include "logger.h"
#include "calibration.h"
#include "robot.h" // For setRobotState

// --- Global Hardware Objects (now owned by the HAL) ---
VL53L0X tofSensor;
MPU6050 mpu(Wire);
// ArduinoOTA does not require instantiation; use its static methods directly.

// --- Global System State (accessed by HAL) ---
extern SystemStatus sysStatus;
extern SensorData sensors;
extern CalibrationData calibData;
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
    initializeSensors(); // This now runs autoDetectSensors()

    // --- Calibration ---
    CalibrationResult loadResult = loadCalibrationData();
    if (loadResult != CALIB_SUCCESS || shouldForceRecalibration()) {
        Serial.println("WARN: No calibration data found. Running auto-calibration.");
        setRobotState(ROBOT_CALIBRATING);
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
    pollSensors(); // Get initial sensor readings
    lastLeftEncoder = sensors.leftEncoderCount;
    lastRightEncoder = sensors.rightEncoderCount;
    lastHeading = sensors.headingAngle;
    currentPose.heading = lastHeading;

    Serial.println("✅ WheelieHAL Initialized.");
    return true; // Init successful
}

void WheelieHAL::update() {
    // --- Poll Hardware ---
    pollSensors();      // Read ToF, IMU, Digital
    updateOdometry();   // Update internal pose (x, y, heading)

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

void WheelieHAL::pollSensors() {
    // This is the logic moved from the old `sensors.cpp`
    if (sysStatus.tofAvailable) {
        sensors.distance = tofSensor.readRangeContinuousMillimeters();
        if (tofSensor.timeoutOccurred()) {
            sensors.distance = 8190;
        }
    }

    if (sysStatus.mpuAvailable) {
        mpu.update();
        sensors.tiltX = mpu.getAngleX();
        sensors.tiltY = mpu.getAngleY();
        sensors.headingAngle = mpu.getAngleZ();
    }
    
    // TODO: Read encoders via interrupts
    // sensors.leftEncoderCount = ...
    // sensors.rightEncoderCount = ...

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
    float avgHeadingRad = avgHeading * M_PI / 180.0;

    // Convert from robot-centric (forward) to world-centric (x, y)
    // Per HAL_DOCUMENTATION.md: X+ is Forward, Y+ is Left
    // float deltaX = deltaDistance * cos(avgHeadingRad); // This is standard math (0-deg=East)
    // float deltaY = deltaDistance * sin(avgHeadingRad);
    // Let's stick to the HAL doc's coordinate system: 90=Forward
    float avgHeadingRad_HAL = avgHeading * M_PI / 180.0; // Assuming 0=East, 90=North
    
    currentPose.position.x += deltaDistance * cos(avgHeadingRad_HAL);
    currentPose.position.y += deltaDistance * sin(avgHeadingRad_HAL);
    
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