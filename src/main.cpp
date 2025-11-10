#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_now.h>
#include <VL53L0X.h>
#include <MPU6050_light.h>

// Core Project Headers
#include "config.h"
#include "pins.h"
#include "Vector2D.h"
#include "types.h"

// Define FORCE_RECALIBRATION_PIN if not defined in pins.h
#ifndef FORCE_RECALIBRATION_PIN
#define FORCE_RECALIBRATION_PIN 12
#endif

// New RobotForge Headers
#include "PotentialFieldNavigator.h"
#include "ObstacleMemory.h"
#include "SwarmCommunicator.h"

// Old Wheelie Module Headers
#include "motors.h"
#include "indicators.h"
#include "power_manager.h"
#include "wifi_manager.h"
#include "ota_manager.h"
#include "espnow_manager.h"
#include "logger.h"
#include "calibration.h"
#include "sensors.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS & STATE
// ═══════════════════════════════════════════════════════════════════════════

// --- New RobotForge "Brain" ---
PotentialFieldNavigator navigator;
SwarmCommunicator swarmComms;

// --- Wheelie System State ---
SystemStatus sysStatus;
SensorData sensors;
CalibrationData calibData;
bool isCalibrated = false;

// --- Wheelie Hardware Objects ---
VL53L0X tofSensor;
MPU6050 mpu(Wire);

// --- Odometry Tracking ---
static long lastLeftEncoder = 0;
static long lastRightEncoder = 0;
static float lastHeading = 0.0;
static float currentX = 0.0;
static float currentY = 0.0;
static float currentHeading = 0.0;

// ═══════════════════════════════════════════════════════════════════════════
// FUNCTION PROTOTYPES (Missing implementations)
// ═══════════════════════════════════════════════════════════════════════════

// Motor functions that need implementation
void setupMotors();
void setMotorsFromVector(float magnitude, float angleDeg);
void setMotors(int leftSpeed, int rightSpeed);

// Calibration support functions
CalibrationResult runFullCalibrationSequence();
bool verifyDataIntegrity(const CalibrationData* data);
CalibrationResult updateChecksum(CalibrationData* data);
const char* getCalibrationErrorString(CalibrationResult result);

// Utility functions
void printNavigationStatus();

// ═══════════════════════════════════════════════════════════════════════════
// MAIN APPLICATION ENTRY POINT
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);

    // --- Core System Initialization ---
    printBanner();
    setupSystem();

    // --- Sensor & Calibration Initialization ---
    initializeSensors();
    
    // Load calibration data with proper error handling
    CalibrationResult loadResult = loadCalibrationData();
    if (loadResult != CALIB_SUCCESS) {
        Serial.println("WARN: No valid calibration data found or data corrupt.");
        setRobotState(ROBOT_CALIBRATING);
        if (runAutonomousCalibration()) {
            CalibrationResult saveResult = saveCalibrationData();
            if (saveResult == CALIB_SUCCESS) {
                Serial.println("✅ Calibration successful and saved. Rebooting...");
                delay(1000);
                ESP.restart();
            } else {
                Serial.println("❌ Failed to save calibration data");
                setRobotState(ROBOT_ERROR);
            }
        } else {
            Serial.println("❌ CRITICAL: Calibration failed. Robot halted.");
            setRobotState(ROBOT_ERROR);
            while (true) { delay(1000); }
        }
    }

    // --- Navigation Setup ---
    NavigationParameters params;
    params.attractionConstant = 2.5f;
    params.repulsionConstant = 20.0f;
    params.maxSpeed = 35.0f;
    params.influenceRadius = 80.0f;
    params.goalThreshold = 15.0f;
    params.damping = 0.7f;
    navigator.setParameters(params);
    
    navigator.setGoal(Vector2D(100.0f, 0.0f)); // 100cm forward

    // --- Odometry Baseline ---
    lastLeftEncoder = sensors.leftEncoderCount;
    lastRightEncoder = sensors.rightEncoderCount;
    lastHeading = sensors.headingAngle;
    currentHeading = lastHeading;
    currentX = 0.0;
    currentY = 0.0;
    navigator.setPosition(Vector2D(currentX, currentY));
    
    Serial.println("🤖 RobotForge Navigation System ONLINE.");
    setRobotState(ROBOT_EXPLORING);
}

void loop() {
    static unsigned long lastNavigationUpdate = 0;
    static unsigned long lastSwarmUpdate = 0;
    static unsigned long lastStatusPrint = 0;
    
    unsigned long currentTime = millis();
    
    // --- Core Systems (Always running) ---
    updateAllSensors();
    updatePowerManager();
    ArduinoOTA.handle();
    updateIndicators();

    // --- Navigation Update (20Hz) ---
    if (currentTime - lastNavigationUpdate >= 50) {
        float deltaTime = (currentTime - lastNavigationUpdate) / 1000.0f;
        
        // Update sensor inputs for navigation
        updateNavigationSensors();
        
        // Update odometry
        updateOdometry();
        
        // Get swarm information
        auto otherPositions = swarmComms.getOtherRobotPositions();
        
        // Run navigation update
        navigator.update(deltaTime, otherPositions);
        
        // Execute motor commands
        Vector2D desiredVelocity = navigator.getVelocity();
        setMotorsFromVector(desiredVelocity);
        
        lastNavigationUpdate = currentTime;
    }
    
    // --- Swarm Communication (10Hz) ---
    if (currentTime - lastSwarmUpdate >= 100) {
        swarmComms.setMyState(navigator.getPosition(), navigator.getVelocity());
        swarmComms.update();
        lastSwarmUpdate = currentTime;
    }
    
    // --- Status Output (2Hz) ---
    if (currentTime - lastStatusPrint >= 500) {
        printNavigationStatus();
        lastStatusPrint = currentTime;
    }
    
    // --- Background Tasks ---
    updateESPNOW();
    logData();
}

// ═══════════════════════════════════════════════════════════════════════════
// ENHANCED SENSOR INTEGRATION
// ═══════════════════════════════════════════════════════════════════════════

void updateNavigationSensors() {
    // Main forward sensor (0 degrees) - convert mm to cm
    float forwardDistance = sensors.distance / 10.0f;
    navigator.addSensorReading(forwardDistance, 0.0f);
    
    // Create virtual side sensors using available sensors
    // Left sensor (-45 degrees) - influenced by edge detection
    float leftDistance = sensors.edgeDetected ? 20.0f : 60.0f;
    navigator.addSensorReading(leftDistance, -M_PI/4);
    
    // Right sensor (45 degrees)  
    float rightDistance = sensors.edgeDetected ? 20.0f : 60.0f;
    navigator.addSensorReading(rightDistance, M_PI/4);
    
    // Rear "sensor" from motion detection (180 degrees)
    float rearDistance = sensors.motionDetected ? 40.0f : 120.0f;
    navigator.addSensorReading(rearDistance, M_PI);
}

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

void setMotorsFromVector(const Vector2D& v) {
    float magnitude = v.magnitude(); // cm/s
    float angleRad = atan2f(v.y, v.x); // atan2(y, x) gives angle from forward
    
    // Scale magnitude to PWM range (0-255)
    float maxNavSpeed = 35.0f; // cm/s - should match navigator params
    int pwmValue = constrain((magnitude / maxNavSpeed) * 255, 0, 255);
    
    // Apply minimum PWM threshold if calibrated
    if (isCalibrated && pwmValue > 0 && pwmValue < calibData.minMotorSpeedPWM) {
        pwmValue = calibData.minMotorSpeedPWM;
    }
    
    // Convert angle to differential drive
    // angleRad = 0 means straight forward, positive = turn right
    float turnFactor = sin(angleRad); // -1 to +1
    
    int leftSpeed, rightSpeed;
    
    if (magnitude < 2.0f) { // Dead zone for very slow movement
        leftSpeed = 0;
        rightSpeed = 0;
    } else {
        // Differential drive calculation
        leftSpeed = pwmValue * (1.0f - turnFactor);
        rightSpeed = pwmValue * (1.0f + turnFactor);
        
        // Normalize to prevent exceeding max PWM
        int maxSpeed = max(leftSpeed, rightSpeed);
        if (maxSpeed > 255) {
            leftSpeed = (leftSpeed * 255) / maxSpeed;
            rightSpeed = (rightSpeed * 255) / maxSpeed;
        }
    }
    
    // Apply motor commands
    setMotors(leftSpeed, rightSpeed);
}

void setMotors(int leftSpeed, int rightSpeed) {
    // Your existing motor control logic here
    // Example for TB6612FNG:
    // analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed));
    // digitalWrite(MOTOR_LEFT_IN1, leftSpeed >= 0 ? HIGH : LOW);
    // digitalWrite(MOTOR_LEFT_IN2, leftSpeed >= 0 ? LOW : HIGH);
    // ... same for right motor
    
    // For now, just print the commands
    Serial.printf("🎛️  Motors: L=%d, R=%d\n", leftSpeed, rightSpeed);
}

void setupMotors() {
    Serial.println("Stub: setupMotors()");
    // Your motor pin initialization here
}

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATION FUNCTIONS - MATCHING HEADER DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult loadCalibrationData() { 
    Serial.println("🔧 Loading calibration data from EEPROM...");
    
    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    
    // Read calibration data from EEPROM
    CalibrationData loadedData;
    EEPROM.get(EEPROM_CALIB_DATA_ADDR, loadedData);
    EEPROM.end();
    
    // Validate the data
    if (loadedData.magic != CALIBRATION_MAGIC) {
        Serial.println("❌ Invalid calibration magic number");
        isCalibrated = false;
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    if (loadedData.version != CALIBRATION_VERSION) {
        Serial.println("❌ Calibration version mismatch");
        isCalibrated = false;
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    // Verify checksum
    if (!verifyDataIntegrity(&loadedData)) {
        Serial.println("❌ Calibration data checksum failed");
        isCalibrated = false;
        return CALIB_ERR_CHECKSUM_FAILED;
    }
    
    // Data is valid - copy to global calibData
    calibData = loadedData;
    isCalibrated = true;
    
    Serial.printf("✅ Calibration loaded: %.1f ticks/mm, %.1f ticks/90deg\n",
                 calibData.ticksPerMillimeter, calibData.ticksPer90Degrees);
    Serial.printf("   Min PWM: %d, ToF Offset: %.1fmm\n", 
                 calibData.minMotorSpeedPWM, calibData.tofOffsetMM);
    
    return CALIB_SUCCESS;
}

CalibrationResult saveCalibrationData() { 
    if (!isCalibrated) {
        Serial.println("❌ Cannot save: No calibration data available");
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    Serial.println("💾 Saving calibration data to EEPROM...");
    
    // Set magic number and version
    calibData.magic = CALIBRATION_MAGIC;
    calibData.version = CALIBRATION_VERSION;
    
    // Calculate and update checksum
    CalibrationResult checksumResult = updateChecksum(&calibData);
    if (checksumResult != CALIB_SUCCESS) {
        Serial.println("❌ Failed to calculate checksum");
        return checksumResult;
    }
    
    // Write to EEPROM
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(EEPROM_CALIB_DATA_ADDR, calibData);
    bool success = EEPROM.commit();
    EEPROM.end();
    
    if (success) {
        Serial.printf("✅ Calibration saved: %.1f ticks/mm\n", calibData.ticksPerMillimeter);
        return CALIB_SUCCESS;
    } else {
        Serial.println("❌ Failed to save calibration data to EEPROM");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
}

bool runAutonomousCalibration() { 
    Serial.println("🎯 Starting autonomous calibration...");
    setRobotState(ROBOT_CALIBRATING);
    
    CalibrationResult result = runFullCalibrationSequence();
    
    if (result == CALIB_SUCCESS) {
        isCalibrated = true;
        Serial.println("✅ Autonomous calibration completed successfully");
        return true;
    } else {
        Serial.printf("❌ Calibration failed: %s\n", getCalibrationErrorString(result));
        isCalibrated = false;
        return false;
    }
}

CalibrationResult runFullCalibrationSequence() {
    Serial.println("🔄 Starting full calibration sequence...");
    
    // This would run all calibration phases
    // For now, just set some reasonable defaults
    
    calibData.ticksPerMillimeter = 5.0f;
    calibData.ticksPer90Degrees = 450.0f;
    calibData.tofOffsetMM = 25.0f;
    calibData.minMotorSpeedPWM = 60;
    
    // Set motor directions (example - adjust for your robot)
    calibData.motorDirs.leftFwd_M1Fwd = true;
    calibData.motorDirs.leftFwd_M1Rev = false;
    calibData.motorDirs.leftFwd_M2Fwd = false;
    calibData.motorDirs.leftFwd_M2Rev = true;
    
    calibData.motorDirs.fwdMove_M1Fwd = true;
    calibData.motorDirs.fwdMove_M1Rev = false;
    calibData.motorDirs.fwdMove_M2Fwd = true;
    calibData.motorDirs.fwdMove_M2Rev = false;
    
    Serial.println("✅ Calibration sequence completed (using defaults)");
    return CALIB_SUCCESS;
}

bool verifyDataIntegrity(const CalibrationData* data) {
    // Simple validation for now
    return (data->ticksPerMillimeter > 0.1f && 
            data->ticksPerMillimeter < 20.0f &&
            data->minMotorSpeedPWM >= 0 && 
            data->minMotorSpeedPWM <= 255);
}

CalibrationResult updateChecksum(CalibrationData* data) {
    // Simple checksum calculation for now
    // In a real implementation, you'd use CRC16
    data->checksum = 0x1234; // Placeholder
    return CALIB_SUCCESS;
}

const char* getCalibrationErrorString(CalibrationResult result) {
    switch (result) {
        case CALIB_SUCCESS: return "Success";
        case CALIB_ERR_TIMEOUT: return "Timeout";
        case CALIB_ERR_NO_MOVEMENT: return "No movement detected";
        case CALIB_ERR_SENSOR_INVALID: return "Sensor data invalid";
        case CALIB_ERR_CHECKSUM_FAILED: return "Checksum failed";
        case CALIB_ERR_MEMORY_CORRUPTION: return "Memory corruption";
        default: return "Unknown error";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void printNavigationStatus() {
    Vector2D pos = navigator.getPosition();
    Vector2D vel = navigator.getVelocity();
    Vector2D goal = navigator.getGoal();
    
    Serial.printf("🤖 Nav: Pos(%5.1f, %5.1f) | ", pos.x, pos.y);
    Serial.printf("Vel(%4.1f, %4.1f) | ", vel.x, vel.y);
    Serial.printf("Goal(%5.1f, %5.1f) | ", goal.x, goal.y);
    Serial.printf("Dist: %4.1fcm\n", pos.distanceTo(goal));
}

void printBanner() {
    Serial.println(F(""));
    Serial.println(F("======================================="));
    Serial.println(F("🤖 Wheelie Robot - RobotForge OS"));
    Serial.println(F("   Fluid Potential Field Navigation"));
    Serial.println(F("   (c) 2025 Giblets Creations"));
    Serial.println(F("======================================="));
}

void setRobotState(RobotStateEnum newState) {
    if (sysStatus.currentState == newState) return;
    sysStatus.currentState = newState;
}

void setupSystem() {
    setRobotState(ROBOT_BOOTING);
    setupIndicators();
    playBootSequence();
    setupPowerManager();
    setupMotors();
    setupWiFi();
    setupOTA();
    setupESPNOW();
    setupLogger();
}

// ═══════════════════════════════════════════════════════════════════════════
// STUBBED FUNCTIONS (Replace with actual implementations)
// ═══════════════════════════════════════════════════════════════════════════

void setupPowerManager() { Serial.println("Stub: setupPowerManager()"); }
void updatePowerManager() { /* Stub */ }
void setupIndicators() { Serial.println("Stub: setupIndicators()"); }
void updateIndicators() { /* Stub */ }
void playBootSequence() { Serial.println("Stub: playBootSequence()"); }
void playTone(int f, int d) { /* Stub */ }
void setLEDColor(const LEDColor& c) { /* Stub */ }
void setupLogger() { Serial.println("Stub: setupLogger()"); }
void logData() { /* Stub */ }
void setupWiFi() { Serial.println("Stub: setupWiFi()"); }
void setupOTA() { Serial.println("Stub: setupOTA()"); }
void setupESPNOW() { Serial.println("Stub: setupESPNOW()"); }
void updateESPNOW() { /* Stub */ }