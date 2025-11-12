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

// Define FORCE_RECALIBRATION_PIN if not defined in pins.h
#ifndef FORCE_RECALIBRATION_PIN
#define FORCE_RECALIBRATION_PIN 12 // <-- Set this to the correct pin number
#endif

// Stub implementations for other required functions
bool checkCalibrationStatus() { 
  return isCalibrated; 
}

bool shouldForceRecalibration() { 
  return digitalRead(FORCE_RECALIBRATION_PIN) == LOW; 
}

void setupEncoders() {
  // Your encoder setup code here
  Serial.println("Stub: setupEncoders()");
}

void resetEncoders() {
  // Your encoder reset code here  
}

long getLeftEncoderCount() { 
  return sensors.leftEncoderCount; 
}

long getRightEncoderCount() { 
  return sensors.rightEncoderCount; 
}
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
#define FORCE_RECALIBRATION_PIN 12 // <-- Set this to the correct pin number
#endif

// New RobotForge Headers
#include "PotentialFieldNavigator.h"
#include "ObstacleMemory.h"
#include "SwarmCommunicator.h"

// Old Wheelie Module Headers (for function declarations)
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
// ArduinoOTA is a namespace; do not instantiate it.

// --- Odometry Tracking ---
static long lastLeftEncoder = 0;
static long lastRightEncoder = 0;
static float lastHeading = 0.0;
static float currentX = 0.0;
static float currentY = 0.0;
static float currentHeading = 0.0;


// ═══════════════════════════════════════════════════════════════════════════
// FUNCTION PROTOTYPES (from all Wheelie modules)
// ═══════════════════════════════════════════════════════════════════════════

// --- Main System (robot.cpp) ---
void printBanner();
void setupSystem();
void setRobotState(RobotStateEnum newState);

// --- Sensors (sensors.cpp) ---
void initializeSensors();
void updateAllSensors();

// --- Power (power_manager.cpp) ---
void setupPowerManager();
void updatePowerManager();

// --- Indicators (indicators.cpp) ---
void setupIndicators();
void updateIndicators();
void playBootSequence();
void playTone(int frequency, int duration);
void setLEDColor(const LEDColor& color);

// --- Calibration (calibration.cpp) ---
CalibrationResult loadCalibrationData();
CalibrationResult saveCalibrationData();
bool runAutonomousCalibration();

// --- Odometry (navigation.cpp) ---
void updateOdometry();
void setMotorsFromVector(const Vector2D& v);

// --- Logger (logger.cpp) ---
void setupLogger();
void logData();

// --- WiFi (wifi_manager.cpp) ---
void setupWiFi();

// --- OTA (ota_manager.cpp) ---
void setupOTA();

// --- ESP-NOW (espnow_manager.cpp) ---
void setupESPNOW();
void updateESPNOW();


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
        setRobotState(static_cast<RobotState>(ROBOT_CALIBRATING));
        if (runAutonomousCalibration()) {
            CalibrationResult saveResult = saveCalibrationData();
            if (saveResult == CALIB_SUCCESS) {
                Serial.println("✅ Calibration successful and saved. Rebooting...");
                delay(1000);
                ESP.restart();
            } else {
                Serial.println("❌ Failed to save calibration data");
                setRobotState(static_cast<RobotState>(ROBOT_ERROR));
            }
        } else {
            Serial.println("❌ CRITICAL: Calibration failed. Robot halted.");
            setRobotState(static_cast<RobotState>(ROBOT_ERROR));
            while (true) { delay(1000); }
        }
    }

    // --- Navigation Setup ---
    NavigationParameters params;
    params.attractionConstant = 2.5f;
    params.repulsionConstant = 20.0f;
    params.maxSpeed = 35.0f;
    navigator.setParameters(params);
    
  // Use calibrated max speed if available (disabled: calibData has no maxSpeed)
  // if (isCalibrated && calibData.maxSpeed > 0) {
  //     params.maxSpeed = calibData.maxSpeed;
  //     navigator.setParameters(params);
  // }
    
    navigator.setGoal(Vector2D(1000, 0));

    // --- Odometry Baseline ---
    lastLeftEncoder = sensors.leftEncoderCount;
    lastRightEncoder = sensors.rightEncoderCount;
    lastHeading = sensors.headingAngle;
    currentHeading = lastHeading;
    currentX = 0.0;
    currentY = 0.0;
    navigator.setPosition(Vector2D(currentX, currentY));
    
    Serial.println("🤖 RobotForge Navigation System ONLINE.");
    setRobotState(static_cast<RobotState>(ROBOT_EXPLORING));
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdate) / 1000.0f;

  // --- Core Systems (No Change) ---
  updateAllSensors();     // Read ToF, IMU, Encoders
  updatePowerManager();   // Check battery
  ArduinoOTA.handle();    // Handle OTA updates

  // Run navigation logic at 20Hz
  if (deltaTime >= 0.05f) {
    
    // --- 1. SENSOR INPUT (Connect real sensors) ---
    // (Wheelie has one forward ToF sensor, angle = 0.0 radians)
    navigator.addSensorReading(sensors.distance, 0.0f);

    // --- 2. ODOMETRY INPUT (Connect real odometry) ---
    updateOdometry(); // This function now updates the navigator's position internally

    // --- 3. SWARM INPUT (Connect ESP-NOW) ---
    auto otherPositions = swarmComms.getOtherRobotPositions();

    // --- 4. RUN NAVIGATION UPDATE ---
    navigator.update(deltaTime, otherPositions);

    // --- 5. MOTOR OUTPUT (Connect real motors) ---
    Vector2D desiredVelocity = navigator.getVelocity();
    setMotorsFromVector(desiredVelocity);

    // --- 6. SWARM OUTPUT (Broadcast our new state) ---
    swarmComms.setMyState(navigator.getPosition(), navigator.getVelocity());

    lastUpdate = currentTime;
  }

  // --- Background Tasks ---
  swarmComms.update();      // Handle ESP-NOW send/receive
  updateESPNOW();           // Handle ESP-NOW (old system, might be redundant)
  updateIndicators();     // Update status LED
  logData();                // Log to CSV
}

// ═══════════════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS (Implementations from Wheelie modules)
// ═══════════════════════════════════════════════════════════════════════════

// --- From robot.cpp ---
void printBanner() {
  Serial.println(F(""));
  Serial.println(F("======================================="));
  Serial.println(F("🤖 Wheelie Robot - RobotForge OS"));
  Serial.println(F("   Fluid Potential Field Navigation"));
  Serial.println(F("   (c) 2025 Giblets Creations"));
  Serial.println(F("======================================="));
}

void setRobotState(RobotStateEnum newState) {
  if (sysStatus.currentState == newState) return; // No change
  sysStatus.currentState = newState;
  // Serial.printf("State changed to: %d\n", newState); // Debug
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

// --- From sensors.cpp ---
void initializeSensors() {
    Serial.println("🔧 Initializing sensor array...");
    Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK);

    // VL53L0X ToF Sensor
    Serial.print("   📏 VL53L0X ToF sensor... ");
    tofSensor.setTimeout(500);
    if (tofSensor.init()) {
        tofSensor.startContinuous();
        sysStatus.tofAvailable = true;
        Serial.println("✅ ONLINE");
    } else {
        Serial.println("❌ OFFLINE");
    }

    // MPU6050 IMU
    Serial.print("   🔄 MPU6050 IMU... ");
    if (mpu.begin() == 0) {
        sysStatus.mpuAvailable = true;
        mpu.calcOffsets(); // Auto-calibrate on boot
        Serial.println("✅ ONLINE (Calibrated)");
    } else {
        Serial.println("❌ OFFLINE");
    }

    // Encoder Pins
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    // Attach interrupts (assuming encoders.cpp logic)
    // TODO: Add attachInterrupt calls for encoders

    // Other digital sensors
    pinMode(SOUND_SENSOR_PIN, INPUT);
    pinMode(EDGE_SENSOR_PIN, INPUT_PULLUP);
    pinMode(PIR_SENSOR_PIN, INPUT);
    
    Serial.println("✅ Sensor initialization complete.");
}

void updateAllSensors() {
    // Read ToF
    if (sysStatus.tofAvailable) {
        sensors.distance = tofSensor.readRangeContinuousMillimeters();
        if (tofSensor.timeoutOccurred()) {
            sensors.distance = 8190; // Max range on timeout
        }
    }

    // Read IMU
    if (sysStatus.mpuAvailable) {
        mpu.update();
        sensors.tiltX = mpu.getAngleX();
        sensors.tiltY = mpu.getAngleY();
        sensors.headingAngle = mpu.getAngleZ();
    }

    // Read Encoders (assuming this is done by interrupts)
    // sensors.leftEncoderCount and sensors.rightEncoderCount
    // are updated by ISRs (Interrupt Service Routines)

    // Read other sensors
    sensors.soundDetected = (digitalRead(SOUND_SENSOR_PIN) == HIGH);
    sensors.edgeDetected = (digitalRead(EDGE_SENSOR_PIN) == HIGH);
    sensors.motionDetected = (digitalRead(PIR_SENSOR_PIN) == HIGH);
}

// --- From navigation.cpp (old) ---
void updateOdometry() {
    long currentLeftEncoder = sensors.leftEncoderCount;
    long currentRightEncoder = sensors.rightEncoderCount;
    currentHeading = sensors.headingAngle;
    
    long deltaLeft = currentLeftEncoder - lastLeftEncoder;
    long deltaRight = currentRightEncoder - lastRightEncoder;
    
    float ticksPerMm = (isCalibrated && calibData.ticksPerMillimeter > 0) ? calibData.ticksPerMillimeter : 1.0;
    float deltaDistance = ((float)(deltaLeft + deltaRight) / 2.0) / ticksPerMm;
    
    float avgHeading = (currentHeading + lastHeading) / 2.0;
    if (abs(currentHeading - lastHeading) > 180.0) {
        avgHeading += 180.0;
    }
    float avgHeadingRad = avgHeading * M_PI / 180.0;

    float deltaX = deltaDistance * cos(avgHeadingRad);
    float deltaY = deltaDistance * sin(avgHeadingRad);
    
    currentX += deltaX;
    currentY += deltaY;
    
    navigator.setPosition(Vector2D(currentX, currentY));
    
    lastLeftEncoder = currentLeftEncoder;
    lastRightEncoder = currentRightEncoder;
    lastHeading = currentHeading;
}

// --- Stubbed functions (to make it compile) ---
// These functions exist in the original project but are complex.
// They are stubbed here to allow the main logic to be complete.

void setupPowerManager() { Serial.println("Stub: setupPowerManager()"); }
void updatePowerManager() { /* Stub */ }
void setupIndicators() { Serial.println("Stub: setupIndicators()"); }
void updateIndicators() { /* Stub */ }
void playBootSequence() { Serial.println("Stub: playBootSequence()"); }
void playTone(int f, int d) { /* Stub */ }
void setLEDColor(const LEDColor& c) { /* Stub */ }

// --- CORRECTED STUBS ---


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
  setRobotState(static_cast<RobotState>(ROBOT_CALIBRATING));
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

// ═══════════════════════════════════════════════════════════════════════════
// SUPPORTING FUNCTIONS (Stubs for now - implement as needed)
// ═══════════════════════════════════════════════════════════════════════════

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

void setupLogger() { Serial.println("Stub: setupLogger()"); }
void logData() { /* Stub */ }
void setupWiFi() { Serial.println("Stub: setupWiFi()"); }
void setupOTA() { Serial.println("Stub: setupOTA()"); }
void setupESPNOW() { Serial.println("Stub: setupESPNOW()"); }
void updateESPNOW() { /* Stub */ }