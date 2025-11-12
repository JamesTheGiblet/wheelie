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
#include "pins.h"
#include "Vector2D.h"
#include "types.h"

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
#include "calibration.h"
#include "sensors.h"
#include "robot.h" // Include the main robot header

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


// --- Odometry Tracking ---
static long lastLeftEncoder = 0;
static long lastRightEncoder = 0;
static float lastHeading = 0.0;
static float currentX = 0.0;
static float currentY = 0.0;
static float currentHeading = 0.0;


// ═══════════════════════════════════════════════════════════════════════════
// MAIN APPLICATION ENTRY POINT
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);

    // --- Core System Initialization ---
    printBanner();
    setupSystem();

    // Handle calibration
    handleCalibration();

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
    setRobotState(ROBOT_EXPLORING);
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