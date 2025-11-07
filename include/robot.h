#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "pins.h"
#include "config.h"
#include "types.h"
#include "motors.h"
#include "sensors.h"
#include "indicators.h"
#include "wifi_manager.h"
#include "espnow_manager.h"
#include "calibration.h"
#include "navigation.h"

// ═══════════════════════════════════════════════════════════════════════════
// ROBOT CORE - Main robot control and coordination functions
// ═══════════════════════════════════════════════════════════════════════════
//
// This is the high-level robot controller that coordinates all subsystems:
// • Motors, Sensors, Navigation, Communication, Calibration
// • Provides clean encapsulation - no low-level timing variables exposed
// • Each subsystem manages its own internal state and timing
// • Main loop calls subsystem update functions for loose coupling
//
// Architecture Pattern:
// robot.cpp calls: sensors_update() → navigation_update() → motors respond
// Each subsystem is responsible for its own timing and state management
//
// ═══════════════════════════════════════════════════════════════════════════

// Global system status and sensor data
extern SystemStatus sysStatus;
extern SensorData sensors;

// System Initialization
void setupSystem();
void printBanner();
void printSystemInfo();

// Main Operation
void normalOperation();

// Robot State Management (Encapsulated)
RobotState getCurrentState();
void setRobotState(RobotState newState);
bool isValidTransition(RobotState from, RobotState to);

// Safety and Emergency
void emergencyStop();
bool checkAllSafety();
void logSafetyEvent();

// Battery Management
float getBatteryVoltage();

// System Health
void checkStackUsage();

// Forward declarations for main.cpp functions
void appendToLogBuffer(String entry);
void flushLogBuffer();
void updateBatteryVoltage();
void estimateRemainingRuntime();
void exitLowPowerMode();
void enterEconomyMode();
void enterLowPowerMode();
void enterCriticalPowerMode();
void initiateEmergencyShutdown();

// Calibration Functions
bool attemptStaticCalibration();
void loadFactoryCalibration();

// Extended Testing Functions
bool testIMU();
bool testDistanceSensor(); 
bool testEncoders();
bool testESPNOW();
bool testCommunications();

// Sensor Data Access
int getDistanceCm();

// System Control
void setMaxSpeed(float speedRatio);
void setLEDBrightness(int brightness);
void setSensorUpdateRate(float rate);
void setObstacleAvoidanceMode(int mode);
void setNavigationMode(int mode);
void setCommunicationMode(int mode);

// Main Loop Functions
void handlePeriodicCommunications();
void processSensorData();
void executeStateBehavior();
void updateActuators();
void handleDiagnostics();

// Power Management (non-conflicting)
void handlePowerModeChange(int oldMode, int newMode);

// Communication Management
void updateCommunications();
void broadcastSensorData();
void broadcastStatusUpdate();

// Diagnostics and Testing
void runDiagnostics();
bool testMotors();
void testIndicators();
bool testSensors();

#endif // ROBOT_H