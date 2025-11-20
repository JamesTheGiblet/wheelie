#ifndef MAIN_H
#define MAIN_H

#include "globals.h"
#include "WheelieHAL.h"
#include "LearningNavigator.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECT & STATE DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════
// These are the declarations for the global objects defined in main.cpp.
// Including this header gives other modules safe, read-only access to them.

extern WheelieHAL hal;
extern LearningNavigator navigator;

extern SystemStatus sysStatus;
extern SensorData sensors;
extern CalibrationData calibData;
extern SensorHealth_t sensorHealth;
extern bool isCalibrated;

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL FUNCTION DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════

void setRobotState(RobotStateEnum newState);
RobotStateEnum getCurrentState();
const char* getRobotStateString(RobotStateEnum state);
void printSystemInfo();
void printNavigationStatus();

#endif // MAIN_H