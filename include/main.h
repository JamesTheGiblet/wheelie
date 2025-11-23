
#ifndef MAIN_H
#define MAIN_H

#include "MissionController.h"
#include "globals.h"
#include "calibration.h"
#include "LearningNavigator.h"
#include "WheelieHAL.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECT & STATE DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════
// These are the declarations for the global objects defined in main.cpp.
// Including this header gives other modules safe, read-only access to them.

class WheelieHAL; // Forward declaration to break circular dependency

extern WheelieHAL hal;
extern LearningNavigator navigator;

extern SystemStatus sysStatus;
extern SensorData sensors;
extern CalibrationData calibData;
extern SensorHealth_t sensorHealth;
extern bool isCalibrated;

// Mission Controller global instance
extern MissionController missionController;

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL FUNCTION DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════

void setRobotState(RobotStateEnum newState);
RobotStateEnum getCurrentState();
const char* getRobotStateString(RobotStateEnum state);
void printSystemInfo();
void printNavigationStatus();

#endif // MAIN_H