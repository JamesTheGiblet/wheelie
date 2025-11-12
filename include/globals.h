#ifndef GLOBALS_H
#define GLOBALS_H

#include "types.h"
#include "calibration.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL SYSTEM STATE
// ═══════════════════════════════════════════════════════════════════════════
// This header provides `extern` declarations for system-wide global variables.
// It should be included by any .cpp file that needs to access these globals.
// It should NOT be included by other .h files to avoid circular dependencies.
// The actual variables are defined in main.cpp.
// ═══════════════════════════════════════════════════════════════════════════

extern SystemStatus sysStatus;
extern SensorData sensors;
extern CalibrationData calibData;
extern bool isCalibrated;

#endif // GLOBALS_H