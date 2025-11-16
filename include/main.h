#ifndef MAIN_H
#define MAIN_H

#include "types.h" // For RobotStateEnum

// This header provides forward declarations for functions defined in main.cpp
// so that other parts of the system (CLI, Web Server, HAL, etc.) can call them.

// Robot State Management
void setRobotState(RobotStateEnum newState);
RobotStateEnum getCurrentState();
const char* getRobotStateString(RobotStateEnum state);

// System Information
void printBanner();
void printSystemInfo();
void printNavigationStatus();


#endif // MAIN_H
