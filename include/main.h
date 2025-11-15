#ifndef MAIN_H
#define MAIN_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void printSystemInfo();
void printNavigationStatus();
const char* getRobotStateString(RobotStateEnum state);

#ifdef __cplusplus
}
#endif

// Function declarations for functions defined in main.cpp
void setRobotState(RobotStateEnum newState);
RobotStateEnum getCurrentState();
void printBanner();

#endif // MAIN_H