#ifndef MAIN_H
#define MAIN_H

#include "types.h"

// Function declarations for functions defined in main.cpp
void setRobotState(RobotStateEnum newState);
RobotStateEnum getCurrentState();
void printBanner();

#endif // MAIN_H