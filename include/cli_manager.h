#ifndef CLI_MANAGER_H
#define CLI_MANAGER_H

#include <Arduino.h>

void initializeCLI();
void handleCLI();
String processCommandAndGetResponse(String command); // <-- ADD THIS

#endif // CLI_MANAGER_H