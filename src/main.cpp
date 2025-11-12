#include <Arduino.h>
#include "PotentialFieldNavigator.h"
#include "SwarmCommunicator.h"
#include "HAL.h" // <-- The generic interface

// --- BOT-SPECIFIC ---
#include "WheelieHAL.h" // <-- The *only* line you change for a new bot!

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

// --- Layer 1: The "Body" ---
WheelieHAL hal; // <-- Create the specific robot body
HAL* robotHAL = &hal; // Pointer to the generic interface

// --- Layer 2: The "Brain" ---
PotentialFieldNavigator navigator;
SwarmCommunicator swarmComms;

// --- System State (DEFINED here, declared as extern in globals.h) ---
SystemStatus sysStatus;
SensorData sensors;
CalibrationData calibData;
bool isCalibrated = false;

// ═══════════════════════════════════════════════════════════════════════════
// MAIN APPLICATION ENTRY POINT
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Prints the startup banner.
 */
void printBanner() {
    Serial.println(F("\n======================================="));
    Serial.println(F("🤖 RobotForge - Project Jumbo"));
    Serial.println(F("   Layer 1: WheelieHAL"));
    Serial.println(F("   Layer 2: PotentialFieldNavigator"));
    Serial.println(F("======================================="));
}

/**
 * @brief Sets the global robot state.
 */
void setRobotState(RobotStateEnum newState) {
    if (sysStatus.currentState == newState) return;
    // TODO: Add state transition validation
    sysStatus.currentState = newState;
    // The HAL is responsible for indicating the state (e.g., LED color)
    robotHAL->setStatusLED(LEDColors::BLUE); // Example
}


void setup() {
    printBanner();

    // Initialize the Hardware Abstraction Layer.
    // This single call handles all hardware setup, sensor discovery, and calibration.
    if (!robotHAL->init()) {
        // HAL initialization failed critically.
        // The HAL's init() function is responsible for indicating the error (e.g., blinking red LED).
        // We halt execution here.
        while (true) {
            delay(1000);
        }
    }

    // Set initial state after successful setup
    setRobotState(ROBOT_IDLE);
}

void loop() {
    // 1. Update the HAL
    // This single call polls all sensors, updates odometry, and runs background tasks (OTA, Power, etc.)
    robotHAL->update();

    // 2. Run the "Brain" (Navigation & Swarm Logic)
    // Get current state from the HAL
    RobotPose currentPose = robotHAL->getPose();
    navigator.setPosition(currentPose.position);

    // TODO: Add logic to run navigator.update() and swarmComms.update()

    // 3. Send commands from the Brain to the HAL
    robotHAL->setVelocity(navigator.getVelocity());
}