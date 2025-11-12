#include <Arduino.h>
#include "types.h"
#include "Vector2D.h"
#include "PotentialFieldNavigator.h"
#include "SwarmCommunicator.h"
#include "HAL.h" // <-- The generic interface

// --- BOT-SPECIFIC ---
#include "WheelieHAL.h" // <-- The *only* line you change for a new bot!
// #include "GizmoHAL.h"
// #include "CybotHAL.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

// --- Layer 1: The "Body" ---
WheelieHAL hal; // <-- Create the specific robot body

// --- Layer 2: The "Brain" ---
PotentialFieldNavigator navigator;
SwarmCommunicator swarmComms;

// --- System State (used by HAL and Brain) ---
// These are global so the HAL and Brain can share state.
SystemStatus sysStatus;
SensorData sensors;
CalibrationData calibData;
SensorHealth_t sensorHealth;
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
    sysStatus.currentState = newState;
}

RobotStateEnum getCurrentState() {
    return sysStatus.currentState;
}


void setup() {
    // 1. Initialize the Hardware Abstraction Layer
    // This single call handles setup, sensor discovery, and calibration.
    if (!hal.init()) {
        // HAL init failed (e.g., calibration failed)
        // Robot is already in an error state.
        while (true) { delay(100); }
    }

    // 2. Initialize the Brain (Layer 2)
    // We can pull settings from the MCP or use defaults.
    // For now, we'll use defaults.
    NavigationParameters params;
    params.attractionConstant = 2.5f;
    params.repulsionConstant = 20.0f;
    params.maxSpeed = 35.0f; // mm/s
    navigator.setParameters(params);
    
    // Set initial goal 1m (1000mm) forward (HAL: X+)
    navigator.setGoal(Vector2D(1000, 0)); 
    
    setRobotState(ROBOT_EXPLORING);
    Serial.println("🤖 RobotForge Brain Online. Engaging fluid motion.");
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdate) / 1000.0f;

    // --- 1. UPDATE HARDWARE (LAYER 1) ---
    // This single call polls all sensors, updates odometry,
    // and runs all background tasks (OTA, Power, etc.)
    hal.update();

    // --- 2. RUN NAVIGATION (20Hz) ---
    if (deltaTime >= 0.05f) {
        
        // --- A. GET DATA FROM HAL (Layer 1) ---
        RobotPose pose = hal.getPose();
        Vector2D obstacleForce = hal.getObstacleRepulsion();
        
        // --- B. GET DATA FROM SWARM (Layer 2) ---
        auto otherPositions = swarmComms.getOtherRobotPositions();
        Vector2D swarmForce = navigator.calculateSwarmForce(otherPositions);

        // --- C. RUN BRAIN (Layer 2) ---
        navigator.setPosition(pose.position);
        // Pass in the *combined* force from obstacles and swarm
        navigator.update(deltaTime, obstacleForce + swarmForce); 

        // --- D. SEND COMMANDS TO HAL (Layer 1) ---
        hal.setVelocity(navigator.getVelocity());

        // --- E. UPDATE SWARM (Layer 2) ---
        swarmComms.setMyState(pose.position, navigator.getVelocity());

        lastUpdate = currentTime;
    }

    // --- Background Tasks ---
    swarmComms.update(); // Handle ESP-NOW send/receive
}