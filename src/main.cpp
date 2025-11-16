#include <Arduino.h>
#include "types.h"
#include "main.h"
#include "Vector2D.h"
#include "LearningNavigator.h" // <-- Use the enhanced brain
#include "SwarmCommunicator.h"
#include "HAL.h" // <-- The generic interface
#include "web_server.h"
#include "ota_manager.h" // <-- ADD THIS
#include "cli_manager.h" // For serial commands

#include "WheelieHAL.h" // <-- The *only* line you change for a new bot!
// #include "GizmoHAL.h"
// #include "CybotHAL.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

// --- Layer 1: The "Body" ---
WheelieHAL hal; // <-- Create the specific robot body

// --- Layer 2: The "Brain" ---
LearningNavigator navigator; // <-- Use the learning-capable navigator
// SwarmCommunicator swarmComms; // Now a singleton

// --- System State (used by HAL and Brain) ---
// These are global so the HAL and Brain can share state.
SystemStatus sysStatus;
SensorData sensors;
CalibrationData calibData;
SensorHealth_t sensorHealth;
bool isCalibrated = false;

// ═══════════════════════════════════════════════════════════════════════════
// MULTI-CORE TASKING
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Task to handle data logging on the second core (Core 0).
 * This prevents blocking file I/O from affecting the main loop.
 */
void loggerTask(void *pvParameters) {
    for (;;) {
        periodicDataLogging();
        vTaskDelay(pdMS_TO_TICKS(100)); // Check if logging is needed every 100ms
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN APPLICATION ENTRY POINT
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Prints the startup banner.
 */
void printBanner() {
    Serial.println(F("\n======================================="));
    Serial.println(F("   🚀 WIRELESS UPDATE COMPLETE! 🚀"));
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

/**
 * @brief Converts a RobotStateEnum to a human-readable string.
 * @param state The state to convert.
 * @return A string representation of the state.
 */
const char* getRobotStateString(RobotStateEnum state) {
    switch (state) {
        case ROBOT_BOOTING:             return "Booting";
        case ROBOT_IDLE:                return "Idle";
        case ROBOT_TESTING:             return "Testing";
        case ROBOT_CALIBRATING:         return "Calibrating";
        case ROBOT_EXPLORING:           return "Exploring";
        case ROBOT_AVOIDING_OBSTACLE:   return "Avoiding";
        case ROBOT_PLANNING_ROUTE:      return "Planning";
        case ROBOT_RECOVERING_STUCK:    return "Recovering";
        case ROBOT_SOUND_TRIGGERED:     return "Sound Triggered";
        case ROBOT_MOTION_TRIGGERED:    return "Motion Triggered";
        case ROBOT_SAFETY_STOP_TILT:    return "Safety Stop (Tilt)";
        case ROBOT_SAFETY_STOP_EDGE:    return "Safety Stop (Edge)";
        case ROBOT_SAFE_MODE:           return "Safe Mode";
        case ROBOT_ERROR:               return "Error";
        default:                        return "Unknown";
    }
}

/**
 * @brief Prints a comprehensive system status report to the Serial monitor.
 */
void printSystemInfo() {
    Serial.println("\n📊 SYSTEM STATUS REPORT");
    Serial.println("════════════════════════════════════════════════");
    Serial.printf("🔧 Platform: ESP32 @ %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("💾 Free Heap: %d bytes\n", esp_get_free_heap_size());
    Serial.printf("💡 Uptime: %lu seconds\n", millis() / 1000);
    Serial.printf("🤖 Current State: %s\n", getRobotStateString(sysStatus.currentState));
    Serial.printf("🔌 Sensors Active: %d\n", sysStatus.sensorsActive);
    Serial.printf("📶 WiFi: %s (%s)\n", sysStatus.wifiConnected ? "Connected" : "Disconnected", sysStatus.ipAddress);
    Serial.printf("📡 ESP-NOW: %s (%d peers)\n", sysStatus.espnowActive ? "Active" : "Inactive", sysStatus.espnowStatus.peerCount);
    Serial.println("════════════════════════════════════════════════");
}

/**
 * @brief Prints the status of the navigation system to the Serial monitor.
 */
void printNavigationStatus() {
    Serial.println("\n🧭 NAVIGATION STATUS:");
    Serial.println("──────────────────────────────────────────");
    RobotPose pose = hal.getPose();
    Vector2D goal = navigator.getGoal();
    Serial.printf("  Pose: Pos(%.1f, %.1f) | Heading: %.1f°\n", pose.position.x, pose.position.y, pose.heading);
    Serial.printf("  Goal: (%.1f, %.1f) | Distance: %.1f mm\n", goal.x, goal.y, pose.position.distanceTo(goal));
    Serial.printf("  Velocity Vector: %s\n", navigator.getVelocity().toString().c_str());
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
    
    // Start the first learning episode
    navigator.startEpisode();

    // Initialize Swarm Communicator
    SwarmCommunicator::getInstance().begin();


    // 3. Initialize Web Server
    initializeWebServer();

    // 3.5. Initialize OTA Update Service
    initializeOTA();

    // 4. Initialize Command Line Interface
    initializeCLI();

    // --- Create Background Task for Data Logging ---
    xTaskCreatePinnedToCore(
        loggerTask,         /* Task function. */
        "LoggerTask",       /* name of task. */
        4096,               /* Stack size of task */
        NULL,               /* parameter of the task */
        1,                  /* priority of the task */
        NULL,               /* Task handle to keep track of created task */
        0);                 /* pin task to core 0 */

    setRobotState(ROBOT_EXPLORING);
    Serial.println("🤖 RobotForge Brain Online. Engaging fluid motion.");
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdate) / 1000.0f;

    // --- 1. UPDATE HARDWARE (LAYER 1) ---
    // This single call polls all sensors, updates odometry,
    // and runs all background tasks (Power, etc.)
    hal.update();

    // --- 2. RUN NAVIGATION (20Hz) ---
    if (deltaTime >= 0.05f) {
        
        // --- A. GET DATA FROM HAL (Layer 1) ---
        RobotPose pose = hal.getPose();
        Vector2D obstacleForce = hal.getObstacleRepulsion();

        // --- B. GET DATA FROM SWARM (Layer 2) ---
        auto otherPositions = SwarmCommunicator::getInstance().getOtherRobotPositions();
        Vector2D swarmForce = navigator.calculateSwarmForce(otherPositions);

        // --- C. RUN BRAIN (Layer 2) ---
        navigator.setPosition(pose.position);
        // Pass in the *combined* force from obstacles and swarm
        navigator.updateWithLearning(deltaTime, obstacleForce + swarmForce); 

        // --- D. SEND COMMANDS TO HAL (Layer 1) ---
        hal.setVelocity(navigator.getVelocity());

        // --- E. UPDATE SWARM (Layer 2) ---
        SwarmCommunicator::getInstance().setMyState(pose.position, navigator.getVelocity());

        lastUpdate = currentTime;
    }

    // --- Background Tasks ---
    SwarmCommunicator::getInstance().update(); // Correctly handles all ESP-NOW logic
    handleWebServer();   // Handle incoming web requests
    handleOTA();         // Handle incoming OTA update requests
    handleCLI();         // Handle serial monitor commands
}