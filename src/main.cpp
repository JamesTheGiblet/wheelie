#include <esp_task_wdt.h> // ESP32 Watchdog Timer
#include <Arduino.h>
#include "globals.h"
#include "Vector2D.h"

#include "LearningNavigator.h" // <-- Use the enhanced brain
#include "SwarmCommunicator.h"
#include "web_server.h"
#include "HAL.h" // <-- The generic interface
#include "ota_manager.h" // <-- ADD THIS
#include "MissionController.h"
#include <ArduinoOTA.h>

#include "WheelieHAL.h" // <-- The *only* line you change for a new bot!
#include <cli_manager.h>

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

// --- Layer 1: The "Body" ---
WheelieHAL hal; // <-- Create the specific robot body

// Forward declarations for functions in this file
void handleCLI();

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

// Mission Controller global instance
MissionController missionController;

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

/**
 * @brief Task to handle networking (OTA, Web Server) on Core 0.
 * This prevents network services from being blocked by the main navigation loop.
 */
void networkTask(void *pvParameters) {
    Serial.println("✅ Network Task started on Core 0.");
    for (;;) {
        handleOTA();
        // If the web server were active, its handler would also go here.
        // handleWebServer(); 
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks
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
    // Enable the watchdog timer (timeout: 5 seconds)
    // This must be done inside a function, like setup().
    esp_task_wdt_init(5, true); // 5 seconds, panic on timeout
    esp_task_wdt_add(NULL);     // Add the main loop task to the watchdog

    // 1. Initialize the Hardware Abstraction Layer
    // This single call handles setup, sensor discovery, and calibration.
    if (!hal.init()) {
        // HAL init failed (e.g., calibration failed)
        // Robot is already in an error state.
        while (true) { delay(100); }
    }

    // Initialize WiFi Manager (NON-BLOCKING)
    initializeWiFi();

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

    // Initialize Web Server
    // initializeWebServer(); // DISABLED: Causes reboot loop on ESP32


    // Initialize Swarm Communicator
    SwarmCommunicator::getInstance().begin();

    Serial.println("\n🧪 Testing Swarm Communication:");
    SwarmCommunicator::getInstance().printSwarmInfo();

    // Initialize MissionController with robot ID from SwarmCommunicator
    // Use public getter for MAC address instead of accessing _myState directly
    // This is the correct, encapsulated way to get the ID.
    missionController.setRobotId(SwarmCommunicator::getInstance().getRobotId());
    Serial.println("🎯 Mission Controller initialized");

    // 3.5. Initialize OTA Update Service (This is now called by wifi_manager on connect)
    // initializeOTA(); // DEPRECATED: Called automatically on WiFi connect

    // --- Configure OTA Callbacks for safety ---
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        Serial.println("🛑 OTA: Starting update for " + type);

        // CRITICAL: Stop the robot and enter a safe state
        hal.setVelocity(Vector2D(0, 0)); // Command a stop
        hal.emergencyStop();             // Engage motor brakes
        setRobotState(ROBOT_IDLE);       // Set state to Idle to stop navigation logic
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n✅ OTA: Update complete!");
        // Add a small delay to allow the TCP stack to send the final ACK
        delay(100);
        // CRITICAL: Force a reboot to apply the update.
        // This is the most reliable way to ensure the new firmware runs.
        ESP.restart();
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("❌ OTA Error[%u]: ", error);
        // Full error descriptions can be found in the ArduinoOTA source
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        
        // Resume normal operation if the update fails
        setRobotState(ROBOT_EXPLORING);
    });

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

    xTaskCreatePinnedToCore(
        networkTask,        /* Task function. */
        "NetworkTask",      /* name of task. */
        8192,               /* Stack size of task */
        NULL,               /* parameter of the task */
        1,                  /* priority of the task */
        NULL,               /* Task handle to keep track of created task */
        0);                 /* pin task to core 0 */

    setRobotState(ROBOT_IDLE); // Default to idle
    Serial.println("🤖 RobotForge Brain Online. Waiting for CLI command.");
}

void loop() {
    // Feed the watchdog at the start of each loop to prevent a timeout reboot.
    // This proves the main loop is not stuck.
    esp_task_wdt_reset();

    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdate) / 1000.0f;

    // --- 1. UPDATE HARDWARE (LAYER 1) ---
    // This single call polls all sensors, updates odometry,
    // and runs all background tasks (Power, etc.)
    hal.update();

    // --- Handle WiFi connection state machine ---
    checkWiFiConnection();

    // --- 2. RUN NAVIGATION (only if in a mobile state) ---
    RobotStateEnum currentState = getCurrentState();
    // Only run navigation if not idle
    if (currentState != ROBOT_IDLE && (currentState == ROBOT_EXPLORING || currentState == ROBOT_AVOIDING_OBSTACLE)) {
        if (deltaTime >= 0.05f) { // Run at 20Hz
            // --- A. GET DATA FROM HAL (Layer 1) ---
            RobotPose pose = hal.getPose(); // Get current position and heading

            // --- B. GET DATA FROM SWARM (Layer 2) ---
            auto otherPositions = SwarmCommunicator::getInstance().getOtherRobotPositions();
            Vector2D swarmForce = navigator.calculateSwarmForce(otherPositions);

            // --- C. RUN BRAIN (Layer 2) ---
            navigator.setPosition(pose.position);
            // Use the modern update function that takes live sensor data
            navigator.update(deltaTime, sensors.frontDistanceCm * 10, sensors.rearDistanceCm * 10); // Convert cm to mm

            // --- D. SEND COMMANDS TO HAL (Layer 1) ---
            hal.setVelocity(navigator.getVelocity());

            // --- E. UPDATE SWARM (Layer 2) ---
            SwarmCommunicator::getInstance().setMyState(pose.position, navigator.getVelocity());

            // --- F. PUSH TELEMETRY TO WEB CLIENTS ---
            pushTelemetryToClients();

            lastUpdate = currentTime;
        }

        // Update mission controller
        RobotPose pose = hal.getPose();
        missionController.update(pose.position);

        // Apply role-based parameters if we have a role
        if (missionController.getRole() != ROLE_NONE) {
            navigator.setParameters(missionController.getRoleParameters());
        }

        // Update navigator goal from active mission
        if (missionController.isMissionActive()) {
            Mission currentMission = missionController.getCurrentMission();
            if (currentMission.type == MISSION_GOTO_WAYPOINT ||
                currentMission.type == MISSION_RETURN_TO_BASE) {
                navigator.setGoal(currentMission.targetPosition);
            }
        }
    }

    // --- Background Tasks ---
    SwarmCommunicator::getInstance().update(); // Correctly handles all ESP-NOW logic

    // Enhanced periodic diagnostics: print system info, navigation, learning, and swarm every 10 seconds
    static unsigned long lastDiagnostic = 0;
    if (millis() - lastDiagnostic > 10000) {  // Every 10 seconds
        Serial.println("\n════════════════════════════════════════");
        printSystemInfo();
        printNavigationStatus();
        navigator.printLearningStats();
        SwarmCommunicator::getInstance().printSwarmInfo();
        Serial.println("════════════════════════════════════════\n");
        lastDiagnostic = millis();
    }

    // handleOTA() is now in networkTask on Core 0
    handleCLI(); // Handle serial monitor commands
}