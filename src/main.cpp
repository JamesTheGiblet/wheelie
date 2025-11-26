#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "config.h"
#include "globals.h"
#include "pins.h"
#include "credentials.h" // Use the actual credentials
#include "wifi_manager.h"  // Use the non-blocking WiFi manager
#include "ota_manager.h"   // OTA is handled by the wifi manager now
#include "cli_manager.h"
#include "web_server.h"
#include "power_manager.h"
#include "indicators.h" // For LED and buzzer
#include "logger.h"     // For state change logging
#include "WheelieHAL.h" // For the hal object
#include <motors.h>
#include "MissionController.h"
#include "SwarmCommunicator.h"
// #include "calibration.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECT INSTANTIATIONS
// ═══════════════════════════════════════════════════════════════════════════
// Define the global objects that are declared as 'extern' in other files.

WheelieHAL hal;
SystemStatus sysStatus;
SensorData sensors;
LearningNavigator navigator;
SensorHealth_t sensorHealth;

MissionController missionController;
CalibrationData calibData;

volatile bool otaInitialized = false;
bool isCalibrated = false;

// Internal state variable, managed by functions below
static RobotStateEnum currentRobotState = ROBOT_BOOTING;


// ═══════════════════════════════════════════════════════════════════════════
// FUNCTION PROTOTYPES
// ═══════════════════════════════════════════════════════════════════════════
void handleSerialCommands();
float readUltrasonic();
// No prototypes needed here now
// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    // Start serial communication first
    Serial.begin(SERIAL_BAUD);
    Serial.println("\n\nBooting Wheelie Robot in OTA_TEST mode...");

    // Initialize I2C bus with custom pins before any sensor setup
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100); // Allow I2C bus to settle

    // Initialize core hardware (this also sets up indicators, motors, etc.)
    hal.init();
    setLEDColor(LEDColors::YELLOW); // Yellow for Booting

    // Initialize power management system
    initializePowerManagement();

    // Initialize only the ultrasonic sensor pins
    pinMode(FRONT_ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ULTRASONIC_ECHO_PIN, INPUT);

    // Start WiFi and OTA services
    initializeWiFi();
    setRobotState(ROBOT_IDLE);

    // --- Swarm Communicator (Disabled for OTA stability) ---
    // Enabling this increases RAM usage and can cause OTA to fail.
    // Set to 'true' when you are ready to test multi-robot communication.
    if (false) {
        SwarmCommunicator::getInstance().begin();
    }

    // Initialize the Navigator
    navigator.setPosition(hal.getPose().position);
    navigator.setParameters(missionController.getRoleParameters());
    navigator.startEpisode(); // Start the first learning episode

    Serial.println("✅ Setup complete. Reading sensors and waiting for OTA.");
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

unsigned long lastScanTime = 0;
unsigned long lastNavTime = 0;
const unsigned long SCAN_INTERVAL_MS = 1000;  // Scan and display every 200ms

/**
 * @brief Converts a distance reading into a human-readable status.
 * @param distanceCm The distance from the sensor in centimeters.
 * @return A const char* with the status label.
 */
const char* getDistanceStatus(float distanceCm) {
    // These thresholds are based on the constants in config.h
    const float HALT_DISTANCE_CM = 10.0f;
    const float DANGER_DISTANCE_CM = 20.0f;  // OBSTACLE_DISTANCE
    const float WARNING_DISTANCE_CM = 35.0f; // WARNING_DISTANCE

    if (distanceCm < HALT_DISTANCE_CM) {
        return "HALT!";
    } else if (distanceCm < DANGER_DISTANCE_CM) {
        return "DANGER";
    } else if (distanceCm < WARNING_DISTANCE_CM) {
        return "Warning";
    } else {
        return "Safe";
    }
}
/**
 * @brief Processes a sensor value for display, applying a "zero" threshold.
 * If the value is within the threshold, it's treated as 0.
 * @param value The raw sensor value.
 * @param threshold The tolerance around zero.
 * @return The processed value.
 */
float zeroClamp(float value, float threshold = 0.5f) {
    if (abs(value) < threshold) {
        return 0.0f;
    }
    return value;
}

void loop() {

    unsigned long currentTime = millis();

    // Handle networking and OTA updates
    checkWiFiConnection();
    handleOTA();

    // Monitor battery and power state
    monitorPower();

    // Update Swarm Communicator (conditionally disabled)
    if (false) {
        SwarmCommunicator::getInstance().update();
    }

    // --- High-Frequency Navigation Loop (e.g., 50Hz) ---
    if (currentTime - lastNavTime >= 20) {
        float dt = (currentTime - lastNavTime) / 1000.0f;
        lastNavTime = currentTime;

        // Get current state and mission details
        RobotStateEnum currentState = getCurrentState();
        Mission currentMission = missionController.getCurrentMission();

        if (currentState == ROBOT_EXPLORING || currentState == ROBOT_NAVIGATING) {
            // Update the navigator's goal from the mission controller
            if (missionController.isMissionActive()) {
                navigator.setGoal(currentMission.targetPosition);
            }

            // Run the navigator's brain
            navigator.update(dt, sensors.frontDistanceCm, sensors.rearDistanceCm);

            // Command the robot to move at the velocity calculated by the navigator
            hal.setVelocity(navigator.getVelocity());
        }
    }

    // Scan sensors and display all data in one line at fixed interval
    if (currentTime - lastScanTime >= SCAN_INTERVAL_MS) {
        // CRITICAL: Service OTA during this long-running block to prevent timeouts
        // during filesystem uploads.
        handleOTA();

        hal.update(); // Update all HAL components (sensors, odometry, etc.)
        const char* usStatus = getDistanceStatus(sensors.frontDistanceCm);
        const char* tofStatus = getDistanceStatus(sensors.frontDistanceCm / 10.0f);
            Serial.printf(
                "Tilt(X:%.1f, Y:%.1f), GyroZ:%.1f, Temp:%.1fC\n",
                zeroClamp(sensors.tiltX),
                zeroClamp(sensors.tiltY),
                zeroClamp(sensors.gyroZ, 0.1f),
                sensors.temperature
            );
        
        // Update our own state for the swarm
        if (false) {
            RobotPose currentPose = hal.getPose();
            SwarmCommunicator::getInstance().setMyState(currentPose.position, hal.getVelocity());
        }

        // Push the latest data to any connected web clients
        pushTelemetryToClients();
        lastScanTime = currentTime;
    }

    // Handle any incoming serial commands
    handleSerialCommands();
}

// ═══════════════════════════════════════════════════════════════════════════
// FUNCTION DEFINITIONS
// GLOBAL STATE MANAGEMENT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

float readUltrasonic() {
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);
    long duration = pulseIn(FRONT_ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
    float distance = duration * 0.034 / 2.0;
    return distance;
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        Serial.printf("Received Serial Command: '%s'\n", command.c_str());
        if (command == "status")
        {
            Serial.println("Status: OK, Mode: SENSOR_TEST");
            if (sysStatus.wifiConnected) {
                Serial.printf("WiFi: Connected, IP: %s\n", sysStatus.ipAddress);
            } else {
                Serial.println("WiFi: Disconnected");
            }
        } else if (command == "reboot") {
            Serial.println("Rebooting now...");
            ESP.restart();
        } else if (command == "fwd") {
            Serial.println("Moving forward...");
            moveForward(TEST_SPEED);
        } else if (command == "rev") {
            Serial.println("Moving reverse...");
            moveBackward(TEST_SPEED);
        } else if (command == "left") {
            Serial.println("Turning left...");
            rotateLeft(TURN_SPEED);
        } else if (command == "right") {
            Serial.println("Turning right...");
            rotateRight(TURN_SPEED);
        } else if (command == "stop") {
            Serial.println("Stopping motors.");
            allStop();
        } else {
            Serial.println("Unknown command. Use: status, reboot, fwd, rev, left, right, stop");
        }
    }
}

void setRobotState(RobotStateEnum newState) {
    if (currentRobotState != newState) {
        logStateChange(currentRobotState, newState);
        currentRobotState = newState;
        // sysStatus.currentState = newState; // sysStatus doesn't have this field in this version
    }
}

RobotStateEnum getCurrentState() {
    return currentRobotState;
}

 const char* getRobotStateString(RobotStateEnum state) {
    switch (state) {
        case ROBOT_BOOTING: return "BOOTING";
        case ROBOT_IDLE: return "IDLE";
        case ROBOT_EXPLORING: return "EXPLORING";
        case ROBOT_AVOIDING_OBSTACLE: return "AVOIDING";
        case ROBOT_PLANNING_ROUTE: return "PLANNING";
        case ROBOT_RECOVERING_STUCK: return "RECOVERING";
        case ROBOT_SAFE_MODE: return "SAFE_MODE";
        case ROBOT_ERROR: return "ERROR";
        case ROBOT_CALIBRATING: return "CALIBRATING";
        case ROBOT_TESTING: return "TESTING";
        case ROBOT_SAFETY_STOP_TILT: return "TILT_STOP";
        case ROBOT_SAFETY_STOP_EDGE: return "EDGE_STOP";
        default: return "UNKNOWN";
    }
 }
 
 void printSystemInfo() {
    Serial.println("\n--- System Status ---");
    Serial.printf("State: %s (%d)\n", getRobotStateString(getCurrentState()), getCurrentState());
    Serial.printf("WiFi: %s, IP: %s\n", sysStatus.wifiConnected ? "Connected" : "Disconnected", sysStatus.ipAddress);
    Serial.printf("Free Heap: %u bytes\n", esp_get_free_heap_size());
    Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
    Serial.println("---------------------\n");
 }
 
 void printNavigationStatus() {
    Serial.println("\n--- Navigation Status ---");
    RobotPose pose = hal.getPose();
    Serial.printf("Position: (X: %.1f, Y: %.1f) mm\n", pose.position.x, pose.position.y);
    Serial.printf("Heading: %.2f degrees\n", pose.heading);
    // You can add more details from the navigator or mission controller here
    Serial.println("-------------------------\n");
 }
 