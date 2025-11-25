#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "config.h"
#include "globals.h"
#include "pins.h"
#include "credentials.h" // Use the actual credentials
#include "wifi_manager.h"  // Use the non-blocking WiFi manager
#include "ota_manager.h"   // OTA is handled by the wifi manager now
// #include "cli_manager.h"
// #include "web_server.h"
// #include "power_manager.h"
#include "indicators.h" // For LED and buzzer
#include "logger.h"     // For state change logging
#include "WheelieHAL.h" // For the hal object
// #include "MissionController.h"
// #include "SwarmCommunicator.h"
// #include "calibration.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECT INSTANTIATIONS
// ═══════════════════════════════════════════════════════════════════════════
// Define the global objects that are declared as 'extern' in other files.

WheelieHAL hal;
SystemStatus sysStatus;
SensorData sensors;
SensorHealth_t sensorHealth;

// MissionController missionController;
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

    // Initialize only the ultrasonic sensor pins
    pinMode(FRONT_ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ULTRASONIC_ECHO_PIN, INPUT);

    // Start WiFi and OTA services
    initializeWiFi();
    setRobotState(ROBOT_IDLE);

    Serial.println("✅ Setup complete. Reading sensors and waiting for OTA.");
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

unsigned long lastScanTime = 0;
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

    // Scan sensors and display all data in one line at fixed interval
    if (currentTime - lastScanTime >= SCAN_INTERVAL_MS) {
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
            setMotorPWM(TEST_SPEED, TEST_SPEED);
        } else if (command == "rev") {
            Serial.println("Moving reverse...");
            setMotorPWM(-TEST_SPEED, -TEST_SPEED);
        } else if (command == "left") {
            Serial.println("Turning left...");
            setMotorPWM(-TURN_SPEED, TURN_SPEED);
        } else if (command == "right") {
            Serial.println("Turning right...");
            setMotorPWM(TURN_SPEED, -TURN_SPEED);
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
/*
 const char* getRobotStateString(RobotStateEnum state) {
     // ... implementation ...
 }
 
 void printSystemInfo() {
     // ... implementation ...
 }
 
 void printNavigationStatus() {
     // ... implementation ...
 }
 */