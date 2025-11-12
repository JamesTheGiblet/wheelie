#include <WebServer.h>
#include <ArduinoJson.h>
#include "main.h"
#include "WheelieHAL.h"
#include "power_manager.h"
#include "types.h"
#include "globals.h"

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

extern WheelieHAL hal;
extern SystemStatus sysStatus;
extern SensorData sensors;
extern PowerMode_t currentPowerMode;
extern BatteryMonitor_t battery;

WebServer server(80);

const char* getPowerModeString(PowerMode_t mode) {
    switch (mode) {
        case POWER_NORMAL: return "Normal";
        case POWER_ECONOMY: return "Economy";
        case POWER_LOW: return "Low";
        case POWER_CRITICAL: return "Critical";
        case POWER_SHUTDOWN: return "Shutdown";
        default: return "Unknown";
    }
}

const char* getRobotStateString(RobotStateEnum state) {
    switch (state) {
        case ROBOT_BOOTING: return "Booting";
        case ROBOT_IDLE: return "Idle";
        case ROBOT_TESTING: return "Testing";
        case ROBOT_CALIBRATING: return "Calibrating";
        case ROBOT_EXPLORING: return "Exploring";
        case ROBOT_AVOIDING_OBSTACLE: return "Avoiding Obstacle";
        case ROBOT_PLANNING_ROUTE: return "Planning Route";
        case ROBOT_RECOVERING_STUCK: return "Recovering";
        case ROBOT_SOUND_TRIGGERED: return "Sound Triggered";
        case ROBOT_MOTION_TRIGGERED: return "Motion Triggered";
        case ROBOT_SAFETY_STOP_TILT: return "Safety Stop (Tilt)";
        case ROBOT_SAFETY_STOP_EDGE: return "Safety Stop (Edge)";
        case ROBOT_SAFE_MODE: return "Safe Mode";
        case ROBOT_ERROR: return "Error";
        default: return "Unknown";
    }
}

void handleApiStatus() {
    StaticJsonDocument<512> doc;

    doc["state"] = getRobotStateString(sysStatus.currentState);
    doc["uptime"] = millis() / 1000;
    doc["wifi"] = sysStatus.wifiConnected ? "Connected" : "Disconnected";
    doc["ip"] = sysStatus.ipAddress;
    doc["peers"] = sysStatus.espnowStatus.peerCount;

    doc["voltage"] = String(battery.voltage, 2);
    doc["percent"] = String(battery.percentage, 1);
    doc["power_mode"] = getPowerModeString(currentPowerMode);

    doc["dist"] = sensors.distance;
    doc["tilt"] = String(sensors.tiltX, 1) + " / " + String(sensors.tiltY, 1);
    doc["heading"] = String(sensors.headingAngle, 1);
    doc["edge"] = sensors.edgeDetected ? "YES" : "No";

    RobotPose pose = hal.getPose();
    doc["nav_mode"] = "Potential Field"; // Placeholder
    doc["pos"] = String(pose.position.x, 0) + " / " + String(pose.position.y, 0);
    doc["nav_heading"] = String(pose.heading, 1);
    doc["stuck"] = "No"; // Placeholder

    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

void handleStart() {
    setRobotState(ROBOT_EXPLORING);
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Starting exploration...");
}

void handleStop() {
    setRobotState(ROBOT_IDLE);
    hal.setVelocity(Vector2D(0,0)); // Command motors to stop immediately
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Stopping...");
}

void handleRoot() {
    // This should serve the index.html from SPIFFS/LittleFS
    // For now, we'll send a simple message.
    server.send(200, "text/html", "<h1>Wheelie Robot</h1><p>Please upload the 'data' directory to the filesystem.</p>");
}

void initializeWebServer() {
    server.on("/", handleRoot);
    server.on("/api/status", handleApiStatus);
    server.on("/start", handleStart);
    server.on("/stop", handleStop);
    server.begin();
    Serial.println("✅ Web server initialized.");
}

void handleWebServer() {
    server.handleClient();
}