#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "main.h"
#include "robot.h"      // For getRobotStateString()
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

void handleApiStatus() {
    StaticJsonDocument<512> doc;

    doc["state"] = getRobotStateString(sysStatus.currentState);
    doc["uptime"] = millis() / 1000;
    doc["wifi"] = sysStatus.wifiConnected ? "Connected" : "Disconnected";
    doc["ip"] = sysStatus.ipAddress;
    doc["peers"] = sysStatus.espnowStatus.peerCount;

    doc["voltage"] = battery.voltage;
    doc["percent"] = battery.percentage;
    doc["power_mode"] = getPowerModeString(currentPowerMode);

    doc["dist"] = sensors.distance;
    JsonObject tilt = doc.createNestedObject("tilt");
    tilt["x"] = sensors.tiltX;
    tilt["y"] = sensors.tiltY;
    doc["heading"] = sensors.headingAngle;
    doc["edge"] = sensors.edgeDetected ? "YES" : "No";

    RobotPose pose = hal.getPose();
    doc["nav_mode"] = "Potential Field"; // Placeholder for now
    JsonObject pos = doc.createNestedObject("pos");
    pos["x"] = pose.position.x;
    pos["y"] = pose.position.y;
    doc["nav_heading"] = pose.heading;
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
    // Try to serve index.html from LittleFS
    // The filesystem is already initialized by the logger
    File file = LittleFS.open("/index.html", "r");
    if (file) {
        server.streamFile(file, "text/html");
        file.close();
        return;
    }
    // Fallback: show placeholder message
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