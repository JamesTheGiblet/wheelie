#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "web_server.h"
#include "robot.h"

WebServer server(80);

extern SystemStatus sysStatus;
extern SensorData sensors;
extern NavigationData navigation;
extern PowerMode_t currentPowerMode;
extern BatteryMonitor_t battery;

void handleRoot();
void handleApiStatus();
void handleStart();
void handleStop();
void handleNotFound();

void initializeWebServer() {
    if (!sysStatus.wifiConnected) {
        Serial.println("⚠️ Web server cannot start, WiFi is not connected.");
        return;
    }
    if(!SPIFFS.begin(true)){
        Serial.println("❌ An Error has occurred while mounting SPIFFS");
        return;
    }
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleApiStatus);
    server.on("/start", HTTP_GET, handleStart);
    server.on("/stop", HTTP_GET, handleStop);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("✅ Web Server started. Access at http://" + String(sysStatus.ipAddress));
}

void handleWebServer() {
    server.handleClient();
}

void handleRoot() {
    File file = SPIFFS.open("/index.html", "r");
    if (!file) {
        server.send(404, "text/plain", "Error: index.html not found in SPIFFS. Did you upload the filesystem?");
        return;
    }
    server.streamFile(file, "text/html");
    file.close();
}

void handleApiStatus() {
    StaticJsonDocument<1024> doc;
    doc["state"] = sysStatus.currentState;
    doc["uptime"] = sysStatus.uptime / 1000;
    doc["wifi"] = sysStatus.wifiConnected ? "Connected" : "Disconnected";
    doc["ip"] = String(sysStatus.ipAddress);
    doc["peers"] = sysStatus.espnowStatus.peerCount;

    doc["voltage"] = String(battery.voltage, 2);
    doc["percent"] = String(battery.percentage, 1);

    switch(currentPowerMode) {
        case POWER_ECONOMY: doc["power_mode"] = "ECONOMY"; break;
        case POWER_LOW: doc["power_mode"] = "LOW"; break;
        case POWER_CRITICAL: doc["power_mode"] = "CRITICAL"; break;
        case POWER_SHUTDOWN: doc["power_mode"] = "SHUTDOWN"; break;
        default: doc["power_mode"] = "NORMAL";
    }

    doc["dist"] = sysStatus.tofAvailable ? String(sensors.distance) : "OFFLINE";
    doc["tilt"] = sysStatus.mpuAvailable ? (String(sensors.tiltX, 1) + "° / " + String(sensors.tiltY, 1) + "°") : "OFFLINE";
    doc["heading"] = sysStatus.mpuAvailable ? String(sensors.headingAngle, 1) : "OFFLINE";
    doc["edge"] = sensors.edgeDetected ? "YES" : "No";

    switch(navigation.mode) {
        case NAV_OBSTACLE_AVOIDING: doc["nav_mode"] = "AVOIDING"; break;
        case NAV_STUCK_RECOVERY: doc["nav_mode"] = "RECOVERING"; break;
        case NAV_ROUTE_PLANNING: doc["nav_mode"] = "PLANNING"; break;
        default: doc["nav_mode"] = "EXPLORING";
    }
    doc["pos"] = String(navigation.currentX, 0) + " / " + String(navigation.currentY, 0) + " mm";
    doc["nav_heading"] = String(navigation.headingAngle, 1);
    doc["stuck"] = navigation.isStuck ? "Yes" : "No";
    String jsonOutput;
    serializeJson(doc, jsonOutput);
    server.send(200, "application/json", jsonOutput);
}

void handleStart() {
    Serial.println("WEB: Received /start command.");
    setRobotState(ROBOT_EXPLORING);
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Starting exploration...");
}

void handleStop() {
    Serial.println("WEB: Received /stop command.");
    allStop();
    setRobotState(ROBOT_IDLE);
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Stopping...");
}

void handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}
