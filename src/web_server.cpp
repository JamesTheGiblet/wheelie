#include <Arduino.h>
#include "ESPAsyncWebServer.h"
#include "LittleFS.h"
#include "ArduinoJson.h"
#include "main.h"
#include "power_manager.h" // For battery info

// --- Global Objects ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void pushTelemetryToClients(); // Forward declaration

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

void initializeWebServer() {
  Serial.println("🌐 Initializing Web Server...");

  // --- Mount Filesystem ---
  if (!LittleFS.begin()) {
    Serial.println("❌ LittleFS mount failed. Dashboard will not be available.");
    return;
  }

  // --- WebSocket Server ---
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // --- HTTP Routes ---

  // Serve the main dashboard page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  // Simple command handlers
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("[WEB] Start command received");
    setRobotState(ROBOT_EXPLORING);
    request->send(200, "text/plain", "OK, starting exploration.");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("[WEB] Stop command received");
    setRobotState(ROBOT_IDLE);
    hal.emergencyStop();
    request->send(200, "text/plain", "OK, stopping.");
  });

  // --- Start Server ---
  server.begin();
  Serial.println("✅ Web Server started. Dashboard available at http://" + WiFi.localIP().toString());
}

void pushTelemetryToClients() {
    // Only push data if there are clients connected
    if (ws.count() == 0) {
        return;
    }

    // Create a JSON document
    // Use JsonDocument for stack allocation, which is safer on ESP32
    JsonDocument doc;

    // System Status
    doc["state"] = getRobotStateString(getCurrentState());
    doc["uptime"] = millis() / 1000;
    doc["ip"] = sysStatus.ipAddress;
    doc["peers"] = sysStatus.espnowStatus.peerCount;
    doc["free_heap"] = esp_get_free_heap_size();

    // Power Status
    doc["voltage"] = hal.getBatteryVoltage();
    doc["percent"] = getBatteryPercentage();

    // Sensor Data
    doc["dist_f"] = sensors.frontDistanceCm;
    doc["dist_r"] = sensors.rearDistanceCm;
    doc["tilt_x"] = sensors.tiltX;
    doc["tilt_y"] = sensors.tiltY;

    // Navigation Data
    RobotPose pose = hal.getPose();
    doc["pos_x"] = pose.position.x;
    doc["pos_y"] = pose.position.y;
    doc["heading"] = pose.heading;

    // Serialize JSON to a string and send it
    String jsonString;
    serializeJson(doc, jsonString);
    ws.textAll(jsonString);
}