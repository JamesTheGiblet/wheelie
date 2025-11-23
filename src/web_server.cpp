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

  // Mission control endpoints
  server.on("/mission/explore", HTTP_POST, [](AsyncWebServerRequest *request){
      Mission m;
      m.type = MISSION_EXPLORE;
      m.timeoutMs = 120000; // 2 minutes
      missionController.setMission(m);
      request->send(200, "text/plain", "Exploration mission started");
  });

  server.on("/mission/patrol", HTTP_POST, [](AsyncWebServerRequest *request){
      Mission m;
      m.type = MISSION_PATROL;
      m.targetPosition = hal.getPose().position; // Patrol around current position
      m.patrolRadius = 500.0f; // 50cm radius
      m.timeoutMs = 180000; // 3 minutes
      missionController.setMission(m);
      request->send(200, "text/plain", "Patrol mission started");
  });

  server.on("/mission/return", HTTP_POST, [](AsyncWebServerRequest *request){
      Mission m;
      m.type = MISSION_RETURN_TO_BASE;
      m.targetPosition = Vector2D(0, 0); // Home position
      missionController.setMission(m);
      request->send(200, "text/plain", "Returning to base");
  });

  server.on("/mission/abort", HTTP_POST, [](AsyncWebServerRequest *request){
      missionController.abortMission();
      request->send(200, "text/plain", "Mission aborted");
  });

  server.on("/waypoint", HTTP_POST, [](AsyncWebServerRequest *request){
      if (request->hasParam("x") && request->hasParam("y")) {
          float x = request->getParam("x")->value().toFloat();
          float y = request->getParam("y")->value().toFloat();
          
          Mission m;
          m.type = MISSION_GOTO_WAYPOINT;
          m.targetPosition = Vector2D(x, y);
          m.timeoutMs = 60000; // 1 minute
          missionController.setMission(m);
          
          char response[100];
          snprintf(response, sizeof(response), "Waypoint set to (%.1f, %.1f)", x, y);
          request->send(200, "text/plain", response);
      } else {
          request->send(400, "text/plain", "Missing x or y parameter");
      }
  });

  server.on("/role", HTTP_POST, [](AsyncWebServerRequest *request){
      String roleStr;
      if (request->hasParam("role")) {
          roleStr = request->getParam("role")->value();
      } else {
          request->send(400, "text/plain", "Missing role parameter");
          return;
      }
      RobotRole role = ROLE_NONE;
      
      if (roleStr == "leader") role = ROLE_LEADER;
      else if (roleStr == "scout") role = ROLE_SCOUT;
      else if (roleStr == "worker") role = ROLE_WORKER;
      else if (roleStr == "none") role = ROLE_NONE;
      else {
          request->send(400, "text/plain", "Unknown role");
          return;
      }
      
      missionController.setRole(role);
      
      char response[50];
      snprintf(response, sizeof(response), "Role set to %s", roleStr.c_str());
      request->send(200, "text/plain", response);
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

    // Mission status
    if (missionController.isMissionActive()) {
        Mission m = missionController.getCurrentMission();
        doc["mission"]["type"] = missionController.getMissionTypeName(m.type);
        doc["mission"]["target_x"] = m.targetPosition.x;
        doc["mission"]["target_y"] = m.targetPosition.y;
        doc["mission"]["runtime"] = (millis() - m.startTime) / 1000;
        doc["mission"]["status"] = "Active";
    } else {
        doc["mission"]["type"] = "IDLE";
        doc["mission"]["status"] = "Idle";
    }

    doc["role"] = missionController.getRoleName(missionController.getRole());
    // Serialize JSON to a string and send it
    String jsonString;
    serializeJson(doc, jsonString);
    ws.textAll(jsonString);
}