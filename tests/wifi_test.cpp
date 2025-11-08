
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "credentials.h"

WebServer server(80);

unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 10000; // 10 seconds
int reconnectAttempts = 0;

void printWiFiStatus() {
  int status = WiFi.status();
  Serial.print("[DEBUG] WiFi.status() = ");
  Serial.print(status);
  switch (status) {
    case WL_IDLE_STATUS: Serial.print(" (IDLE_STATUS)"); break;
    case WL_NO_SSID_AVAIL: Serial.print(" (NO_SSID_AVAIL)"); break;
    case WL_SCAN_COMPLETED: Serial.print(" (SCAN_COMPLETED)"); break;
    case WL_CONNECTED: Serial.print(" (CONNECTED)"); break;
    case WL_CONNECT_FAILED: Serial.print(" (CONNECT_FAILED)"); break;
    case WL_CONNECTION_LOST: Serial.print(" (CONNECTION_LOST)"); break;
    case WL_DISCONNECTED: Serial.print(" (DISCONNECTED)"); break;
    default: Serial.print(" (UNKNOWN)"); break;
  }
  Serial.println();
}

void handleRoot() {
  String html = "<html><body><h2>ESP32 WiFi Test: Connected!</h2>";
  html += "<p>IP: " + WiFi.localIP().toString() + "</p>";
  html += "<p>RSSI: " + String(WiFi.RSSI()) + " dBm</p>";
  html += "<p>Reconnects: " + String(reconnectAttempts) + "</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
    printWiFiStatus();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("[ERROR] WiFi connection timed out. Will retry in loop.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[WiFi Robust Test] Connecting...");
  connectToWiFi();
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  server.handleClient();
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > reconnectInterval) {
      Serial.println("[WARN] WiFi lost. Attempting reconnect...");
      reconnectAttempts++;
      connectToWiFi();
      lastReconnectAttempt = now;
    }
  }
  delay(100);
}
