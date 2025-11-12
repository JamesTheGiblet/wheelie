# MCP ESP32 Bridge Firmware

This firmware forwards ESP-NOW packets to/from the PC over Serial. It listens for ESP-NOW packets, sends them to the PC, and relays packets from the PC to ESP-NOW.

- Receives ESP-NOW packets and sends them to Serial as raw bytes.
- Receives Serial data and sends as ESP-NOW packets.
- Uses a fixed broadcast MAC for provisioning.

---

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define SERIAL_BAUD 115200
#define BRIDGE_CHANNEL 1

// Buffer sizes
#define MAX_PACKET_SIZE 64

// Broadcast MAC address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // Forward received ESP-NOW packet to Serial
  Serial.write((const char *)data, len);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = BRIDGE_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  Serial.println("MCP Bridge Ready");
}

void loop() {
  // Forward Serial data to ESP-NOW
  if (Serial.available()) {
    uint8_t buffer[MAX_PACKET_SIZE];
    int len = Serial.readBytes(buffer, MAX_PACKET_SIZE);
    if (len > 0) {
      esp_now_send(broadcastAddress, buffer, len);
    }
  }
}
