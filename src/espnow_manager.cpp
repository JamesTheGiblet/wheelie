#include "espnow_manager.h"
#include "indicators.h"

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW MANAGER IMPLEMENTATION - Peer-to-peer communication system
// ═══════════════════════════════════════════════════════════════════════════

// Global Variables
ESPNowStatus espnowStatus;
ESPNowPeer peers[ESPNOW_MAX_PEERS];
uint16_t sequenceNumber = 0;
uint8_t localMacAddress[6];
uint8_t deviceId = 1; // Unique device ID for this robot

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW CORE FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

bool initializeESPNow() {
  Serial.println();
  Serial.println("🔗 INITIALIZING ESP-NOW COMMUNICATION");
  Serial.println("════════════════════════════════════════════════════════════════");
  
  // Get local MAC address
  WiFi.macAddress(localMacAddress);
  Serial.print("📍 Local MAC Address: ");
  Serial.println(macAddressToString(localMacAddress));
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed!");
    setLEDColor(LEDColors::RED);
    indicateError();
    return false;
  }
  
  // Register callbacks
  esp_now_register_send_cb(onESPNowDataSent);
  esp_now_register_recv_cb(onESPNowDataReceived);
  
  // Add broadcast peer for discovery
  esp_now_peer_info_t broadcastPeer = {};
  const uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(broadcastPeer.peer_addr, broadcastMac, 6);
  broadcastPeer.channel = ESPNOW_CHANNEL;
  broadcastPeer.encrypt = false;
  
  if (esp_now_add_peer(&broadcastPeer) != ESP_OK) {
      Serial.println("❌ Failed to add broadcast peer");
      return false;
  }
  
  // Initialize status
  espnowStatus.initialized = true;
  espnowStatus.channel = ESPNOW_CHANNEL;
  espnowStatus.peerCount = 0;
  espnowStatus.lastActivity = millis();
  
  // Initialize peer array
  for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
    peers[i].isActive = false;
    peers[i].deviceId = 0;
    peers[i].lastSeen = 0;
    peers[i].packetsReceived = 0;
    peers[i].packetsSent = 0;
  }
  
  Serial.println("✅ ESP-NOW initialized successfully!");
  Serial.print("📡 Channel: ");
  Serial.println(ESPNOW_CHANNEL);
  
  setLEDColor(LEDColors::GREEN);
  indicateSuccess();
  delay(500);
  clearLEDs();
  
  return true;
}

void shutdownESPNow() {
  if (espnowStatus.initialized) {
    esp_now_deinit();
    espnowStatus.initialized = false;
    Serial.println("🔗 ESP-NOW shutdown complete");
  }
}

bool isESPNowReady() {
  return espnowStatus.initialized;
}

// ═══════════════════════════════════════════════════════════════════════════
// PEER MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

bool addPeer(const uint8_t* macAddress, uint8_t peerDeviceId) {
  if (!isESPNowReady()) return false;
  
  // Check if peer already exists
  int existingIndex = findPeerIndex(macAddress);
  if (existingIndex >= 0) {
    peers[existingIndex].isActive = true;
    peers[existingIndex].lastSeen = millis();
    return true;
  }
  
  // Find empty slot
  int emptyIndex = -1;
  for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
    if (!peers[i].isActive) {
      emptyIndex = i;
      break;
    }
  }
  
  if (emptyIndex == -1) {
    Serial.println("⚠️ No available peer slots!");
    return false;
  }
  
  // Add peer to ESP-NOW
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, macAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add ESP-NOW peer");
    return false;
  }
  
  // Add to our peer list
  memcpy(peers[emptyIndex].macAddress, macAddress, 6);
  peers[emptyIndex].deviceId = peerDeviceId;
  peers[emptyIndex].isActive = true;
  peers[emptyIndex].lastSeen = millis();
  peers[emptyIndex].packetsReceived = 0;
  peers[emptyIndex].packetsSent = 0;
  
  espnowStatus.peerCount++;
  
  Serial.print("✅ Added peer: ");
  Serial.print(macAddressToString(macAddress));
  Serial.print(" (ID: ");
  Serial.print(peerDeviceId);
  Serial.println(")");
  
  return true;
}

bool removePeer(const uint8_t* macAddress) {
  if (!isESPNowReady()) return false;
  
  int index = findPeerIndex(macAddress);
  if (index < 0) return false;
  
  // Remove from ESP-NOW
  esp_now_del_peer(macAddress);
  
  // Remove from our list
  peers[index].isActive = false;
  espnowStatus.peerCount--;
  
  Serial.print("🗑️ Removed peer: ");
  Serial.println(macAddressToString(macAddress));
  
  return true;
}

int findPeerIndex(const uint8_t* macAddress) {
  for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
    if (peers[i].isActive && memcmp(peers[i].macAddress, macAddress, 6) == 0) {
      return i;
    }
  }
  return -1;
}

void updatePeerActivity(const uint8_t* macAddress, int rssi) {
  int index = findPeerIndex(macAddress);
  if (index >= 0) {
    peers[index].lastSeen = millis();
    peers[index].rssi = rssi;
    peers[index].packetsReceived++;
  }
}

void cleanupInactivePeers() {
  unsigned long currentTime = millis();
  const unsigned long PEER_TIMEOUT = 30000; // 30 seconds
  
  for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
    if (peers[i].isActive && (currentTime - peers[i].lastSeen) > PEER_TIMEOUT) {
      Serial.print("🧹 Cleaning up inactive peer: ");
      Serial.println(macAddressToString(peers[i].macAddress));
      removePeer(peers[i].macAddress);
    }
  }
}

int getActivePeerCount() {
  return espnowStatus.peerCount;
}

// ═══════════════════════════════════════════════════════════════════════════
// MESSAGE HANDLING
// ═══════════════════════════════════════════════════════════════════════════

bool sendMessage(const ESPNowMessage& message, const uint8_t* targetMac) {
  if (!isESPNowReady()) return false;
  
  // Send to specific peer (targetMac must not be null)
  esp_err_t result = esp_now_send(targetMac, (uint8_t*)&message, sizeof(ESPNowMessage));
  
  if (result == ESP_OK) {
    espnowStatus.messagesSent++;
    espnowStatus.lastActivity = millis();
    return true;
  } else {
    espnowStatus.sendFailures++;
    
    // Reduce spam by only showing "no peers" message every 10 failures
    static int noPeerMessageCount = 0;
    
    Serial.print("📡 ESP-NOW: ");
    
    // Provide descriptive error messages
    switch(result) {
      case ESP_ERR_ESPNOW_NOT_INIT:
        Serial.println("❌ Not initialized - ESP-NOW system not ready");
        break;
      case ESP_ERR_ESPNOW_ARG:
        Serial.println("❌ Invalid argument - Message format error");
        break;
      case ESP_ERR_ESPNOW_INTERNAL:
        Serial.println("❌ Internal error - ESP-NOW system fault");
        break;
      case ESP_ERR_ESPNOW_NO_MEM:
        Serial.println("❌ Out of memory - Too many pending messages");
        break;
      case ESP_ERR_ESPNOW_NOT_FOUND:
        if (noPeerMessageCount == 0) {
          Serial.println("🔍 No peer robots found - Operating solo");
          Serial.println("   💡 This is normal for single robot operation");
          Serial.println("   📡 Add more Wheelie robots for mesh networking");
        }
        noPeerMessageCount = (noPeerMessageCount + 1) % 20; // Show message every 20 failures
        break;
      case ESP_ERR_ESPNOW_IF:
        Serial.println("❌ WiFi interface error - Check WiFi connection");
        break;
      default:
        Serial.print("❌ Send failed (code ");
        Serial.print(result);
        Serial.println(") - Unknown ESP-NOW error");
        break;
    }
    return false;
  }
}

bool sendBroadcastMessage(const ESPNowMessage& message) {
  if (!isESPNowReady()) return false;
  
  // Send to the global broadcast MAC address
  const uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_err_t result = esp_now_send(broadcastMac, (uint8_t*)&message, sizeof(ESPNowMessage));
  
  if (result == ESP_OK) {
    espnowStatus.messagesSent++;
    espnowStatus.lastActivity = millis();
    return true;
  } else {
    espnowStatus.sendFailures++;
    Serial.println("📡 ESP-NOW: Broadcast failed");
    return false;
  }
}

bool sendHeartbeat() {
  uint8_t heartbeatData[4];
  heartbeatData[0] = deviceId;
  heartbeatData[1] = espnowStatus.peerCount;
  heartbeatData[2] = (uint8_t)(millis() >> 16); // High byte of uptime
  heartbeatData[3] = (uint8_t)(millis() >> 8);  // Mid byte of uptime
  
  ESPNowMessage message = createMessage(MSG_HEARTBEAT, heartbeatData, 4);
  return sendBroadcastMessage(message);
}

bool sendSensorData(const SensorData& sensorData) {
  // Pack sensor data into message
  uint8_t data[20];
  memcpy(data, &sensorData.distance, sizeof(int));           // 0-3: distance
  memcpy(data + 4, &sensorData.tiltX, sizeof(float));        // 4-7: tiltX
  memcpy(data + 8, &sensorData.tiltY, sizeof(float));        // 8-11: tiltY
  data[12] = sensorData.edgeDetected ? 1 : 0;                // 12: edge
  data[13] = sensorData.soundDetected ? 1 : 0;               // 13: sound
  data[14] = sensorData.motionDetected ? 1 : 0;              // 14: motion
  
  ESPNowMessage message = createMessage(MSG_SENSOR_DATA, data, 15);
  return sendBroadcastMessage(message);
}

bool sendStatusUpdate(const SystemStatus& status) {
  uint8_t data[10];
  data[0] = status.tofAvailable ? 1 : 0;
  data[1] = status.mpuAvailable ? 1 : 0;
  data[2] = status.pirAvailable ? 1 : 0;
  data[3] = status.wifiConnected ? 1 : 0;
  data[4] = status.espnowActive ? 1 : 0;
  data[5] = status.sensorsActive;
  memcpy(data + 6, &status.uptime, sizeof(uint32_t));        // 6-9: uptime
  
  ESPNowMessage message = createMessage(MSG_STATUS, data, 10);
  return sendBroadcastMessage(message);
}

bool sendCommand(uint8_t command, const uint8_t* cmdData, size_t dataSize, const uint8_t* targetMac) {
  if (dataSize > 239) return false; // Max payload - 1 byte for command
  
  uint8_t data[240];
  data[0] = command;
  if (cmdData && dataSize > 0) {
    memcpy(data + 1, cmdData, dataSize);
  }
  
  ESPNowMessage message = createMessage(MSG_COMMAND, data, dataSize + 1);
  return sendMessage(message, targetMac);
}

bool sendPairingRequest() {
  Serial.println("📡 ESP-NOW: Broadcasting pairing request...");
  
  uint8_t pairingData[8];
  pairingData[0] = deviceId;                                    // Our device ID
  pairingData[1] = ESPNOW_MAX_PEERS - espnowStatus.peerCount;   // Available peer slots
  memcpy(pairingData + 2, localMacAddress, 6);                 // Our MAC address
  
  ESPNowMessage message = createMessage(MSG_PAIRING, pairingData, 8);
  return sendBroadcastMessage(message);
}

// ═══════════════════════════════════════════════════════════════════════════
// MESSAGE UTILITIES
// ═══════════════════════════════════════════════════════════════════════════

ESPNowMessage createMessage(ESPNowMessageType type, const uint8_t* data, size_t dataSize) {
  ESPNowMessage message = {};
  
  message.type = type;
  message.deviceId = deviceId;
  message.timestamp = millis();
  message.sequenceNumber = ++sequenceNumber;
  
  if (data && dataSize > 0 && dataSize <= 240) {
    memcpy(message.data, data, dataSize);
  }
  
  message.checksum = calculateChecksum(message);
  
  return message;
}

uint8_t calculateChecksum(const ESPNowMessage& message) {
  uint8_t checksum = 0;
  checksum ^= message.type;
  checksum ^= message.deviceId;
  checksum ^= (uint8_t)(message.timestamp & 0xFF);
  checksum ^= (uint8_t)(message.sequenceNumber & 0xFF);
  
  for (int i = 0; i < 240; i++) {
    checksum ^= message.data[i];
  }
  
  return checksum;
}

bool validateMessage(const ESPNowMessage& message) {
  return calculateChecksum(message) == message.checksum;
}

void printMessage(const ESPNowMessage& message, bool incoming) {
  Serial.print(incoming ? "📨 RX: " : "📤 TX: ");
  Serial.print("Type=");
  Serial.print(message.type);
  Serial.print(" DeviceID=");
  Serial.print(message.deviceId);
  Serial.print(" Seq=");
  Serial.print(message.sequenceNumber);
  Serial.print(" Size=");
  Serial.print(sizeof(ESPNowMessage));
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════════════════
// CALLBACK FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void onESPNowDataReceived(const uint8_t* macAddress, const uint8_t* data, int dataLength) {
  if (dataLength != sizeof(ESPNowMessage)) {
    Serial.println("❌ Invalid message size received");
    return;
  }
  
  ESPNowMessage message;
  memcpy(&message, data, sizeof(ESPNowMessage));
  
  if (!validateMessage(message)) {
    Serial.println("❌ Message checksum validation failed");
    return;
  }
  
  espnowStatus.messagesReceived++;
  espnowStatus.lastActivity = millis();
  
  updatePeerActivity(macAddress);
  printMessage(message, true);
  
  processReceivedMessage(message, macAddress);
}

void onESPNowDataSent(const uint8_t* macAddress, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    // Update peer packet count if specific target
    if (macAddress != nullptr) {
      int index = findPeerIndex(macAddress);
      if (index >= 0) {
        peers[index].packetsSent++;
      }
    }
  } else {
    espnowStatus.sendFailures++;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// MESSAGE PROCESSING
// ═══════════════════════════════════════════════════════════════════════════

void processReceivedMessage(const ESPNowMessage& message, const uint8_t* senderMac) {
  switch (message.type) {
    case MSG_HEARTBEAT:
      handleHeartbeat(message, senderMac);
      break;
    case MSG_SENSOR_DATA:
      handleSensorData(message, senderMac);
      break;
    case MSG_COMMAND:
      handleCommand(message, senderMac);
      break;
    case MSG_STATUS:
      handleStatusUpdate(message, senderMac);
      break;
    case MSG_PAIRING:
      handlePairingRequest(message, senderMac);
      break;
    case MSG_ACK:  // <-- ADD THIS
      handleAck(message, senderMac);
      break;
    default:
      Serial.print("❓ Unknown message type: ");
      Serial.println(message.type);
  }
}

void handleHeartbeat(const ESPNowMessage& message, const uint8_t* senderMac) {
  // Auto-add peer if not already known
  if (findPeerIndex(senderMac) < 0) {
    addPeer(senderMac, message.deviceId);
  }
  
  Serial.print("💓 Heartbeat from device ");
  Serial.print(message.deviceId);
  Serial.print(" (");
  Serial.print(macAddressToString(senderMac));
  Serial.println(")");
}

void handleSensorData(const ESPNowMessage& message, const uint8_t* senderMac) {
  Serial.print("📊 Sensor data from device ");
  Serial.print(message.deviceId);
  
  // Decode sensor data
  int distance;
  float tiltX, tiltY;
  memcpy(&distance, message.data, sizeof(int));
  memcpy(&tiltX, message.data + 4, sizeof(float));
  memcpy(&tiltY, message.data + 8, sizeof(float));
  
  Serial.print(" - Distance: ");
  Serial.print(distance);
  Serial.print("mm, Tilt: ");
  Serial.print(tiltX);
  Serial.print("°, ");
  Serial.print(tiltY);
  Serial.println("°");
}

void handleCommand(const ESPNowMessage& message, const uint8_t* senderMac) {
  uint8_t command = message.data[0];
  Serial.print("🎛️ Command ");
  Serial.print(command);
  Serial.print(" from device ");
  Serial.println(message.deviceId);
  
  // Handle specific commands here
  // Example: if (command == 1) { /* move forward */ }
}

void handleStatusUpdate(const ESPNowMessage& message, const uint8_t* senderMac) {
  Serial.print("📊 Status update from device ");
  Serial.print(message.deviceId);
  Serial.print(" - Sensors: ");
  Serial.print(message.data[5]);
  Serial.println(" active");
}

void handlePairingRequest(const ESPNowMessage& message, const uint8_t* senderMac) {
  Serial.print("🤝 Pairing request from device ");
  Serial.print(message.deviceId);
  Serial.print(" (");
  Serial.print(macAddressToString(senderMac));
  Serial.println(")");

  // Only add the peer if we don't know them already
  if (findPeerIndex(senderMac) < 0) {
    // Add the requesting peer
    addPeer(senderMac, message.deviceId);
    // Send an acknowledgment back so they can add us
    ESPNowMessage ack = createMessage(MSG_ACK, nullptr, 0);
    sendMessage(ack, senderMac);
  }
}

void handleAck(const ESPNowMessage& message, const uint8_t* senderMac) {
    Serial.print("🤝 Pairing ACK from device ");
    Serial.println(message.deviceId);
    
    // This is the final step: add the peer that acknowledged us
    addPeer(senderMac, message.deviceId);
}

// ═══════════════════════════════════════════════════════════════════════════
// STATUS AND INFORMATION
// ═══════════════════════════════════════════════════════════════════════════

void printESPNowStatus() {
  Serial.println();
  Serial.println("📡 ESP-NOW STATUS REPORT");
  Serial.println("════════════════════════════════════════════════════════════════");
  Serial.print("🔗 Status: ");
  Serial.println(espnowStatus.initialized ? "Active" : "Inactive");
  Serial.print("📻 Channel: ");
  Serial.println(espnowStatus.channel);
  Serial.print("👥 Active Peers: ");
  Serial.println(espnowStatus.peerCount);
  Serial.print("📤 Messages Sent: ");
  Serial.println(espnowStatus.messagesSent);
  Serial.print("📥 Messages Received: ");
  Serial.println(espnowStatus.messagesReceived);
  Serial.print("❌ Send Failures: ");
  Serial.println(espnowStatus.sendFailures);
  Serial.print("⏰ Last Activity: ");
  Serial.print((millis() - espnowStatus.lastActivity) / 1000);
  Serial.println(" seconds ago");
  Serial.print("📍 MAC Address: ");
  Serial.println(macAddressToString(localMacAddress));
  Serial.println("════════════════════════════════════════════════════════════════");
}

void printPeerList() {
  Serial.println();
  Serial.println("👥 ACTIVE PEERS LIST");
  Serial.println("════════════════════════════════════════════════════════════════");
  
  if (espnowStatus.peerCount == 0) {
    Serial.println("   No active peers");
  } else {
    for (int i = 0; i < ESPNOW_MAX_PEERS; i++) {
      if (peers[i].isActive) {
        Serial.print("🤖 Device ID ");
        Serial.print(peers[i].deviceId);
        Serial.print(": ");
        Serial.print(macAddressToString(peers[i].macAddress));
        Serial.print(" - Last seen: ");
        Serial.print((millis() - peers[i].lastSeen) / 1000);
        Serial.print("s ago, RX: ");
        Serial.print(peers[i].packetsReceived);
        Serial.print(", TX: ");
        Serial.println(peers[i].packetsSent);
      }
    }
  }
  Serial.println("════════════════════════════════════════════════════════════════");
}

String getESPNowStatusString() {
  return String("ESP-NOW: ") + (espnowStatus.initialized ? "Active" : "Inactive") + 
         " (" + String(espnowStatus.peerCount) + " peers)";
}

uint8_t* getLocalMacAddress() {
  return localMacAddress;
}

String macAddressToString(const uint8_t* macAddress) {
  char buffer[18];
  sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",
          macAddress[0], macAddress[1], macAddress[2],
          macAddress[3], macAddress[4], macAddress[5]);
  return String(buffer);
}

// ═══════════════════════════════════════════════════════════════════════════
// MAINTENANCE
// ═══════════════════════════════════════════════════════════════════════════

void updateESPNowStatus() {
  // Update general status
  espnowStatus.lastActivity = millis();
  
  // Clean up inactive peers periodically
  static unsigned long lastCleanup = 0;
  if (millis() - lastCleanup > 60000) { // Every 60 seconds
    cleanupInactivePeers();
    lastCleanup = millis();
  }
}

void performESPNowMaintenance() {
  updateESPNowStatus();
  
  // Send periodic heartbeat
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast > ESPNOW_HEARTBEAT_INTERVAL) {
    // Always send a heartbeat. This serves for both discovery and keep-alive.
    // New robots will hear this and send a pairing request.
    sendHeartbeat();
    lastBroadcast = millis();
  }
}