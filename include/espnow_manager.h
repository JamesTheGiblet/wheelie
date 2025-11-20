#ifndef ESPNOW_MANAGER_H
#define ESPNOW_MANAGER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW MANAGER - Peer-to-peer communication system
// ═══════════════════════════════════════════════════════════════════════════
// 
// This manager provides a complete, robust communication system for ESP-NOW
// peer-to-peer networking. Features include:
//
// • Automatic peer discovery and pairing (sendPairingRequest/handlePairingRequest)
// • Message validation with checksums (validateMessage/calculateChecksum)
// • Peer activity tracking and cleanup (updatePeerActivity/cleanupInactivePeers)
// • Unambiguous API design (sendMessage requires MAC, sendBroadcastMessage for broadcasts)
// • Comprehensive error handling and status reporting
// • Support for multiple message types: heartbeat, sensor data, commands, status
//
// API Design Philosophy:
// - Explicit over implicit: sendMessage() requires a target MAC address
// - Clear intent: sendBroadcastMessage() for broadcasts, not nullable parameters
// - Complete pairing: both proactive (sendPairingRequest) and reactive (handlePairingRequest)
// ═══════════════════════════════════════════════════════════════════════════

// Global ESP-NOW status
extern ESPNowStatus espnowStatus;
extern ESPNowPeer peers[ESPNOW_MAX_PEERS];

// ESP-NOW Core Functions
bool initializeESPNow();
void shutdownESPNow();
bool isESPNowReady();

// Peer Management
bool addPeer(const uint8_t* macAddress, uint8_t deviceId = 0);
bool removePeer(const uint8_t* macAddress);
int findPeerIndex(const uint8_t* macAddress);
void updatePeerActivity(const uint8_t* macAddress, int rssi = 0);
void cleanupInactivePeers();
int getActivePeerCount();

// Message Handling
bool sendMessage(const ESPNowMessage& message, const uint8_t* targetMac);  // Send to specific MAC address (required)
bool sendBroadcastMessage(const ESPNowMessage& message);                   // Broadcast to all nearby devices
bool sendHeartbeat();                                                      // Send periodic heartbeat signal
bool sendSensorData(const SensorData& sensorData);                         // Broadcast sensor readings
bool sendStatusUpdate(const SystemStatus& status);                         // Broadcast system status
bool sendCommand(uint8_t command, const uint8_t* data, size_t dataSize, const uint8_t* targetMac);  // Send command to specific device
bool sendPairingRequest();                                                 // Broadcast pairing request to discover peers

// Message Utilities
ESPNowMessage createMessage(ESPNowMessageType type, const uint8_t* data, size_t dataSize);
uint8_t calculateChecksum(const ESPNowMessage& message);
bool validateMessage(const ESPNowMessage& message);
void printMessage(const ESPNowMessage& message, bool incoming = true);

// Status and Information
void printESPNowStatus();
void printPeerList();
String getESPNowStatusString();
uint8_t* getLocalMacAddress();
String macAddressToString(const uint8_t* macAddress);

// Callback Functions (implemented in cpp file)
void onESPNowDataReceived(const uint8_t* macAddress, const uint8_t* data, int dataLength);
void onESPNowDataSent(const uint8_t* macAddress, esp_now_send_status_t status);

// Message Processing
void processReceivedMessage(const ESPNowMessage& message, const uint8_t* senderMac);
void handleHeartbeat(const ESPNowMessage& message, const uint8_t* senderMac);
void handleSensorData(const ESPNowMessage& message, const uint8_t* senderMac);
void handleCommand(const ESPNowMessage& message, const uint8_t* senderMac);
void handleStatusUpdate(const ESPNowMessage& message, const uint8_t* senderMac);
void handlePairingRequest(const ESPNowMessage& message, const uint8_t* senderMac);
void handleAck(const ESPNowMessage& message, const uint8_t* senderMac);

// Maintenance
void updateESPNowStatus();
void performESPNowMaintenance();

#endif // ESPNOW_MANAGER_H