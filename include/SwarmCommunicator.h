#pragma once
#include <esp_now.h>
#include <WiFi.h>
#include "Vector2D.h"
#include <vector>

#define MAX_SWARM_SIZE 10
#define BROADCAST_INTERVAL 100  // ms

struct RobotState {
    uint8_t mac[6];
    Vector2D position;
    Vector2D velocity;
    uint32_t timestamp;
    uint16_t robotId;
    uint8_t sequence;
};

class SwarmCommunicator {
private:
    RobotState myState;
    RobotState swarmStates[MAX_SWARM_SIZE];
    uint8_t swarmSize;
    esp_now_peer_info_t peerInfo;
    uint16_t sequenceNumber;
    unsigned long lastBroadcast;

    // Static callback declaration
    static void onDataReceived(const uint8_t* mac, const uint8_t* data, int len);
    
public:
    SwarmCommunicator() : swarmSize(0), sequenceNumber(0), lastBroadcast(0) {
        // Initialize my state
        WiFi.macAddress(myState.mac);
        myState.robotId = (myState.mac[4] << 8) | myState.mac[5];  // Unique ID from MAC
        myState.sequence = 0;
        
        // Initialize ESP-NOW
        WiFi.mode(WIFI_STA);
        if (esp_now_init() != ESP_OK) {
            Serial.println("Error initializing ESP-NOW");
            return;
        }
        
        // Register callback
        esp_now_register_recv_cb(onDataReceived);
        
        // Add broadcast peer
        memset(&peerInfo, 0, sizeof(peerInfo));
        for (int i = 0; i < 6; i++) {
            peerInfo.peer_addr[i] = 0xFF;  // Broadcast address
        }
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        
        esp_now_add_peer(&peerInfo);
    }
    
    void setMyState(const Vector2D& position, const Vector2D& velocity) {
        myState.position = position;
        myState.velocity = velocity;
        myState.timestamp = millis();
        myState.sequence = sequenceNumber++;
    }
    
    void update() {
        unsigned long currentTime = millis();
        
        // Broadcast state periodically
        if (currentTime - lastBroadcast >= BROADCAST_INTERVAL) {
            broadcastState();
            lastBroadcast = currentTime;
        }
        
        // Clean up stale swarm states (older than 2 seconds)
        cleanStaleStates();
    }
    
    void broadcastState() {
        esp_now_send(peerInfo.peer_addr, (uint8_t*)&myState, sizeof(myState));
    }
    
    std::vector<Vector2D> getOtherRobotPositions() {
        std::vector<Vector2D> positions;
        unsigned long currentTime = millis();
        
        for (int i = 0; i < swarmSize; i++) {
            // Only include recent positions (last 1 second)
            if (currentTime - swarmStates[i].timestamp < 1000) {
                positions.push_back(swarmStates[i].position);
            }
        }
        
        return positions;
    }
    
    void printSwarmInfo() {
        Serial.printf("Swarm size: %d\n", swarmSize);
        for (int i = 0; i < swarmSize; i++) {
            Serial.printf("Robot %d: Pos%s Vel%s Age: %lums\n", 
                         swarmStates[i].robotId,
                         swarmStates[i].position.toString().c_str(),
                         swarmStates[i].velocity.toString().c_str(),
                         millis() - swarmStates[i].timestamp);
        }
    }
    
private:
    static void onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
        if (len == sizeof(RobotState)) {
            SwarmCommunicator* instance = getInstance();
            instance->processReceivedState(*(RobotState*)data);
        }
    }
    
    void processReceivedState(const RobotState& receivedState) {
        // Don't process our own messages
        if (memcmp(receivedState.mac, myState.mac, 6) == 0) {
            return;
        }
        
        // Update or add swarm state
        int existingIndex = -1;
        for (int i = 0; i < swarmSize; i++) {
            if (memcmp(swarmStates[i].mac, receivedState.mac, 6) == 0) {
                existingIndex = i;
                break;
            }
        }
        
        if (existingIndex >= 0) {
            // Update existing - only if sequence is newer
            if (isSequenceNewer(receivedState.sequence, swarmStates[existingIndex].sequence)) {
                swarmStates[existingIndex] = receivedState;
            }
        } else if (swarmSize < MAX_SWARM_SIZE) {
            // Add new robot to swarm
            swarmStates[swarmSize] = receivedState;
            swarmSize++;
        }
    }
    
    bool isSequenceNewer(uint8_t newSeq, uint8_t oldSeq) {
        // Handle sequence number wrap-around
        return ((newSeq > oldSeq) && (newSeq - oldSeq < 128)) ||
               ((oldSeq > newSeq) && (oldSeq - newSeq > 128));
    }
    
    void cleanStaleStates() {
        unsigned long currentTime = millis();
        int writeIndex = 0;
        
        for (int readIndex = 0; readIndex < swarmSize; readIndex++) {
            if (currentTime - swarmStates[readIndex].timestamp < 3000) {  // 3 second timeout
                if (writeIndex != readIndex) {
                    swarmStates[writeIndex] = swarmStates[readIndex];
                }
                writeIndex++;
            }
        }
        
        swarmSize = writeIndex;
    }
    
    static SwarmCommunicator* getInstance() {
        static SwarmCommunicator instance;
        return &instance;
    }
};