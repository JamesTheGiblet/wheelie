#ifndef SWARM_COMMUNICATOR_H
#define SWARM_COMMUNICATOR_H

#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <Arduino.h>
#include "Vector2D.h" // For position and velocity
#include "main.h"     // For sysStatus access

#define MAX_SWARM_SIZE 10
#define BROADCAST_INTERVAL 100     // ms
#define PEER_TIMEOUT_MS 3000       // 3 seconds


// Internal struct to track other robots in the swarm
struct SwarmPeer {
    uint8_t mac[6];
    uint16_t robotId;
    SwarmState lastState;
    unsigned long lastSeen;
};

class SwarmCommunicator {
public:
    // --- Singleton Access ---
    static SwarmCommunicator& getInstance();

    // --- Public API ---
    void begin();
    void update();
    void setMyState(const Vector2D& position, const Vector2D& velocity);
    void broadcastLogMessage(const String& message); // Will use SwarmState as a marker only
    std::vector<Vector2D> getOtherRobotPositions();
    void printSwarmInfo();

private:
    // --- Private Methods & Callbacks ---
    SwarmCommunicator(); // Private constructor for singleton
    void _processIncomingState(const uint8_t* mac_addr, const SwarmState& state);
    void _sendBroadcast(const SwarmState& state);
    void _cleanStalePeers();
    bool _isSequenceNewer(uint8_t newSeq, uint8_t oldSeq);

    // Static callback that wraps the instance method
    static void onDataReceived(const uint8_t* mac_addr, const uint8_t* incomingData, int len);

    // --- Member Variables ---
    bool _isInitialized;
    SwarmState _myState;
    SwarmPeer _swarmPeers[MAX_SWARM_SIZE];
    uint8_t _swarmSize;
    uint8_t _sequenceNumber;
    unsigned long _lastBroadcastTime;
    esp_now_peer_info_t _broadcastPeerInfo;

    // Deleted copy constructor and assignment operator to enforce singleton
    SwarmCommunicator(const SwarmCommunicator&) = delete;
    void operator=(const SwarmCommunicator&) = delete;
};

#endif // SWARM_COMMUNICATOR_H