#ifndef SWARM_COMMUNICATOR_H
#define SWARM_COMMUNICATOR_H

#include <esp_now.h> // For ESP-NOW functions
#include <vector>    // For std::vector
#include "globals.h"

// Define PeerInfo if not already defined elsewhere
struct PeerInfo {
    uint8_t mac_addr[6];
    SwarmState state;
    unsigned long lastSeen;
    uint8_t lastSequence;
};

class SwarmCommunicator {
public:
    static SwarmCommunicator& getInstance();
    void begin();
    void update();
    void setMyState(const Vector2D& position, const Vector2D& velocity);
    std::vector<Vector2D> getOtherRobotPositions();
    uint16_t getRobotId() const;
    void printSwarmInfo();
    String getSwarmInfoString(); // <-- ADD THIS
    void broadcastLogMessage(const String& message);

private:
    SwarmCommunicator();
    SwarmCommunicator(const SwarmCommunicator&) = delete;
    void operator=(const SwarmCommunicator&) = delete;

    void _initializeEspNow();
    void _processIncomingState(const uint8_t* mac_addr, const SwarmState& state);
    void _cleanStalePeers();
    bool _isSequenceNewer(uint8_t newSeq, uint8_t oldSeq);
    void _sendBroadcast(const SwarmState& state);

    static void onDataReceived(const uint8_t* mac_addr, const uint8_t* incomingData, int len);

    bool _isInitialized;
    SwarmState _myState;
    PeerInfo _swarmPeers[MAX_SWARM_SIZE];
    int _swarmSize;
    uint8_t _sequenceNumber;
    unsigned long _lastBroadcastTime;
    esp_now_peer_info_t _broadcastPeerInfo;
};

#endif // SWARM_COMMUNICATOR_H