
#include "SwarmCommunicator.h"
#include <cstring>
#include <vector>
#include <globals.h>
#include <WiFi.h>

SwarmCommunicator& SwarmCommunicator::getInstance() {
    static SwarmCommunicator instance;
    return instance;
}

SwarmCommunicator::SwarmCommunicator() : _isInitialized(false), _swarmSize(0), _sequenceNumber(0), _lastBroadcastTime(0) {
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        memset(_swarmPeers[i].mac, 0, sizeof(_swarmPeers[i].mac));
        _swarmPeers[i].robotId = 0;
        _swarmPeers[i].lastSeen = 0;
        memset(&_swarmPeers[i].lastState, 0, sizeof(SwarmState));
    }
}

void SwarmCommunicator::begin() {
    // Initialize WiFi in STA mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register receive callback
    esp_now_register_recv_cb(SwarmCommunicator::onDataReceived);

    // Setup broadcast peer info
    memset(&_broadcastPeerInfo, 0, sizeof(_broadcastPeerInfo));
    for (int i = 0; i < 6; ++i) _broadcastPeerInfo.peer_addr[i] = 0xFF;
    _broadcastPeerInfo.channel = 0;
    _broadcastPeerInfo.encrypt = false;
    if (!esp_now_is_peer_exist(_broadcastPeerInfo.peer_addr)) {
        esp_now_add_peer(&_broadcastPeerInfo);
    }

    // Add known peers (if any)
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        if (_swarmPeers[i].robotId != 0 && memcmp(_swarmPeers[i].mac, _broadcastPeerInfo.peer_addr, 6) != 0) {
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, _swarmPeers[i].mac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            if (!esp_now_is_peer_exist(peerInfo.peer_addr)) {
                esp_now_add_peer(&peerInfo);
            }
        }
    }

    _isInitialized = true;
}

void SwarmCommunicator::update() {
    if (!_isInitialized) return;

    unsigned long now = millis();
    // Broadcast state every 100ms
    if (now - _lastBroadcastTime > 100) {
        _sendBroadcast(_myState);
        _lastBroadcastTime = now;
    }

    _cleanStalePeers();
}

void SwarmCommunicator::setMyState(const Vector2D& position, const Vector2D& velocity) {
    _myState.position = position;
    _myState.velocity = velocity;
}

std::vector<Vector2D> SwarmCommunicator::getOtherRobotPositions() {
    std::vector<Vector2D> positions;
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        if (_swarmPeers[i].lastSeen > 0) {
            positions.push_back(_swarmPeers[i].lastState.position);
        }
    }
    return positions;
}

void SwarmCommunicator::printSwarmInfo() {
    Serial.println("Swarm Info:");
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        if (_swarmPeers[i].lastSeen > 0) {
            Serial.print("Robot ");
            Serial.print(_swarmPeers[i].robotId);
            Serial.print(" at ");
            Serial.print(_swarmPeers[i].lastState.position.x);
            Serial.print(", ");
            Serial.print(_swarmPeers[i].lastState.position.y);
            Serial.print(" last seen: ");
            Serial.println(_swarmPeers[i].lastSeen);
        }
    }
}


void SwarmCommunicator::onDataReceived(const uint8_t* mac_addr, const uint8_t* incomingData, int len) {
    if (len != sizeof(SwarmState)) {
        return;
    }
    SwarmCommunicator& instance = getInstance();
    SwarmState state;
    memcpy(&state, incomingData, sizeof(SwarmState));
    instance._processIncomingState(mac_addr, state);
}


void SwarmCommunicator::broadcastLogMessage(const String& message) {
    if (!_isInitialized) return;
    // Quick solution: reuse SwarmState and send log message as a special state
    SwarmState logState = _myState;
    logState.velocity.x = -9999; // Marker for log event
    esp_now_send(_broadcastPeerInfo.peer_addr, (uint8_t*)&logState, sizeof(logState));
    // If you want to send the actual text, you must extend SwarmState or use a new struct.
}


void SwarmCommunicator::_sendBroadcast(const SwarmState& state) {
    if (!_isInitialized) return;
    esp_now_send(_broadcastPeerInfo.peer_addr, (uint8_t*)&state, sizeof(state));
}