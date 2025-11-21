#include "SwarmCommunicator.h"
#include <cstring>
#include <vector>
#include <globals.h>
#include <WiFi.h>

SwarmCommunicator& SwarmCommunicator::getInstance() {
    static SwarmCommunicator instance;
    return instance;
}

SwarmCommunicator::SwarmCommunicator() 
    : _isInitialized(false), _swarmSize(0), _sequenceNumber(0), _lastBroadcastTime(0) {
    // Initialize peer array
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        memset(_swarmPeers[i].mac, 0, sizeof(_swarmPeers[i].mac));
        _swarmPeers[i].robotId = 0;
        _swarmPeers[i].lastSeen = 0;
        memset(&_swarmPeers[i].lastState, 0, sizeof(SwarmState));
    }
    
    // Initialize my MAC address and robot ID
    WiFi.macAddress(_myState.mac);
    _myState.robotId = (_myState.mac[4] << 8) | _myState.mac[5]; // Unique ID from MAC
    _myState.sequence = 0;
    _myState.timestamp = 0;
}

void SwarmCommunicator::begin() {
    Serial.println("📡 Initializing ESP-NOW Swarm Communication...");
    
    // Initialize WiFi in STA mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ Error initializing ESP-NOW");
        return;
    }

    // Register receive callback
    esp_now_register_recv_cb(SwarmCommunicator::onDataReceived);

    // Setup broadcast peer info
    memset(&_broadcastPeerInfo, 0, sizeof(_broadcastPeerInfo));
    for (int i = 0; i < 6; ++i) {
        _broadcastPeerInfo.peer_addr[i] = 0xFF; // Broadcast address
    }
    _broadcastPeerInfo.channel = 0;
    _broadcastPeerInfo.encrypt = false;
    
    if (!esp_now_is_peer_exist(_broadcastPeerInfo.peer_addr)) {
        esp_now_add_peer(&_broadcastPeerInfo);
    }

    _isInitialized = true;
    Serial.printf("✅ ESP-NOW initialized. Robot ID: %d\n", _myState.robotId);
}

void SwarmCommunicator::update() {
    if (!_isInitialized) return;

    unsigned long now = millis();
    
    // Broadcast state every 100ms
    if (now - _lastBroadcastTime >= SWARM_BROADCAST_INTERVAL_MS) {
        // Update timestamp and sequence before sending
        _myState.timestamp = now;
        _myState.sequence = _sequenceNumber++;
        
        _sendBroadcast(_myState);
        _lastBroadcastTime = now;
    }

    // Clean up stale peers
    _cleanStalePeers();
}

void SwarmCommunicator::setMyState(const Vector2D& position, const Vector2D& velocity) {
    _myState.position = position;
    _myState.velocity = velocity;
    // timestamp and sequence are updated in update() just before broadcast
}

std::vector<Vector2D> SwarmCommunicator::getOtherRobotPositions() {
    std::vector<Vector2D> positions;
    unsigned long now = millis();
    
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        // Only include recent positions (within 1 second)
        if (_swarmPeers[i].robotId != 0 && (now - _swarmPeers[i].lastSeen < 1000)) {
            positions.push_back(_swarmPeers[i].lastState.position);
        }
    }
    
    return positions;
}

void SwarmCommunicator::printSwarmInfo() {
    Serial.println("\n📡 SWARM STATUS:");
    Serial.println("──────────────────────────────────────");
    Serial.printf("My Robot ID: %d\n", _myState.robotId);
    Serial.printf("Active Peers: %d\n", _swarmSize);
    
    unsigned long now = millis();
    int activePeers = 0;
    
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        if (_swarmPeers[i].robotId != 0) {
            unsigned long age = now - _swarmPeers[i].lastSeen;
            bool isRecent = age < 1000;
            
            Serial.printf("  Robot %d: Pos(%.1f, %.1f) Vel(%.1f, %.1f) - %lums ago %s\n", 
                _swarmPeers[i].robotId,
                _swarmPeers[i].lastState.position.x,
                _swarmPeers[i].lastState.position.y,
                _swarmPeers[i].lastState.velocity.x,
                _swarmPeers[i].lastState.velocity.y,
                age,
                isRecent ? "✅" : "⚠️"
            );
            
            if (isRecent) activePeers++;
        }
    }
    
    Serial.printf("Recently Active: %d peers\n", activePeers);
    Serial.println("──────────────────────────────────────");
}

void SwarmCommunicator::broadcastLogMessage(const String& message) {
    if (!_isInitialized) return;
    
    // Create a special log marker state
    // This is a HACK - ideally you'd create a separate message type
    SwarmState logState = _myState;
    logState.velocity.x = -9999.0f; // Magic marker for log events
    logState.timestamp = millis();
    
    esp_now_send(_broadcastPeerInfo.peer_addr, (uint8_t*)&logState, sizeof(logState));
    
    // Note: The actual text message is NOT transmitted in this implementation.
    // To send text, you'd need to define a separate LogMessage struct.
}

// ═══════════════════════════════════════════════════════════════════════════
// PRIVATE METHODS
// ═══════════════════════════════════════════════════════════════════════════

void SwarmCommunicator::_processIncomingState(const uint8_t* mac_addr, const SwarmState& state) {
    // Ignore our own broadcasts
    if (memcmp(state.mac, _myState.mac, 6) == 0) {
        return;
    }
    
    // Ignore log marker messages (hack from broadcastLogMessage)
    if (state.velocity.x == -9999.0f) {
        return; // This is a log broadcast, not actual state
    }
    
    // Find existing peer by MAC address
    int peerIndex = -1;
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        if (_swarmPeers[i].robotId != 0 && memcmp(_swarmPeers[i].mac, mac_addr, 6) == 0) {
            peerIndex = i;
            break;
        }
    }
    
    if (peerIndex != -1) {
        // Update existing peer - but only if sequence is newer
        if (_isSequenceNewer(state.sequence, _swarmPeers[peerIndex].lastState.sequence)) {
            _swarmPeers[peerIndex].lastState = state;
            _swarmPeers[peerIndex].lastSeen = millis();
        }
        // else: This is an old/duplicate packet, ignore it
    } else {
        // Add new peer - find an empty slot
        for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
            if (_swarmPeers[i].robotId == 0) {
                memcpy(_swarmPeers[i].mac, mac_addr, 6);
                _swarmPeers[i].robotId = state.robotId;
                _swarmPeers[i].lastState = state;
                _swarmPeers[i].lastSeen = millis();
                _swarmSize++; // Increment count
                
                Serial.printf("🤝 New peer discovered: Robot ID %d\n", state.robotId);
                break;
            }
        }
    }
}

void SwarmCommunicator::_cleanStalePeers() {
    unsigned long now = millis();
    
    for (int i = 0; i < MAX_SWARM_SIZE; ++i) {
        if (_swarmPeers[i].robotId != 0 && (now - _swarmPeers[i].lastSeen > PEER_TIMEOUT_MS)) {
            Serial.printf("👋 Peer timeout: Robot ID %d\n", _swarmPeers[i].robotId);
            
            // Clear the peer slot
            _swarmPeers[i].robotId = 0;
            memset(_swarmPeers[i].mac, 0, sizeof(_swarmPeers[i].mac));
            memset(&_swarmPeers[i].lastState, 0, sizeof(SwarmState));
            _swarmPeers[i].lastSeen = 0;
            _swarmSize--; // Decrement count
        }
    }
}

bool SwarmCommunicator::_isSequenceNewer(uint8_t newSeq, uint8_t oldSeq) {
    // Handle 8-bit sequence number wrap-around
    // Sequence numbers wrap at 255 -> 0
    // We assume packets won't be more than 128 out of order
    
    int diff = newSeq - oldSeq;
    
    // Handle wrap-around
    if (diff < -128) diff += 256;
    if (diff > 128) diff -= 256;
    
    return diff > 0;
}

void SwarmCommunicator::_sendBroadcast(const SwarmState& state) {
    if (!_isInitialized) return;
    
    esp_err_t result = esp_now_send(_broadcastPeerInfo.peer_addr, (uint8_t*)&state, sizeof(state));
    
    if (result != ESP_OK) {
        Serial.printf("⚠️ ESP-NOW broadcast failed: %d\n", result);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// STATIC CALLBACK
// ═══════════════════════════════════════════════════════════════════════════

void SwarmCommunicator::onDataReceived(const uint8_t* mac_addr, const uint8_t* incomingData, int len) {
    // Validate packet size
    if (len != sizeof(SwarmState)) {
        return; // Invalid packet size, ignore
    }
    
    // Get singleton instance
    SwarmCommunicator& instance = getInstance();
    
    // Deserialize the state
    SwarmState state;
    memcpy(&state, incomingData, sizeof(SwarmState));
    
    // Process it
    instance._processIncomingState(mac_addr, state);
}