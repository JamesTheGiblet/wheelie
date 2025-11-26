#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h> // For uint8_t, uint16_t, etc.
#include "Vector2D.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL DATA STRUCTURES & ENUMS
// ═══════════════════════════════════════════════════════════════════════════
// This file is the single source of truth for all custom data types,
// structs, and enums used across the project.
// ═══════════════════════════════════════════════════════════════════════════

// Robot State Enumeration
enum RobotStateEnum {
    ROBOT_BOOTING,
    ROBOT_IDLE,
    ROBOT_TESTING,
    ROBOT_CALIBRATING,
    ROBOT_EXPLORING,
    ROBOT_AVOIDING_OBSTACLE,
    ROBOT_NAVIGATING,
    ROBOT_PLANNING_ROUTE,
    ROBOT_RECOVERING_STUCK,
    ROBOT_SOUND_TRIGGERED,
    ROBOT_MOTION_TRIGGERED,
    ROBOT_SAFETY_STOP_TILT,
    ROBOT_SAFETY_STOP_EDGE,
    ROBOT_SAFE_MODE,
    ROBOT_ERROR
};

// Mission & Role Types
enum MissionType {
    MISSION_NONE,
    MISSION_EXPLORE,
    MISSION_PATROL,
    MISSION_RETURN_TO_BASE,
    MISSION_GOTO_WAYPOINT,
    MISSION_MONITOR_TARGET,
    MISSION_FORMATION
};

enum RobotRole {
    ROLE_NONE,
    ROLE_LEADER,
    ROLE_SCOUT,
    ROLE_WORKER
};

// Navigation Parameters based on role
struct NavigationParameters {
    // General
    float maxSpeed;
    float mass;
    float damping;

    // Potential Field Forces
    float attractionConstant;
    float repulsionConstant;
    float maxAttraction;
    float influenceRadius;

    // State Thresholds
    float goalThreshold;
    float oscillationThreshold;
    float escapeStrength;
};

// System Operation Modes
enum ObstacleAvoidanceMode { AGGRESSIVE_MODE, CONSERVATIVE_MODE, PASSIVE_MODE };
enum NavigationMode { SENSOR_FUSION, ENCODER_ONLY, IMU_ONLY, MANUAL_MODE };
enum CommunicationMode { FULL_COMM, OFFLINE_MODE, ESPNOW_ONLY, WIFI_ONLY };

// ESP-NOW Message Types
enum ESPNowMessageType {
    MSG_HEARTBEAT = 0x01,
    MSG_SENSOR_DATA,
    MSG_COMMAND,
    MSG_STATUS,
    MSG_PAIRING,
    MSG_ACK
};

// ESP-NOW Status Structure
struct ESPNowStatus {
    bool initialized = false;
    uint8_t channel = 0;
    int peerCount = 0;
    uint16_t messagesSent = 0;
    uint16_t messagesReceived = 0;
    uint16_t sendFailures = 0;
    unsigned long lastActivity = 0;
};

// ESP-NOW Peer Information
struct ESPNowPeer {
    uint8_t macAddress[6];
    uint8_t deviceId;
    bool isActive;
    unsigned long lastSeen;
    int rssi;
    uint16_t packetsReceived;
    uint16_t packetsSent;
};

// ESP-NOW Message Structure
struct ESPNowMessage {
    uint8_t type;
    uint8_t deviceId;
    uint32_t timestamp;
    uint16_t sequenceNumber;
    uint8_t data[240];
    uint8_t checksum;
};

// System Status Structure
struct SystemStatus {
    RobotStateEnum currentState = ROBOT_IDLE;
    bool tofAvailable = false;
    bool mpuAvailable = false;
    bool ultrasonicAvailable = false;
    bool pirAvailable = false;
    bool wifiConnected = false;
    bool espnowActive = false;
    int sensorsActive = 0;
    unsigned long uptime = 0;
    char ipAddress[16] = "0.0.0.0";
    ESPNowStatus espnowStatus;
};

// Sensor Data Structure
struct SensorData {
    float frontDistanceCm = 819.0;
    float rearDistanceCm = 400.0;
    float tiltX = 0;
    float tiltY = 0;
    float headingAngle = 0.0;
    float gyroZ = 0;
    float accelZ = 0;
    float temperature = 0;
    bool edgeDetected = false;
    bool soundDetected = false;
    bool motionDetected = false;
    long leftEncoderCount = 0;
    long rightEncoderCount = 0;
};

// Sensor Health Structure
typedef struct {
    bool tofHealthy = true;
    bool mpuHealthy = true;
    bool ultrasonicHealthy = true;
    bool pirHealthy = true;
    bool edgeHealthy = true;
    bool soundHealthy = true;
} SensorHealth_t;

// LED Color Structure
struct LEDColor {
    bool red;
    bool green;
    bool blue;
};

// Predefined LED Colors
namespace LEDColors {
    const LEDColor OFF = {false, false, false};
    const LEDColor RED = {true, false, false};
    const LEDColor GREEN = {false, true, false};
    const LEDColor BLUE = {false, false, true};
    const LEDColor YELLOW = {true, true, false};
    const LEDColor CYAN = {false, true, true};
    const LEDColor MAGENTA = {true, false, true};
    const LEDColor WHITE = {true, true, true};
}

// State of a robot within the swarm
struct SwarmState {
    uint8_t mac[6];
    Vector2D position;
    Vector2D velocity;
    uint32_t timestamp;
    uint16_t robotId;
    uint8_t sequence;
};

// Mission Data Structure
struct Mission {
    MissionType type = MISSION_NONE;
    uint32_t missionId = 0;
    Vector2D targetPosition;
    float patrolRadius = 0.0f;
    unsigned long startTime = 0;
    unsigned long timeoutMs = 0;
    bool isComplete = true;
};

// Task Bidding Structure for auctions
struct TaskBid {
    uint32_t taskId;
    uint16_t robotId;
    float cost;
    unsigned long timestamp;
};

// Constants
#define MAX_SWARM_SIZE 10
#define PEER_TIMEOUT_MS 5000


// --- Global State Management Functions (defined in main.cpp) ---
// These are declared here to be accessible from any file that includes globals.h

void setRobotState(RobotStateEnum newState);
RobotStateEnum getCurrentState();

#endif // GLOBALS_H