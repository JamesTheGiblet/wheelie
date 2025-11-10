#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h> // For uint8_t, uint16_t, etc.
#include "Vector2D.h"

// ═══════════════════════════════════════════════════════════════════════════
// DATA TYPES - Structures and enums for robot data management
// ═══════════════════════════════════════════════════════════════════════════

// Robot State Enumeration (must be defined before SystemStatus)
// This is the single source of truth for the robot's state.
enum RobotStateEnum {
    // Boot & Idle States
    ROBOT_BOOTING,              // System is starting (in setup())
    ROBOT_IDLE,                 // Stationary and ready
    ROBOT_TESTING,              // Running self-tests
    ROBOT_CALIBRATING,          // Running the one-time calibration

    // Navigation States (Goals from the "brain")
    ROBOT_EXPLORING,            // Moving, no obstacles
    ROBOT_AVOIDING_OBSTACLE,    // Actively avoiding (running the avoidance state machine)
    ROBOT_PLANNING_ROUTE,       // Paused to decide on a new path
    ROBOT_RECOVERING_STUCK,     // Attempting to free itself

    // Interactive States
    ROBOT_SOUND_TRIGGERED,      // Responding to sound
    ROBOT_MOTION_TRIGGERED,     // Responding to motion
    
    // Safety & Error States (Highest priority)
    ROBOT_SAFETY_STOP_TILT,     // Halted due to unsafe tilt
    ROBOT_SAFETY_STOP_EDGE,     // Halted due to edge detection
    ROBOT_SAFE_MODE,            // Limited function due to non-critical error
    ROBOT_ERROR                 // Halted due to critical system error
};

// System Operation Modes
enum ObstacleAvoidanceMode {
  AGGRESSIVE_MODE = 0,
  CONSERVATIVE_MODE = 1,
  PASSIVE_MODE = 2
};

enum NavigationMode {
  SENSOR_FUSION = 0,
  ENCODER_ONLY = 1, 
  IMU_ONLY = 2,
  MANUAL_MODE = 3
};

enum CommunicationMode {
  FULL_COMM = 0,
  OFFLINE_MODE = 1,
  ESPNOW_ONLY = 2,
  WIFI_ONLY = 3
};

// ESP-NOW Message Types
enum ESPNowMessageType {
  MSG_HEARTBEAT = 0x01,             // Heartbeat/keepalive message
  MSG_SENSOR_DATA = 0x02,           // Sensor data broadcast
  MSG_COMMAND = 0x03,               // Remote command
  MSG_STATUS = 0x04,                // Robot status update
  MSG_PAIRING = 0x05,               // Peer pairing request
  MSG_ACK = 0x06                    // Acknowledgment message
};

// ESP-NOW Status Structure (defined before SystemStatus)
struct ESPNowStatus {
  bool initialized = false;         // ESP-NOW initialization status
  uint8_t channel = 0;              // Current ESP-NOW channel
  int peerCount = 0;                // Number of active peers
  uint16_t messagesSent = 0;        // Total messages sent
  uint16_t messagesReceived = 0;    // Total messages received
  uint16_t sendFailures = 0;        // Send failure count
  unsigned long lastActivity = 0;   // Last communication activity
};

// ESP-NOW Peer Information
struct ESPNowPeer {
  uint8_t macAddress[6];            // Peer MAC address
  uint8_t deviceId;                 // Peer device ID
  bool isActive;                    // Peer activity status
  unsigned long lastSeen;           // Last communication timestamp
  int rssi;                         // Signal strength
  uint16_t packetsReceived;         // Received packet count
  uint16_t packetsSent;             // Sent packet count
};

// ESP-NOW Message Structure (max 250 bytes)
struct ESPNowMessage {
  uint8_t type;                     // Message type (ESPNowMessageType)
  uint8_t deviceId;                 // Sender device ID
  uint32_t timestamp;               // Message timestamp
  uint16_t sequenceNumber;          // Message sequence number
  uint8_t data[240];                // Message payload
  uint8_t checksum;                 // Simple checksum for integrity
};

// System Status Structure (uses ESPNowStatus defined above)
struct SystemStatus {
  RobotStateEnum currentState = ROBOT_IDLE; // Current robot state
  bool tofAvailable = false;        // VL53L0X Time-of-Flight sensor status
  bool mpuAvailable = false;        // MPU6050 IMU sensor status
  bool pirAvailable = false;        // PIR motion sensor status (currently disabled)
  bool wifiConnected = false;       // WiFi connection status
  bool espnowActive = false;        // ESP-NOW communication status
  int sensorsActive = 0;            // Count of active sensors
  unsigned long uptime = 0;         // System uptime in milliseconds
  char ipAddress[16] = "0.0.0.0";   // Robot's IP address when connected (safe char array)
  ESPNowStatus espnowStatus;        // ESP-NOW communication status
};

// Sensor Data Structure
struct SensorData {
  int distance = 2000;              // Distance reading from ToF sensor (mm)
  float tiltX = 0;                  // Tilt angle X-axis (degrees)
  float tiltY = 0;                  // Tilt angle Y-axis (degrees)
  float headingAngle = 0.0;         // Absolute heading from IMU (degrees)
  bool edgeDetected = false;        // Edge sensor state
  bool soundDetected = false;       // Sound sensor state
  bool motionDetected = false;      // Motion sensor state (PIR)
  long leftEncoderCount = 0;        // Left wheel encoder count (real-time)
  long rightEncoderCount = 0;       // Right wheel encoder count (real-time)
};

// Sensor Health Structure
typedef struct {
  bool tofHealthy = true;      // ToF sensor health
  bool mpuHealthy = true;      // IMU sensor health
  bool pirHealthy = true;      // PIR sensor health
  bool edgeHealthy = true;     // Edge sensor health
  bool soundHealthy = true;    // Sound sensor health
  // Add more fields as needed for other sensors
} SensorHealth_t;

// Extern global sensor health status
extern SensorHealth_t sensorHealth;

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

// Swarm Robot State for ESP-NOW communication
struct RobotState {
    uint8_t mac[6];
    Vector2D position;
    Vector2D velocity;
    uint32_t timestamp;
    uint16_t robotId;
    uint8_t sequence;
};

#endif // TYPES_H

// Global system status instance (defined in robot.cpp)
extern SystemStatus sysStatus;