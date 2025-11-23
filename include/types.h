#ifndef TYPES_H
#define TYPES_H

// This file contains fundamental data structures and enumerations used
// throughout the Wheelie project. It should not include other project headers
// to avoid circular dependencies.

#include <Arduino.h> // For types like uint8_t, bool, etc.

/**
 * @brief Defines the main operational states of the robot.
 */
enum RobotStateEnum {
  ROBOT_BOOTING,
  ROBOT_IDLE,
  ROBOT_CALIBRATING,
  ROBOT_EXPLORING,
  ROBOT_AVOIDING_OBSTACLE,
  ROBOT_PLANNING_ROUTE,
  ROBOT_RECOVERING_STUCK,
  ROBOT_TESTING,
  ROBOT_SAFE_MODE,
  ROBOT_ERROR,
  ROBOT_SAFETY_STOP_EDGE,
  ROBOT_SAFETY_STOP_TILT
};

/**
 * @brief Represents an RGB color for the status LED.
 */
struct LEDColor {
  bool red;
  bool green;
  bool blue;
};

/**
 * @brief Holds the current status of the entire system.
 */
typedef struct {
  unsigned long uptime;
  bool mpuAvailable;
  bool tofAvailable;
  bool ultrasonicAvailable;
  uint8_t sensorsActive;
} SystemStatus_t;

/**
 * @brief Holds the latest readings from all sensors.
 */
typedef struct {
  float tiltX;
  float tiltY;
  float headingAngle;
  float gyroZ;
  float frontDistanceCm;
  float rearDistanceCm;
  long leftEncoderCount;
  long rightEncoderCount;
  bool soundDetected;
  bool edgeDetected;
  bool motionDetected;
} SensorData_t;

/**
 * @brief Tracks the health status of each sensor.
 */
typedef struct {
    bool mpuHealthy;
    bool tofHealthy;
    bool edgeHealthy;
    bool motorAHealthy;
    bool motorBHealthy;
} SensorHealth_t;


#endif // TYPES_H