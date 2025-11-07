#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "config.h"
#include "sensors.h"
#include "calibration.h"  // For precise movement capabilities

// ═══════════════════════════════════════════════════════════════════════════
// INTELLIGENT NAVIGATION SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

// Navigation states
enum NavigationState {
  NAV_EXPLORING,        // Moving forward exploring
  NAV_OBSTACLE_AVOIDING, // Actively avoiding obstacles
  NAV_ROUTE_PLANNING,   // Deciding best direction
  NAV_STUCK_RECOVERY    // Recovering from stuck situation
};

// Sub-states for non-blocking avoidance maneuvers
enum AvoidanceSubState {
  AVOID_IDLE,
  AVOID_START,
  AVOID_BACKUP_START,
  AVOID_BACKUP_WAIT,
  AVOID_SCAN_LEFT_START,
  AVOID_SCAN_LEFT_WAIT,
  AVOID_SCAN_LEFT_SETTLE,     // <-- ADD THIS
  AVOID_SCAN_RIGHT_START,
  AVOID_SCAN_RIGHT_WAIT,
  AVOID_SCAN_RIGHT_SETTLE,    // <-- ADD THIS
  AVOID_RETURN_TO_CENTER_START, // <-- ADD THIS
  AVOID_RETURN_TO_CENTER_WAIT,  // <-- ADD THIS
  AVOID_DECIDE_DIRECTION,
  AVOID_EXECUTE_TURN_START,
  AVOID_EXECUTE_TURN_WAIT,
  AVOID_EXECUTE_FORWARD_START,
  AVOID_EXECUTE_FORWARD_WAIT
};

// Direction preferences
enum Direction {
  DIR_FORWARD,
  DIR_BACKWARD,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_NONE
};

// Obstacle memory structure
struct ObstacleMemory {
  float x, y;           // Absolute (X, Y) position of the obstacle
  unsigned long timestamp;
  bool isActive;
};

// Navigation data
struct NavigationData {
  NavigationState mode;
  Direction currentDirection;
  Direction preferredDirection;
  
  // Odometry & Sensing
  float headingAngle;           // Current heading (0-360°) from IMU
  float currentX;               // Robot's estimated X position (mm)
  float currentY;               // Robot's estimated Y position (mm)

  // State Timers
  unsigned long lastMoveTime;
  unsigned long stuckStartTime;
  bool isStuck;

  // Obstacle Logic
  int consecutiveObstacles;
  ObstacleMemory obstacles[10]; // Remember last 10 obstacles
  int obstacleCount;

  // Non-blocking state machine variables
  AvoidanceSubState avoidanceState;
  unsigned long stateStartTime;
  float targetAngle;
  long targetTicks;
  int leftScanDist, rightScanDist;
};

extern NavigationData navigation;

// ═══════════════════════════════════════════════════════════════════════════
// NAVIGATION FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Initializes all navigation variables and odometry.
 */
void initializeNavigation();

/**
 * @brief Main update loop, call this in main.cpp loop().
 * Runs the odometry and the navigation state machine.
 */
void navigation_update();

/**
 * @brief [INTERNAL] Updates robot's (X, Y) position.
 * Uses encoder deltas and IMU heading. Called by navigation_update().
 */
void updateOdometry();

/**
 * @brief [INTERNAL] Runs the core navigation state machine.
 * Called by navigation_update().
 */
void runNavigationStateMachine();

// State Machine Actions
void exploreEnvironment();
void avoidObstacleIntelligently();
void handleStuckSituation();
Direction analyzeEscapeRoute();

// Movement & Memory
void moveInDirection(Direction dir);
void rememberObstacle(); // Remembers an obstacle in front of the robot
bool isPathClear(Direction dir);
void printNavigationStatus();

#endif