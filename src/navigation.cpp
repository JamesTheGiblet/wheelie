#include "navigation.h"
#include "motors.h"
#include "indicators.h"
#include "robot.h"

// External references to global data
extern SystemStatus sysStatus;
extern SensorData sensors;

// ═══════════════════════════════════════════════════════════════════════════
// INTELLIGENT NAVIGATION IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

NavigationData navigation;

// Odometry tracking variables
static long lastLeftEncoder = 0;
static long lastRightEncoder = 0;
static float lastHeading = 0.0;

void initializeNavigation() {
  navigation.mode = NAV_EXPLORING;
  navigation.currentDirection = DIR_FORWARD;
  navigation.preferredDirection = DIR_FORWARD;
  
  // Initialize odometry
  navigation.headingAngle = 0.0;
  navigation.currentX = 0.0;
  navigation.currentY = 0.0;
  
  // Initialize timing
  navigation.lastMoveTime = millis();
  navigation.stuckStartTime = 0;
  navigation.isStuck = false;
  
  // Initialize obstacle tracking
  navigation.consecutiveObstacles = 0;
  navigation.obstacleCount = 0;
  
  // Initialize non-blocking state machine
  navigation.avoidanceState = AVOID_IDLE;
  navigation.stateStartTime = 0;
  navigation.targetAngle = 0.0;
  navigation.targetTicks = 0;

  // Clear obstacle memory
  for (int i = 0; i < 10; i++) {
    navigation.obstacles[i].isActive = false;
  }
  
  // Initialize odometry baseline
  lastLeftEncoder = getLeftEncoderCount();
  lastRightEncoder = getRightEncoderCount();
  lastHeading = sensors.headingAngle; // Read from sensor struct
  navigation.headingAngle = lastHeading;
  
  Serial.println("🧠 Intelligent Navigation initialized");
  Serial.println("   🎯 Mode: Exploration");
  Serial.println("   📍 Odometry: (0.0, 0.0) at 0.0°");
  Serial.println("   🧭 Ready for smart obstacle avoidance");
}

void updateOdometry() {
  // Read current sensor values
  long currentLeftEncoder = getLeftEncoderCount();
  long currentRightEncoder = getRightEncoderCount();
  // NON-BLOCKING: Read heading from the sensor struct, which is updated in the main loop
  float currentHeading = sensors.headingAngle;
  
  // Calculate encoder deltas
  long deltaLeft = currentLeftEncoder - lastLeftEncoder;
  long deltaRight = currentRightEncoder - lastRightEncoder;
  
  // Calculate distance moved (average of both wheels)
  float deltaDistance = ((float)(deltaLeft + deltaRight) / 2.0) / (isCalibrated ? calibData.ticksPerMillimeter : 10.0);
  
  // Update heading from IMU (single source of truth)
  navigation.headingAngle = currentHeading;
  
  // ACCURACY REFINEMENT: Use average heading for this tick
  float avgHeading = (currentHeading + lastHeading) / 2.0;
  // Handle heading wrap-around for the average
  if (abs(currentHeading - lastHeading) > 180) {
    avgHeading += 180.0;
  }
  float avgHeadingRad = avgHeading * PI / 180.0;
  float deltaX = deltaDistance * cos(avgHeadingRad);
  float deltaY = deltaDistance * sin(avgHeadingRad);
  
  // Update absolute position
  navigation.currentX += deltaX;
  navigation.currentY += deltaY;
  
  // Store current values for next iteration
  lastLeftEncoder = currentLeftEncoder;
  lastRightEncoder = currentRightEncoder;
  lastHeading = currentHeading;
}

void navigation_update() {
  // Step 1: Update robot's position (critical for all navigation)
  updateOdometry();
  
  // Step 2: Run the navigation state machine
  runNavigationStateMachine();
}

void runNavigationStateMachine() {
  // Check if robot has been stuck too long
  if (navigation.isStuck) {
    if (millis() - navigation.stuckStartTime > 10000) { // 10 seconds
      // If we are not already in recovery, start it.
      if (navigation.mode != NAV_STUCK_RECOVERY) {
        Serial.println("🆘 Robot has been stuck for 10s. Initiating recovery maneuver.");
        navigation.mode = NAV_STUCK_RECOVERY;
        navigation.avoidanceState = RECOVER_START; // Use the avoidance state machine for this
      }
      handleStuckSituation();
    }
  }
  
  // Expire old obstacles (older than 30 seconds)
  for (int i = 0; i < 10; i++) {
    if (navigation.obstacles[i].isActive && 
        millis() - navigation.obstacles[i].timestamp > 30000) {
      navigation.obstacles[i].isActive = false;
      Serial.println("🗑️  Expired old obstacle from memory");
    }
  }
  
  // Execute current navigation mode
  switch (navigation.mode) {
    case NAV_EXPLORING:
      exploreEnvironment();
      break;
    case NAV_OBSTACLE_AVOIDING:
      avoidObstacleIntelligently();
      break;
    case NAV_STUCK_RECOVERY:
      handleStuckSituation();
      break;
  }
}

void exploreEnvironment() {
  Serial.print("🔍 Exploring");
  if (sysStatus.tofAvailable) {
    Serial.print(" | 📏");
    Serial.print(sensors.distance);
    Serial.print("mm");
  }
  if (sysStatus.mpuAvailable) {
    Serial.print(" | 📐");
    Serial.print(sensors.tiltX, 0);
    Serial.print("°/");
    Serial.print(sensors.tiltY, 0);
    Serial.print("°");
  }
  Serial.print(" | ⏱️");
  Serial.print(millis()/1000);
  Serial.println("s");
  
  // Move forward cautiously
  if (sysStatus.tofAvailable && sensors.distance < WARNING_DISTANCE && sensors.distance > OBSTACLE_DISTANCE) {
    calibratedMoveForward(SLOW_SPEED);
    navigation.currentDirection = DIR_FORWARD;
  } else if (sysStatus.tofAvailable && sensors.distance <= OBSTACLE_DISTANCE) {
    // Obstacle detected! Switch to avoidance mode
    Serial.println("🚧 Obstacle detected! Analyzing escape route...");
    allStop();
    
    // Remember this obstacle
    rememberObstacle();
    
    navigation.mode = NAV_OBSTACLE_AVOIDING; // Go directly to avoidance state machine
    navigation.avoidanceState = AVOID_START;
    navigation.consecutiveObstacles++;
    
    // Check if we're getting stuck
    if (navigation.consecutiveObstacles >= 3) {
      navigation.isStuck = true;
      navigation.stuckStartTime = millis();
      navigation.mode = NAV_STUCK_RECOVERY;
    }
    
    return;
  } else {
    calibratedMoveForward(TEST_SPEED);
    navigation.currentDirection = DIR_FORWARD;
    navigation.consecutiveObstacles = 0; // Reset if we're moving freely
  }
  
  navigation.lastMoveTime = millis();
}

void avoidObstacleIntelligently() {
  // This is the new non-blocking state machine for avoiding obstacles.
  // It replaces both analyzeEscapeRoute and the old avoidObstacleIntelligently.
  
  unsigned long timeInState = millis() - navigation.stateStartTime;

  switch (navigation.avoidanceState) {
    case AVOID_IDLE:
      // Should not be in this state while avoiding. Go back to exploring.
      navigation.mode = NAV_EXPLORING;
      break;

    case AVOID_START:
      Serial.println("🧭 Analyzing escape routes...");
      navigation.avoidanceState = AVOID_BACKUP_START;
      break;

    case AVOID_BACKUP_START:
      Serial.println("   ⬅️  Backing up for better analysis...");
      resetEncoders();
      calibratedMoveBackward(TEST_SPEED);
      navigation.targetTicks = (long)(50.0 * calibData.ticksPerMillimeter); // Back up 5cm
      navigation.avoidanceState = AVOID_BACKUP_WAIT;
      break;

    case AVOID_BACKUP_WAIT:
      if (getAverageEncoderCount() >= navigation.targetTicks) {
        allStop();
        navigation.avoidanceState = AVOID_SCAN_LEFT_START;
      }
      break;

    case AVOID_SCAN_LEFT_START:
      Serial.println("   🔍 Checking left side...");
      resetEncoders();
      calibratedTurnLeft(TURN_SPEED);
      navigation.targetTicks = calibData.ticksPer90Degrees;
      navigation.avoidanceState = AVOID_SCAN_LEFT_WAIT;
      break;

    case AVOID_SCAN_LEFT_WAIT:
      if (getAverageEncoderCount() >= navigation.targetTicks) {
        allStop();
        // delay(200); // <-- REMOVE
        navigation.stateStartTime = millis(); // Start settle timer
        navigation.avoidanceState = AVOID_SCAN_LEFT_SETTLE; // <-- GO TO NEW STATE
      }
      break;

    case AVOID_SCAN_LEFT_SETTLE: // <-- ADD THIS NEW CASE
      if (millis() - navigation.stateStartTime > 200) { // Wait 200ms
        navigation.leftScanDist = sensors.distance;
        Serial.printf("   📏 Left space: %dmm\n", navigation.leftScanDist);
        navigation.avoidanceState = AVOID_SCAN_RIGHT_START;
      }
      break;

    case AVOID_SCAN_RIGHT_START:
      Serial.println("   🔍 Checking right side...");
      resetEncoders();
      calibratedTurnRight(TURN_SPEED);
      navigation.targetTicks = calibData.ticksPer90Degrees * 2; // Turn 180 deg from left
      navigation.avoidanceState = AVOID_SCAN_RIGHT_WAIT;
      break;

    case AVOID_SCAN_RIGHT_WAIT:
      if (getAverageEncoderCount() >= navigation.targetTicks) {
        allStop();
        // delay(200); // <-- REMOVE
        navigation.stateStartTime = millis(); // Start settle timer
        navigation.avoidanceState = AVOID_SCAN_RIGHT_SETTLE; // <-- GO TO NEW STATE
      }
      break;

    case AVOID_SCAN_RIGHT_SETTLE: // <-- ADD THIS NEW CASE
      if (millis() - navigation.stateStartTime > 200) { // Wait 200ms
        navigation.rightScanDist = sensors.distance;
        Serial.printf("   📏 Right space: %dmm\n", navigation.rightScanDist);
        navigation.avoidanceState = AVOID_RETURN_TO_CENTER_START; // <-- Change this
      }
      break;

    case AVOID_RETURN_TO_CENTER_START: // <-- ADD THIS
      Serial.println("   🔄 Returning to center...");
      resetEncoders();
      calibratedTurnLeft(TURN_SPEED);
      navigation.targetTicks = calibData.ticksPer90Degrees;
      navigation.avoidanceState = AVOID_RETURN_TO_CENTER_WAIT;
      break;

    case AVOID_RETURN_TO_CENTER_WAIT: // <-- ADD THIS
      if (getAverageEncoderCount() >= navigation.targetTicks) {
        allStop();
        navigation.avoidanceState = AVOID_DECIDE_DIRECTION;
      }
      break;
        
    case AVOID_DECIDE_DIRECTION:
      // Decide best direction
      if (navigation.leftScanDist > OBSTACLE_DISTANCE + 100 && navigation.rightScanDist > OBSTACLE_DISTANCE + 100) {
        navigation.preferredDirection = (navigation.leftScanDist > navigation.rightScanDist) ? DIR_LEFT : DIR_RIGHT;
      } else if (navigation.leftScanDist > OBSTACLE_DISTANCE + 100) {
        navigation.preferredDirection = DIR_LEFT;
      } else if (navigation.rightScanDist > OBSTACLE_DISTANCE + 100) {
        navigation.preferredDirection = DIR_RIGHT;
      } else {
        navigation.preferredDirection = DIR_BACKWARD;
      }
      Serial.printf("   ✅ Decision: Go %s\n", (navigation.preferredDirection == DIR_LEFT ? "LEFT" : (navigation.preferredDirection == DIR_RIGHT ? "RIGHT" : "BACKWARD")));
      navigation.avoidanceState = AVOID_EXECUTE_TURN_START;
      break;

    case AVOID_EXECUTE_TURN_START:
      resetEncoders();
      if (navigation.preferredDirection == DIR_LEFT) {
        calibratedTurnLeft(TURN_SPEED);
        navigation.targetTicks = calibData.ticksPer90Degrees;
      } else if (navigation.preferredDirection == DIR_RIGHT) {
        calibratedTurnRight(TURN_SPEED);
        navigation.targetTicks = calibData.ticksPer90Degrees;
      } else { // Backward
        calibratedTurnRight(TURN_SPEED); // Turn 180
        navigation.targetTicks = calibData.ticksPer90Degrees * 2;
      }
      navigation.avoidanceState = AVOID_EXECUTE_TURN_WAIT;
      break;

    case AVOID_EXECUTE_TURN_WAIT:
      if (getAverageEncoderCount() >= navigation.targetTicks) {
        allStop();
        navigation.avoidanceState = AVOID_EXECUTE_FORWARD_START;
      }
      break;

    case AVOID_EXECUTE_FORWARD_START:
      resetEncoders();
      calibratedMoveForward(TEST_SPEED);
      navigation.targetTicks = (long)(400.0 * calibData.ticksPerMillimeter); // Move 40cm
      navigation.avoidanceState = AVOID_EXECUTE_FORWARD_WAIT;
      break;

    case AVOID_EXECUTE_FORWARD_WAIT:
      // Also check for new obstacles while moving
      if (getAverageEncoderCount() >= navigation.targetTicks || (sysStatus.tofAvailable && sensors.distance <= OBSTACLE_DISTANCE)) {
        allStop();
        navigation.avoidanceState = AVOID_FINISH;
      }
      break;

    case AVOID_FINISH:
      Serial.println("   ✅ Obstacle avoidance maneuver complete.");
      navigation.mode = NAV_EXPLORING;
      navigation.avoidanceState = AVOID_IDLE;
      break;
  }
}

void rememberObstacle() {
  // Calculate obstacle position based on robot's current position and heading
  // Assume obstacle is detected at a fixed distance in front of the robot
  float obstacleDistance = 200.0; // mm, typical ToF detection distance
  
  float headingRad = navigation.headingAngle * PI / 180.0;
  float obstacleX = navigation.currentX + obstacleDistance * cos(headingRad);
  float obstacleY = navigation.currentY + obstacleDistance * sin(headingRad);
  
  // Find an empty slot or overwrite the oldest
  int slotIndex = -1;
  unsigned long oldestTime = millis();
  
  for (int i = 0; i < 10; i++) {
    if (!navigation.obstacles[i].isActive) {
      slotIndex = i;
      break;
    } else if (navigation.obstacles[i].timestamp < oldestTime) {
      oldestTime = navigation.obstacles[i].timestamp;
      slotIndex = i;
    }
  }
  
  if (slotIndex >= 0) {
    navigation.obstacles[slotIndex].x = obstacleX;
    navigation.obstacles[slotIndex].y = obstacleY;
    navigation.obstacles[slotIndex].timestamp = millis();
    navigation.obstacles[slotIndex].isActive = true;
    navigation.obstacleCount = min(navigation.obstacleCount + 1, 10);
    
    Serial.print("🧠 Remembered obstacle at current position");
    Serial.print(" - Total: ");
    Serial.println(navigation.obstacleCount);
  }
}

void handleStuckSituation() {
  // This is the new NON-BLOCKING state machine for stuck recovery.
  switch (navigation.avoidanceState) {
    case RECOVER_START:
      Serial.println("🆘 STUCK RECOVERY MODE");
      Serial.println("   🔄 Attempting to break free...");
      playTone(1000, 200); // Non-blocking tone
      navigation.avoidanceState = RECOVER_BACKUP_START;
      break;

    case RECOVER_BACKUP_START:
      Serial.println("   1. Aggressive reverse...");
      resetEncoders();
      calibratedMoveBackward(MAX_SPEED); // Use max speed
      navigation.targetTicks = (long)(200.0 * calibData.ticksPerMillimeter); // Back up 20cm
      navigation.avoidanceState = RECOVER_BACKUP_WAIT;
      break;

    case RECOVER_BACKUP_WAIT:
      if (getAverageEncoderCount() >= navigation.targetTicks) {
        allStop();
        navigation.avoidanceState = RECOVER_TURN_START;
      }
      break;

    case RECOVER_TURN_START:
      Serial.println("   2. Aggressive random turn...");
      resetEncoders();
      if (millis() % 2 == 0) {
        calibratedTurnLeft(MAX_SPEED);
      } else {
        calibratedTurnRight(MAX_SPEED);
      }
      navigation.targetTicks = calibData.ticksPer90Degrees * 1.5; // Turn ~135 degrees
      navigation.avoidanceState = RECOVER_TURN_WAIT;
      break;

    case RECOVER_TURN_WAIT:
      if (getAverageEncoderCount() >= navigation.targetTicks) {
        allStop();
        // Since we are done, reset the state and go back to exploring
        Serial.println("   ✅ Recovery maneuver complete, resuming exploration.");
        navigation.isStuck = false;
        navigation.consecutiveObstacles = 0;
        navigation.mode = NAV_EXPLORING;
        navigation.avoidanceState = AVOID_IDLE;
      }
      break;

    default:
      // If in an unexpected sub-state, reset to exploring
      navigation.mode = NAV_EXPLORING;
      navigation.avoidanceState = AVOID_IDLE;
      break;
  }
}

void printNavigationStatus() {
  Serial.println("🧭 NAVIGATION STATUS");
  Serial.printf("   Position: (%.1f, %.1f) mm\n", navigation.currentX, navigation.currentY);
  Serial.printf("   Heading: %.1f°\n", navigation.headingAngle);
  Serial.print("   Mode: ");
  switch (navigation.mode) {
    case NAV_EXPLORING: Serial.println("Exploring"); break;
    case NAV_OBSTACLE_AVOIDING: Serial.println("Avoiding Obstacle"); break;
    case NAV_ROUTE_PLANNING: Serial.println("Planning Route"); break;
    case NAV_STUCK_RECOVERY: Serial.println("Stuck Recovery"); break;
  }
  Serial.print("   Obstacles remembered: ");
  Serial.println(navigation.obstacleCount);
  Serial.print("   Consecutive obstacles: ");
  Serial.println(navigation.consecutiveObstacles);
  Serial.print("   Is stuck: ");
  Serial.println(navigation.isStuck ? "Yes" : "No");
}
