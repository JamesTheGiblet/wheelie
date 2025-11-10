#include "navigation.h"
#include "sensors.h"        // For reading global sensor struct
#include "motors.h"         // For calling setMotorsFromVector
#include "calibration.h"    // For odometry calibration data (calibData)
#include "types.h"          // For global SensorData and SystemStatus
#include "config.h"         // For navigation constants
#include "PotentialFieldNavigator.h" // For PotentialFieldNavigator class
#include "SwarmCommunicator.h"       // For SwarmCommunicator class

// ═══════════════════════════════════════════════════════════════════════════
// ROBOTFORGE FLUID NAVIGATION - IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

// --- Global Navigator Instances ---
PotentialFieldNavigator navigator;
SwarmCommunicator swarmComms;

// --- External Global Data ---
extern SensorData sensors;
extern SystemStatus sysStatus;
extern CalibrationData calibData;
extern bool isCalibrated;

// --- Odometry Tracking Variables ---
static long lastLeftEncoder = 0;
static long lastRightEncoder = 0;
static float lastHeading = 0.0;
static float currentX = 0.0;
static float currentY = 0.0;
static float currentHeading = 0.0;

void initializeNavigation() {
    // 1. Initialize Odometry Baseline
    lastLeftEncoder = sensors.leftEncoderCount;
    lastRightEncoder = sensors.rightEncoderCount;
    lastHeading = sensors.headingAngle;
    currentHeading = lastHeading;
    currentX = 0.0;
    currentY = 0.0;

    // 2. Set Navigator Parameters
    NavigationParameters params;
    params.attractionConstant = 2.5f;
    params.repulsionConstant = 20.0f;
    params.maxSpeed = 35.0f; // Max speed in cm/s
    // ... (tune other parameters from PotentialFieldNavigator.h as needed)
    navigator.setParameters(params);

    // 3. Set Initial State
    navigator.setPosition(Vector2D(currentX, currentY));
    navigator.setGoal(Vector2D(1000, 0)); // Set initial goal 1m (100cm) forward

    // 4. swarmComms initializes itself via its constructor

    Serial.println("🤖 RobotForge Fluid Navigation Initialized.");
    Serial.println("   🎯 Initial Goal: (1000, 0)");
}

void updateOdometry() {
    // This function is preserved from your old file, as it works well.
    // It now feeds its result directly to the new navigator.

    // Read current sensor values
    long currentLeftEncoder = sensors.leftEncoderCount;
    long currentRightEncoder = sensors.rightEncoderCount;
    currentHeading = sensors.headingAngle; // Updated from sensors.cpp
    
    // Calculate encoder deltas
    long deltaLeft = currentLeftEncoder - lastLeftEncoder;
    long deltaRight = currentRightEncoder - lastRightEncoder;
    
    // Calculate distance moved (average of both wheels)
    // Convert ticks to millimeters (using 1.0 as a safe default)
    float ticksPerMm = (isCalibrated && calibData.ticksPerMillimeter > 0) ? calibData.ticksPerMillimeter : 1.0;
    float deltaDistance = ((float)(deltaLeft + deltaRight) / 2.0) / ticksPerMm;
    
    // Use average heading for this tick's movement calculation
    float avgHeading = (currentHeading + lastHeading) / 2.0;
    // Handle heading wrap-around for the average
    if (abs(currentHeading - lastHeading) > 180.0) {
        avgHeading += 180.0;
    }
    float avgHeadingRad = avgHeading * M_PI / 180.0;

    // Calculate delta X and Y
    float deltaX = deltaDistance * cos(avgHeadingRad);
    float deltaY = deltaDistance * sin(avgHeadingRad);
    
    // Update absolute position
    currentX += deltaX;
    currentY += deltaY;
    
    // --- CRITICAL: Update the Navigator ---
    // Feed the new position into the potential field "brain"
    navigator.setPosition(Vector2D(currentX, currentY));
    
    // Store current values for next iteration
    lastLeftEncoder = currentLeftEncoder;
    lastRightEncoder = currentRightEncoder;
    lastHeading = currentHeading;
}

void navigation_update(float deltaTime) {
    // 1. Update our position belief from encoders/IMU
    updateOdometry();
    
    // 2. Feed sensor data into the navigator
    // (Wheelie has one forward ToF sensor, angle = 0.0 radians)
    navigator.addSensorReading(sensors.distance, 0.0f);
    
    // 3. Get swarm data
    auto otherPositions = swarmComms.getOtherRobotPositions();
    
    // 4. Run the core "brain" calculation
    navigator.update(deltaTime, otherPositions);
    
    // 5. Get the resulting velocity vector from the "brain"
    Vector2D desiredVelocity = navigator.getVelocity();
    
    // 6. Translate the vector into motor commands
    setMotorsFromVector(desiredVelocity);
    
    // 7. Broadcast our new state to the swarm
    swarmComms.setMyState(navigator.getPosition(), navigator.getVelocity());
    swarmComms.update(); // Handle background send/receive
}

void setMotorsFromVector(Vector2D velocity) {
    // This function translates the navigator's 2D velocity vector
    // into the magnitude and angle that the motors.cpp driver understands.

    // 1. Get the speed (magnitude)
    // Note: The navigator's `maxSpeed` is in cm/s. We need to map this
    // to a PWM value (0-255). This is a simple 1:1 for now.
    float magnitude = velocity.magnitude();
    
    // 2. Get the angle (in radians) and convert to degrees
    float angleRad = velocity.angle();
    float angleDeg = angleRad * 180.0 / M_PI;

    // 3. Call the motor driver function (from motors.cpp)
    // This function handles all the differential drive mixing.
    setMotorsFromVector(magnitude, angleDeg);
}