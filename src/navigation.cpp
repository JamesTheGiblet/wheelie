
#include "navigation.h"
#include "sensors.h"
#include "motors.h"
#include "calibration.h"
#include "types.h"
#include "PotentialFieldNavigator.h"
#include "SwarmCommunicator.h"

// ═══════════════════════════════════════════════════════════════════════════
// FLUID NAVIGATION: Potential Field + Swarm
// ═══════════════════════════════════════════════════════════════════════════

// --- Global Navigation State ---
extern PotentialFieldNavigator navigator;
extern SwarmCommunicator swarmComms;

extern SensorData sensors;
extern SystemStatus sysStatus;
extern CalibrationData calibData;
extern bool isCalibrated;

// Odometry state
static long lastLeftEncoder = 0;
static long lastRightEncoder = 0;
static float lastHeading = 0.0;
static float currentX = 0.0;
static float currentY = 0.0;
static float currentHeading = 0.0;

void initializeNavigation() {
    lastLeftEncoder = sensors.leftEncoderCount;
    lastRightEncoder = sensors.rightEncoderCount;
    lastHeading = sensors.headingAngle;
    currentHeading = lastHeading;
    currentX = 0.0;
    currentY = 0.0;

    NavigationParameters params;
    params.attractionConstant = 2.5f;
    params.repulsionConstant = 20.0f;
    params.maxSpeed = 35.0f;
    navigator.setParameters(params);
    navigator.setPosition(Vector2D(currentX, currentY));
    navigator.setGoal(Vector2D(1000, 0));

    Serial.println("🤖 RobotForge Fluid Navigation Initialized.");
    Serial.println("   🎯 Initial Goal: (1000, 0)");
}

extern void updateOdometry();

void navigation_update() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdate) / 1000.0f;

    updateOdometry();
    navigator.addSensorReading(sensors.distance, 0.0f);
    auto otherPositions = swarmComms.getOtherRobotPositions();
    navigator.update(deltaTime, otherPositions);
    Vector2D desiredVelocity = navigator.getVelocity();
    setMotorsFromVector(desiredVelocity); // This now correctly calls the Vector2D overload
    swarmComms.setMyState(navigator.getPosition(), navigator.getVelocity());
    swarmComms.update();

    lastUpdate = currentTime;
}
