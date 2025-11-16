# Adding New Robots to Project Jumbo

## How to Scale Your Robot Swarm by Creating New HAL Implementations

---

## 🎯 Overview

Project Jumbo's architecture separates the **Brain** (navigation intelligence) from the **Body** (hardware). This means you can create unlimited robot types, and they'll all work with the same intelligent navigation system.

**The Process:**

1. Build your physical robot
2. Create a new HAL class (e.g., `GizmoHAL`)
3. Change **one line** in `main.cpp`
4. Done! Your new robot has fluid navigation, swarm intelligence, and all features

---

## 📐 Architecture Recap

```txt
┌─────────────────────────────────────────┐
│  LAYER 2: THE BRAIN (Universal)        │
│  • PotentialFieldNavigator              │
│  • SwarmCommunicator                    │
│  • Never changes for new robots         │
└─────────────────────────────────────────┘
                    ↕
         **HAL INTERFACE** (Contract)
                    ↕
┌─────────────────────────────────────────┐
│  LAYER 1: THE BODY (Robot-Specific)    │
│  • WheelieHAL  ← Differential drive    │
│  • GizmoHAL    ← Mecanum wheels         │
│  • TankBotHAL  ← Tank treads            │
│  • SpiderHAL   ← Hexapod walker         │
└─────────────────────────────────────────┘
```

---

## 🛠️ Step-by-Step: Creating GizmoHAL

### **Example Robot: GISMO**

- **Drive System**: Mecanum wheels (omnidirectional)
- **Sensors**: Ultrasonic array (5 sensors), MPU6050
- **Special Feature**: Can strafe sideways

---

### **Step 1: Create the Header File**

**File**: `include/GizmoHAL.h`

```cpp
#ifndef GISMO_HAL_H
#define GISMO_HAL_H

#include "HAL.h"
#include "motors.h"
#include "indicators.h"
#include "power_manager.h"
#include "calibration.h"

/**
 * @brief GISMO's HAL implementation - Mecanum drive with ultrasonic array
 */
class GizmoHAL : public HAL {
public:
    GizmoHAL();
    virtual ~GizmoHAL() {}

    // --- HAL Interface Implementation ---
    virtual bool init() override;
    virtual void update() override;
    virtual Vector2D getObstacleRepulsion() override;
    virtual RobotPose getPose() override;
    virtual void setVelocity(const Vector2D& velocity) override;
    
    // --- Power & Safety ---
    virtual void setMaxSpeed(float speedRatio) override;
    virtual void setLEDBrightness(int brightness) override;
    virtual void emergencyStop() override;
    
    // --- Utilities ---
    virtual void setStatusLED(const LEDColor& color) override;
    virtual void playTone(int frequency, int duration) override;
    virtual float getBatteryVoltage() override;

private:
    void autoDetectSensors();
    void initializeSensors();
    void updateOdometry();
    CalibrationResult runGizmoCalibration(); // GISMO-specific calibration
    void updateAllSensors();
    
    // GISMO-specific: Ultrasonic array
    void readUltrasonicArray();
    float ultrasonicDistances[5]; // Front, Front-Left, Front-Right, Left, Right
    
    RobotPose currentPose;

    // Internal state for non-blocking ultrasonic array
    enum GizmoUltrasonicState { GIZMO_US_IDLE, GIZMO_US_TRIGGERED, GIZMO_US_ECHO_IN_PROGRESS };
    GizmoUltrasonicState usState = GIZMO_US_IDLE;
    int currentUsSensor = 0;
    unsigned long usTriggerTime = 0;
    unsigned long usEchoStartTime = 0;

    long lastFrontLeftEncoder = 0;
    long lastFrontRightEncoder = 0;
    long lastBackLeftEncoder = 0;
    long lastBackRightEncoder = 0;
    float lastHeading = 0.0;
};

#endif // GISMO_HAL_H
```

---

### **Step 2: Implement the Core Functions**

**File**: `src/GizmoHAL.cpp` (Example Implementation)

```cpp
#include "GizmoHAL.h"
#include <Arduino.h>
#include <Wire.h>
#include "main.h"

// --- External globals ---
extern SystemStatus sysStatus;
extern SensorData sensors; // The new HAL will populate this
extern CalibrationData calibData;
extern bool isCalibrated;

// --- GISMO's ultrasonic pins ---
#define ULTRA_FRONT_TRIG 12
#define ULTRA_FRONT_ECHO 13
#define ULTRA_FL_TRIG 14
#define ULTRA_FL_ECHO 15
#define ULTRA_FR_TRIG 16
#define ULTRA_FR_ECHO 17
#define ULTRA_LEFT_TRIG 18
#define ULTRA_LEFT_ECHO 19
#define ULTRA_RIGHT_TRIG 20
#define ULTRA_RIGHT_ECHO 21

// --- GISMO's mecanum motor pins ---
#define MOTOR_FL_PWM 22
#define MOTOR_FL_DIR 23
#define MOTOR_FR_PWM 24
#define MOTOR_FR_DIR 25
#define MOTOR_BL_PWM 26
#define MOTOR_BL_DIR 27
#define MOTOR_BR_PWM 28
#define MOTOR_BR_DIR 29

GizmoHAL::GizmoHAL() {
    currentPose.position = Vector2D(0, 0);
    currentPose.heading = 0.0;
    for (int i = 0; i < 5; i++) {
        ultrasonicDistances[i] = 0;
    }
}

bool GizmoHAL::init() {
    Serial.begin(115200);
    delay(1000);
    
    setRobotState(ROBOT_BOOTING);
    
    // Initialize subsystems
    setupIndicators(); // From indicators.h
    startupAnimation();
    initializePowerManagement(); // From power_manager.h
    
    // Setup mecanum motors
    pinMode(MOTOR_FL_PWM, OUTPUT);
    pinMode(MOTOR_FL_DIR, OUTPUT);
    pinMode(MOTOR_FR_PWM, OUTPUT);
    pinMode(MOTOR_FR_DIR, OUTPUT);
    pinMode(MOTOR_BL_PWM, OUTPUT);
    pinMode(MOTOR_BL_DIR, OUTPUT);
    pinMode(MOTOR_BR_PWM, OUTPUT);
    pinMode(MOTOR_BR_DIR, OUTPUT);
    
    // Setup ultrasonic sensors
    pinMode(ULTRA_FRONT_TRIG, OUTPUT);
    pinMode(ULTRA_FRONT_ECHO, INPUT);
    pinMode(ULTRA_FL_TRIG, OUTPUT);
    pinMode(ULTRA_FL_ECHO, INPUT);
    pinMode(ULTRA_FR_TRIG, OUTPUT);
    pinMode(ULTRA_FR_ECHO, INPUT);
    pinMode(ULTRA_LEFT_TRIG, OUTPUT);
    pinMode(ULTRA_LEFT_ECHO, INPUT);
    pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
    pinMode(ULTRA_RIGHT_ECHO, INPUT);
    
    initializeSensors();
    
    // --- Calibration ---
    // A new robot can have its own calibration sequence or skip it.
    CalibrationResult loadResult = loadCalibrationData();
    if (shouldForceRecalibration() || loadResult != CALIB_SUCCESS) {
        if (runGizmoCalibration() != CALIB_SUCCESS) {
            setRobotState(ROBOT_ERROR);
            return false; // Init failed
        }
    }
    
    Serial.println("✅ GizmoHAL Initialized.");
    return true;
}

void GizmoHAL::update() {
    updateAllSensors();
    updateOdometry();
    monitorPower();
    indicators_update();
}

void GizmoHAL::updateAllSensors() {
    readUltrasonicArray();
    
    // Read MPU if available
    if (sysStatus.mpuAvailable) {
        // Read MPU data...
        // sensors.headingAngle = mpu.getAngleZ();
    }
}

void GizmoHAL::readUltrasonicArray() {
    // This non-blocking state machine reads one sensor at a time to avoid crosstalk.
    unsigned long currentMicros = micros();
    const unsigned long US_TIMEOUT = 38000;

    // Array of Trig/Echo pins for easy iteration
    const int trigPins[] = {ULTRA_FRONT_TRIG, ULTRA_FL_TRIG, ULTRA_FR_TRIG, ULTRA_LEFT_TRIG, ULTRA_RIGHT_TRIG};
    const int echoPins[] = {ULTRA_FRONT_ECHO, ULTRA_FL_ECHO, ULTRA_FR_ECHO, ULTRA_LEFT_ECHO, ULTRA_RIGHT_ECHO};

    if (usState == GIZMO_US_IDLE) {
        // --- 1. Start a new reading for the current sensor ---
        digitalWrite(trigPins[currentUsSensor], HIGH);
        usTriggerTime = currentMicros;
        usState = GIZMO_US_TRIGGERED;

    } else if (usState == GIZMO_US_TRIGGERED) {
        // --- 2. End the trigger pulse after 10us ---
        if (currentMicros - usTriggerTime >= 10) {
            digitalWrite(trigPins[currentUsSensor], LOW);
            usState = GIZMO_US_ECHO_IN_PROGRESS;
            usEchoStartTime = currentMicros; // Start timeout timer
        }

    } else if (usState == GIZMO_US_ECHO_IN_PROGRESS) {
        // --- 3. Wait for the echo pulse to finish or time out ---
        if (digitalRead(echoPins[currentUsSensor]) == LOW) {
            long duration = currentMicros - usEchoStartTime;
            if (duration > 100 && duration < US_TIMEOUT) {
                ultrasonicDistances[currentUsSensor] = duration * 0.0343 / 2.0;
            }
            // Move to the next sensor
            currentUsSensor = (currentUsSensor + 1) % 5;
            usState = GIZMO_US_IDLE;
        } else if (currentMicros - usEchoStartTime > US_TIMEOUT) {
            // Timeout, record max range and move on
            ultrasonicDistances[currentUsSensor] = 999.0f;
            currentUsSensor = (currentUsSensor + 1) % 5;
            usState = GIZMO_US_IDLE;
        }
    }
}

Vector2D GizmoHAL::getObstacleRepulsion() {
    // GISMO's Translation: 5 ultrasonic sensors → single repulsion vector
    Vector2D totalRepulsion(0, 0);
    
    const float INFLUENCE_RADIUS = 50.0f; // 50cm
    const float REPULSION_STRENGTH = 25.0f;
    
    // Front sensor (0° - straight ahead)
    if (ultrasonicDistances < INFLUENCE_RADIUS && ultrasonicDistances > 2.0f) {
        float strength = REPULSION_STRENGTH * (1.0f - (ultrasonicDistances[0] / INFLUENCE_RADIUS));
        totalRepulsion += Vector2D(-strength, 0); // Push backward
    }
    
    // Front-Left sensor (45°)
    if (ultrasonicDistances < INFLUENCE_RADIUS && ultrasonicDistances > 2.0f) {
        float strength = REPULSION_STRENGTH * (1.0f - (ultrasonicDistances[1] / INFLUENCE_RADIUS));
        float angle = 45.0f * M_PI / 180.0f;
        totalRepulsion += Vector2D(-strength * cos(angle), strength * sin(angle)); // Push away from front-left
    }
    
    // Front-Right sensor (-45°)
    if (ultrasonicDistances < INFLUENCE_RADIUS && ultrasonicDistances > 2.0f) {
        float strength = REPULSION_STRENGTH * (1.0f - (ultrasonicDistances[2] / INFLUENCE_RADIUS));
        float angle = -45.0f * M_PI / 180.0f;
        totalRepulsion += Vector2D(-strength * cos(angle), strength * sin(angle)); // Push away from front-right
    }
    
    // Left sensor (90°)
    if (ultrasonicDistances < INFLUENCE_RADIUS && ultrasonicDistances > 2.0f) {
        float strength = REPULSION_STRENGTH * (1.0f - (ultrasonicDistances[3] / INFLUENCE_RADIUS));
        totalRepulsion += Vector2D(0, -strength); // Push right
    }
    
    // Right sensor (-90°)
    if (ultrasonicDistances < INFLUENCE_RADIUS && ultrasonicDistances > 2.0f) {
        float strength = REPULSION_STRENGTH * (1.0f - (ultrasonicDistances[4] / INFLUENCE_RADIUS));
        totalRepulsion += Vector2D(0, strength); // Push left
    }
    
    return totalRepulsion;
}

RobotPose GizmoHAL::getPose() {
    return currentPose;
}

void GizmoHAL::setVelocity(const Vector2D& velocity) {
    // GISMO's Translation: 2D velocity → Mecanum wheel speeds
    // Mecanum allows true omnidirectional movement!
    
    float vx = velocity.x; // Forward/backward
    float vy = velocity.y; // Strafe left/right
    float vr = 0.0f;        // Rotation (for now, zero)
    
    // Mecanum wheel equations
    float frontLeft  = vx - vy - vr;
    float frontRight = vx + vy + vr;
    float backLeft   = vx + vy - vr;
    float backRight  = vx - vy + vr;
    
    // Normalize if any wheel exceeds max speed
    float maxWheel = max(max(abs(frontLeft), abs(frontRight)), 
                         max(abs(backLeft), abs(backRight)));
    if (maxWheel > 255.0f) {
        frontLeft  = (frontLeft  / maxWheel) * 255.0f;
        frontRight = (frontRight / maxWheel) * 255.0f;
        backLeft   = (backLeft   / maxWheel) * 255.0f;
        backRight  = (backRight  / maxWheel) * 255.0f;
    }
    
    // Set motor speeds and directions
    digitalWrite(MOTOR_FL_DIR, frontLeft >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_FL_PWM, abs((int)frontLeft));
    
    digitalWrite(MOTOR_FR_DIR, frontRight >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_FR_PWM, abs((int)frontRight));
    
    digitalWrite(MOTOR_BL_DIR, backLeft >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_BL_PWM, abs((int)backLeft));
    
    digitalWrite(MOTOR_BR_DIR, backRight >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_BR_PWM, abs((int)backRight));
}

void GizmoHAL::updateOdometry() {
    // Mecanum odometry using 4-wheel encoders
    // This is more complex than differential drive
    // For now, simplified version
    currentPose.heading = sensors.headingAngle;
}

void GizmoHAL::autoDetectSensors() {
    Wire.begin();
    // Scan I2C bus for MPU6050...
}

void GizmoHAL::initializeSensors() {
    autoDetectSensors();
    // Initialize MPU6050 if found...
}

CalibrationResult GizmoHAL::runGizmoCalibration() {
    // Example: Gizmo might only need to calibrate its IMU.
    // It could skip the motor calibration if its mecanum drive is precise enough.
    Serial.println("Running GISMO-specific calibration...");
    // ... call MPU calibration logic ...
    return CALIB_SUCCESS;
}
// --- Utility functions ---
void GizmoHAL::setMaxSpeed(float speedRatio) {
    Serial.printf("GISMO: Max speed set to %.1f%%\n", speedRatio * 100);
}

void GizmoHAL::setLEDBrightness(int brightness) {
    Serial.printf("GISMO: LED brightness set to %d\n", brightness);
}

void GizmoHAL::emergencyStop() {
    analogWrite(MOTOR_FL_PWM, 0);
    analogWrite(MOTOR_FR_PWM, 0);
    analogWrite(MOTOR_BL_PWM, 0);
    analogWrite(MOTOR_BR_PWM, 0);
    setRobotState(ROBOT_ERROR);
    Serial.println("🚨 GISMO: Emergency Stop!");
}

void GizmoHAL::setStatusLED(const LEDColor& color) {
    setLEDColor(color);
}

void GizmoHAL::playTone(int frequency, int duration) {
    ::playTone(frequency, duration);
}

float GizmoHAL::getBatteryVoltage() {
    return getBatteryVoltage(); // From power_manager.h
}
```

---

### **Step 3: Update main.cpp (ONE LINE CHANGE!)**

**File**: `src/main.cpp`

```cpp
#include <Arduino.h>
#include "types.h"
#include "Vector2D.h"
#include "PotentialFieldNavigator.h"
#include "SwarmCommunicator.h"
#include "HAL.h"

// --- BOT-SPECIFIC: Change this line for different robots ---
// #include "WheelieHAL.h"  // ← Wheelie (differential drive)
#include "GizmoHAL.h"       // ← GISMO (mecanum wheels)
// #include "TankBotHAL.h"  // ← Future tank robot

// --- Create the robot body ---
// WheelieHAL hal;  // ← Wheelie
GizmoHAL hal;       // ← GISMO
// TankBotHAL hal;  // ← Tank

// --- Everything else stays EXACTLY the same ---
PotentialFieldNavigator navigator;
SwarmCommunicator swarmComms;

SystemStatus sysStatus;
SensorData sensors;
CalibrationData calibData;
bool isCalibrated = false;

void setup() {
    if (!hal.init()) {
        while (true) { delay(100); }
    }
    
    NavigationParameters params;
    params.attractionConstant = 2.5f;
    params.repulsionConstant = 20.0f;
    params.maxSpeed = 35.0f;
    navigator.setParameters(params);
    navigator.setGoal(Vector2D(1000, 0));
    
    setRobotState(ROBOT_EXPLORING);
    Serial.println("🤖 GISMO Brain Online!");
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdate) / 1000.0f;

    hal.update();

    if (deltaTime >= 0.05f) {
        RobotPose pose = hal.getPose();
        Vector2D obstacleForce = hal.getObstacleRepulsion();
        auto otherPositions = swarmComms.getOtherRobotPositions();
        
        navigator.setPosition(pose.position);
        navigator.update(deltaTime, obstacleForce);
        
        hal.setVelocity(navigator.getVelocity());
        swarmComms.setMyState(pose.position, navigator.getVelocity());
        
        lastUpdate = currentTime;
    }

    swarmComms.update();
}
```

---

## 🎯 That's It

You've just created a **completely different robot** with:

- ✅ Different drive system (mecanum vs differential)
- ✅ Different sensor array (5 ultrasonics vs 1 ToF)
- ✅ Different capabilities (can strafe sideways!)

But the **brain stays the same**. GISMO automatically gets:

- ✅ Fluid navigation
- ✅ Obstacle avoidance
- ✅ Swarm intelligence
- ✅ Local minima escape
- ✅ All the smarts!

---

## 📋 Checklist for New Robots

When creating a new robot HAL:

### **1. Hardware Translation (Required)**

- [ ] `init()` - Initialize all hardware
- [ ] `update()` - Poll sensors and background tasks
- [ ] `getObstacleRepulsion()` - Convert YOUR sensors → repulsion vector
- [ ] `getPose()` - Track position and heading
- [ ] `setVelocity()` - Convert velocity vector → YOUR motor commands

### **2. Utility Functions (Required)**

- [ ] `setStatusLED()` - Visual feedback
- [ ] `playTone()` - Audio feedback
- [ ] `getBatteryVoltage()` - Power monitoring
- [ ] `emergencyStop()` - Safety cutoff

### **3. Power & Speed (Optional but Recommended)**

- [ ] `setMaxSpeed()` - Speed limiting
- [ ] `setLEDBrightness()` - LED control

### **4. Custom Hardware Support**

- [ ] Private helper functions for your specific sensors
- [ ] Odometry calculation for your drive system
- [ ] Calibration routines (if needed)

---

## 🚀 Advanced: Multi-Robot Swarms

Once you have multiple robots:

```cpp
// Wheelie: Fast scout
WheelieHAL wheelie;
wheelie.setMaxSpeed(1.0f); // Full speed

// GISMO: Precise manipulator
GizmoHAL gizmo;
gizmo.setMaxSpeed(0.6f); // Slower, more precise

// TankBot: Heavy lifter
TankBotHAL tank;
tank.setMaxSpeed(0.4f); // Slow but powerful
```

They'll **automatically coordinate** via ESP-NOW mesh networking!

---

## 💡 Pro Tips

1. **Start Simple**: Get basic movement working first, then add sensors
2. **Test Incrementally**: Test each HAL function individually
3. **Reuse Code**: Copy from WheelieHAL and modify for your hardware
4. **Document Hardware**: Add pin definitions and sensor specs to comments
5. **Consider Calibration**: Some robots benefit from auto-calibration like Wheelie

---

## 🎓 Examples of HAL Implementations

### **Easy Difficulty**

- **DifferentialBot** - Two wheels + caster (like Wheelie)
- **TricycleBot** - Front steering wheel + two drive wheels

### **Medium Difficulty**

- **MecanumBot** - Four mecanum wheels (like GISMO)
- **AckermannBot** - Car-like steering

### **Hard Difficulty**

- **HexapodBot** - Six-legged walker
- **DroneHAL** - Quadcopter (3D navigation!)
- **HybridBot** - Wheels + legs

---

## 🏁 Conclusion

The HAL architecture means:

- **Brain development** happens once (Layer 2)
- **Body development** is parallel (Layer 1)
- **Each robot** is independent but compatible
- **Swarms emerge** automatically when robots meet

**You're not building robots anymore. You're building a robot ecosystem.**

Go forth and multiply your fleet! 🤖🤖🤖
