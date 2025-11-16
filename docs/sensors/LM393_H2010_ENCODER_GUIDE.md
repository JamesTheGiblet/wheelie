# LM393 H2010 Photoelectric Sensor Guide

## Overview

The **LM393 H2010 Photoelectric Sensor** is an opposite-type infrared photoelectric sensor designed for speed detection, counting applications, and motor encoder implementations in robotics projects.

## Specifications

- **Model**: LM393 H2010
- **Type**: Opposite-type photoelectric sensor
- **Comparator**: LM393 dual comparator IC
- **Operating Voltage**: 3.3V - 5V
- **Current Consumption**: < 15mA
- **Detection Range**: 5mm - 10mm (adjustable)
- **Output**: Digital (HIGH/LOW)

## Key Features

### Opposite-Type Detection

- **Infrared transmitter** and **receiver** on opposite sides
- **Object interruption** triggers detection
- **Precise counting** for rotating objects (wheels, encoder discs)
- **Reliable operation** in various lighting conditions

### LM393 Comparator Circuit

- **Built-in signal conditioning** with LM393 dual comparator
- **Adjustable sensitivity** via onboard potentiometer
- **Clean digital output** with hysteresis
- **Noise immunity** for stable operation

### Motor Speed Sensing

- **Wheel encoder application** for robot navigation
- **RPM measurement** for motor speed control
- **Pulse counting** for distance measurement
- **Direction sensing** (with dual sensors)

## Pin Configuration

### Standard Pinout

```txt
VCC  - Power supply (3.3V - 5V)
GND  - Ground
OUT  - Digital output signal
```

### Dual Sensor Setup

For your robot's wheel encoding system (updated for current hardware):

```txt
Left Encoder (LM393 #1):
VCC  → 3.3V (ESP32)
GND  → GND (ESP32)
OUT  → GPIO 33 (ESP32)

Right Encoder (LM393 #2):
VCC  → 3.3V (ESP32)
GND  → GND (ESP32)
OUT  → GPIO 5 (ESP32)
```

*Note: Pin assignments updated to match current hardware. Left encoder: GPIO 33, Right encoder: GPIO 5.*

## Mechanical Installation

### Encoder Disc Setup

If your DollaTek chassis doesn't include encoder discs:

1. **Create encoder discs**:
   - Cut 20-slot discs from cardboard/plastic
   - Diameter: 30-40mm to fit motor shaft
   - Alternating opaque/transparent sections

2. **Mount on motor shafts**:
   - Attach securely to motor output shaft
   - Ensure disc rotates freely
   - Align with sensor gap

### Sensor Mounting

- **Position sensors** to straddle encoder disc
- **Adjust gap** to 5-10mm between transmitter and receiver
- **Secure mounting** to prevent vibration
- **Align precisely** with disc path

## Wiring Diagram

```txt
[Left Motor with Encoder Disc]
     │
  [LM393 #1]
  │ OUT → GPIO 33
  │ VCC → 3.3V
  │ GND → GND
  │
[ESP32 Development Board]
  │
  │ OUT ← GPIO 5
  │ VCC ← 3.3V  
  │ GND ← GND
  [LM393 #2]
     │
[Right Motor with Encoder Disc]
```

## Programming

### Basic Encoder Reading

```cpp
// Pin definitions (updated for current hardware)
const int LEFT_ENCODER_PIN = 33;
const int RIGHT_ENCODER_PIN = 5;

// Variables for counting
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void setup() {
  Serial.begin(115200);
  
  // Configure encoder pins
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  
  // Attach interrupts for accurate counting
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), 
                  leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), 
                  rightEncoderISR, FALLING);
                  
  Serial.println("LM393 H2010 Encoders Initialized (Left: GPIO 33, Right: GPIO 5)");
}

// Interrupt service routines
void leftEncoderISR() {
  leftEncoderCount++;
}

void rightEncoderISR() {
  rightEncoderCount++;
}

void loop() {
  // Display counts every second
  static unsigned long lastDisplay = 0;
  
  if (millis() - lastDisplay > 1000) {
    Serial.print("Left: ");
    Serial.print(leftEncoderCount);
    Serial.print(" | Right: ");
    Serial.println(rightEncoderCount);
    
    lastDisplay = millis();
  }
}
```

### Speed Calculation

```cpp
// Calculate RPM from encoder pulses
float calculateRPM(long pulseCount, unsigned long timeInterval, int pulsesPerRevolution) {
  float rpm = (pulseCount * 60000.0) / (timeInterval * pulsesPerRevolution);
  return rpm;
}

// Distance calculation (for navigation)
float calculateDistance(long pulseCount, float wheelCircumference, int pulsesPerRevolution) {
  float revolutions = pulseCount / (float)pulsesPerRevolution;
  float distance = revolutions * wheelCircumference;
  return distance;
}
```

### Integration with Robot Navigation

```cpp
// PID motor control with encoder feedback
class EncoderMotorControl {
private:
  int motorPin1, motorPin2, enablePin;
  int encoderPin;
  volatile long encoderCount;
  long targetCount;
  
public:
  void setTarget(long target) {
    targetCount = target;
    encoderCount = 0;
  }
  
  bool reachedTarget() {
    return abs(encoderCount - targetCount) < 5; // 5 pulse tolerance
  }
  
  void updateMotor() {
    long error = targetCount - encoderCount;
    int speed = constrain(abs(error) * 2, 100, 255); // Simple P controller
    
    if (error > 0) {
      moveForward(speed);
    } else if (error < 0) {
      moveBackward(speed);
    } else {
      stopMotor();
    }
  }
};
```

## Calibration & Setup

### Sensitivity Adjustment

1. **Position encoder disc** in sensor gap
2. **Rotate disc manually** while observing LED indicator
3. **Adjust potentiometer** until LED toggles consistently
4. **Test at different speeds** to ensure reliable detection

### Optimal Settings

| Condition | Potentiometer Setting | Notes |
|-----------|----------------------|-------|
| Bright Environment | Higher sensitivity (CCW) | Compensate for ambient light |
| Dark Environment | Lower sensitivity (CW) | Prevent false triggers |
| Fast Rotation | Medium sensitivity | Balance speed vs accuracy |
| Precision Counting | Higher sensitivity | Detect all pulses |

## Troubleshooting

### Common Issues

#### No Pulse Detection

- **Check power connections**: Verify 3.3V supply
- **Verify sensor alignment**: Ensure disc passes through gap
- **Adjust sensitivity**: Turn potentiometer counterclockwise
- **Test LED indicator**: Should flash during rotation

#### Missed Pulses

- **Increase sensitivity**: Turn potentiometer counterclockwise
- **Check disc quality**: Ensure clean opaque/transparent sections
- **Reduce speed**: Test at slower motor speeds first
- **Verify interrupt setup**: Ensure proper ISR configuration

#### False Pulses

- **Decrease sensitivity**: Turn potentiometer clockwise
- **Shield from light**: Cover sensor from ambient light
- **Check mounting**: Ensure stable sensor position
- **Add debouncing**: Implement software debouncing

#### Inconsistent Counting

- **Secure connections**: Check for loose wires
- **Stable mounting**: Prevent sensor vibration
- **Clean encoder disc**: Remove dust/debris
- **Power supply**: Ensure stable 3.3V supply

## Robot Applications

### Navigation Features

1. **Straight-Line Movement**
   - Compare left/right encoder counts
   - Adjust motor speeds to maintain straight path
   - Implement closed-loop control

2. **Precise Turning**
   - Calculate turn angles from encoder differences
   - Execute precise degree turns
   - Maintain position accuracy

3. **Distance Measurement**
   - Convert encoder pulses to distance traveled
   - Track robot position (odometry)
   - Return to starting position

4. **Speed Control**
   - Monitor actual wheel speed vs commanded speed
   - Implement PID control for consistent movement
   - Compensate for motor differences

### Advanced Applications

```cpp
// Odometry system using both encoders
class RobotOdometry {
private:
  float x, y, heading; // Robot position
  float wheelBase;     // Distance between wheels
  float wheelRadius;   // Wheel radius
  int pulsesPerRev;    // Encoder pulses per wheel revolution
  
public:
  void updatePosition(long leftPulses, long rightPulses, float deltaTime) {
    float leftDistance = pulsesToDistance(leftPulses);
    float rightDistance = pulsesToDistance(rightPulses);
    
    float distance = (leftDistance + rightDistance) / 2.0;
    float deltaHeading = (rightDistance - leftDistance) / wheelBase;
    
    x += distance * cos(heading + deltaHeading / 2.0);
    y += distance * sin(heading + deltaHeading / 2.0);
    heading += deltaHeading;
  }
};
```

## Performance Optimization

### Interrupt Efficiency

- **Keep ISRs short**: Minimal processing in interrupt routines
- **Use volatile variables**: Ensure proper variable access
- **Atomic operations**: Use proper data types for counters
- **Disable interrupts briefly**: When reading counter values

### Noise Reduction

- **Software debouncing**: Ignore rapid successive pulses
- **Hardware filtering**: Add capacitors to reduce electrical noise
- **Proper grounding**: Ensure good ground connections
- **Shielded cables**: Use for longer wire runs

## Specifications Summary

| Parameter | Value | Notes |
|-----------|-------|-------|
| Supply Voltage | 3.3V - 5V | ESP32 compatible |
| Current Draw | < 15mA | Low power operation |
| Detection Gap | 5mm - 10mm | Adjustable spacing |
| Response Time | < 1ms | Fast pulse detection |
| Operating Temp | -10°C to +70°C | Standard range |
| Pulse Frequency | Up to 1kHz | High-speed counting |

## Advantages for Robot Encoders

### Compared to Magnetic Encoders

- **No magnet required** - Simpler mechanical setup
- **Higher resolution** - More pulses per revolution possible
- **Better accuracy** - Optical detection is precise
- **Environmental immunity** - Not affected by magnetic fields

### Compared to Basic Switches

- **Non-contact operation** - No mechanical wear
- **Higher speed capability** - Handles fast rotation
- **Better reliability** - No contact bounce issues
- **Longer lifespan** - Optical components don't wear out

### For Autonomous Navigation

- **Precise movement control** - Accurate distance measurement
- **Closed-loop feedback** - Real-time speed monitoring
- **Path accuracy** - Maintain straight-line movement
- **Position tracking** - Implement odometry systems

---

**Perfect for**: Wheel encoders, motor speed sensing, precise robot navigation, autonomous movement control

**Key Advantage**: The LM393 H2010 provides reliable, high-precision optical encoding for accurate robot navigation and speed control with easy integration into ESP32-based systems.
