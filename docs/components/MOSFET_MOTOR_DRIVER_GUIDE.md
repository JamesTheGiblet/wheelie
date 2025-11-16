# 🚗 MOS-FET Motor Driver Guide

## Overview

Your Wheelie robot uses an **advanced MOS-FET based dual H-bridge motor driver** that's significantly more efficient than traditional L298N drivers. This compact module provides superior performance with built-in thermal protection.

## 🔧 **Motor Driver Specifications**

### 📊 **Technical Specifications**

- **Module Type**: Dual H-Bridge with built-in MOS-FET switches
- **Input Voltage**: 2V - 10V DC
- **Operating Current**: 1.5A per channel (continuous)
- **Peak Current**: 2.5A per channel
- **Standby Current**: < 0.1µA (ultra-low power)
- **Size**: 24.7 × 21 × 5mm (ultra-compact)
- **Weight**: 5g (lightweight)
- **Mounting Holes**: 2mm diameter

### 🏆 **Advantages Over L298N**

- ✅ **Higher Efficiency** - MOS-FET vs bipolar transistors
- ✅ **Lower Heat Generation** - No heat sink required
- ✅ **Smaller Size** - 75% smaller than L298N modules
- ✅ **Lower Power Consumption** - Near-zero standby current
- ✅ **Built-in Protection** - Thermal protection with hysteresis
- ✅ **Better Performance** - Higher current capacity in smaller package

## 🔌 **Pin Configuration**

### ESP32 Connections

```txt
ESP32 Pin → Motor Driver → Function
GPIO 23   → IN1         → Left motor control 1 (PWM capable)
GPIO 22   → IN2         → Left motor control 2 (PWM capable)  
GPIO 19   → IN3         → Right motor control 1 (PWM capable)
GPIO 18   → IN4         → Right motor control 2 (PWM capable)
VIN       → VCC         → Power input (7.4V from battery)
GND       → GND         → Common ground
```

### Motor Connections

```txt
Driver Output → Motor Wire
OUT1         → Left motor terminal 1
OUT2         → Left motor terminal 2
OUT3         → Right motor terminal 1
OUT4         → Right motor terminal 2
```

## 🎛️ **Control Method**

### PWM Speed Control

Unlike L298N which uses separate enable pins, this driver controls speed directly through PWM on the IN pins:

**Left Motor Control:**

- **Forward**: IN1=PWM (0-255), IN2=0
- **Reverse**: IN1=0, IN2=PWM (0-255)
- **Stop**: IN1=0, IN2=0
- **Brake**: IN1=255, IN2=255

**Right Motor Control:**

- **Forward**: IN3=PWM (0-255), IN4=0
- **Reverse**: IN3=0, IN4=PWM (0-255)
- **Stop**: IN3=0, IN4=0
- **Brake**: IN3=255, IN4=255

### Code Implementation

```cpp
// Move forward at speed 200
setMotorDirection(200, 200);

// Turn left (left motor reverse, right motor forward)
setMotorDirection(-150, 150);

// Move backward
setMotorDirection(-180, -180);

// Emergency brake
stopWithBrake();
```

## 🛡️ **Safety Features**

### Built-in Protection

- **Thermal Protection**: Automatic shutdown when overheating
- **Overcurrent Protection**: Limits current to safe levels
- **Short Circuit Protection**: Protects against output shorts
- **Reverse Polarity Protection**: Prevents damage from wrong connections
- **Hysteresis Control**: Prevents oscillation during protection events

### Software Safety

- **Speed Limiting**: PWM values constrained to 0-255 range
- **Direction Validation**: Prevents invalid control combinations
- **Emergency Stop**: Immediate brake function available
- **Timeout Protection**: Automatic stop if communication lost

## ⚙️ **Operating Characteristics**

### Performance Metrics

- **Response Time**: < 1ms for direction changes
- **PWM Frequency**: 5kHz for smooth operation
- **Efficiency**: > 90% (vs ~70% for L298N)
- **Heat Generation**: Minimal (no heat sink required)
- **Noise Level**: Low electromagnetic interference

### Power Consumption

```txt
Operating Mode     | Current Draw
-------------------|-------------
Both Motors Stop   | < 0.1µA
Single Motor 50%   | ~750mA
Both Motors 100%   | ~3000mA
Standby Mode       | < 0.1µA
```

## 🔧 **Configuration Options**

### PWM Settings (config.h)

```cpp
const int PWM_FREQ = 5000;          // 5kHz PWM frequency
const int PWM_RESOLUTION = 8;       // 8-bit resolution (0-255)
const int TEST_SPEED = 200;         // Default test speed
const int MAX_SPEED = 255;          // Maximum speed
```

### Motor Calibration

```cpp
// Left motor may run slightly faster/slower than right
// Adjust in code if needed:
int leftMotorSpeed = speed * 0.95;   // 5% reduction if needed
int rightMotorSpeed = speed;
```

## 🛠️ **Troubleshooting**

### Common Issues

#### **Motors not moving**

- Check power supply (7.4V battery charged?)
- Verify motor connections to OUT1-OUT4
- Confirm ESP32 pin connections (23, 22, 19, 18)
- Check PWM signal generation in code

#### **Motors running but no speed control**

- Verify PWM is enabled on correct pins
- Check PWM frequency setting (5kHz recommended)
- Ensure PWM resolution is 8-bit (0-255)

#### **Motors running in wrong direction**

- Swap motor wires (OUT1↔OUT2 or OUT3↔OUT4)
- Or adjust code: change `setMotorDirection(speed, speed)` to `setMotorDirection(-speed, -speed)`

#### **Thermal protection activating**

- Check for mechanical binding in wheels/gears
- Verify voltage is within 2V-10V range
- Ensure adequate ventilation around driver
- Check motor current draw is < 2.5A peak

### Diagnostic Code

```cpp
// Test individual motors
testMotorA();  // Test left motor forward/reverse
testMotorB();  // Test right motor forward/reverse

// Check PWM output
Serial.println("PWM Channel 0: " + String(ledcRead(0)));
Serial.println("PWM Channel 1: " + String(ledcRead(1)));
```

## 📊 **Performance Comparison**

| Feature | MOS-FET Driver | L298N | Advantage |
|---------|----------------|-------|-----------|
| Efficiency | >90% | ~70% | +20% |
| Size | 24×21×5mm | 43×43×26mm | 75% smaller |
| Heat Generation | Minimal | High | No heat sink needed |
| Current Capacity | 2.5A peak | 2A peak | +25% |
| Standby Current | <0.1µA | ~6mA | 60,000× lower |
| Cost | Low | Low | Similar |
| Complexity | Simple | Complex | Easier wiring |

## 🔮 **Advanced Features**

### Encoder Support

Your robot includes encoder pins for closed-loop control:

```cpp
#define ENCODER_LEFT_PIN 33    // Left encoder (LM393)
#define ENCODER_RIGHT_PIN 5    // Right encoder (LM393)
```

*Note: Pin assignments updated to match current hardware and documentation. Left encoder: GPIO 33, Right encoder: GPIO 5.*

### Speed Profiling

```cpp
// Gradual acceleration
for(int speed = 0; speed <= 200; speed += 10) {
  setMotorDirection(speed, speed);
  delay(100);
}
```

### Precision Control

```cpp
// Fine speed control for precise movements
setMotorDirection(50, 55);   // Slight curve correction
delay(1000);
setMotorDirection(200, 200); // Full speed ahead
```

---

**Your MOS-FET motor driver provides professional-grade motor control with maximum efficiency and reliability!** 🚗💨

*Optimized for battery-powered autonomous robots with superior performance over traditional L298N drivers.*
