# H-1-0332 Sound Sensor Module Guide

## Overview

The **H-1-0332** is a high-sensitivity sound detection module designed for audio-reactive projects and robotics applications. It provides reliable digital output for sound detection with adjustable sensitivity.

## Specifications

- **Model**: H-1-0332
- **Type**: Sound sensor with digital output
- **Operating Voltage**: 3.3V - 5V
- **Current Consumption**: < 10mA
- **Detection Range**: Adjustable via onboard potentiometer
- **Output**: Digital (HIGH/LOW)

## Key Features

### High-Sensitivity Detection

- **Built-in microphone** with amplification circuit
- **Adjustable sensitivity** via onboard potentiometer
- **Fast response time** for real-time applications
- **Stable operation** in various environmental conditions

### Digital Output

- **Clean digital signal** (3.3V/5V logic compatible)
- **No analog conversion needed** - direct digital read
- **Debounced output** for reliable detection
- **LED indicator** shows detection status

## Pin Configuration

### Standard Pinout

```txt
VCC  - Power supply (3.3V - 5V)
GND  - Ground
OUT  - Digital output signal
```

### Connection to ESP32

```txt
H-1-0332 Pin    →    ESP32 Pin
VCC             →    3.3V
GND             →    GND  
OUT             →    GPIO 17 (Digital Input)
```

## Wiring Diagram

```txt
[H-1-0332 Sound Sensor]
     VCC  ──────────── 3.3V (ESP32)
     GND  ──────────── GND (ESP32)
     OUT  ──────────── GPIO 17 (ESP32)
```

## Programming

### Basic Code Example

```cpp
// Pin definition
const int SOUND_SENSOR_PIN = 17;

void setup() {
  Serial.begin(115200);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  Serial.println("H-1-0332 Sound Sensor Initialized");
}

void loop() {
  int soundDetected = digitalRead(SOUND_SENSOR_PIN);
  
  if (soundDetected == HIGH) {
    Serial.println("Sound detected!");
    // Add your sound-reactive behavior here
    delay(100); // Debounce delay
  }
  
  delay(50);
}
```

### Integration with Robot Code

```cpp
// In your main robot code
bool checkAndHandleSound() {
  bool soundDetected = digitalRead(17); // GPIO 17
  
  if (soundDetected) {
    Serial.println("Sound triggered - activating response");
    
    // Example responses:
    setStatusLED(255, 255, 0); // Yellow for sound detection
    playTone(1000, 200);       // Audio feedback
    
    // Stop and look around
    stopMotors();
    delay(500);
    
    return true;
  }
  return false;
}
```

## Calibration & Setup

### Sensitivity Adjustment

1. **Locate the potentiometer** on the H-1-0332 module
2. **Test with ambient noise**:
   - Turn counterclockwise for higher sensitivity
   - Turn clockwise for lower sensitivity
3. **Monitor the onboard LED**:
   - Should light up when sound is detected
   - Should stay off during quiet periods
4. **Test with target sounds**:
   - Clapping, whistling, voice commands
   - Adjust until reliable detection achieved

### Optimal Settings

| Environment | Sensitivity Setting | Notes |
|-------------|-------------------|-------|
| Quiet Indoor | High (CCW) | Detect whispers, small sounds |
| Normal Indoor | Medium | Balanced for speech, claps |
| Noisy Environment | Low (CW) | Only loud sounds trigger |
| Outdoor | Very Low | Avoid wind/ambient noise |

## Installation on Robot

### Mounting Position

- **Top of robot chassis** for omnidirectional detection
- **Front-facing** for directional sound response
- **Away from motors** to minimize mechanical noise
- **Secure mounting** to prevent vibration interference

### Cable Management

- **Short wires** to minimize electrical interference
- **Route away** from power lines and motor cables
- **Strain relief** at connection points
- **Label connections** for easy troubleshooting

## Troubleshooting

### Common Issues

#### No Detection

- **Check power**: Verify 3.3V supply
- **Verify connections**: Ensure proper wiring
- **Adjust sensitivity**: Turn potentiometer counterclockwise
- **Test LED**: Should illuminate during detection

#### False Triggers

- **Lower sensitivity**: Turn potentiometer clockwise
- **Check placement**: Move away from vibration sources
- **Shield from EMI**: Route wires away from motors
- **Add delay**: Implement debouncing in code

#### Intermittent Operation

- **Secure connections**: Check for loose wires
- **Stable power**: Ensure adequate power supply
- **Temperature effects**: Allow warm-up time
- **Code timing**: Avoid blocking delays in loop

## Robot Behavior Applications

### Sound-Reactive Modes

1. **Follow Sound**
   - Turn toward loudest sound source
   - Move in direction of detected audio
   - Stop and listen periodically

2. **Sound Alarm**
   - Activate when unexpected sounds detected
   - Flash LEDs and sound buzzer
   - Record timestamp and location

3. **Voice Commands**
   - Detect clap patterns for commands
   - Different responses for different sound levels
   - Combine with timing for command sequences

4. **Environmental Monitoring**
   - Log sound events with timestamps
   - Detect unusual noise patterns
   - Send alerts for loud sounds

### Integration with Other Sensors

```cpp
// Combined sensor response
void autonomousNavigation() {
  // Check all sensors
  bool soundDetected = checkAndHandleSound();
  bool obstacleDetected = checkObstacle();
  bool edgeDetected = checkEdge();
  
  if (soundDetected) {
    // Sound takes priority - investigate source
    turnTowardSound();
  } else if (obstacleDetected) {
    // Normal obstacle avoidance
    avoidObstacle();
  } else {
    // Continue normal navigation
    moveForward();
  }
}
```

## Performance Optimization

### Power Efficiency

- **Sleep mode**: Disable when not needed
- **Interrupt-driven**: Use GPIO interrupt instead of polling
- **Batch processing**: Group sound events
- **Adaptive sensitivity**: Adjust based on environment

### Response Speed

- **Minimal delays**: Keep loop cycle fast
- **Interrupt handling**: Use attachInterrupt() for immediate response
- **Priority handling**: Sound detection before other tasks
- **Efficient code**: Optimize detection algorithms

## Specifications Summary

| Parameter | Value | Notes |
|-----------|-------|-------|
| Supply Voltage | 3.3V - 5V | ESP32 compatible |
| Current Draw | < 10mA | Low power consumption |
| Response Time | < 50ms | Fast detection |
| Sensitivity Range | Adjustable | Via potentiometer |
| Output Logic | 3.3V/5V | TTL compatible |
| Operating Temp | -20°C to +70°C | Standard range |
| Detection Distance | 0.5m - 5m | Depends on sound level |

## Advantages of H-1-0332

### Compared to Basic Microphones

- **Digital output** - No ADC conversion needed
- **Built-in amplification** - Higher sensitivity
- **Stable threshold** - Consistent triggering
- **Onboard processing** - Reduces CPU load

### For Robot Applications

- **Real-time response** - Immediate digital signal
- **Low power consumption** - Suitable for battery operation
- **Robust design** - Handles vibration and EMI
- **Easy integration** - Simple digital I/O

---

**Perfect for**: Sound-reactive robots, voice-activated systems, security applications, interactive projects

**Key Advantage**: The H-1-0332 provides reliable digital sound detection with adjustable sensitivity, making it ideal for autonomous robots that need to respond to audio cues.
