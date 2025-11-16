# MPU6050 GY-521 IMU Module Guide

## Overview

The **MPU6050 GY-521** is a 6-axis Inertial Measurement Unit (IMU) that combines a 3-axis accelerometer and 3-axis gyroscope on a single chip. This module provides comprehensive motion sensing capabilities for your autonomous robot.

## Specifications

- **Model**: MPU6050 GY-521 6-Axis IMU Module
- **Accelerometer**: 3-axis, ±2g/±4g/±8g/±16g selectable range
- **Gyroscope**: 3-axis, ±250°/s to ±2000°/s selectable range
- **ADC Resolution**: 16-bit for high precision measurements
- **Interface**: I2C (TWI) compatible
- **Operating Voltage**: 3.3V - 5V
- **Current Consumption**: 3.9mA (normal mode)
- **Dimensions**: 20mm x 16mm PCB module

## Key Features

### 6 Degrees of Freedom (6 DOF)

- **3-axis Accelerometer**: Measures linear acceleration (X, Y, Z axes)
- **3-axis Gyroscope**: Measures angular velocity (pitch, roll, yaw)
- **Combined Data**: Enables comprehensive motion and orientation tracking
- **Sensor Fusion**: Calculate precise tilt angles and movement patterns

### High Precision Sensing

- **16-bit ADC**: Provides 65,536 discrete measurement levels
- **Programmable Ranges**: Adjustable sensitivity for different applications
- **Built-in Filtering**: Reduces noise and improves measurement stability
- **Temperature Compensation**: Maintains accuracy across temperature ranges

### I2C Communication

- **Shared Bus Capability**: Works with VL53L0X and other I2C sensors
- **Standard Address**: 0x68 (default) or 0x69 (with AD0 pulled high)
- **Fast Mode**: Supports up to 400kHz I2C clock speed
- **Low Overhead**: Simple register-based communication protocol

## Pin Configuration

### GY-521 Module Pinout

```txt
VCC  - Power supply (3.3V or 5V)
GND  - Ground
SCL  - I2C Clock line
SDA  - I2C Data line
XDA  - Auxiliary I2C Data (not used in basic setup)
XCL  - Auxiliary I2C Clock (not used in basic setup)
AD0  - I2C Address select (LOW=0x68, HIGH=0x69)
INT  - Interrupt output (optional)
```

### ESP32 Connection

```txt
MPU6050 GY-521    →    ESP32
VCC               →    3.3V
GND               →    GND
SCL               →    GPIO 27 (I2C Clock)
SDA               →    GPIO 26 (I2C Data)
```

*Note: SDA = GPIO 26, SCL = GPIO 27. These lines are shared with the VL53L0X ToF sensor on the same I2C bus.*

## Wiring & Integration

### I2C Bus Sharing

The MPU6050 shares the I2C bus with the VL53L0X ToF sensor:

```txt
ESP32 I2C Bus:
├── GPIO 26 (SDA) ──┬── VL53L0X SDA
│                   └── MPU6050 SDA
└── GPIO 27 (SCL) ──┬── VL53L0X SCL
          └── MPU6050 SCL

Power Distribution:
3.3V ──┬── VL53L0X VCC
  └── MPU6050 VCC
GND  ──┬── VL53L0X GND
  └── MPU6050 GND
```

*Note: Both sensors must be powered from 3.3V. Ensure solid connections for reliable I2C communication.*

### Physical Mounting

- **Secure attachment**: Mount firmly to robot chassis
- **Vibration isolation**: Use foam padding if motors cause vibration
- **Orientation**: Align X-axis with robot forward direction
- **Level placement**: Module should be as level as possible when robot is on flat surface

## Programming & Code Integration

### Library Setup

The robot code uses the MPU6050 for tilt detection and safety monitoring:

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Wire.begin();
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}
```

### Basic Reading Functions

```cpp
void readMPU6050Data() {
  int16_t ax, ay, az;  // Accelerometer values
  int16_t gx, gy, gz;  // Gyroscope values
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to meaningful units
  float accelX = ax / 16384.0;  // g-force
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  float gyroX = gx / 131.0;     // degrees/second
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;
}
```

### Tilt Detection Implementation

```cpp
bool checkTiltSafety() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Calculate tilt angles
  float pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  float roll = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI;
  
  // Safety threshold (30 degrees)
  if (abs(pitch) > 30 || abs(roll) > 30) {
    Serial.println("TILT DETECTED - Emergency stop!");
    return false;  // Unsafe tilt detected
  }
  
  return true;  // Safe to continue
}
```

## Robot Safety Integration

### Tilt Detection Safety

Your robot uses the MPU6050 for critical safety functions:

1. **Continuous Monitoring**: Checks tilt angles every control loop (pitch/roll from accelerometer)
2. **Emergency Stop**: Stops motors if tilt exceeds 30° (configurable threshold)
3. **Recovery Logic**: Waits for stable orientation before resuming
4. **Status Indication**: RGB LED (KY-009) and buzzer (KY-006) alerts for tilt conditions

### Motion Sensing Applications

- **Movement Detection**: Detects when robot is being moved or handled
- **Collision Detection**: Sudden acceleration changes indicate impacts
- **Orientation Tracking**: Maintains awareness of robot's spatial orientation
- **Navigation Aid**: Helps maintain straight-line movement

## Calibration & Setup

### Initial Calibration

1. **Place robot on level surface**: Ensure chassis is perfectly level
2. **Power on and wait**: Allow 30 seconds for sensor to stabilize
3. **Record offsets**: Note baseline accelerometer and gyroscope values
4. **Adjust in code**: Apply offset corrections for accurate readings

### Calibration Process

```cpp
void calibrateMPU6050() {
  long ax_offset = 0, ay_offset = 0, az_offset = 0;
  long gx_offset = 0, gy_offset = 0, gz_offset = 0;
  
  // Take multiple readings for averaging
  for (int i = 0; i < 1000; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    ax_offset += ax;
    ay_offset += ay;
    az_offset += (az - 16384);  // Subtract 1g from Z-axis
    gx_offset += gx;
    gy_offset += gy;
    gz_offset += gz;
    
    delay(2);
  }
  
  // Calculate average offsets
  ax_offset /= 1000;
  ay_offset /= 1000;
  az_offset /= 1000;
  gx_offset /= 1000;
  gy_offset /= 1000;
  gz_offset /= 1000;
  
  // Apply offsets to MPU6050
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
}
```

## Troubleshooting

### Common Issues

#### No I2C Communication

- **Check wiring**: Verify SDA/SCL connections to GPIO 26/27
- **Power supply**: Ensure 3.3V is stable
- **I2C address**: Verify 0x68 address (default with AD0 low)
- **Pull-up resistors**: May need 4.7kΩ resistors on SDA/SCL lines

#### Erratic Readings

- **Vibration interference**: Isolate sensor from motor vibrations
- **Power noise**: Add decoupling capacitors near sensor
- **Calibration**: Recalibrate sensor on level surface
- **Temperature effects**: Allow warm-up time after power-on

#### Incorrect Tilt Detection

- **Mounting orientation**: Verify sensor axes align with robot
- **Calibration offsets**: Ensure proper zero-point calibration
- **Threshold tuning**: Adjust 30° threshold based on application
- **Sensor fusion**: Combine accelerometer and gyroscope data

#### I2C Bus Conflicts

- **Address conflicts**: Ensure unique addresses for all I2C devices
- **Bus timing**: Check I2C clock speed (400kHz max)
- **Multiple devices**: Verify proper power distribution
- **Shared connections**: Ensure solid connections to all devices

## Performance Optimization

### Sensor Configuration

```cpp
void configureMPU6050() {
  // Set accelerometer range (±2g for most robot applications)
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
  // Set gyroscope range (±250°/s for normal robot movements)
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  // Enable low-pass filter to reduce noise
  mpu.setDLPFMode(MPU6050_DLPF_BW_21);
  
  // Set sample rate to 50Hz for robot control
  mpu.setRate(19);  // Rate = 1000Hz / (1 + 19) = 50Hz
}
```

### Data Processing

- **Filtering**: Apply complementary filter for angle estimation
- **Averaging**: Use moving averages to smooth noisy readings
- **Threshold hysteresis**: Prevent oscillation around threshold values
- **Rate limiting**: Limit how often safety checks trigger actions

## Advanced Features

### Interrupt-Based Operation

```cpp
// Setup interrupt for motion detection
void setupMotionInterrupt() {
  mpu.setMotionDetectionThreshold(2);  // Sensitivity level
  mpu.setMotionDetectionDuration(40);  // Duration threshold
  mpu.setIntMotionEnabled(true);       // Enable motion interrupt
  
  // Connect INT pin to ESP32 interrupt
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), motionISR, RISING);
}

void motionISR() {
  // Handle motion detection interrupt
  motionDetected = true;
}
```

### Sensor Fusion Algorithm

```cpp
float complementaryFilter(float accel_angle, float gyro_rate, float dt) {
  static float filtered_angle = 0;
  float alpha = 0.98;  // Filter coefficient
  
  // Complementary filter combining accelerometer and gyroscope
  filtered_angle = alpha * (filtered_angle + gyro_rate * dt) + 
                   (1 - alpha) * accel_angle;
  
  return filtered_angle;
}
```

## Robot Applications

### Safety Systems

- **Tilt Protection**: Prevents robot from falling or tipping over (emergency stop if unsafe tilt)
- **Collision Detection**: Detects impacts and stops movement
- **Orientation Monitoring**: Ensures robot maintains proper orientation
- **Emergency Response**: Triggers safety protocols when needed (LED/buzzer feedback)

### Navigation Enhancement

- **Dead Reckoning**: Track movement when other sensors unavailable
- **Turn Detection**: Monitor rotation during navigation
- **Level Maintenance**: Keep robot level during movement
- **Stability Control**: Adjust movement based on stability feedback

### Behavioral Features

- **Sleep Detection**: Detect when robot is stationary
- **Handling Detection**: Know when robot is being picked up
- **Surface Detection**: Identify different surface types by vibration
- **Activity Monitoring**: Track robot's daily activity patterns

## Specifications Summary

| Parameter | Value | Notes |
|-----------|-------|-------|
| Supply Voltage | 3.3V - 5V | ESP32 compatible |
| Current Draw | 3.9mA | Low power operation |
| Accelerometer Range | ±2g to ±16g | Selectable full scale |
| Gyroscope Range | ±250°/s to ±2000°/s | Selectable full scale |
| ADC Resolution | 16-bit | High precision |
| I2C Address | 0x68/0x69 | Selectable via AD0 pin |
| Sample Rate | Up to 1kHz | Configurable output rate |
| Operating Temp | -40°C to +85°C | Industrial temperature range |

## Advantages for Robot Control

### Comprehensive Motion Sensing

- **6 DOF**: Complete orientation and motion information
- **High Precision**: 16-bit resolution for accurate measurements
- **Fast Response**: Real-time motion detection and response
- **Robust Design**: Industrial-grade sensor for reliable operation

### Safety Enhancement

- **Tilt Detection**: Prevents dangerous situations
- **Collision Detection**: Immediate response to impacts
- **Orientation Awareness**: Maintains spatial understanding
- **Emergency Stop**: Quick response to safety conditions

### Navigation Support

- **Motion Tracking**: Dead reckoning capabilities
- **Orientation Reference**: Maintains heading information
- **Stability Monitoring**: Ensures safe movement
- **Surface Adaptation**: Responds to different terrain types

---

**Perfect for**: Autonomous robots, drones, self-balancing vehicles, motion-controlled systems

**Key Advantage**: The MPU6050 GY-521 provides comprehensive 6-axis motion sensing with high precision and robust I2C integration, making it essential for safe and intelligent robot navigation and control.
