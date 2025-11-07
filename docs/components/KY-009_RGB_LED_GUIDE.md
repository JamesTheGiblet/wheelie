# KY-009 RGB LED Module Guide

## Overview

The **KY-009 RGB LED Module** is a compact 3-color SMD LED board that provides visual status indication for your robot. This module features built-in current limiting resistors and can produce millions of colors by combining red, green, and blue LEDs.

## Specifications

- **LED Type**: SMD RGB LED (Red, Green, Blue)
- **Operating Voltage**: DC 5V
- **Current Limiting**: Built-in resistors (no external resistors needed)
- **Control Method**: Common anode design
- **Colors**: 16.7 million possible color combinations
- **Size**: Compact module design (~18mm x 15mm)
- **Pins**: 4-pin interface (VCC, R, G, B)
- **LED Type**: High-brightness SMD LEDs

## Pin Configuration

```txt
KY-009 Module Pins:
┌─────────────────┐
│  KY-009 Module  │
│   ┌───────┐     │
│   │ RGB   │     │  Pin Functions:
│   │ LED   │     │  - VCC: Power supply (+5V)
│   └───────┘     │  - R: Red LED control
│ VCC R  G  B     │  - G: Green LED control  
└─────────────────┘  - B: Blue LED control
```

## Connection Diagram

### ESP32 to KY-009 Wiring

```txt
ESP32 Pin  →  KY-009 Pin  →  Function
GPIO 15    →  R           →  Red LED Control
GPIO 2     →  G           →  Green LED Control
GPIO 4     →  B           →  Blue LED Control
5V         →  VCC         →  Power Supply
GND        →  GND         →  Ground Reference
```

### Power Requirements

- **Supply Voltage**: 5V DC (from ESP32 5V pin or XL4015 output)
- **Current Draw**:
  - Red LED: ~20mA (max)
  - Green LED: ~20mA (max)  
  - Blue LED: ~20mA (max)
  - Total: ~60mA (all colors on)

## Code Implementation

### Basic RGB Control Functions

```cpp
// KY-009 RGB LED control pins
#define LED_RED_PIN    15
#define LED_GREEN_PIN  2
#define LED_BLUE_PIN   4

// PWM channels for LED control
#define LED_RED_CHANNEL   0
#define LED_GREEN_CHANNEL 1
#define LED_BLUE_CHANNEL  2

void setupRGBLED() {
    // Configure PWM for RGB LED control
    ledcSetup(LED_RED_CHANNEL, 5000, 8);     // 5kHz, 8-bit resolution
    ledcSetup(LED_GREEN_CHANNEL, 5000, 8);
    ledcSetup(LED_BLUE_CHANNEL, 5000, 8);
    
    // Attach pins to PWM channels
    ledcAttachPin(LED_RED_PIN, LED_RED_CHANNEL);
    ledcAttachPin(LED_GREEN_PIN, LED_GREEN_CHANNEL);
    ledcAttachPin(LED_BLUE_PIN, LED_BLUE_CHANNEL);
    
    // Initialize to off state
    setRGBColor(0, 0, 0);
}

void setRGBColor(int red, int green, int blue) {
    // Set PWM values for each color (0-255)
    ledcWrite(LED_RED_CHANNEL, red);
    ledcWrite(LED_GREEN_CHANNEL, green);
    ledcWrite(LED_BLUE_CHANNEL, blue);
}
```

### Robot Status Indication

```cpp
// Status LED color definitions
void setStatusColor(String status) {
    if (status == "IDLE") {
        setRGBColor(0, 0, 255);      // Blue - Ready/Idle
    } else if (status == "FORWARD") {
        setRGBColor(0, 255, 0);      // Green - Moving forward
    } else if (status == "TURNING") {
        setRGBColor(255, 255, 0);    // Yellow - Turning
    } else if (status == "REVERSING") {
        setRGBColor(255, 165, 0);    // Orange - Backing up
    } else if (status == "OBSTACLE") {
        setRGBColor(255, 0, 0);      // Red - Obstacle detected
    } else if (status == "ERROR") {
        // Red blinking for errors
        for (int i = 0; i < 6; i++) {
            setRGBColor(255, 0, 0);
            delay(200);
            setRGBColor(0, 0, 0);
            delay(200);
        }
    }
}
```

### Advanced Features

```cpp
// Rainbow color cycling for startup/demo
void rainbowCycle(int duration) {
    unsigned long startTime = millis();
    
    while (millis() - startTime < duration) {
        for (int hue = 0; hue < 360; hue += 5) {
            // Convert HSV to RGB
            int rgb[3];
            hsvToRgb(hue, 255, 255, rgb);
            setRGBColor(rgb[0], rgb[1], rgb[2]);
            delay(50);
        }
    }
}

// Breathing effect for idle state
void breathingEffect(int red, int green, int blue) {
    for (int brightness = 0; brightness <= 255; brightness += 5) {
        setRGBColor((red * brightness) / 255, 
                   (green * brightness) / 255, 
                   (blue * brightness) / 255);
        delay(20);
    }
    for (int brightness = 255; brightness >= 0; brightness -= 5) {
        setRGBColor((red * brightness) / 255, 
                   (green * brightness) / 255, 
                   (blue * brightness) / 255);
        delay(20);
    }
}
```

## Installation Guide

### Physical Mounting

1. **Location Selection**: Mount on top of robot chassis for maximum visibility
2. **Orientation**: LED should face upward or toward operator
3. **Secure Mounting**: Use double-sided tape or small screws
4. **Wire Management**: Route wires to avoid moving parts

### Electrical Connection

1. **Power Connection**: Connect VCC to 5V rail (XL4015 output or ESP32 5V pin)
2. **Ground Connection**: Connect to common ground
3. **Control Signals**: Wire R, G, B pins to designated ESP32 GPIO pins
4. **Verify Connections**: Check continuity before powering on

## Testing & Validation

### Basic Functionality Test

```cpp
void testRGBLED() {
    Serial.println("Testing KY-009 RGB LED Module...");
    
    // Test individual colors
    Serial.println("Red");
    setRGBColor(255, 0, 0);
    delay(1000);
    
    Serial.println("Green");
    setRGBColor(0, 255, 0);
    delay(1000);
    
    Serial.println("Blue");
    setRGBColor(0, 0, 255);
    delay(1000);
    
    // Test color mixing
    Serial.println("Yellow (Red + Green)");
    setRGBColor(255, 255, 0);
    delay(1000);
    
    Serial.println("Purple (Red + Blue)");
    setRGBColor(255, 0, 255);
    delay(1000);
    
    Serial.println("Cyan (Green + Blue)");
    setRGBColor(0, 255, 255);
    delay(1000);
    
    // Test white
    Serial.println("White (All colors)");
    setRGBColor(255, 255, 255);
    delay(1000);
    
    // Turn off
    Serial.println("Off");
    setRGBColor(0, 0, 0);
    
    Serial.println("KY-009 test complete!");
}
```

### Expected Results

- **Individual Colors**: Clear red, green, and blue illumination
- **Color Mixing**: Proper yellow, purple, and cyan colors
- **Brightness**: Uniform illumination across all colors
- **Responsiveness**: Immediate color changes with no flicker

## Troubleshooting

### Common Issues

#### No LED Illumination

**Symptoms**: LED remains off
**Causes & Solutions**:

- Check 5V power supply connection
- Verify ESP32 GPIO pin assignments in code
- Confirm PWM channel setup
- Test with multimeter for voltage at VCC pin

#### Incorrect Colors

**Symptoms**: Wrong colors displayed
**Causes & Solutions**:

- Verify pin connections match code definitions
- Check for crossed R/G/B wires
- Confirm common anode configuration
- Test individual color channels

#### Dim or Flickering LED

**Symptoms**: Weak or unstable illumination
**Causes & Solutions**:

- Check power supply capacity (5V rail)
- Verify PWM frequency settings (5kHz recommended)
- Ensure solid connections at all pins
- Check for voltage drop in wiring

#### One Color Missing

**Symptoms**: Only 2 of 3 colors work
**Causes & Solutions**:

- Test continuity on affected color wire
- Check ESP32 GPIO pin functionality
- Verify PWM channel assignment
- Replace module if LED element is damaged

## Integration with Robot Systems

### Status Indication Framework

```cpp
// Robot state management with LED feedback
enum RobotState {
    ROBOT_IDLE,
    ROBOT_MOVING,
    ROBOT_TURNING,
    ROBOT_OBSTACLE,
    ROBOT_ERROR
};

void updateStatusLED(RobotState state) {
    switch (state) {
        case ROBOT_IDLE:
            breathingEffect(0, 0, 255);  // Blue breathing
            break;
        case ROBOT_MOVING:
            setRGBColor(0, 255, 0);      // Solid green
            break;
        case ROBOT_TURNING:
            setRGBColor(255, 255, 0);    // Solid yellow
            break;
        case ROBOT_OBSTACLE:
            setRGBColor(255, 0, 0);      // Solid red
            break;
        case ROBOT_ERROR:
            setStatusColor("ERROR");      // Blinking red
            break;
    }
}
```

### Multi-Sensor Integration

The KY-009 provides visual feedback for multiple sensor inputs:

- **VL53L0X ToF Sensor**: Red for obstacles detected
- **MPU6050 IMU**: Yellow for tilt warnings
- **Sound Sensor**: Purple for sound activation
- **Encoder Feedback**: Green for normal movement
- **Battery Status**: Color intensity indicates charge level

## Specifications Summary

| Feature | Specification |
|---------|---------------|
| **LED Type** | SMD RGB (Red, Green, Blue) |
| **Voltage** | DC 5V |
| **Current** | 60mA total (20mA per color) |
| **Colors** | 16.7 million combinations |
| **Control** | PWM (3 channels) |
| **Pins** | 4-pin (VCC, R, G, B) |
| **Size** | ~18mm x 15mm |
| **Resistors** | Built-in current limiting |

## Advantages for Robot Projects

- **No External Resistors**: Built-in current limiting simplifies wiring
- **High Brightness**: Excellent visibility in various lighting conditions
- **Color Variety**: Millions of colors for detailed status indication
- **Compact Design**: Minimal space requirements on robot chassis
- **Professional Look**: Clean, integrated appearance
- **Reliable Operation**: SMD LEDs for durability and longevity

**Perfect for**: Status indication, user feedback, debugging visual aid, aesthetic enhancement, multi-state indication systems
