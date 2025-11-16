# Wheelie Robot - Complete Setup Guide

## Overview

This guide will walk you through building the Wheelie autonomous robot using the DollaTek 2WD Smart Robot Car Chassis Kit and ESP32 microcontroller.

## Required Tools

- Screwdrivers (Phillips and flathead)
- Wire strippers
- Soldering iron and solder (optional but recommended)
- Multimeter
- Hot glue gun (optional)
- Double-sided tape or mounting brackets

## Assembly Time

- **Beginner**: 3-4 hours
- **Intermediate**: 2-3 hours  
- **Advanced**: 1-2 hours

---

## Phase 1: Mechanical Assembly

### Chassis Preparation

1. **Unpack the DollaTek kit** and verify all parts:
   - Acrylic chassis plates (top and bottom)
   - 2x TT gear motors
   - 2x wheels
   - Speed encoder discs and sensors
   - Battery box (4x AA)
   - Front caster wheel
   - Rear caster ball
   - Screws and spacers

2. **Assemble the base chassis**:
   - Attach the 4 corner spacers between top and bottom plates
   - Install the front caster wheel assembly
   - Mount the rear caster ball bearing

3. **Install the motors**:
   - Mount TT motors in the side brackets
   - Ensure motors are aligned and secure
   - Attach wheels to motor shafts
   - Install speed encoder discs on motor shafts

4. **Mount the battery box**:
   - Secure battery box under the chassis
   - Route power wires through chassis opening
   - Test fit 4x AA batteries

### Weight Distribution Check

- Place batteries in box
- Ensure robot sits level on all contact points
- Adjust caster positions if needed for balance

---

## Phase 2: Electronics Layout

### Connection Strategy Choice

#### Option A: Direct Breadboard Connections

- Use jumper wires from ESP32 pins to breadboard
- More flexible for prototyping and changes
- Good for temporary builds and experimentation

#### Option B: ESP32 Breakout Board (Recommended)

- Mount ESP32 on breakout board with screw terminals
- More reliable connections for permanent robot
- Easier troubleshooting with labeled terminals
- Professional appearance and durability

### Component Placement Strategy

```txt
[Front of Robot]
    VL53L0X Sensor
    
    ESP32     Breadboard
   MOSFET H-Bridge     MPU6050
    
    Battery Box
[Rear of Robot]
```

### Mounting Locations

1. **ESP32** (Type-C, CH340C, 30-pin): Center-front of top plate for easy access
   - Ensure Type-C port is accessible for programming/power
   - If using breakout board: Mount breakout board, then insert ESP32
   - Orient with pins facing outward for breadboard connections
2. **ESP32 Breakout Board** (if used): Mount securely to chassis
   - Provides screw terminals for all GPIO connections
   - Reduces wiring complexity and improves reliability
3. **MOSFET H-Bridge**: Near ESP32, with good ventilation
4. **Breadboard**: Adjacent to ESP32 for short connections (if not using breakout)
5. **VL53L0X**: Front edge, facing forward, ~2cm above ground
6. **MPU6050**: Secure location with minimal vibration
7. **Edge sensor**: Bottom of chassis, pointing down
8. **Sound sensor**: Top front, facing forward

---

## Phase 3: Power System Wiring

### Power Distribution

```txt
Battery (7.4V Li-Po or 6V AA) → MOSFET H-Bridge Motor Supply (direct)
                             → XL4015 Buck Converter → Adjustable 5V/3.3V → ESP32 + Sensors
                             → Battery Indicator (if using Li-Po)
```

**XL4015 Buck Converter Benefits:**

- **High Efficiency**: 96% efficiency at optimal load
- **Wide Input Range**: 4-38V input, 1.25-36V output
- **High Current**: Up to 5A output capability
- **Constant Current**: LED driver capability for stable power
- **Adjustable**: Precise voltage control with potentiometer

**Important Safety Notes:**

- Always connect ground first
- Double-check polarity before applying power
- Use fuses or current-limiting resistors where appropriate
- Monitor battery voltage to prevent over-discharge (especially with Li-Po)
- Set XL4015 output voltage BEFORE connecting to ESP32

**Battery Options:**

- **4x AA (6V)**: Simple, safe, readily available
- **7.4V Li-Po**: Higher capacity, lighter weight, requires voltage monitoring

### Power Connections

1. **XL4015 Buck Converter Setup**:
   - Battery + → XL4015 IN+
   - Battery - → XL4015 IN- (and common ground)
   - XL4015 OUT+ → ESP32 VIN (adjust to 5.0V first!)
   - XL4015 OUT- → Common ground

2. **XL4015 Voltage Adjustment** (CRITICAL):
   - Connect multimeter to XL4015 output
   - Apply battery power to XL4015 input
   - Adjust potentiometer to exactly 5.0V output
   - Disconnect power before connecting ESP32

3. **Motor Power** (Direct from Battery):
   - Battery + → MOSFET H-Bridge VM (motors need full voltage)
   - Battery - → MOSFET H-Bridge GND (common ground)

4. **Alternative Development Power**:
   - Option A: USB-C power during programming/testing
   - Option B: Bench power supply for initial setup

5. **Sensor Power** (3.3V):
   - Use ESP32's 3.3V output
   - Maximum current: 600mA total
   - Add decoupling capacitors if needed

---

## Phase 4: Motor Control Wiring

### MOSFET H-Bridge to TT Motors

```txt
MOSFET H-Bridge Output → Motor Wire
OUT1/OUT2 → Left Motor
OUT3/OUT4 → Right Motor
```

### ESP32 to MOSFET H-Bridge Control

```txt
Function        ESP32 Pin → H-Bridge Pin
Left Dir 1      GPIO 23   → IN1
Left Dir 2      GPIO 22   → IN2
Right Dir 1     GPIO 19   → IN3
Right Dir 2     GPIO 18   → IN4
Ground          GND       → GND
```

### Motor Direction Testing

Before final assembly, test motor directions:

1. Upload test code
2. Verify "forward" moves both wheels forward
3. Verify "turn left" rotates correctly
4. Swap motor wires if direction is wrong

---

## Phase 5: Sensor Integration

### Ultrasonic Sensor (HC-SR04)

```txt
ESP32 GPIO 16 → HC-SR04 Trig
ESP32 GPIO 32 → HC-SR04 Echo
5V            → HC-SR04 VCC
GND           → HC-SR04 GND
```

- **Purpose**: Rear obstacle detection
- **Mounting**: Rear of robot, facing backward
- **Notes**: 5V logic, use voltage divider on Echo if needed

### I2C Bus Setup (VL53L0X + MPU6050)

Both sensors share the I2C bus:

```txt
ESP32 Pin → Sensor Connection
GPIO 26   → SDA (both sensors)
GPIO 27   → SCL (both sensors)  
3.3V      → VCC (both sensors)
GND       → GND (both sensors)
```

**I2C Considerations:**

- Use 4.7kΩ pull-up resistors on SDA/SCL if experiencing issues
- Keep I2C wires short (<20cm)
- Avoid running I2C wires parallel to motor wires

### Individual Sensor Connections

#### VL53L0X GY-VL53L0XV2 Time-of-Flight Sensor

```txt
ESP32 GPIO 26 → VL53L0X SDA (I2C Data)
ESP32 GPIO 27 → VL53L0X SCL (I2C Clock)
3.3V          → VL53L0X VCC
GND           → VL53L0X GND
```

- **Model**: VL53L0X GY-VL53L0XV2 ranging sensor module
- **Technology**: 940nm laser Time-of-Flight measurement
- **Range**: 20mm to 2000mm precision distance detection
- **Interface**: I2C communication with shared bus capability
- **Purpose**: Front obstacle detection and navigation support
- **Mounting**: Front of robot, 2-5cm above ground level
- **Angle**: Slightly downward (10-15°) to detect ground obstacles

#### MPU6050 GY-521 IMU

```txt
ESP32 GPIO 26 → MPU6050 SDA (I2C Data)
ESP32 GPIO 27 → MPU6050 SCL (I2C Clock)
3.3V          → MPU6050 VCC
GND           → MPU6050 GND
```

- **Model**: MPU6050 GY-521 6-axis sensor module
- **Features**: 3-axis accelerometer + 3-axis gyroscope (6 DOF)
- **Resolution**: 16-bit ADC for high precision motion sensing
- **Interface**: I2C communication with shared bus
- **Purpose**: Tilt detection, motion sensing, and orientation tracking
- **Mounting**: Secure to chassis with minimal vibration exposure
- **Orientation**: Align X/Y axes with robot's forward/side directions

#### Edge/Cliff Sensor

```txt
ESP32 GPIO 15 → Sensor Signal (optional)
3.3V         → Sensor VCC
GND          → Sensor GND
```

- **Purpose**: Prevent falling off tables/edges
- **Mounting**: Bottom of chassis, pointing down
- **Distance**: 2-3cm from ground when on flat surface

#### H-1-0332 Sound Sensor

```txt
ESP32 GPIO 17 → H-1-0332 OUT (Digital)
3.3V         → H-1-0332 VCC  
GND          → H-1-0332 GND
```

- **Model**: H-1-0332 high-sensitivity sound detection module
- **Purpose**: Sound-reactive behaviors and audio detection
- **Output**: Digital signal (HIGH when sound detected)
- **Mounting**: Top of robot, facing forward or up  
- **Sensitivity**: Adjust onboard potentiometer for desired threshold
- **Features**: Built-in amplifier and trigger circuit

---

## Phase 6: Visual & Audio Feedback

### KY-009 RGB LED Module Setup

Using KY-009 3-color SMD LED board with built-in current limiting:

```txt
ESP32 Pin → KY-009 Pin
GPIO 14   → R (Red)
GPIO 12   → G (Green)
GPIO 13   → B (Blue)
3.3V      → VCC (+)
GND       → GND (-)
```

**LED Color Codes** (as programmed):

- 🔴 Red: Stopped/Error state
- 🟢 Green: Moving forward
- 🔵 Blue: Turning
- 🟡 Yellow: Backing up
- 🟣 Purple: Turning right
- 🔶 Cyan: Slow forward

### KY-006 Piezoelectric Buzzer

```txt
ESP32 GPIO 21 → KY-006 Signal Pin (S)
3.3V          → KY-006 VCC (+)
GND           → KY-006 Ground (-)
```

- **Type**: KY-006 piezoelectric buzzer module
- **Operating Voltage**: 3-5V (ESP32 3.3V compatible)
- **Control**: PWM signal on GPIO 21
- **Volume**: Adjust with PWM duty cycle (0-255)
- **Mounting**: Away from sensors to avoid interference
- **Features**: Built-in driving circuit, reliable tone generation

### Battery Capacity Indicator (Optional)

```txt
Lithium Battery Capacity Indicator Module:
Battery + → VCC (3.7V Li-Po positive)
Battery - → GND (3.7V Li-Po negative)
B+ → Connect to battery positive terminal
B- → Connect to battery negative terminal
```

- **Purpose**: Real-time battery voltage/capacity monitoring
- **Display**: 4-segment blue LED bar showing charge level
- **Voltage Range**: 3.0V - 4.2V (optimized for Li-Po batteries)
- **Installation**: Mount on chassis where LEDs are visible
- **Wiring**: Use included 26AWG wire for connections
- **Note**: This module works independently and doesn't connect to ESP32

---

## Phase 7: Software Configuration

### Development Environment

1. **Install VS Code** with PlatformIO extension
2. **Clone the project** or download source code
3. **Open project folder** in VS Code
4. **Let PlatformIO install** dependencies automatically

### First Upload

1. **Install CH340C driver** (if not automatically detected):
   - Windows: Usually auto-installs via Windows Update
   - Manual driver: Available from manufacturer or Arduino IDE
2. **Connect ESP32** via Type-C USB cable
   - Use a quality USB-C cable with data capability
   - Some charge-only cables won't work for programming
3. **Select correct COM port** in PlatformIO
   - Look for "USB-SERIAL CH340" in device manager
4. **Build project**: `Ctrl+Shift+P` → "PlatformIO: Build"
5. **Upload firmware**: `Ctrl+Shift+P` → "PlatformIO: Upload"
6. **Open serial monitor**: `Ctrl+Shift+P` → "PlatformIO: Serial Monitor"

### Initial Testing Sequence

The robot will automatically run diagnostics on startup:

1. **System initialization** - Check all sensors
2. **LED test** - Cycle through all colors  
3. **Audio test** - Play startup melody
4. **Motor test** - Test each motor individually
5. **Sensor readings** - Display all sensor values

---

## Phase 8: Calibration & Tuning

### Sensor Calibration

1. **VL53L0X**: Clean lens, verify detection range
2. **MPU6050**: Place on level surface for gyro calibration
3. **Edge sensor**: Test detection height over table edge
4. **Sound sensor**: Adjust sensitivity potentiometer

### Motion Tuning

Adjust these parameters in `src/main.cpp`:

```cpp
const int TEST_SPEED = 200;           // Motor speed (0-255)
const int OBSTACLE_DISTANCE = 200;    // mm - stop distance
const int WARNING_DISTANCE = 350;     // mm - slow down distance  
const float TILT_THRESHOLD = 30.0;    // degrees - emergency stop
```

### Behavior Customization

- **Obstacle avoidance**: Modify `checkAndHandleObstacle()`
- **Sound reactions**: Adjust `checkAndHandleSound()`
- **Navigation patterns**: Customize `normalOperation()`

---

## Phase 9: Testing & Validation

### Safety Tests

1. **Tilt protection**: Gradually tilt robot to verify shutdown
2. **Edge detection**: Place near table edge
3. **Obstacle avoidance**: Test with various objects
4. **Emergency stop**: Verify immediate response

### Performance Tests  

1. **Battery life**: Monitor operation time
2. **Speed consistency**: Test on different surfaces
3. **Turning accuracy**: Verify consistent turn angles
4. **Sensor reliability**: Test in various lighting conditions

### Troubleshooting Checklist

- [ ] All connections secure and correct polarity
- [ ] ESP32 receiving adequate power (5V, >500mA)
- [ ] I2C sensors responding (check serial monitor)
- [ ] Motors moving in correct directions
- [ ] LEDs functioning and showing correct colors
- [ ] Buzzer producing sounds
- [ ] No loose wires or short circuits

---

## Phase 10: Advanced Features (Optional)

### LM393 H2010 Speed Encoders

Using the **2x LM393 H2010 Photoelectric Sensors** for wheel encoding:

**Connections:**

```txt
Right Encoder (LM393 #1):
ESP32 GPIO 5  → Right Encoder OUT (Digital)
3.3V          → Right Encoder VCC
GND           → Right Encoder GND

Left Encoder (LM393 #2):
ESP32 GPIO 33 → Left Encoder OUT (Digital)
3.3V          → Left Encoder VCC
GND           → Left Encoder GND
```

**Implementation:**

1. **Mount H2010 sensors** to detect wheel rotation
2. **Connect to interrupt pins** (GPIO 32, 33) for accurate counting
3. **Install encoder discs** on motor shafts (if not included)
4. **Modify software** to count pulses and calculate speed
5. **Implement PID control** for precise movement and navigation
6. **Add odometry** for position tracking and autonomous navigation

### Additional Sensors

Consider adding:

- **Ultrasonic sensor** (HC-SR04) for backup obstacle detection
- **Camera module** for vision-based navigation
- **Compass module** for directional guidance
- **GPS module** for outdoor navigation

### Wireless Control

Add remote control capabilities:

- **WiFi web interface** for browser control
- **Bluetooth** for smartphone app control
- **ESP-NOW** for direct ESP32-to-ESP32 communication

---

## Maintenance & Upgrades

### Regular Maintenance

- **Clean sensors** monthly or after dusty environments
- **Check connections** for corrosion or loosening
- **Lubricate wheels** and casters if needed
- **Update firmware** for new features and bug fixes

### Common Upgrades

1. **Larger battery** for extended runtime
2. **Better wheels** for improved traction
3. **Shock dampening** for sensor protection
4. **Weatherproofing** for outdoor use

---

## Safety Considerations

### Electrical Safety

- Never exceed component voltage ratings
- Use appropriate fuses and current limiting
- Avoid short circuits between battery terminals
- Disconnect power when modifying circuits

### Mechanical Safety  

- Secure all components to prevent damage from falls
- Ensure robot cannot tip over during normal operation
- Use appropriate mounting hardware for component weights
- Test emergency stop functions regularly

### Environmental Safety

- Avoid operating near stairs or high surfaces without edge detection
- Test in controlled environment before autonomous operation
- Supervise initial testing until behavior is predictable
- Consider fail-safe modes for loss of communication

---

## Support & Resources

### Documentation

- [ESP32 Pinout Reference](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
- [L298N Motor Driver Guide](https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/)
- [VL53L0X Sensor Documentation](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html)

### Community

- **GitHub Issues**: Report bugs and request features
- **Arduino Forums**: General ESP32 and sensor help  
- **Reddit r/robotics**: Project sharing and advice

### Troubleshooting

If you encounter issues:

1. Check the troubleshooting section in README.md
2. Review wiring diagrams in WIRING.md
3. Enable debug output in serial monitor
4. Post detailed issue description with photos if needed

---

## Happy Building! 🤖

We hope you enjoy building and customizing your Wheelie robot. Share your projects and improvements with the community!
