# Wheelie Robot - Enterprise ESP32 Autonomous Robot

An advanced autonomous robot with enterprise-grade features including intelligent power management, secure OTA updates, comprehensive data logging, system health monitoring, and graceful degradation capabilities.

## 🚀 Enterprise Features

- 🤖 **Intelligent Navigation** - 6-state robot management with adaptive behavior
- 🛡️ **System Health Monitoring** - Real-time memory, performance, and sensor health tracking
- 🔋 **Smart Power Management** - 5-level battery management with automatic power scaling
- 📡 **Secure OTA Updates** - Remote firmware updates with authentication and progress tracking
- 📊 **Professional Data Logging** - SPIFFS-based CSV logging with analytics and rotation
- 🔄 **Graceful Degradation** - Automatic adaptation when sensors fail
- � **Multi-Sensor Fusion** - ToF, IMU, edge detection with comprehensive safety
- 🎵 **Sound-Reactive Behaviors** - Audio-triggered interaction modes
- 💡 **RGB Status Indicators** - Real-time system status visualization
- 🔊 **Audio Feedback System** - Professional diagnostic and status reporting

## Materials List

### Chassis & Motors

- **DollaTek 2WD Smart Robot Car Chassis Kit**
  - Includes: Acrylic chassis, 2x TT motors with wheels, speed encoders, battery box, front/rear casters
  - Alternative: Any 2WD robot chassis with TT motors

### Electronics - Core Components

- **ESP32 Development Board** - Type-C, CH340C chip, 30-pin, 2.4GHz WiFi/Bluetooth
- **ESP32 Breakout/Expansion Board** - 30-pin breakout for easier connections
- **XL4015 Buck Converter** - DC-DC step-down voltage regulator (4-38V to 1.25-36V, 5A max)
- **Fasizi L298N Motor Driver** - Dual H-bridge PWM motor controller (2V-10V, ultra-compact)
- **Breadboard or Perfboard** - For connections (400-point recommended)
- **Jumper Wires** - Male-to-male, male-to-female, female-to-female
- **Power Supply** - 2x 3.7V 2000mAh Li-Po batteries (903042) in series for 7.4V 2000mAh
- **Voltage Divider Resistors** - 10kΩ and 3.3kΩ for battery monitoring circuit

### Sensors

- **VL53L0X GY-VL53L0XV2 Time-of-Flight Sensor** - 940nm laser ranging for obstacle detection (20-2000mm range)
- **MPU6050 GY-521 6-Axis IMU** - 3-axis accelerometer + 3-axis gyroscope for tilt detection and motion sensing
- **IR Edge/Cliff Sensor** - Prevents falling off tables
- **H-1-0332 Sound Sensor Module** - Sound-reactive behaviors and audio detection
- **2x LM393 H2010 Photoelectric Sensors** - Motor speed encoders and wheel rotation counting
- **PIR Motion Sensor** (Optional) - Motion detection

### Indicators & Feedback

- **KY-009 RGB LED Module** - 3-color SMD LED board for status indication (DC 5V)
- **KY-006 Piezoelectric Buzzer** - Sound emitting device for audio feedback
- **Lithium Battery Capacity Indicator Module** - 3.7V Blue LED Display Board
- **2S Li-Po Charger Module** - USB-C boost charger for 7.4V battery pack (2S 1A)
- **Resistors** - 220Ω for LEDs, various values for sensors

### Hardware & Tools

- **M3 Screws and Nuts** - For mounting sensors and boards
- **Double-sided tape or mounting brackets**
- **Wire strippers and screwdrivers**
- **Multimeter** (for testing connections)

### Optional Upgrades

- **Ultrasonic Sensor (HC-SR04)** - Additional obstacle detection
- **Camera Module** - For vision-based navigation
- **Servo Motor** - For sensor scanning
- **LED Strip** - Enhanced visual feedback

## Hardware Requirements

## Assembly & Setup Guide

### Step 1: Chassis Assembly

1. **Assemble the DollaTek chassis** following the included instructions
2. **Install TT motors** in the chassis motor mounts
3. **Attach wheels** to the motor shafts
4. **Install casters** (front and rear) for stability
5. **Mount battery box** under the chassis
6. **Test motor rotation** by connecting directly to battery

### Step 2: Electronics Mounting

1. **Mount ESP32** on the chassis using double-sided tape or brackets
2. **Mount L298N motor driver** near the ESP32
3. **Install breadboard** for sensor connections
4. **Position sensors** for optimal operation:
   - VL53L0X facing forward for obstacle detection
   - MPU6050 mounted securely (avoid vibration)
   - Edge sensor pointing downward at chassis edge
   - Sound sensor facing forward/upward

### Step 3: Power Wiring & Battery Monitoring

⚠️ **Always double-check polarity before connecting power!**

```txt
Battery Box (7.4V 2S LiPo) → L298N VCC
ESP32 VIN ← L298N +5V (if available) OR separate 5V supply
All GND connections → Common ground rail

Battery Voltage Monitoring Circuit:
Battery + (7.4V) → 10kΩ Resistor → Junction → 3.3kΩ Resistor → GND
                                   ↓
                              ESP32 GPIO34 (ADC)

This voltage divider scales 8.4V max → 2.32V for safe ADC input
```

### Step 4: Motor Connections

```txt
L298N → TT Motors
OUT1  → Left Motor +
OUT2  → Left Motor -
OUT3  → Right Motor +
OUT4  → Right Motor -
```

### Step 5: ESP32 to L298N Connections

```txt
ESP32 GPIO → L298N Pin
GPIO 25    → ENA (Left Motor Speed)
GPIO 23    → IN1 (Left Motor Direction)
GPIO 22    → IN2 (Left Motor Direction)
GPIO 14    → ENB (Right Motor Speed)
GPIO 19    → IN3 (Right Motor Direction)
GPIO 18    → IN4 (Right Motor Direction)
GND        → GND
```

### Step 6: Sensor Wiring

```txt
VL53L0X ToF Sensor:
ESP32 GPIO 26 → SDA
ESP32 GPIO 27 → SCL
3.3V → VCC, GND → GND

MPU6050 IMU:
ESP32 GPIO 26 → SDA (shared I2C bus)
ESP32 GPIO 27 → SCL (shared I2C bus)
3.3V → VCC, GND → GND

Edge Sensor:
ESP32 GPIO 34 → Signal
3.3V → VCC, GND → GND

Sound Sensor:
ESP32 GPIO 17 → Digital Out
3.3V → VCC, GND → GND
```

### Step 7: Indicators & Feedback

```txt
KY-009 RGB LED Module:
ESP32 GPIO 15 → R (Red)
ESP32 GPIO 2  → G (Green)
ESP32 GPIO 4  → B (Blue)
5V → VCC (+)
GND → GND (-)

Buzzer:
ESP32 GPIO 21 → Buzzer +
GND → Buzzer -
```

### Step 8: Testing & Calibration

1. **Power on** and check all connections
2. **Upload the code** using PlatformIO
3. **Open serial monitor** to view comprehensive diagnostics
4. **Test each system**:
   - System health monitoring displays memory and performance stats
   - Motors should move in diagnostic sequence with power management
   - LEDs should cycle through colors indicating system status
   - Buzzer should play startup melody with diagnostic tones
   - Sensors should report valid readings with health monitoring
   - Battery voltage should display current level and power mode
   - OTA updates should be available over WiFi for remote programming

### Step 9: Advanced Features Setup

1. **WiFi Configuration** - Robot connects automatically to configured network
2. **OTA Updates** - Enable secure remote firmware updates
3. **Data Logging** - SPIFFS file system logs operational data to CSV files
4. **Power Management** - System automatically adjusts performance based on battery level
5. **Health Monitoring** - Real-time system diagnostics and performance tracking

### Step 10: Mechanical Adjustments & Final Setup

1. **Balance the robot** - ensure it doesn't tip with battery and components
2. **Adjust caster heights** for smooth movement
3. **Secure all wiring** to prevent interference, especially voltage divider circuit
4. **Add cable management** for clean appearance and safety
5. **Test power management** - verify battery monitoring and power modes
6. **Validate OTA functionality** - ensure remote update capability works

### Troubleshooting Common Issues

- **Motors don't move**: Check power supply and L298N connections, verify power mode
- **Robot tips over**: Redistribute weight, lower center of gravity, check battery placement
- **Sensors not working**: Verify I2C connections and 3.3V power, check sensor health monitoring
- **Random resets**: Check power supply capacity (min 2A recommended), monitor battery voltage
- **Poor obstacle detection**: Clean VL53L0X lens, check mounting angle, verify sensor health
- **OTA updates fail**: Check WiFi connection, verify password, ensure sufficient flash space
- **Battery monitoring inaccurate**: Verify voltage divider resistor values (10kΩ + 3.3kΩ)
- **Performance issues**: Check data logging space, monitor memory usage, review system health
- **Power management not working**: Verify battery voltage reading, check calibration values

## Getting Started

### Quick Start

1. **Get the parts** - See [docs/assembly/SHOPPING_LIST.md](docs/assembly/SHOPPING_LIST.md) for complete materials list
2. **Assemble the robot** - Follow [docs/assembly/SETUP_GUIDE.md](docs/assembly/SETUP_GUIDE.md) for detailed assembly instructions  
3. **Wire the electronics** - Use [docs/assembly/WIRING.md](docs/assembly/WIRING.md) for connection diagrams
4. **Setup development** - Install VS Code with PlatformIO extension
5. **Upload and test** - Build and upload the firmware

### Documentation

#### 📁 Assembly & Setup

- 📋 **[Shopping List](docs/assembly/SHOPPING_LIST.md)** - Complete parts list with cost estimates
- 🔧 **[Setup Guide](docs/assembly/SETUP_GUIDE.md)** - Step-by-step assembly instructions
- 🔌 **[Wiring Diagram](docs/assembly/WIRING.md)** - Pin connections and electrical setup

#### 📁 Core Components

- �️ **[ESP32 Type-C Guide](docs/components/ESP32_TYPE_C_GUIDE.md)** - Microcontroller board specifications
- 🔗 **[Breakout Board Guide](docs/components/BREAKOUT_BOARD_GUIDE.md)** - ESP32 expansion board setup
- 🚗 **[Fasizi L298N Motor Driver](docs/components/FASIZI_L298N_GUIDE.md)** - Ultra-compact motor control
- 💡 **[KY-009 RGB LED Module](docs/components/KY-009_RGB_LED_GUIDE.md)** - Status indication system

#### 📁 Power Management

- 🔋 **[Li-Po Battery Pack Guide](docs/power/LIPO_BATTERY_PACK_GUIDE.md)** - 2S battery configuration and safety
- ⚡ **[XL4015 Buck Converter](docs/power/XL4015_POWER_GUIDE.md)** - Efficient power regulation
- 🔌 **[USB-C 2S Charger](docs/power/USB_C_2S_CHARGER_GUIDE.md)** - Battery charging system
- 📊 **[Battery Indicator](docs/power/BATTERY_INDICATOR.md)** - Charge level monitoring

#### 📁 Sensors & Feedback

- 🧭 **[MPU6050 6-Axis IMU](docs/sensors/MPU6050_GY521_GUIDE.md)** - Motion and tilt detection
- 📏 **[VL53L0X ToF Sensor](docs/sensors/VL53L0X_GY_VL53L0XV2_GUIDE.md)** - Laser distance measurement
- 🎤 **[H-1-0332 Sound Sensor](docs/sensors/H-1-0332_SOUND_SENSOR_GUIDE.md)** - Audio detection and response
- ⚙️ **[LM393 H2010 Encoders](docs/sensors/LM393_H2010_ENCODER_GUIDE.md)** - Wheel speed and position

#### 💻 Source Code

- 🤖 **[Main Firmware](src/main.cpp)** - Complete robot control system with safety features

### PlatformIO Setup

1. Install VS Code
2. Install PlatformIO IDE extension
3. Open this project folder
4. PlatformIO will automatically install dependencies
5. Build and upload to your ESP32

### Pin Configuration & Advanced Features

See `src/main.cpp` for detailed pin definitions and wiring instructions.

**New Enterprise Features:**

- Battery voltage monitoring on GPIO34 with voltage divider circuit
- Real-time system health monitoring and performance analytics
- SPIFFS-based data logging with CSV export capabilities
- Secure OTA updates with WiFi connectivity
- 5-level intelligent power management system
- Graceful sensor failure handling and adaptive behavior

## Building and Uploading

```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## License

MIT License
