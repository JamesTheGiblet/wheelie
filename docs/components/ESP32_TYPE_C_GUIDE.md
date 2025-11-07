# ESP32 Type-C Development Board - Quick Reference

## Board Specifications

### Hardware Details

- **Model**: ESP32 Development Board with Type-C
- **USB Interface**: USB Type-C (reversible connector)
- **Serial Chip**: CH340C (USB-to-Serial converter)
- **Pin Configuration**: 30-pin layout
- **WiFi**: 2.4GHz 802.11 b/g/n
- **Bluetooth**: Classic + BLE (Bluetooth Low Energy)
- **Flash Memory**: 4MB (typically)
- **RAM**: 520KB SRAM

### Key Features

- **Modern Connector**: Type-C is more durable than micro-USB
- **Reliable Serial**: CH340C provides stable communication
- **Compact Design**: 30-pin layout saves space
- **Built-in LED**: GPIO 2 connected to onboard LED
- **Auto-Reset**: Automatic programming mode entry

## ESP32 Breakout/Expansion Board

### Purpose and Benefits

The ESP32 Breakout Board provides easier access to all 30 pins of the ESP32 development board through:

- **Screw Terminals**: Secure, removable connections
- **Labeled Pins**: Clear GPIO numbering and function labels
- **Power Distribution**: Dedicated 3.3V and 5V terminals
- **Ground Rails**: Multiple GND connections for sensors
- **Compact Layout**: Organized pin arrangement for clean wiring

### Connection Method

1. **ESP32 Insertion**: Insert the 30-pin ESP32 board into the breakout board socket
2. **Secure Fit**: Ensure all pins make good contact
3. **Wiring**: Use screw terminals instead of jumper wires for permanent connections
4. **Programming**: USB-C remains accessible for programming and power

### Advantages for Robot Projects

- **Reliability**: Screw terminals won't come loose during robot movement
- **Professional**: Clean, organized appearance
- **Maintenance**: Easy to change connections without soldering
- **Troubleshooting**: Clearly labeled pins simplify debugging
- **Durability**: Reduces wear on ESP32 pin headers

## Pin Layout (30-Pin Configuration)

### Left Side (from USB end)

```txt
EN    │  GPIO 36 (SVP)
GPIO 35 (SVN) │  GPIO 26
GPIO 32 │  GPIO 18
GPIO 33 │  GPIO 19
GPIO 25 │  GPIO 21
GPIO 26 │  RX0 (GPIO 3)
GPIO 27 │  TX0 (GPIO 1)
GPIO 14 │  GPIO 22
GPIO 12 │  GPIO 23
GND   │  GND
GPIO 13 │  GPIO 15
GPIO 9  │  GPIO 2
GPIO 10 │  GPIO 0
GPIO 11 │  GPIO 4
5V    │  GPIO 16
3.3V  │  GPIO 17
```

### GPIO Usage in Wheelie Robot

```txt
Pin Assignment for Robot Components:
GPIO 25 → Motor A Speed (ENA)
GPIO 23 → Motor A Direction 1 (IN1)
GPIO 22 → Motor A Direction 2 (IN2)
GPIO 14 → Motor B Speed (ENB)
GPIO 19 → Motor B Direction 1 (IN3)
GPIO 18 → Motor B Direction 2 (IN4)
GPIO 26 → I2C SDA (VL53L0X, MPU6050)
GPIO 27 → I2C SCL (VL53L0X, MPU6050)
GPIO 34 → Edge Sensor (Analog Input)
GPIO 17 → Sound Sensor (Digital Input)
GPIO 35 → PIR Sensor (Digital Input) - Optional
GPIO 15 → RGB LED Red
GPIO 2  → RGB LED Green
GPIO 4  → RGB LED Blue
GPIO 21 → Buzzer/Speaker
```

## CH340C Driver

### Installation

**Windows 10/11**: Usually auto-installs via Windows Update
**Manual Installation**:

1. Download from manufacturer or Arduino IDE
2. Install driver package
3. Restart computer if needed
4. Verify in Device Manager under "Ports (COM & LPT)"

### Identification

- Device appears as "USB-SERIAL CH340" in Device Manager
- COM port number assigned automatically
- Green LED on board indicates power
- Blue LED indicates serial communication

## Programming Setup

### PlatformIO Configuration

The board is compatible with standard ESP32 settings:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
```

### Arduino IDE Setup

1. **Board**: "ESP32 Dev Module"
2. **Upload Speed**: 921600 (or 115200 if issues)
3. **CPU Frequency**: 240MHz
4. **Flash Frequency**: 80MHz
5. **Flash Mode**: DIO
6. **Flash Size**: 4MB

## Type-C Connection Benefits

### Physical Advantages

- **Reversible**: No wrong way to insert cable
- **Durable**: Better connector retention and lifecycle
- **Modern**: Future-proof connector standard
- **Power Delivery**: Supports higher charging currents

### Programming Reliability

- **Stable Connection**: Less prone to connection issues
- **Better Contact**: More reliable data transmission
- **Reduced Wear**: Longer lasting connector

## Troubleshooting

### Common Issues

#### "Port Not Found" Error

1. **Check Cable**: Ensure USB-C cable supports data (not charge-only)
2. **Driver Issue**: Reinstall CH340C driver
3. **Port Conflict**: Close other serial applications
4. **Windows Issue**: Try different USB port

#### Upload Failures

1. **Hold BOOT Button**: Press and hold during upload start
2. **Reset Sequence**: Press RESET after upload begins
3. **Baud Rate**: Try lower upload speed (115200)
4. **Cable Quality**: Use shorter, higher-quality USB-C cable

#### Serial Monitor Issues

1. **Wrong Port**: Verify correct COM port selected
2. **Baud Rate**: Ensure matching baud rate (115200)
3. **Driver**: Reinstall CH340C driver
4. **Buffer**: Clear serial buffer and reconnect

### Hardware Checks

- **Power LED**: Should be solid green when connected
- **Programming LED**: Should flash blue during upload
- **Continuity**: Check GPIO connections with multimeter
- **Voltage**: Verify 3.3V and 5V outputs

## Compatibility Notes

### Software Compatibility

- **Arduino IDE**: Fully compatible
- **PlatformIO**: Standard ESP32 support
- **ESP-IDF**: Native framework support
- **MicroPython**: Full support

### Pin Compatibility

- Same GPIO functionality as other ESP32 boards
- Pin numbers identical to ESP32-DevKitC
- All robot code works without modification
- Same I2C, SPI, ADC capabilities

### Differences from Other ESP32 Boards

- **Connector**: Type-C vs micro-USB
- **Serial Chip**: CH340C vs CP2102/FTDI
- **Layout**: 30-pin vs 38-pin variants
- **Size**: Slightly more compact

## Power Specifications

### Power Input

- **USB Type-C**: 5V from computer/adapter
- **VIN Pin**: 6-12V external power
- **3.3V Pin**: 3.3V regulated (output only)

### Power Output

- **3.3V**: Max 600mA for sensors
- **5V**: Available when powered via USB
- **Current Draw**: ~80mA idle, ~200mA active

### Power Management

- **Sleep Modes**: Deep sleep, light sleep available
- **Wake Sources**: Timer, GPIO, touch, ULP
- **Battery Operation**: Can run on Li-Po with regulator

## Advanced Features

### WiFi Capabilities

- **Station Mode**: Connect to existing network
- **Access Point**: Create hotspot for robot control
- **WiFi Direct**: Peer-to-peer communication
- **Range**: ~100m outdoor, ~30m indoor

### Bluetooth Features

- **Classic Bluetooth**: For smartphone apps
- **BLE**: Low energy sensor communication
- **Dual Mode**: Both classic and BLE simultaneously
- **Pairing**: Standard Bluetooth pairing process

### Sensor Interfaces

- **I2C**: Hardware I2C on GPIO 21/22 (default) or any GPIO
- **SPI**: Hardware SPI available
- **ADC**: 12-bit ADC on select pins
- **PWM**: All GPIO pins support PWM output

---

**This ESP32 variant provides the same powerful features as other ESP32 boards with the added benefit of modern Type-C connectivity and reliable CH340C serial communication.**
