# 📍 Hardware Pin Mapping

## 🔌 ESP32 Pin Assignments (Actual Hardware)

### 🚗 **Motor Control System**

| Component | ESP32 Pin | Function | Wire Color | Notes |
|-----------|-----------|----------|------------|-------|
| IN1 (Left Motor Control 1) | GPIO 23 | Left motor PWM control | Yellow | MOS-FET H-Bridge driver |
| IN2 (Left Motor Control 2) | GPIO 22 | Left motor PWM control | Green | MOS-FET H-Bridge driver |
| IN3 (Right Motor Control 1) | GPIO 19 | Right motor PWM control | Yellow | MOS-FET H-Bridge driver |
| IN4 (Right Motor Control 2) | GPIO 18 | Right motor PWM control | Orange | MOS-FET H-Bridge driver |

### 📊 **Encoder Feedback**

| Component | ESP32 Pin | Function | Wire Color | Notes |
|-----------|-----------|----------|------------|-----------------------|
| Right Encoder | GPIO 5 | Right wheel encoder | Blue | LM393 H2010, 5V power |
| Left Encoder | GPIO 33 | Left wheel encoder | Purple | LM393 H2010, 5V power |

### 💡 **Visual Indicators**

| Component | ESP32 Pin | Function | Wire Color | Notes |
|-----------|-----------|----------|------------|-------|
| Red LED | GPIO 14 | Status indicator - Red channel | Red | RGB LED module |
| Green LED | GPIO 12 | Status indicator - Green channel | Green | RGB LED module |
| Blue LED | GPIO 13 | Status indicator - Blue channel | Blue | RGB LED module |

### 🔊 **Audio System**

| Component | ESP32 Pin | Function | Wire Color | Notes |
|-----------|-----------|----------|------------|-------|
| Buzzer | GPIO 21 | Audio alerts and feedback | Red | Piezo buzzer, 3.3V power |
| Sound Sensor | GPIO 17 | Sound detection input | - | H-1-0332, 3.3V power |

### 🛡️ **Safety Sensors**

| Component | ESP32 Pin | Function | Status |
|-----------|-----------|----------|--------|
| Edge Sensor | GPIO 15 | Cliff detection | Not installed |
| PIR Motion | GPIO 39 | Motion detection | Not installed |

### 🔗 **I2C Bus (Sensor Communication)**

| Component | ESP32 Pin | Function | Connected Devices |
|-----------|-----------|---------------|-------------------------|
| SDA (Data) | GPIO 26 | I2C data line | VL53L0X ToF, MPU6050 IMU |
| SCL (Clock) | GPIO 27 | I2C clock line | VL53L0X ToF, MPU6050 IMU |

## 🔋 **Power Distribution**

### Power Rails

- **Battery**: 8.4V Li-Po (2x 4.2V cells, 4000mAh total)
- **XL4015 Input**: 8.4V from battery
- **XL4015 Output**: 5V regulated to ESP32 VIN
- **Motor Driver**: 8.4V direct from battery
- **ESP32 3.3V**: Sensors (MPU6050, VL53L0X, Sound Sensor)
- **ESP32 5V**: Encoders (LM393 H2010)
- **GND**: Common ground for all components

### Power Connections

| Component | Power Source | Voltage | Notes |
|-----------|-------------|---------|-------|
| ESP32 | XL4015 output | 5V | Via VIN pin |
| Motor Driver | Battery direct | 8.4V | MOS-FET H-Bridge |
| MPU6050 | ESP32 3.3V | 3.3V | I2C sensor |
| VL53L0X | ESP32 3.3V | 3.3V | I2C sensor |
| Sound Sensor | ESP32 3.3V | 3.3V | Digital input |
| Encoders | ESP32 5V | 5V | LM393 modules |
| LEDs | ESP32 GPIO | 3.3V | Direct GPIO control |
| Buzzer | ESP32 3.3V | 3.3V | GPIO + 3.3V power |
| Battery Monitor | Battery direct | 8.4V | Independent LED display |

### Current Draw (Estimated)

- **ESP32**: ~240mA (with WiFi + ESP-NOW active)
- **Motors**: ~1500mA each (under load)
- **Sensors**: ~50mA total (MPU6050, VL53L0X, Sound)
- **Encoders**: ~30mA total (2x LM393)
- **LEDs**: ~60mA total (all on)
- **Battery Monitor**: ~5mA
- **Total Peak**: ~3.4A

## 🛠️ **Hardware Layout Notes**

### Pin Selection Rationale

- **PWM Pins (25, 32)**: Selected for motor speed control compatibility
- **Input-Only Pin (34)**: Used for encoder that doesn't need pullups
- **High-Speed Pins**: Used for time-critical motor control
- **I2C Pins (26, 27)**: Standard I2C pins for sensor communication

### Board Orientation

- Robot assembled with ESP32 development board in specific orientation
- Pin numbering adjusted for actual physical layout
- Wiring optimized for minimal crossing and interference

### Signal Integrity

- Motor control pins separated from sensitive sensor inputs
- I2C bus isolated from high-current motor switching
- Power and ground distribution optimized for noise reduction

## 🔧 **Connection Verification**

### Sensors

```txt
VL53L0X ToF Sensor:
VCC → 3.3V
GND → GND
SDA → GPIO 26
SCL → GPIO 27

MPU6050 IMU:
VCC → 3.3V
GND → GND
SDA → GPIO 26
SCL → GPIO 27
```

### Status LEDs

```txt
Common Cathode RGB LED:
Red Anode   → GPIO 14 (through 220Ω resistor)
Green Anode → GPIO 12 (through 220Ω resistor)
Blue Anode  → GPIO 13 (through 220Ω resistor)
Cathode     → GND
```

## ⚠️ **Important Notes**

### Pin Constraints

- **GPIO 34**: Input only, no pullup resistor available
- **GPIO 39**: Input only, no pullup resistor available
- **Boot Pins**: Avoid using GPIO 0, 2 during boot sequence
- **Flash Pins**: GPIO 6-11 reserved for flash memory

### Signal Levels

- **ESP32 Logic**: 3.3V
- **Motor Driver**: 5V logic compatible
- **Sensors**: 3.3V operation

### Safety Considerations

- **Motor Power**: Separate 7.4V rail for motor driver
- **Logic Protection**: Motor driver provides logic level isolation
- **Reverse Protection**: Diodes on motor outputs prevent back-EMF damage

## 📋 **Testing Checklist**

### Power-On Tests

- [ ] 3.3V rail stable
- [ ] 5V rail stable
- [ ] LED indicators functional
- [ ] Buzzer operational

### Motor Tests

- [ ] Left motor forward/reverse
- [ ] Right motor forward/reverse
- [ ] PWM speed control functional
- [ ] No unexpected noise or heating

### Sensor Tests

- [ ] I2C bus communication
- [ ] VL53L0X distance readings
- [ ] MPU6050 orientation data
- [ ] Sound sensor triggering

### Communication Tests

- [ ] WiFi connection established
- [ ] ESP-NOW peer discovery
- [ ] Serial monitor output clear

---

**Hardware pin mapping updated for actual assembled robot configuration.** 🔌🤖
