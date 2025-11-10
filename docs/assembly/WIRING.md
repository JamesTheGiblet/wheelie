# Wheelie Robot Wiring Guide

## ESP32 Pin Connections

*Using ESP32 Development Board with Type-C USB, CH340C chip, 30-pin configuration*
*Optional: ESP32 Breakout Board for easier screw terminal connections*

### Connection Methods

- **Direct**: Use jumper wires from ESP32 pins to components
- **Breakout Board**: Use screw terminals on expansion board for secure connections

### Power System (XL4015 Buck Converter)

```txt
Battery  →  XL4015 Input
Battery+ →  IN+ (Input Positive)
Battery- →  IN- (Input Negative)

XL4015   →  ESP32 Power
OUT+     →  ESP32 VIN (set to 5V)
OUT-     →  ESP32 GND

XL4015   →  Motor Driver
Battery+ →  MOSFET H-Bridge VM (direct from battery)
Battery- →  MOSFET H-Bridge GND (common ground)
```

### Motor Driver (MOSFET H-Bridge, TB6612FNG or similar)

```txt
ESP32    →  MOSFET H-Bridge
GPIO 25  →  PWMA (Left Motor Speed)
GPIO 23  →  AIN1 (Left Motor Direction 1)
GPIO 22  →  AIN2 (Left Motor Direction 2)
GPIO 14  →  PWMB (Right Motor Speed)
GPIO 19  →  BIN1 (Right Motor Direction 1)
GPIO 18  →  BIN2 (Right Motor Direction 2)
5V       →  VCC
GND      →  GND
A01      →  Left Motor +
A02      →  Left Motor -
B01      →  Right Motor +
B02      →  Right Motor -
```

### Sensors

```txt
ESP32    →  Sensor
GPIO 26  →  VL53L0X GY-VL53L0XV2 SDA (I2C Data)
GPIO 27  →  VL53L0X GY-VL53L0XV2 SCL (I2C Clock)
GPIO 26  →  MPU6050 SDA (I2C Data)
GPIO 27  →  MPU6050 SCL (I2C Clock)
GPIO 34  →  Edge Sensor (Analog Input)
GPIO 17  →  H-1-0332 Sound Sensor OUT (Digital)
GPIO 32  →  Left LM393 H2010 Encoder OUT (Digital)
GPIO 33  →  Right LM393 H2010 Encoder OUT (Digital)
GPIO 35  →  PIR Motion Sensor (Digital Input) - Currently disconnected
3.3V     →  All sensor VCC pins
GND      →  All sensor GND pins
```

### Indicators & Feedback

```txt
ESP32    →  Component
GPIO 15  →  KY-009 Red LED (R pin)
GPIO 2   →  KY-009 Green LED (G pin)  
GPIO 4   →  KY-009 Blue LED (B pin)
GPIO 21  →  KY-006 Buzzer Signal (S pin)
3.3V     →  KY-006 Buzzer VCC
GND      →  All component GND pins

Battery Capacity Indicator (Optional):
Battery+ →  VCC (3.7V Li-Po positive)
Battery- →  GND (3.7V Li-Po negative)
B+       →  Battery positive terminal
B-       →  Battery negative terminal
Note: This module is independent and doesn't connect to ESP32
```

## Power Supply

**Primary Power System (using XL4015 Buck Converter):**

- Battery: 2x 3.7V 2000mAh Li-Po (903042) in series = 7.4V 2000mAh
- XL4015 Input: Direct battery connection (7.4V)
- XL4015 Output: 5V regulated → ESP32 VIN
- Motors: Direct battery connection via MOSFET H-Bridge (7.4V)
- ESP32 Internal: 3.3V → Sensors and logic

**Power Distribution:**

- High efficiency (96%) power conversion
- Stable 5V supply regardless of battery voltage
- Motors get full battery voltage for maximum performance
- Clean power for sensitive electronics
- KY-009 LED: 5V (built-in current limiting)

## Safety Features

- Tilt detection via MPU6050 GY-521 (30° threshold using 6-axis IMU data)
- Edge detection via edge sensor
- Obstacle avoidance via VL53L0X GY-VL53L0XV2 (20cm threshold using 940nm laser)
- Emergency stop on any safety trigger

## Troubleshooting

1. **No sensor readings**: Check I2C connections and power
2. **Motors not moving**: Verify MOSFET H-Bridge connections and power supply
3. **Random resets**: Check power supply capacity
4. **Build errors**: Ensure all libraries are installed via PlatformIO

---

*Legacy/Alternative: L298N/Fasizi wiring and troubleshooting available in [FASIZI_L298N_GUIDE.md](../components/FASIZI_L298N_GUIDE.md) if needed for older builds.*
