# Fasizi L298N Motor Driver Guide

## Overview

The **Fasizi Dual Channel L298N PWM Speed DC Motor Driver Board** is an ultra-compact motor controller designed for battery-powered robots and smart cars.

## Specifications

- **Brand**: Fasizi
- **Supply Voltage**: 2V - 10V
- **Weight**: 5 grams (ultra-lightweight)
- **Form Factor**: Ultra-small size for easy assembly
- **UPC**: 798339964994

## Key Features

### Dual H-Bridge Design

- Can drive **2x DC motors** simultaneously
- Can drive **1x 4-wire 2-phase stepper motor**
- Independent control for each motor channel

### Built-in Protection

- **TSD (Thermal Shutdown)** protection
- No worry about motor stall conditions
- Prevents damage from overheating

### Performance Characteristics

- **Forward/reverse rotation** control
- **Adjustable speed** via PWM
- **Zero standby current** - ideal for battery operation
- Wide voltage range compatibility

## Wiring Connections

### ESP32 to Fasizi L298N

```txt
ESP32 Pin    →  L298N Pin     Function
GPIO 32      →  IN1          Motor A Direction 1
GPIO 33      →  IN2          Motor A Direction 2
GPIO 25      →  ENA          Motor A Speed (PWM)
GPIO 26      →  IN3          Motor B Direction 1
GPIO 27      →  IN4          Motor B Direction 2
GPIO 14      →  ENB          Motor B Speed (PWM)
GND          →  GND          Ground
```

### Power Connections

```txt
Battery +    →  VCC          Motor power supply (6V-10V)
Battery -    →  GND          Ground
ESP32 5V     →  +5V          Logic power (if needed)
```

## Motor Connections

### Left Motor (Motor A)

```txt
Motor A+     →  OUT1
Motor A-     →  OUT2
```

### Right Motor (Motor B)

```txt
Motor B+     →  OUT3
Motor B-     →  OUT4
```

## Programming Notes

### Motor Control Functions

The code uses these functions for motor control:

- `ledcSetup()` - Configure PWM channels
- `ledcAttachPin()` - Attach PWM to GPIO pins
- `ledcWrite()` - Set motor speed (0-255)
- `digitalWrite()` - Set motor direction

### Speed Control

- **PWM Range**: 0-255
- **0**: Motor stopped
- **255**: Maximum speed
- **128**: Approximately 50% speed

### Direction Control

```cpp
// Forward
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);

// Reverse  
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);

// Stop
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
```

## Advantages of Fasizi L298N

### Size & Weight

- **Ultra-compact**: 5g weight vs standard L298N modules
- **Space-efficient**: Fits easily in small robot chassis
- **Minimal footprint**: More room for other components

### Power Efficiency

- **Zero standby current**: Extends battery life
- **Wide voltage range**: Works with various battery types
- **Thermal protection**: Prevents damage and maintains performance

### Ease of Use

- **Drop-in replacement**: Compatible with standard L298N code
- **No external components**: Built-in driving circuits
- **Reliable operation**: Thermal shutdown prevents failures

## Troubleshooting

### Motor Not Moving

- Check power supply voltage (must be 2V-10V)
- Verify wiring connections
- Ensure PWM signals are active
- Check motor connections for shorts

### Overheating

- Reduce motor load
- Check for motor stall conditions
- Verify adequate ventilation
- Consider adding heat sink if needed

### Erratic Behavior

- Check ground connections
- Verify stable power supply
- Ensure proper PWM frequency (1000-5000 Hz recommended)

## Mounting Recommendations

### Placement

- Mount away from heat sources
- Ensure airflow around module
- Secure with double-sided tape or small screws
- Keep wires short to minimize noise

### Orientation

- Component side up for heat dissipation
- Easy access to terminal connections
- Clear view of any status indicators

## Comparison with Alternatives

| Feature | Fasizi L298N | Standard L298N | TB6612FNG |
|---------|--------------|----------------|-----------|
| Weight | 5g | 15-20g | 4g |
| Voltage | 2V-10V | 5V-35V | 2.7V-5.5V |
| Standby Current | 0mA | 10-20mA | 1mA |
| Thermal Protection | Yes | Basic | Yes |
| Size | Ultra-compact | Large | Small |

## Best Practices

### Power Management

- Use regulated 6V-9V supply for optimal performance
- Add capacitors for noise reduction if needed
- Monitor current draw during operation

### Code Optimization

- Use appropriate PWM frequency (1-5 kHz)
- Implement soft start/stop for motors
- Add current limiting in software if needed

### Maintenance

- Check connections periodically
- Clean terminals if oxidation occurs
- Monitor temperature during extended operation

---

**Perfect for**: Battery-powered robots, smart cars, autonomous vehicles, space-constrained projects

**Key Advantage**: The Fasizi L298N offers the same functionality as larger motor drivers in an ultra-compact, lightweight package with zero standby current consumption.
