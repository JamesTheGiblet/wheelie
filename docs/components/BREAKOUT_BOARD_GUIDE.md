# ESP32 Breakout Board - Quick Reference

## Overview

The ESP32 Breakout/Expansion Board is a 30-pin interface board that provides easier access to all ESP32 GPIO pins through screw terminals and clearly labeled connections.

## Key Features

### Screw Terminal Connections

- **Secure Mounting**: Screws hold wires firmly in place
- **No Soldering**: Easy to modify connections without tools
- **Reusable**: Wires can be easily changed or moved
- **Vibration Resistant**: Won't come loose during robot movement

### Clear Labeling

- **GPIO Numbers**: Each pin clearly marked with GPIO number
- **Function Labels**: Power, Ground, and special function pins labeled
- **Color Coding**: Different sections for power, GPIO, and analog pins
- **Pin Grouping**: Logical arrangement for easy identification

### Power Distribution

- **Multiple 3.3V**: Several 3.3V terminals for sensors
- **Multiple 5V**: 5V terminals when powered via USB
- **Ground Rails**: Multiple GND connections
- **Power Indicators**: LEDs show power status

## Physical Specifications

### Dimensions

- **Length**: ~85mm
- **Width**: ~55mm  
- **Height**: ~15mm (with ESP32 inserted)
- **Mounting**: 4x mounting holes (M3 compatible)

### Pin Configuration

- **30-Pin Socket**: Accepts standard 30-pin ESP32 boards
- **Dual Row**: Maintains ESP32's dual-row pin layout
- **Secure Fit**: Spring-loaded contacts ensure good connection
- **Access**: USB-C port remains accessible for programming

## Connection Guide for Wheelie Robot

### Motor Control (L298N)

```txt
Breakout Terminal → L298N Pin
GPIO25           → ENA (Motor A Speed)
GPIO23           → IN1 (Motor A Direction)
GPIO22           → IN2 (Motor A Direction)
GPIO14           → ENB (Motor B Speed)
GPIO19           → IN3 (Motor B Direction)
GPIO18           → IN4 (Motor B Direction)
GND              → GND
```

### I2C Sensors (VL53L0X + MPU6050)

```txt
Breakout Terminal → Sensor Connection
GPIO26           → SDA (both sensors)
GPIO27           → SCL (both sensors)
3.3V             → VCC (both sensors)
GND              → GND (both sensors)
```

### Individual Sensors

```txt
Edge Sensor:
GPIO34 → Signal
3.3V   → VCC
GND    → GND

Sound Sensor:
GPIO17 → Digital Out
3.3V   → VCC
GND    → GND

PIR Sensor (Optional):
GPIO35 → Signal
3.3V   → VCC
GND    → GND
```

### Status Indicators

```txt
RGB LED:
GPIO15 → Red (through 220Ω resistor)
GPIO2  → Green (through 220Ω resistor)
GPIO4  → Blue (through 220Ω resistor)
GND    → Common Cathode

Buzzer:
GPIO21 → Buzzer Positive
GND    → Buzzer Negative
```

## Installation Steps

### Step 1: Mount Breakout Board

1. **Choose Location**: Center of robot chassis for easy access
2. **Mark Holes**: Use breakout board as template for mounting holes
3. **Drill Holes**: 3mm holes for M3 screws
4. **Secure Board**: Use M3 screws and nuts or standoffs
5. **Check Clearance**: Ensure no interference with other components

### Step 2: Install ESP32

1. **Align Pins**: Carefully line up ESP32 with breakout socket
2. **Press Down**: Firmly but gently press ESP32 into socket
3. **Check Seating**: Ensure all pins make contact
4. **Verify USB**: Confirm Type-C port is accessible
5. **Test Power**: Connect USB and verify power LEDs

### Step 3: Wire Connections

1. **Strip Wires**: 6-8mm of insulation for screw terminals
2. **Insert Wires**: Place wire in terminal before tightening
3. **Tighten Screws**: Firm but not over-tight to avoid damage
4. **Test Connection**: Gently tug wires to verify secure connection
5. **Label Wires**: Use colored wires or labels for identification

## Advantages for Robot Projects

### Reliability Benefits

- **No Loose Connections**: Screw terminals eliminate intermittent contacts
- **Vibration Proof**: Secure mounting prevents connection failures
- **Long-term Stability**: Professional connections for permanent installations
- **Field Repairs**: Easy troubleshooting and wire replacement

### Development Benefits

- **Rapid Prototyping**: Quick connection changes during development
- **Clear Identification**: Labeled terminals reduce wiring errors
- **Multiple Ground Points**: Easier power distribution
- **Professional Appearance**: Clean, organized wiring layout

### Maintenance Benefits

- **Easy Access**: All connections visible and accessible
- **Quick Changes**: Modify wiring without desoldering
- **Troubleshooting**: Easy to isolate connection problems
- **Documentation**: Terminal labels make wiring diagrams easier

## Wiring Best Practices

### Wire Management

- **Use Different Colors**: Red=Power, Black=Ground, others=signals
- **Keep Wires Short**: Minimize wire length for reliability
- **Bundle Related Wires**: Group power, I2C, motor wires separately
- **Strain Relief**: Secure wire bundles to prevent stress on terminals

### Connection Quality

- **Proper Strip Length**: 6-8mm exposed wire for good contact
- **Clean Copper**: Ensure wire ends are clean and bright
- **Proper Torque**: Tight enough to secure, not so tight to damage
- **Double Check**: Verify all connections before powering on

### Safety Considerations

- **Power Off**: Always disconnect power when changing connections
- **Check Polarity**: Verify positive/negative before connecting power
- **Avoid Shorts**: Keep bare wires separated to prevent short circuits
- **Test Incrementally**: Add one connection at a time and test

## Troubleshooting

### Common Issues

#### ESP32 Not Recognized

- **Check Seating**: Ensure ESP32 is fully inserted in socket
- **Verify USB**: Confirm Type-C cable supports data transfer
- **Power Indicators**: Check that power LEDs are lit
- **Socket Damage**: Inspect socket pins for damage or corrosion

#### Intermittent Connections

- **Retighten Screws**: Check all terminal connections
- **Wire Condition**: Look for damaged or corroded wire ends
- **Terminal Wear**: Inspect screw terminals for wear
- **Vibration**: Secure breakout board mounting

#### Power Issues

- **Voltage Check**: Measure 3.3V and 5V outputs with multimeter
- **Current Load**: Ensure total sensor current < 600mA on 3.3V
- **Ground Loops**: Verify single ground reference point
- **Power Supply**: Check USB power source capability

### Testing Procedures

1. **Visual Inspection**: Check all connections and component mounting
2. **Continuity Test**: Use multimeter to verify connections
3. **Power Test**: Measure voltages at breakout terminals
4. **Signal Test**: Verify GPIO signals with oscilloscope/logic analyzer
5. **System Test**: Run robot diagnostics to verify all functions

## Alternatives and Upgrades

### Alternative Connection Methods

- **Solderable Breakout**: Permanent solder connections
- **IDC Connectors**: Ribbon cable connections for clean routing
- **PCB Design**: Custom PCB for specific robot requirements
- **Modular Connectors**: Use JST or similar connectors for modularity

### Future Upgrades

- **Additional Sensors**: Easy to add new sensors to spare terminals
- **PWM Expansion**: Add PWM modules for more servos/LEDs
- **Communication Modules**: Add wireless modules via I2C/SPI
- **Protection Circuits**: Add fuses or protection diodes

---

**The ESP32 Breakout Board transforms the development process from prototype to professional robot with reliable, maintainable connections.**
