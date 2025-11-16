# USB-C 2S Li-Po Charger Module Guide

## Overview

The **Multi-Cell 2S Step-Up Boost Li-Po Charger Board** with USB-C input provides a convenient and safe way to charge your 2x 3.7V Li-Po batteries in series (2S configuration, 7.4V pack) for your autonomous robot.

## Specifications

- **Model**: Multi-Cell 2S/3S/4S Step-Up Boost Charger
- **Configuration Used**: 2S (7.4V nominal, 8.4V full charge)
- **Input**: USB-C 5V (from any USB-C power source)
- **Output**: 8.4V (4.2V per cell for 2S Li-Po)
- **Charging Current**: 1A for 2S configuration
- **Battery Compatibility**: 7.4V Li-Po, 18650 lithium cells

## Key Features

### USB-C Convenience

- **Universal Input**: Works with any USB-C charger, power bank, or computer
- **No Special Equipment**: No need for dedicated Li-Po balance chargers
- **Portable Charging**: Charge anywhere with USB-C power available
- **Safe Connector**: Reversible USB-C prevents incorrect connections

### Step-Up Boost Technology

- **5V to 8.4V Conversion**: Efficiently boosts USB 5V to required 8.4V
- **High Efficiency**: Minimal power loss during charging process
- **Regulated Output**: Stable 8.4V output regardless of USB input variations
- **Current Limiting**: Built-in 1A current limit for safe charging

### Automatic Protection

- **Overcharge Protection**: Automatically stops at 4.2V per cell
- **Cell Balancing**: Ensures both cells charge evenly
- **Temperature Monitoring**: Prevents charging if batteries get too hot
- **Reverse Polarity Protection**: Prevents damage from incorrect connections

## Wiring & Connections

### Charger Module Connections

```txt
[USB-C Power Source] → [2S Charger Module] → [2S Li-Po Battery Pack]
                            │
                            └── LED Status Indicators
```

### Battery Pack Connection

```txt
2S Li-Po Pack (standard robot configuration):
Cell 1: 3.7V ────┐
               ├── 7.4V Total → Charger Input
Cell 2: 3.7V ────┘

Charger Output: 8.4V (4.2V per cell when full)
```

*Note: Always disconnect the battery pack from the robot before charging to avoid damage to electronics or charger.*

### Physical Setup

1. **Connect Li-Po pack** to charger module output terminals
2. **Verify polarity**: Red (+) to positive, Black (-) to negative
3. **Plug USB-C cable** into charger module
4. **Connect power source**: USB-C charger, power bank, or computer USB-C port

## Charging Process

### LED Status Indicators

- **Red LED**: Charging in progress
- **Green LED**: Charging complete (8.4V reached)
- **Blinking Red**: Fault condition or battery error
- **No LED**: No power or no battery connected

### Charging Stages

1. **Pre-Charge (3.0V-3.7V)**: Low current to safely wake up batteries
2. **Constant Current (3.7V-4.1V)**: 1A charging current per specifications
3. **Constant Voltage (4.1V-4.2V)**: Tapered current to reach full charge
4. **Charge Complete (4.2V)**: Automatic termination and green LED

### Charging Time Estimation

- **2000mAh Batteries**: ~2-3 hours for full charge
- **From 50%**: ~1-1.5 hours
- **Top-off Charge**: ~30 minutes if nearly full
- **Safety Time**: 4-hour maximum charge time cutoff

## Safety Features

### Built-in Protections

- **Overvoltage Protection**: Prevents charging above 4.2V per cell
- **Overcurrent Protection**: Limits charging current to 1A
- **Thermal Protection**: Reduces current if module gets too hot
- **Input Protection**: Guards against USB power fluctuations

### User Safety Guidelines

1. **Never leave unattended**: Monitor charging process
2. **Stable surface**: Place on non-flammable surface during charging
3. **Ventilation**: Ensure good airflow around batteries and charger
4. **Temperature monitoring**: Stop if batteries become warm (>40°C)
5. **Inspect regularly**: Check for damage before each charge

## Integration with Robot System

### Charging Station Setup

```txt
Robot Power System:
┌─────────────────┐    ┌──────────────────┐
│ 2S Li-Po Pack   │────│ Robot Electronics│
│ (7.4V 2000mAh)  │    │ (XL4015, ESP32,  │
└─────────────────┘    │  Motors, etc.)   │
         │              └──────────────────┘
         │
         ▼ (For Charging)
┌─────────────────┐    ┌──────────────────┐
│ USB-C 2S        │────│ USB-C Power      │
│ Charger Module  │    │ Source (5V)      │
└─────────────────┘    └──────────────────┘
```

### Charging Workflow

1. **Power down robot**: Turn off ESP32 and motor systems
2. **Disconnect main power**: Remove Li-Po pack from robot
3. **Connect to charger**: Attach pack to charging module
4. **Begin charging**: Plug in USB-C power source
5. **Monitor progress**: Watch LED indicators
6. **Charging complete**: Green LED indicates full charge
7. **Reconnect to robot**: Install charged pack back in robot

## Troubleshooting

### Common Issues

#### No Charging (No LEDs)

- **Check USB-C power**: Verify 5V supply is working
- **Battery voltage**: Ensure pack voltage >2.5V per cell
- **Connection polarity**: Verify red(+) and black(-) connections
- **Module fault**: Try different USB-C power source

#### Slow Charging

- **USB power capacity**: Use 2A+ capable USB-C charger
- **Cable quality**: Use high-quality USB-C cable
- **Temperature**: Cool batteries charge faster than warm ones
- **Battery age**: Older batteries may charge more slowly

#### Won't Reach Full Charge

- **Cell imbalance**: Individual cells may have different voltages
- **Battery wear**: Capacity decreases over charge cycles
- **Temperature cutoff**: Hot batteries may not reach full charge
- **Time limit**: 4-hour safety cutoff may activate

#### Blinking Red LED

- **Overvoltage**: Battery voltage too high (>4.3V per cell)
- **Undervoltage**: Battery voltage too low (<2.5V per cell)
- **Temperature**: Batteries too hot or too cold
- **Connection fault**: Poor or intermittent connections

## Maintenance & Care

### Charger Module Care

- **Keep dry**: Avoid moisture and liquid exposure
- **Clean connections**: Wipe terminals with alcohol occasionally
- **Storage**: Store in anti-static bag when not in use
- **Cable care**: Avoid bending USB-C connector

### Battery Integration

- **Regular charging**: Don't let batteries sit discharged
- **Balanced usage**: Rotate between charge cycles if multiple packs
- **Storage voltage**: Charge to ~50% if storing >1 week
- **Temperature monitoring**: Check battery temperature during charging

## Performance Optimization

### Charging Efficiency

- **Quality power source**: Use 2A+ USB-C chargers for best performance
- **Cool environment**: Charge in room temperature or cooler
- **Good connections**: Ensure tight, clean terminal connections
- **Proper ventilation**: Allow airflow around batteries and charger

### Battery Longevity

- **Avoid deep discharge**: Don't let batteries go below 3.0V per cell
- **Partial charging**: Top-off charges are easier on batteries
- **Temperature management**: Keep batteries cool during charging
- **Regular use**: Don't let batteries sit unused for months

## Technical Specifications

| Parameter | Specification | Notes |
|-----------|--------------|-------|
| Input Voltage | 5V USB-C | Standard USB-C power |
| Output Voltage | 8.4V (2S) | 4.2V per cell |
| Charging Current | 1A | For 2S configuration |
| Efficiency | >85% | Step-up conversion |
| Protection | Multi-level | OV, OC, thermal, reverse |
| LED Indicators | Red/Green | Charging/Complete status |
| Operating Temp | 0°C to 45°C | Charging temperature range |
| Battery Types | Li-Po, Li-Ion | 3.7V nominal cells |

## Advantages vs Traditional Chargers

### Convenience

- **No dedicated charger needed**: Uses common USB-C power
- **Portable**: Charge anywhere with USB power available
- **Simple operation**: Just plug in and charge
- **Universal compatibility**: Works with phone chargers, power banks

### Safety

- **Automatic termination**: Prevents overcharging
- **Built-in protections**: Multiple safety systems
- **LED feedback**: Clear charging status indication
- **Temperature monitoring**: Prevents thermal runaway

### Cost Effective

- **Lower cost**: Cheaper than dedicated Li-Po balance chargers
- **Multi-purpose**: Can charge different cell configurations
- **No additional equipment**: Uses existing USB-C infrastructure
- **Replacement friendly**: Easy to replace if damaged

## Battery Pack Compatibility

### Confirmed Compatible

- **2S Li-Po**: 7.4V nominal (your configuration)
- **2S Li-Ion**: 7.4V 18650 cells in series
- **Single Li-Po**: 3.7V individual cells (using 1S mode)

### Power Source Requirements

- **Minimum**: 5V 1A USB-C source
- **Recommended**: 5V 2A+ USB-C charger
- **Compatible**: Phone chargers, laptops, power banks
- **Avoid**: Low-power sources (<1A capability)

---

**Perfect for**: Autonomous robots, portable electronics, RC vehicles with 2S Li-Po power systems

**Key Advantage**: The USB-C 2S charger module provides convenient, safe, and automatic charging for your robot's battery pack using common USB-C power sources, eliminating the need for specialized Li-Po charging equipment.
