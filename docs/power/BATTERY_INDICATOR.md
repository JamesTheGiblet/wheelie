# Battery Capacity Indicator Module - Quick Reference

## Overview

The **Lithium Battery Capacity Indicator Module** provides real-time visual feedback of your robot's battery charge level using a 4-segment blue LED display.

## Specifications

- **Input Voltage**: 3.0V - 4.2V (optimized for single-cell Li-Po)
- **Display**: 4 blue LED segments
- **Wire**: Pre-installed 26AWG wire leads
- **Dimensions**: ~40mm x 13mm x 8mm
- **Current Draw**: <5mA
- **Response Time**: Real-time voltage monitoring

## LED Indicators

```txt
Battery Voltage    LED Display    Charge Level
4.0V - 4.2V       ████ (4/4)     75-100% Full
3.7V - 4.0V       ███  (3/4)     50-75%  Good  
3.4V - 3.7V       ██   (2/4)     25-50%  Medium
3.0V - 3.4V       █    (1/4)     0-25%   Low
<3.0V             ○    (0/4)     Critical - Stop Use!
```

## Installation

### Wiring Connections

```txt
Module Terminal → Connection
VCC            → Battery Positive (+)
GND            → Battery Ground (-)
B+             → Battery Positive (+) [same as VCC]
B-             → Battery Ground (-) [same as GND]
```

### Physical Mounting

1. **Location**: Mount where LEDs are easily visible during operation
2. **Orientation**: LEDs facing upward or toward operator position
3. **Mounting**: Use double-sided tape or small screws through mounting holes
4. **Wire Management**: Route wires away from moving parts and sensors

## Benefits for Robot Applications

### Battery Management

- **Prevents Over-discharge**: Visual warning before battery damage
- **Extended Battery Life**: Avoid deep discharge cycles
- **Predictable Runtime**: Know when to recharge or replace batteries

### Operational Advantages

- **Real-time Monitoring**: No need to check battery with multimeter
- **Autonomous Operation**: Can program robot to return home when low
- **Safety**: Prevents unexpected shutdowns during operation

### Troubleshooting Aid

- **Power Issues**: Quickly identify if problems are battery-related
- **Performance Degradation**: Low battery affects motor performance
- **System Diagnosis**: Rule out power supply issues

## Usage Tips

### Best Practices

- **Check Before Operation**: Always verify charge level before use
- **Monitor During Use**: Glance at indicator during longer runs
- **Recharge Timing**: Recharge when 2 LEDs or fewer are lit
- **Storage**: Store Li-Po batteries at ~3.7V (50% charge)

### Warning Signs

- **Single LED**: Limit operation time, prepare to recharge
- **No LEDs**: Stop operation immediately to prevent battery damage
- **Flickering**: May indicate loose connections or failing battery

## Li-Po Battery Safety

### Voltage Ranges

- **Fully Charged**: 4.2V per cell
- **Nominal**: 3.7V per cell  
- **Low**: 3.4V per cell
- **Critical**: 3.0V per cell (stop use immediately)
- **Damage Threshold**: <3.0V per cell

### Safety Precautions

- Never discharge Li-Po below 3.0V per cell
- Use proper Li-Po charger with balancing
- Store in fireproof container
- Monitor temperature during charging/use
- Dispose of damaged batteries properly

## Integration with Robot Code

### Analog Monitoring (Optional)

While the indicator module works independently, you can also monitor battery voltage through the ESP32:

```cpp
// Add to your robot code for battery monitoring
#define BATTERY_PIN 35  // Analog pin for voltage divider
#define LOW_BATTERY_THRESHOLD 3.4  // Volts

void checkBatteryLevel() {
  float voltage = analogRead(BATTERY_PIN) * (3.3 / 4095.0) * 2; // Voltage divider
  
  if (voltage < LOW_BATTERY_THRESHOLD) {
    Serial.println("⚠️ Low Battery - Returning to base!");
    // Implement low battery behavior
    returnToBase();
  }
}
```

### Voltage Divider Circuit (if using ESP32 monitoring)

```txt
Battery+ ─── 10kΩ ─── ESP32 GPIO35
                 │
               10kΩ
                 │
Battery- ─── GND ─── ESP32 GND
```

## Troubleshooting

### No LED Display

- Check battery voltage with multimeter
- Verify all connections are secure
- Ensure battery is above 3.0V

### Incorrect Reading

- Clean battery terminals
- Check for loose connections
- Verify module is rated for your battery voltage

### Intermittent Display

- Secure all wire connections
- Check for damaged wires
- Test with known good battery

## Alternatives and Upgrades

### Basic Alternatives

- **Simple Voltage Divider**: Use ESP32 ADC + code for monitoring
- **Multimeter**: Manual checking (less convenient)
- **Battery Alarm**: Audible low-voltage warning

### Advanced Options

- **LCD Display**: Show exact voltage numbers
- **Wireless Monitoring**: Send battery status to phone/computer
- **Data Logging**: Record battery performance over time

---

**Note**: This module is designed for single-cell Li-Po batteries (3.7V nominal). For different battery types or multi-cell packs, verify compatibility or use appropriate voltage dividers.
