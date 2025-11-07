# Li-Po Battery Pack Guide (2x 3.7V 2000mAh in Series)

## Overview

Using **2x 3.7V 2000mAh Lithium Polymer batteries (903042)** in series configuration creates a **7.4V 2000mAh battery pack** that provides excellent power characteristics for autonomous robots.

## Battery Specifications

### Individual Battery (903042)

- **Model**: 903042 Li-Po Battery
- **Nominal Voltage**: 3.7V
- **Capacity**: 2000mAh
- **Chemistry**: Lithium Polymer (LiPo)
- **Dimensions**: Approximately 90mm x 30mm x 42mm
- **Weight**: ~40-50g each

### Series Configuration

- **Total Voltage**: 7.4V (3.7V × 2)
- **Total Capacity**: 2000mAh (same as individual)
- **Total Energy**: 14.8Wh (7.4V × 2Ah)
- **Total Weight**: ~80-100g for battery pack

## Series Connection Wiring

### Safe Series Connection

```txt
Battery 1:  [+] ────────── [Robot Power +]
            [-] ──────┐
                      │
Battery 2:  [+] ──────┘
            [-] ────────── [Robot Power -]
```

### Connection Steps

1. **Connect Battery 1 negative** to **Battery 2 positive**
2. **Battery 1 positive** becomes **Pack positive (+)**
3. **Battery 2 negative** becomes **Pack negative (-)**
4. **Result**: 7.4V across pack terminals

## Power Distribution System

### Dual-Rail Power Design

```txt
8.4V Li-Po Pack (2x 4.2V cells, 4000mAh total)
      │
      ├── Direct to MOS-FET Motor Driver (8.4V for motors)
      │
      ├── Battery Monitor (LED display)
      │
      └── XL4015 Buck Converter
          └── 5V Output to ESP32 VIN
              ├── ESP32 Internal: 5V → 3.3V for sensors
              └── ESP32 5V Pin → Encoders (LM393 H2010)
```

### Power Requirements

- **Motors (via MOS-FET driver)**: 8.4V direct (maximum torque)
- **ESP32 Controller**: 5V regulated (from XL4015)
- **Sensors**: 3.3V (from ESP32 internal regulator)
- **Encoders**: 5V (from ESP32 5V pin)
- **Battery Monitor**: 8.4V direct (independent operation)
- **Total Current**: ~1-4A peak (motors + electronics)

## Advantages of 7.4V Li-Po Pack

### Motor Performance

- **Higher Torque**: 7.4V provides more power than 6V AA batteries
- **Better Speed**: Motors run faster at optimal voltage
- **Consistent Performance**: Li-Po maintains voltage under load
- **No Voltage Drop**: Unlike AA batteries, Li-Po voltage stays stable

### Runtime & Capacity

- **2000mAh Capacity**: Significantly more than AA batteries (~1500mAh)
- **Lower Self-Discharge**: Li-Po retains charge longer when stored
- **Rechargeable**: No need to replace disposable batteries
- **Flat Discharge Curve**: Consistent performance until nearly depleted

### Weight & Form Factor

- **Lighter**: Li-Po pack weighs less than 6x AA batteries
- **Compact**: Easier to mount in robot chassis
- **Lower Center of Gravity**: Better robot stability
- **Professional Appearance**: Cleaner installation

## Safety Considerations

### Li-Po Safety Rules

1. **Never Overcharge**: Use proper Li-Po charger only
2. **Monitor Voltage**: Don't discharge below 3.0V per cell (6.0V total)
3. **Balanced Charging**: Use balancer for series cells
4. **Storage**: Store at 3.7-3.8V per cell if unused > 1 week
5. **Temperature**: Avoid extreme heat or cold
6. **Physical Damage**: Don't puncture, crush, or bend

### Protection Recommendations

- **Battery Monitoring**: Use capacity indicator module
- **Voltage Cutoff**: Implement low-voltage protection in code
- **Fusing**: Add 5A fuse for overcurrent protection
- **Secure Mounting**: Prevent movement during robot operation

## Charging System

### USB-C 2S Li-Po Charger Module

- **Type**: Multi-cell step-up boost Li-Po charger board
- **Input**: USB-C (5V from any USB-C power source)
- **Output**: 8.4V (4.2V per cell for 2S configuration)
- **Charging Current**: 1A for 2S batteries
- **Features**: Automatic cell balancing and charge termination
- **Input**: AC 100-240V or DC 12V
- **Output**: 7.4V (2S) with balancing
- **Features**: Automatic cell balancing and charge termination

### Charging Procedure

1. **Connect charger module** to 2S Li-Po battery pack
2. **Plug USB-C cable** into charger module
3. **Connect to power source**: Any USB-C charger, power bank, or computer
4. **Monitor LED indicators**: Charging status and completion signals
5. **Automatic termination**: Stops at 8.4V (4.2V per cell) when full
6. **Charging time**: Approximately 2-3 hours for 2000mAh batteries

### Charging Safety

- **Fire-Safe Location**: Charge in metal container or fire-safe bag
- **Stable Surface**: Avoid carpet, fabric, or flammable surfaces
- **Ventilation**: Ensure good airflow around batteries
- **Supervision**: Monitor charging process regularly

## Integration with Robot Systems

### XL4015 Buck Converter Setup

```txt
Li-Po Pack (7.4V) → XL4015 Input
XL4015 Output (5V) → ESP32 VIN
ESP32 3.3V → All sensors
```

**Adjustment Procedure:**

1. Connect 7.4V Li-Po to XL4015 input
2. Adjust potentiometer to 5.0V output
3. Connect to ESP32 VIN pin
4. Verify 3.3V available on ESP32 3.3V pin

### Motor Driver Connection

```txt
Li-Po Pack (7.4V) → L298N VCC
L298N Motor Outputs → TT Motors
```

**Benefits:**

- Full 7.4V delivered to motors
- Maximum torque and speed available
- Efficient power transfer (no regulation loss)

### Battery Monitoring

```txt
Li-Po Pack + → Voltage Divider → ESP32 GPIO (ADC)
```

**Voltage Monitoring Code:**

```cpp
// Voltage divider: 10kΩ and 4.7kΩ (for 7.4V → 3.3V max)
const float VOLTAGE_DIVIDER_RATIO = (10.0 + 4.7) / 4.7; // 3.13
const float ADC_MAX = 4095.0; // ESP32 12-bit ADC
const float ADC_VOLTAGE = 3.3; // ESP32 ADC reference

float readBatteryVoltage() {
  int adcValue = analogRead(35); // GPIO 35 for voltage monitoring
  float voltage = (adcValue * ADC_VOLTAGE / ADC_MAX) * VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

void checkBatteryLevel() {
  float voltage = readBatteryVoltage();
  
  if (voltage < 6.0) {
    // Critical: Stop robot and indicate low battery
    Serial.println("CRITICAL: Battery voltage too low!");
    // Implement emergency shutdown
  } else if (voltage < 6.8) {
    // Warning: Battery getting low
    Serial.println("WARNING: Battery voltage low");
    // Flash LED or sound alarm
  }
}
```

## Runtime Estimation

### Power Consumption Analysis

- **ESP32 + Sensors**: ~200-300mA
- **Motors (moving)**: ~500-1500mA total
- **Motors (stopped)**: ~50mA (standby)
- **Peak Current**: ~2000mA (acceleration, climbing)

### Runtime Calculations

```txt
Battery Capacity: 4000mAh (2x 2000mAh in series)
Average Current: ~800mA (mixed operation)
Estimated Runtime: 4000mAh ÷ 800mA = 5 hours

Conservative Runtime: 4 hours (accounting for voltage drop)
Aggressive Operation: 3 hours (high motor usage)
Light Operation: 6+ hours (mostly stationary with sensors)
```

## Maintenance & Care

### Daily Operation

- **Check voltage** before each use
- **Inspect connections** for corrosion or looseness
- **Monitor temperature** during operation
- **Secure mounting** to prevent damage

### Weekly Maintenance

- **Full charge cycle** if batteries were used
- **Visual inspection** for swelling or damage
- **Connection cleaning** with contact cleaner
- **Storage charge** if not using for >1 week

### Long-term Storage

- **Charge to 3.7-3.8V per cell** (storage voltage)
- **Store in cool, dry location** (room temperature)
- **Check voltage monthly** and adjust if needed
- **Avoid extreme temperatures** (<0°C or >40°C)

## Troubleshooting

### Common Issues

#### Robot Won't Start

- **Check battery voltage**: Should be >7.0V
- **Verify connections**: Ensure tight, clean connections
- **Test continuity**: Check for broken wires
- **Fuse check**: Verify protection fuse isn't blown

#### Short Runtime

- **Battery age**: Li-Po capacity decreases over time
- **High current draw**: Check for motor binding or overload
- **Voltage drop**: Weak connections cause inefficiency
- **Cold weather**: Battery capacity reduced in cold

#### Charging Problems

- **Balance issues**: Cells may be unbalanced
- **Charger settings**: Verify 2S Li-Po mode selected
- **Connection problems**: Check charger leads
- **Battery damage**: Swollen or damaged cells won't charge

## Specifications Summary

| Parameter | Value | Notes |
|-----------|-------|-------|
| Configuration | 2S1P (2 in series) | 7.4V nominal |
| Capacity | 2000mAh | Same as individual cells |
| Energy | 14.8Wh | 7.4V × 2Ah |
| Discharge Rate | 1-3C | 2A-6A continuous |
| Charge Rate | 0.5-1C | 1A-2A recommended |
| Weight | 80-100g | Both batteries |
| Cycle Life | 300-500 cycles | With proper care |

## Advantages vs Alternatives

### vs 6x AA Batteries

- **Higher Voltage**: 7.4V vs 6V (fresh) / 4.8V (depleted)
- **More Capacity**: 2000mAh vs ~1500mAh
- **Lighter Weight**: ~90g vs ~150g
- **Rechargeable**: No replacement costs
- **Stable Voltage**: Maintains voltage under load

### vs Single 7.4V Battery

- **Redundancy**: If one cell fails, robot still partially functional
- **Easier Replacement**: Standard 3.7V cells widely available
- **Balanced Charging**: Easier to maintain cell balance
- **Cost Effective**: Two smaller batteries often cheaper

---

**Perfect for**: Autonomous robots, RC vehicles, portable electronics requiring 7.4V power

**Key Advantage**: The 2S Li-Po configuration provides optimal voltage and capacity for robot motors while maintaining excellent energy density and rechargeability.
