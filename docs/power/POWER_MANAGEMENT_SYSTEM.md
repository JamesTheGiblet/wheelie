# Power Management System Guide

## 🔋 Overview

The Wheelie robot features an intelligent 5-level power management system that automatically adjusts performance based on battery voltage to maximize operational time and protect the battery from damage.

## ⚡ Power Modes

### 1. NORMAL Mode (7.8V - 8.4V)

- **Full Performance**: All systems running at maximum capability
- **Features Active**: All sensors, full motor speed, continuous data logging
- **LED Indicator**: Green - system operating normally
- **Typical Runtime**: 2-3 hours depending on activity level

### 2. ECONOMY Mode (7.2V - 7.8V)

- **Reduced Performance**: Non-critical features disabled
- **Features Active**: Core sensors, reduced motor speed, limited logging
- **LED Indicator**: Blue - economy mode active
- **Power Savings**: ~20% reduction in power consumption

### 3. LOW Mode (6.8V - 7.2V)

- **Essential Functions Only**: Critical safety systems maintained
- **Features Active**: Obstacle avoidance, basic movement, emergency stop
- **LED Indicator**: Yellow - low power warning
- **Power Savings**: ~40% reduction in power consumption

### 4. CRITICAL Mode (6.4V - 6.8V)

- **Survival Mode**: Minimal systems to return to base
- **Features Active**: Basic navigation, emergency communications
- **LED Indicator**: Red flashing - critical battery level
- **Behavior**: Attempts to return to charging station or safe location

### 5. SHUTDOWN Mode (<6.4V)

- **Battery Protection**: Automatic shutdown to prevent damage
- **Features Active**: None - system powers down safely
- **LED Indicator**: Red solid - system shutting down
- **Protection**: Prevents deep discharge damage to LiPo battery

## 🔌 Hardware Requirements

### Voltage Divider Circuit

The battery monitoring system uses a voltage divider to safely measure the 2S LiPo battery voltage (6.0V - 8.4V) using the ESP32's 3.3V ADC. The standard configuration uses 20kΩ and 10kΩ resistors, and GPIO 35 for analog input.

```txt
Battery + (8.4V max) ----[20kΩ]----+----[10kΩ]---- GND
                            |
                        ESP32 GPIO35 (ADC)
```

**Voltage Scaling:**

- Maximum battery voltage: 8.4V (fully charged)
- Voltage divider ratio: 10kΩ / (20kΩ + 10kΩ) = 0.3333
- Maximum ADC input: 8.4V × 0.3333 = 2.8V (safe for 3.3V ADC)
- Minimum battery voltage: 6.0V (empty)
- Minimum ADC input: 6.0V × 0.3333 = 2.0V

### Component List

| Component | Value | Tolerance | Power Rating |
|-----------|--------|-----------|--------------|
| R1 (High) | 20kΩ | ±1% | 1/4W |
| R2 (Low) | 10kΩ | ±1% | 1/4W |
| ADC Pin | GPIO35 | - | 3.3V max |

## 🔧 Software Implementation

### Battery Monitoring Structure

```cpp
typedef struct {
    float voltage;
    float percentage;
    PowerMode_t mode;
    uint32_t lastUpdate;
    bool isCharging;
    uint32_t lowVoltageStartTime;
} BatteryMonitor_t;
```

### Power Mode Enum

```cpp
typedef enum {
    POWER_NORMAL = 0,
    POWER_ECONOMY,
    POWER_LOW,
    POWER_CRITICAL,
    POWER_SHUTDOWN
} PowerMode_t;
```

### Key Functions

- `updateBatteryStatus()` - Reads voltage and updates power mode
- `managePowerMode()` - Adjusts system behavior based on current mode
- `enterLowPowerMode()` - Activates power saving features
- `checkBatteryHealth()` - Monitors for battery issues

## 📊 Performance Impact

### Feature Scaling by Power Mode

| Feature | Normal | Economy | Low | Critical | Shutdown |
|---------|--------|---------|-----|----------|----------|
| Motor Speed | 100% | 75% | 50% | 25% | 0% |
| Sensor Updates | 50ms | 100ms | 200ms | 500ms | OFF |
| Data Logging | Full | Reduced | Critical | None | None |
| LED Brightness | 100% | 75% | 50% | 25% | OFF |
| WiFi Activity | Full | Reduced | Emergency | OFF | OFF |
| ESP-NOW | Full | Reduced | Emergency | OFF | OFF |

### Estimated Runtime Extension

- **Economy Mode**: +25% runtime compared to normal
- **Low Mode**: +50% runtime compared to normal
- **Critical Mode**: +100% runtime for emergency return

## 🚨 Safety Features

### Battery Protection

1. **Deep Discharge Prevention**: Automatic shutdown at 6.4V
2. **Voltage Monitoring**: Continuous ADC readings with averaging
3. **Graceful Degradation**: Smooth transition between power modes
4. **Emergency Protocols**: Critical mode enables return-to-base behavior

### Error Handling

- **ADC Failure**: Defaults to CRITICAL mode for safety
- **Voltage Fluctuations**: 5-reading average prevents false triggers
- **Mode Hysteresis**: Prevents rapid mode switching
- **Low Voltage Persistence**: Requires 30 seconds in range before mode change

## 🔄 Calibration

### Voltage Calibration Constants

```cpp
#define BATTERY_VOLTAGE_DIVIDER_RATIO 0.3333f  // 10k / (20k + 10k)
#define BATTERY_ADC_REFERENCE 3.3f             // ESP32 ADC reference voltage
#define BATTERY_ADC_RESOLUTION 4095.0f         // 12-bit ADC resolution
```

### Calibration Procedure

1. **Measure Actual Resistor Values**: Use multimeter for precision
2. **Calculate Actual Ratio**: Real ratio = R2 / (R1 + R2)
3. **Test with Known Voltages**: Verify readings with multimeter
4. **Adjust Constants**: Update `BATTERY_VOLTAGE_DIVIDER_RATIO` if needed

### Verification Steps

```cpp
// Expected voltage reading calculation:
// ADC_Value = (Battery_Voltage * Divider_Ratio * 4095) / 3.3V
// For 7.4V battery: (7.4 * 0.3333 * 4095) / 3.3 = 3067 ADC counts
```

## 📈 Data Logging

The power management system logs the following data to CSV files:

- Battery voltage (V)
- Battery percentage (%)
- Current power mode
- Power mode change events
- Runtime in each mode
- Estimated remaining time

### Log File Format

```csv
timestamp,voltage,percentage,power_mode,runtime_hours,estimated_remaining
2025-01-15 10:30:15,7.84,89.2,NORMAL,1.25,2.8
2025-01-15 10:30:30,7.82,88.7,NORMAL,1.26,2.75
```

## 🔧 Troubleshooting

### Common Issues

#### Incorrect Voltage Readings

- Check resistor values with multimeter
- Verify connections to GPIO34
- Ensure common ground between battery and ESP32

#### Premature Low Power Mode

- Check battery condition and age
- Verify load current isn't excessive
- Monitor for voltage sags under load

#### Power Mode Not Changing

- Check hysteresis settings
- Verify 30-second persistence timer
- Monitor for ADC noise or fluctuations

#### System Shutting Down Too Early

- Adjust CRITICAL voltage threshold
- Check for voltage drops under motor load
- Verify battery capacity and health

### Diagnostic Commands

Monitor the serial output for power management debug information:

```txt
🔋 BATTERY STATUS: 7.42V (85%) - NORMAL MODE
⚡ POWER MANAGEMENT: All systems nominal
📊 RUNTIME: 2.3 hours, EST REMAINING: 0.8 hours
```

## 🎯 Best Practices

1. **Use Quality Resistors**: ±1% tolerance for accurate readings
2. **Secure Connections**: Loose connections cause voltage fluctuations
3. **Monitor Trends**: Log data to identify battery degradation
4. **Regular Calibration**: Verify voltage readings monthly
5. **Battery Maintenance**: Proper LiPo storage and charging practices
6. **Load Testing**: Verify readings under motor load conditions

## 🔮 Future Enhancements

- **Predictive Analytics**: Machine learning for runtime estimation
- **Adaptive Thresholds**: Dynamic voltage levels based on usage patterns
- **Charging Integration**: Automatic dock-and-charge capabilities
- **Remote Monitoring**: Web dashboard for battery status
- **Battery Health Tracking**: Long-term capacity and performance analysis
