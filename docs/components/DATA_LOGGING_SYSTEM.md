# Data Logging System Guide

## 📊 Overview

The Wheelie robot features a comprehensive data logging system built on the SPIFFS file system, providing structured CSV logging for operational data, performance metrics, system health, and debugging information. This enterprise-grade logging enables detailed analysis and troubleshooting.

## 🗂️ File System Architecture

### SPIFFS Configuration

- **Total Space**: ~1.5MB available for data storage
- **File Format**: CSV (Comma-Separated Values) for universal compatibility
- **Encoding**: UTF-8 with timestamp headers
- **Compression**: Optional GZIP compression for large files

### Directory Structure

```txt
/spiffs/
├── logs/
│   ├── system_health.csv      # System performance and health data
│   ├── sensor_data.csv        # All sensor readings with timestamps
│   ├── motor_control.csv      # Motor commands and performance
│   ├── power_management.csv   # Battery and power system data
│   ├── navigation.csv         # Movement and navigation decisions
│   └── errors.csv            # Error events and recovery actions
├── config/
│   ├── robot_config.json     # Robot configuration settings
│   └── calibration.json      # Sensor calibration data
└── cache/
    ├── performance_cache.dat  # Short-term performance data
    └── sensor_cache.dat      # Sensor reading buffer
```

## 📝 Logging Categories

### System Health Logging

Records comprehensive system performance metrics:

```csv
timestamp,free_heap,stack_usage,loop_freq,loop_time,cpu_usage,temperature,uptime_hours
2025-01-15 10:30:00,245760,45,18.5,52,67,42.3,2.5
2025-01-15 10:30:15,244832,46,18.2,54,68,42.8,2.51
2025-01-15 10:30:30,243904,47,17.8,58,71,43.1,2.52
```

### Sensor Data Logging

Captures all sensor readings with quality metrics:

```csv
timestamp,tof_distance,tof_status,ultrasonic_distance,imu_accel_x,imu_accel_y,imu_accel_z,imu_gyro_x,imu_gyro_y,imu_gyro_z,edge_detected,sound_level
2025-01-15 10:30:00,1245,OK,800,0.15,-0.02,9.78,0.5,-0.3,0.1,false,23
2025-01-15 10:30:05,1238,OK,805,0.18,-0.05,9.81,0.8,-0.2,0.2,false,25
2025-01-15 10:30:10,1250,OK,810,0.12,-0.01,9.75,0.3,-0.4,0.0,false,22
```

*Note: `tof_distance` is from the front VL53L0X sensor, `ultrasonic_distance` is from the rear HC-SR04 sensor.*

### Motor Control Logging

Tracks motor commands and performance:

```csv
timestamp,left_speed,right_speed,left_direction,right_direction,encoder_left,encoder_right,current_draw
2025-01-15 10:30:00,150,150,FORWARD,FORWARD,1332,5,850
2025-01-15 10:30:01,0,0,STOP,STOP,1332,5,45
2025-01-15 10:30:02,100,-100,FORWARD,REVERSE,1335,8,720
```

*Note: `encoder_left` = GPIO 33, `encoder_right` = GPIO 5.*

### Power Management Logging

Records battery status and power consumption:

```csv
timestamp,battery_voltage,battery_percent,power_mode,current_draw,estimated_runtime,charge_cycles
2025-01-15 10:30:00,7.84,89.2,NORMAL,850,2.8,45
2025-01-15 10:30:30,7.82,88.7,NORMAL,870,2.75,45
2025-01-15 10:31:00,7.80,88.2,NORMAL,890,2.7,45
```

### Navigation Logging

Documents movement decisions and path planning:

```csv
timestamp,robot_state,action_taken,obstacle_distance,turn_angle,duration_ms,success
2025-01-15 10:30:00,NAVIGATING,MOVE_FORWARD,1245,0,1000,true
2025-01-15 10:30:01,OBSTACLE_DETECTED,STOP,125,0,50,true
2025-01-15 10:30:02,AVOIDING,TURN_RIGHT,125,90,500,true
```

### Error Logging

Captures all error events and recovery actions:

```csv
timestamp,error_type,error_code,component,severity,description,recovery_action,success
2025-01-15 10:30:00,SENSOR_TIMEOUT,E001,ToF_Sensor,WARNING,Sensor response timeout,SENSOR_RESET,true
2025-01-15 10:30:15,MEMORY_LOW,E002,System,CRITICAL,Free heap below threshold,FEATURE_DISABLE,true
```

## 🔧 Logging Configuration

### Data Logging Structure

```cpp
typedef struct {
    bool enableSystemLog;
    bool enableSensorLog;
    bool enableMotorLog;
    bool enablePowerLog;
    bool enableNavigationLog;
    bool enableErrorLog;
    bool enableDebugLog;
    uint32_t maxFileSize;        // Maximum file size in bytes
    uint16_t bufferSize;         // Number of entries to buffer
    uint32_t flushInterval;      // Milliseconds between buffer flushes
    bool enableCompression;      // GZIP compression for large files
} DataLogConfig_t;
```

### Configurable Parameters

- **Log Levels**: DEBUG, INFO, WARNING, ERROR, CRITICAL
- **Update Frequencies**: Configurable intervals per log type
- **File Rotation**: Automatic archiving when files reach size limits
- **Buffer Management**: Configurable memory vs. disk trade-offs
- **Compression Options**: Selective compression for storage optimization

## 📊 Data Management Features

### Automatic File Rotation

When log files reach the configured size limit:

1. **Archive Current File**: Rename to include timestamp
2. **Create New File**: Start fresh log with headers
3. **Cleanup Old Files**: Remove oldest archives to free space
4. **Update Index**: Maintain file directory for easy access

Example rotation:

```txt
sensor_data.csv → sensor_data_20250115_103000.csv
→ New sensor_data.csv created
```

### Buffer Management

The logging system uses intelligent buffering to optimize performance:

- **Memory Buffer**: Holds recent entries for fast writing
- **Disk Sync**: Periodic flush to ensure data persistence
- **Emergency Flush**: Immediate write on critical errors
- **Power Loss Protection**: Buffer flush on low battery detection

### Data Compression

For long-term storage optimization:

- **GZIP Compression**: Reduce file sizes by 60-80%
- **Selective Compression**: Compress archived files only
- **Real-time Access**: Keep current files uncompressed
- **Decompression Tools**: Utilities for analysis access

## 🔍 Data Analysis Tools

### Built-in Analysis Functions

```cpp
// Statistical analysis of sensor data
SensorStats_t analyzeSensorPerformance(const char* filename, uint32_t timeRange);

// Performance trend analysis
PerformanceTrend_t analyzeSystemTrends(uint32_t hours);

// Error pattern detection
ErrorAnalysis_t analyzeErrorPatterns(const char* component);

// Power consumption analysis
PowerAnalysis_t analyzePowerUsage(uint32_t timeRange);
```

### Real-time Monitoring

Access live data through serial interface:

```txt
LOG> status
Current log files:
- system_health.csv: 45KB (245 entries)
- sensor_data.csv: 128KB (2,156 entries)
- motor_control.csv: 23KB (892 entries)
- power_management.csv: 15KB (567 entries)

LOG> tail sensor_data 10
Showing last 10 entries from sensor_data.csv...

LOG> analyze performance 24h
Analyzing 24-hour performance trends...
Average loop time: 54ms (±8ms)
Peak memory usage: 89%
Sensor reliability: 99.2%
```

### Export Functions

```cpp
// Export data for external analysis
bool exportToUSB(const char* filename);
bool exportToWiFi(const char* filename, const char* destination);
bool generateReport(const char* reportType, uint32_t timeRange);
```

## 📈 Performance Impact

### Resource Usage

- **Memory Usage**: ~8KB RAM for buffering (configurable)
- **Storage Space**: ~1.5MB available (with rotation)
- **CPU Overhead**: <2% for normal logging operations
- **I/O Impact**: Batched writes minimize disk access

### Optimization Features

- **Selective Logging**: Enable only required log categories
- **Adaptive Frequency**: Reduce logging under resource pressure
- **Background Processing**: Non-blocking writes to maintain performance
- **Priority Queues**: Critical data logged first during resource constraints

## 🛠️ Configuration Options

### Log Level Configuration

```cpp
typedef enum {
    LOG_LEVEL_DEBUG = 0,    // Detailed debugging information
    LOG_LEVEL_INFO,         // General operational information
    LOG_LEVEL_WARNING,      // Warning conditions
    LOG_LEVEL_ERROR,        // Error conditions
    LOG_LEVEL_CRITICAL      // Critical error conditions
} LogLevel_t;
```

### Timing Configuration

```cpp
#define LOG_SYSTEM_INTERVAL     15000    // 15 seconds
#define LOG_SENSOR_INTERVAL     5000     // 5 seconds
#define LOG_MOTOR_INTERVAL      1000     // 1 second
#define LOG_POWER_INTERVAL      30000    // 30 seconds
#define LOG_NAVIGATION_INTERVAL 1000     // 1 second
#define LOG_ERROR_INTERVAL      0        // Immediate
```

### Storage Configuration

```cpp
#define LOG_MAX_FILE_SIZE       262144   // 256KB per file
#define LOG_BUFFER_SIZE         100      // 100 entries per buffer
#define LOG_FLUSH_INTERVAL      10000    // 10 seconds
#define LOG_ARCHIVE_COUNT       5        // Keep 5 archive files
```

## 🔧 Troubleshooting

### Common Issues

#### Storage Full

**Symptoms**: Logging stops, error messages about disk space
**Solutions**:

- Reduce log file size limits
- Increase archive cleanup frequency
- Disable non-essential log categories
- Enable compression for older files

#### Performance Impact

**Symptoms**: Slow loop times, delayed responses
**Solutions**:

- Increase buffer flush intervals
- Reduce logging frequency
- Disable debug-level logging
- Use background logging mode

#### Data Corruption

**Symptoms**: Incomplete or garbled log entries
**Solutions**:

- Check power supply stability
- Verify SPIFFS file system integrity
- Implement checksum validation
- Use emergency flush on low battery

### Diagnostic Commands

```txt
DIAG> filesystem
SPIFFS Status:
Total: 1,572,864 bytes
Used: 456,789 bytes (29%)
Free: 1,116,075 bytes (71%)

DIAG> logs
Active Log Files:
system_health.csv: 45,123 bytes (245 entries)
sensor_data.csv: 128,456 bytes (2,156 entries)
motor_control.csv: 23,789 bytes (892 entries)

DIAG> check logs
Verifying log file integrity...
✅ All files verified successfully
```

## 📊 Data Export and Analysis

### Export Formats

- **CSV**: Raw data for spreadsheet analysis
- **JSON**: Structured data for programming interfaces
- **XML**: Formatted data for reporting tools
- **Binary**: Compressed format for large datasets

### Analysis Integration

- **Excel/LibreOffice**: Direct CSV import for analysis
- **Python/R**: Data science tools for advanced analytics
- **MATLAB**: Engineering analysis and visualization
- **Grafana**: Real-time dashboard creation
- **Custom Tools**: API access for specialized analysis

### Sample Analysis Scripts

#### Python Data Analysis

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load sensor data
df = pd.read_csv('sensor_data.csv')
df['timestamp'] = pd.to_datetime(df['timestamp'])

# Plot ToF sensor readings over time
plt.figure(figsize=(12, 6))
plt.plot(df['timestamp'], df['tof_distance'])
plt.title('ToF Sensor Readings Over Time')
plt.xlabel('Time')
plt.ylabel('Distance (mm)')
plt.show()

# Calculate sensor reliability
reliability = (df['tof_status'] == 'OK').mean() * 100
print(f"ToF Sensor Reliability: {reliability:.1f}%")
```

#### Performance Trend Analysis

```python
# Load system health data
health_df = pd.read_csv('system_health.csv')

# Calculate performance trends
health_df['loop_efficiency'] = 1000 / health_df['loop_time']  # Hz
trend = health_df['loop_efficiency'].rolling(window=100).mean()

plt.figure(figsize=(12, 6))
plt.plot(health_df['timestamp'], trend)
plt.title('System Performance Trend')
plt.xlabel('Time')
plt.ylabel('Loop Frequency (Hz)')
plt.show()
```

## 🔮 Future Enhancements

### Advanced Features

- **Machine Learning Integration**: Predictive analytics on log data
- **Real-time Streaming**: Live data feeds to external systems
- **Cloud Integration**: Automatic upload to cloud storage
- **Anomaly Detection**: Automatic identification of unusual patterns
- **Comparative Analysis**: Multi-robot data comparison

### Enhanced Analytics

- **Pattern Recognition**: Automatic detection of operational patterns
- **Performance Optimization**: AI-driven performance recommendations
- **Failure Prediction**: Predictive maintenance based on log trends
- **Behavioral Analysis**: Understanding robot behavior patterns

## 🎯 Best Practices

### Logging Strategy

1. **Log Purposefully**: Only log data that will be analyzed
2. **Balance Detail vs. Performance**: Find optimal logging frequency
3. **Regular Maintenance**: Clean up old logs to free storage
4. **Monitor Resources**: Watch memory and storage usage
5. **Document Changes**: Maintain log format documentation

### Data Management

1. **Regular Backups**: Export important logs for safekeeping
2. **Version Control**: Track changes to log formats and structure
3. **Privacy Considerations**: Be mindful of sensitive data in logs
4. **Retention Policies**: Define how long to keep different log types
5. **Analysis Preparation**: Structure logs for easy analysis

### Performance Optimization

1. **Batch Operations**: Group log writes for efficiency
2. **Smart Buffering**: Balance memory usage with data safety
3. **Selective Compression**: Compress only when beneficial
4. **Resource Monitoring**: Adapt logging based on system health
5. **Emergency Protocols**: Prioritize critical data during emergencies
