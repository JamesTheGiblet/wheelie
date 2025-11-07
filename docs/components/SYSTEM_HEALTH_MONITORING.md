# System Health Monitoring Guide

## 🏥 Overview

The Wheelie robot features comprehensive system health monitoring that continuously tracks memory usage, performance metrics, sensor health, and overall system stability. This enterprise-grade monitoring system enables proactive maintenance and prevents system failures.

## 📊 Monitoring Components

### Memory Monitoring

- **Heap Usage**: Real-time free heap memory tracking
- **Stack Usage**: Stack pointer monitoring and overflow detection
- **Memory Leaks**: Automatic detection of memory allocation issues
- **Fragmentation**: Heap fragmentation analysis and reporting

### Performance Metrics

- **Loop Timing**: Main loop execution time analysis
- **Sensor Response**: Individual sensor response time monitoring
- **CPU Usage**: Processor utilization tracking
- **Task Scheduling**: Real-time task performance analysis

### Sensor Health

- **Connectivity**: I2C communication health monitoring
- **Response Time**: Sensor reading latency tracking
- **Data Validity**: Sensor reading range and sanity checking
- **Failure Detection**: Automatic sensor fault identification

## 🔧 Health Check Functions

### Core Health Monitoring

```cpp
typedef struct {
    uint32_t freeHeap;
    uint32_t minFreeHeap;
    uint32_t maxAllocHeap;
    float stackUsagePercent;
    uint32_t uptimeSeconds;
    bool memoryLeakDetected;
    uint32_t loopFrequency;
    uint32_t avgLoopTime;
    uint32_t maxLoopTime;
} SystemHealth_t;
```

### Key Monitoring Functions

- `monitorSystemHealth()` - Comprehensive system health check
- `checkMemoryHealth()` - Memory usage and leak detection
- `monitorLoopPerformance()` - Loop timing analysis
- `checkSensorHealth()` - Individual sensor status verification
- `reportSystemStatus()` - Detailed health reporting

## 📈 Performance Thresholds

### Memory Thresholds

| Metric | Warning Level | Critical Level | Action |
|--------|---------------|----------------|--------|
| Free Heap | <50KB | <20KB | Reduce features |
| Stack Usage | >80% | >95% | Function optimization |
| Memory Leak | 1KB/hour | 5KB/hour | Restart required |
| Fragmentation | >60% | >80% | Garbage collection |

### Performance Thresholds

| Metric | Target | Warning | Critical | Action |
|--------|--------|---------|----------|--------|
| Loop Frequency | 20Hz | <15Hz | <10Hz | Performance mode |
| Loop Time | <50ms | >100ms | >200ms | Feature reduction |
| Sensor Response | <10ms | >50ms | >100ms | Sensor reset |
| CPU Usage | <70% | >85% | >95% | Load balancing |

## 🚨 Health Status Levels

### HEALTHY (Green)

- All systems operating within normal parameters
- Memory usage <70%, performance >15Hz
- All sensors responding normally
- No error conditions detected

### WARNING (Yellow)

- One or more systems approaching limits
- Memory usage 70-85%, performance 10-15Hz
- Some sensors showing degraded performance
- Non-critical errors detected

### CRITICAL (Red)

- System resources severely constrained
- Memory usage >85%, performance <10Hz
- Multiple sensor failures detected
- Critical errors requiring intervention

### EMERGENCY (Flashing Red)

- System stability compromised
- Immediate action required to prevent failure
- Automatic safe mode activation
- Data logging and communication prioritized

## 📊 Real-Time Monitoring

### Health Dashboard

```txt
🏥 SYSTEM HEALTH MONITOR
════════════════════════════════════════════════════════════════
💾 Memory Status:
   Free Heap: 245,760 bytes (75%)
   Min Free Heap: 198,432 bytes
   Stack Usage: 45% (2,304 / 5,120 bytes)
   Memory Leak: None detected

⚡ Performance Metrics:
   Loop Frequency: 18.5 Hz
   Average Loop Time: 52ms
   Max Loop Time: 89ms
   CPU Usage: 67%

🔍 Sensor Health:
   IMU (MPU6050): ✅ HEALTHY (8ms response)
   ToF (VL53L0X): ✅ HEALTHY (12ms response)
   Edge Sensor: ✅ HEALTHY (2ms response)
   Sound Sensor: ⚠️  WARNING (45ms response)

🔋 Power Status:
   Battery: 7.8V (92%) - NORMAL mode
   Power Draw: 850mA
   Estimated Runtime: 2.3 hours

🌐 Communication:
   WiFi: ✅ Connected (-42 dBm)
   ESP-NOW: ✅ Active (3 peers)
   OTA: ✅ Ready

🎯 Overall Status: HEALTHY
════════════════════════════════════════════════════════════════
```

### Performance Trends

The system tracks performance trends over time:

- **5-minute averages** for short-term trend analysis
- **1-hour averages** for operational pattern detection
- **24-hour summaries** for daily performance reports
- **Weekly reports** for long-term health assessment

## 🔧 Automatic Health Management

### Self-Healing Features

#### Memory Management

```cpp
void manageMemoryHealth() {
    if (ESP.getFreeHeap() < MEMORY_WARNING_THRESHOLD) {
        // Reduce logging frequency
        dataLogConfig.maxFileSize /= 2;
        
        // Clear non-essential buffers
        clearPerformanceHistory();
        
        // Request garbage collection
        ESP.restart(); // If critically low
    }
}
```

#### Performance Optimization

```cpp
void optimizePerformance() {
    if (avgLoopTime > PERFORMANCE_WARNING_THRESHOLD) {
        // Reduce sensor update frequency
        sensorUpdateInterval *= 2;
        
        // Disable non-essential features
        dataLogging.enableDebugLog = false;
        
        // Simplify LED animations
        indicators.setSimpleMode(true);
    }
}
```

#### Sensor Recovery

```cpp
void recoverFailedSensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorHealth[i].status == SENSOR_FAILED) {
            // Attempt sensor reset
            resetSensor(i);
            
            // Re-initialize if necessary
            if (!initializeSensor(i)) {
                // Mark for degraded operation
                sensorHealth[i].degradedMode = true;
            }
        }
    }
}
```

## 📊 Data Logging Integration

### Health Data Logging

The health monitoring system logs comprehensive data:

```csv
timestamp,free_heap,stack_usage,loop_freq,loop_time,cpu_usage,sensors_ok,battery_voltage,health_status
2025-01-15 10:30:00,245760,45,18.5,52,67,4,7.84,HEALTHY
2025-01-15 10:30:15,244832,46,18.2,54,68,4,7.83,HEALTHY
2025-01-15 10:30:30,243904,47,17.8,58,71,3,7.82,WARNING
```

### Performance Analytics

- **Memory usage patterns** over time
- **Performance degradation trends**
- **Sensor failure frequency analysis**
- **Power consumption correlation**
- **Environmental impact assessment**

## 🔔 Alert System

### Alert Types

#### Memory Alerts

- **Heap Low Warning**: <50KB available
- **Stack Overflow Risk**: >90% usage
- **Memory Leak Detected**: Continuous allocation without deallocation
- **Fragmentation Critical**: >80% fragmentation ratio

#### Performance Alerts

- **Loop Slowdown**: Performance below target thresholds
- **Sensor Timeout**: Response time exceeding limits
- **CPU Overload**: Processing capacity exceeded
- **Communication Lag**: Network response delays

#### System Alerts

- **Sensor Failure**: Hardware malfunction detected
- **Power Critical**: Battery or power system issues
- **Communication Lost**: Network connectivity problems
- **Thermal Issues**: Temperature outside operating range

### Alert Actions

1. **Visual Indicators**: LED status changes and patterns
2. **Audio Alerts**: Specific beep patterns for different alerts
3. **Data Logging**: Detailed event logging for analysis
4. **Automatic Recovery**: Self-healing actions when possible
5. **Safe Mode**: Graceful degradation to essential functions

## 🔧 Manual Health Checks

### Diagnostic Commands

Access detailed health information through serial interface:

```txt
HEALTH> status
Displaying comprehensive system health status...

HEALTH> memory
Free Heap: 245,760 bytes
Min Free Heap: 198,432 bytes
Max Alloc: 32,768 bytes
Stack Usage: 2,304 / 5,120 bytes (45%)

HEALTH> performance
Loop Frequency: 18.5 Hz
Average Loop Time: 52ms
Max Loop Time: 89ms
CPU Usage: 67%

HEALTH> sensors
IMU: HEALTHY (8ms response)
ToF: HEALTHY (12ms response)
Edge: HEALTHY (2ms response)
Sound: WARNING (45ms response)

HEALTH> reset [sensor_name]
Resetting specified sensor...
```

### Health Test Procedures

#### Memory Test

1. Monitor heap usage during normal operation
2. Check for memory leaks over extended periods
3. Verify garbage collection effectiveness
4. Test memory recovery after high-usage scenarios

#### Performance Test

1. Measure loop timing under various loads
2. Test sensor response times individually
3. Monitor CPU usage during peak operations
4. Verify real-time performance requirements

#### Sensor Test

1. Individual sensor connectivity verification
2. Response time measurement under load
3. Data quality assessment over time
4. Failure recovery testing

## 🛠️ Troubleshooting

### Common Health Issues

#### Memory Problems

**Symptoms**: Decreasing free heap, system instability
**Diagnosis**:

- Monitor heap usage trends
- Check for memory leaks in logs
- Analyze stack usage patterns

**Solutions**:

- Reduce data logging frequency
- Clear non-essential buffers
- Optimize memory allocation patterns
- Restart system if critically low

#### Performance Degradation

**Symptoms**: Slow loop times, delayed responses
**Diagnosis**:

- Check CPU usage trends
- Monitor sensor response times
- Analyze task scheduling efficiency

**Solutions**:

- Reduce sensor update frequency
- Disable non-essential features
- Optimize algorithm efficiency
- Implement performance mode

#### Sensor Health Issues

**Symptoms**: Timeouts, invalid readings, communication errors
**Diagnosis**:

- Test individual sensor responses
- Check I2C communication quality
- Monitor power supply stability

**Solutions**:

- Reset failed sensors
- Reinitialize communication buses
- Check wiring and connections
- Replace faulty hardware

## 📈 Health Analytics

### Predictive Maintenance

The system uses historical health data to predict:

- **Memory degradation patterns** leading to future failures
- **Performance trend analysis** for proactive optimization
- **Sensor failure prediction** based on response time trends
- **Battery health assessment** through power consumption analysis

### Health Scoring

Overall system health is calculated using weighted factors:

```cpp
float calculateHealthScore() {
    float memoryScore = (freeHeap / totalHeap) * 100;
    float performanceScore = (targetFreq / actualFreq) * 100;
    float sensorScore = (workingSensors / totalSensors) * 100;
    float powerScore = batteryPercentage;
    
    return (memoryScore * 0.3) + (performanceScore * 0.3) + 
           (sensorScore * 0.25) + (powerScore * 0.15);
}
```

Health scores range from 0-100:

- **90-100**: Excellent health
- **80-89**: Good health
- **70-79**: Fair health (monitoring recommended)
- **60-69**: Poor health (maintenance needed)
- **<60**: Critical health (immediate attention required)

## 🔮 Future Enhancements

### Advanced Monitoring

- **Machine Learning**: Predictive failure analysis
- **Remote Monitoring**: Web dashboard for fleet health
- **Comparative Analysis**: Multi-robot health comparison
- **Environmental Correlation**: Health vs environmental factors

### Automated Recovery

- **Self-Repair**: Automatic hardware reconfiguration
- **Dynamic Optimization**: Real-time performance tuning
- **Preventive Maintenance**: Scheduled maintenance alerts
- **Health-Based Routing**: Path planning considering robot health

## 🎯 Best Practices

### Health Monitoring Guidelines

1. **Regular Monitoring**: Check health status daily
2. **Trend Analysis**: Review weekly health trends
3. **Threshold Tuning**: Adjust thresholds based on usage
4. **Proactive Maintenance**: Address warnings before they become critical
5. **Documentation**: Maintain health event logs for analysis

### Performance Optimization Best Practices

1. **Baseline Establishment**: Record normal operating parameters
2. **Regular Testing**: Periodic performance verification
3. **Load Testing**: Verify health under maximum load
4. **Optimization Cycles**: Regular performance improvement iterations
5. **Capacity Planning**: Monitor growth in resource usage
