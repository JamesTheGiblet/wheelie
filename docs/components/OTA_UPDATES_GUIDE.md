# OTA (Over-The-Air) Updates Guide

## 🚀 Overview

The Wheelie robot features secure Over-The-Air (OTA) update capabilities, allowing remote firmware updates without physical access to the device. This enterprise-grade feature enables field updates, bug fixes, and feature additions through WiFi connectivity.

## 🔐 Security Features

### Authentication

- **Password Protection**: All OTA updates require authentication
- **Default Password**: `wheelie_secure_ota_2024`
- **Hostname**: `wheelie-robot-[MAC_ADDRESS]`
- **Port**: 3232 (ArduinoOTA default)

### Integrity Verification

- **MD5 Checksums**: Automatic verification of uploaded firmware
- **Rollback Protection**: System maintains previous firmware version
- **Update Validation**: Pre-installation compatibility checks

## 📡 Network Requirements

### WiFi Configuration

- **Network**: Robot automatically connects to configured network
- **IP Assignment**: DHCP (dynamic) or static IP configuration
- **Signal Strength**: Minimum -70 dBm for reliable updates
- **Bandwidth**: 1-2 Mbps recommended for optimal transfer speed

### Firewall Settings

- **Inbound Port**: 3232 (TCP) must be accessible
- **mDNS**: Multicast DNS for device discovery
- **Network Discovery**: Ensure device appears in network scan

## 🔧 Update Process

### Automatic Update Discovery

The robot broadcasts its availability for OTA updates when:

1. **WiFi Connected**: Stable connection established
2. **System Healthy**: All critical systems operational
3. **Sufficient Power**: Battery level above CRITICAL mode
4. **Not During Operation**: Robot in IDLE or safe state

### Update Procedure

1. **Device Discovery**: Robot appears as `wheelie-robot-[MAC]` in Arduino IDE
2. **Authentication**: Enter password when prompted
3. **Upload Firmware**: Standard Arduino IDE upload process
4. **Progress Monitoring**: Real-time update progress display
5. **Automatic Restart**: Robot reboots with new firmware

## 💻 Development Setup

### Arduino IDE Configuration

1. **Install ESP32 Board Package**:

   ```txt
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```

2. **Select Board**: ESP32 Dev Module
3. **Upload Method**: Network Port (for OTA)
4. **Partition Scheme**: Default 4MB with OTA

### PlatformIO Configuration

Add to `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
upload_protocol = espota
upload_port = wheelie-robot-24:6f:28:ab:cd:ef.local
upload_flags = --auth=wheelie_secure_ota_2024
```

## 📊 Update Status Monitoring

### Visual Indicators

- **Blue LED Solid**: OTA server listening
- **Blue LED Blinking**: Update in progress
- **Green LED**: Update successful
- **Red LED**: Update failed

### Serial Output

```txt
🔄 OTA UPDATE STATUS
════════════════════════════════════════════════════════════════
📶 WiFi: Connected (192.168.1.100)
🔧 Hostname: wheelie-robot-246f28abcdef
🔐 Authentication: Enabled
📊 Progress: 45% (234,567 / 521,344 bytes)
⏱️ ETA: 2 minutes 15 seconds
🔋 Battery: 78% (sufficient for update)
════════════════════════════════════════════════════════════════
```

### Audio Feedback

- **Single Beep**: Update start
- **Double Beep**: Update progress (every 25%)
- **Triple Beep**: Update successful
- **Long Beep**: Update failed

## 🛡️ Safety Features

### Pre-Update Checks

```cpp
bool isOTAUpdateSafe() {
    // Check battery level
    if (batteryMonitor.mode >= POWER_CRITICAL) {
        return false;
    }
    
    // Check system health
    if (!isSystemHealthy()) {
        return false;
    }
    
    // Check robot state
    if (currentState != ROBOT_IDLE) {
        return false;
    }
    
    return true;
}
```

### Update Safeguards

1. **Battery Check**: Minimum 30% battery required
2. **State Validation**: Robot must be in safe state
3. **System Health**: All critical systems operational
4. **Storage Space**: Sufficient flash memory available
5. **Connection Stability**: Stable WiFi connection required

### Recovery Mechanisms

- **Watchdog Timer**: Prevents firmware corruption
- **Partition Rollback**: Automatic recovery to previous version
- **Factory Reset**: Hardware button recovery option
- **Emergency Boot**: Serial upload fallback mode

## 📱 Remote Management

### Network Discovery

Use network tools to find your robot:

```bash
# Windows - Find devices on network
nmap -sn 192.168.1.0/24

# macOS/Linux - mDNS discovery
dns-sd -B _arduino._tcp local.

# Arduino IDE - Network ports
Tools > Port > Network ports
```

### Update Scripts

Automated update script example:

```bash
#!/bin/bash
# Wheelie Robot OTA Update Script

ROBOT_IP="192.168.1.100"
FIRMWARE_FILE="wheelie_firmware.bin"
PASSWORD="wheelie_secure_ota_2024"

echo "Starting OTA update for Wheelie Robot..."
python espota.py -i $ROBOT_IP -p 3232 -a $PASSWORD -f $FIRMWARE_FILE
echo "Update complete!"
```

## 🔧 Troubleshooting

### Common Issues

#### Update Not Starting

**Symptoms**: Robot not visible in network ports
**Solutions**:

- Verify WiFi connection status
- Check network firewall settings
- Confirm robot is in IDLE state
- Restart Arduino IDE/PlatformIO

#### Authentication Failed

**Symptoms**: "Authentication failed" error
**Solutions**:

- Verify password: `wheelie_secure_ota_2024`
- Check for typos in authentication
- Restart robot and try again
- Clear Arduino IDE cache

#### Update Timeout

**Symptoms**: Upload times out or fails
**Solutions**:

- Check WiFi signal strength (-70 dBm minimum)
- Reduce network traffic during update
- Move closer to WiFi router
- Use ethernet connection for development machine

#### Partial Update

**Symptoms**: Update stops at percentage
**Solutions**:

- Check battery level (>30% required)
- Verify sufficient flash memory
- Restart robot and retry
- Check for network interruptions

### Diagnostic Commands

Monitor OTA status via serial:

```txt
📡 OTA DIAGNOSTICS
════════════════════════════════════════════════════════════════
🔧 OTA Server: ACTIVE
📶 WiFi Signal: -45 dBm (Excellent)
🔋 Battery: 67% (Sufficient)
💾 Free Flash: 1,234,567 bytes
🤖 Robot State: IDLE (Ready for update)
🔐 Auth Required: YES
⏰ Uptime: 2h 34m 12s
📍 Last Update: 2 days ago
════════════════════════════════════════════════════════════════
```

## 🔄 Update Workflow

### Development Cycle

1. **Code Changes**: Modify firmware in development environment
2. **Local Testing**: Test changes on development robot
3. **Build Firmware**: Compile for target ESP32 board
4. **Network Discovery**: Locate target robot on network
5. **Authentication**: Enter OTA password
6. **Upload**: Deploy firmware over WiFi
7. **Verification**: Monitor update progress and completion
8. **Testing**: Verify new functionality operates correctly

### Production Deployment

1. **Staging**: Test updates on staging robot
2. **Validation**: Verify all features work correctly
3. **Backup**: Document current firmware version
4. **Schedule**: Plan update during low-activity periods
5. **Deploy**: Push updates to production robots
6. **Monitor**: Watch for successful completion
7. **Verify**: Confirm robots operate normally post-update

## 📈 Update Analytics

### Logging

OTA updates are logged to SPIFFS with the following information:

```csv
timestamp,update_type,version,duration_seconds,status,error_code
2025-01-15 14:30:00,OTA,v2.1.0,245,SUCCESS,0
2025-01-16 09:15:30,OTA,v2.1.1,198,SUCCESS,0
2025-01-17 16:45:12,OTA,v2.2.0,0,FAILED,AUTH_ERROR
```

### Performance Metrics

- **Update Success Rate**: Track successful vs failed updates
- **Transfer Speed**: Monitor upload speeds for optimization
- **Downtime**: Measure robot unavailability during updates
- **Battery Impact**: Track power consumption during updates

## 🔮 Future Enhancements

### Planned Features

- **Automatic Updates**: Scheduled firmware updates
- **Fleet Management**: Mass deployment to multiple robots
- **Delta Updates**: Incremental patches to reduce transfer time
- **Rollback Commands**: Remote command to revert firmware
- **Update Staging**: A/B testing with gradual rollout
- **Web Interface**: Browser-based update management

### Advanced Security

- **Certificate Validation**: SSL/TLS encryption for updates
- **Code Signing**: Cryptographic verification of firmware
- **Access Control**: Role-based update permissions
- **Audit Logging**: Detailed security event tracking

## 🎯 Best Practices

### Security Recommendations

1. **Change Default Password**: Use unique, strong passwords
2. **Network Segmentation**: Isolate robots on separate VLAN
3. **Regular Updates**: Keep firmware current with security patches
4. **Monitoring**: Track update activities and failures
5. **Backup Strategy**: Maintain known-good firmware versions

### Operational Guidelines

1. **Update Windows**: Schedule updates during maintenance periods
2. **Batch Testing**: Test updates on subset before full deployment
3. **Rollback Plan**: Prepare recovery procedures for failed updates
4. **Documentation**: Maintain update logs and version history
5. **User Training**: Ensure operators understand update procedures

### Performance Optimization

1. **Network Quality**: Ensure strong WiFi signal for reliable transfers
2. **Power Management**: Update during high battery periods
3. **Resource Monitoring**: Track memory and storage usage
4. **Timing**: Avoid updates during critical operations
5. **Validation**: Thoroughly test updates before deployment
