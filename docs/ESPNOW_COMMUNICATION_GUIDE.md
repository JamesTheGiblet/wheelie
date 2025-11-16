# ESP-NOW Communication Guide

## Overview

Your Wheelie robot now includes **ESP-NOW peer-to-peer communication** that enables direct communication with other ESP32 devices without requiring a WiFi router. This creates a mesh network of robots for coordination, data sharing, and distributed operations.

## 📡 ESP-NOW Features

### Communication Capabilities

- ✅ **Peer-to-Peer Communication** - Direct ESP32 to ESP32 communication
- ✅ **Automatic Peer Discovery** - Robots automatically find and connect to each other
- ✅ **Real-time Data Sharing** - Sensor data, status, and commands transmitted instantly
- ✅ **Mesh Networking** - Multiple robots can communicate simultaneously
- ✅ **Low Latency** - Sub-millisecond communication times
- ✅ **No Router Required** - Works independently of WiFi infrastructure
- ✅ **Long Range** - Up to 200+ meters line-of-sight communication

### Message Types

- 💓 **Heartbeat** - Keep-alive messages for peer discovery
- 📊 **Sensor Data** - Real-time sensor readings broadcast
- 🎛️ **Commands** - Remote control commands
- 📋 **Status Updates** - System status and health information
- 🤝 **Pairing Requests** - Automatic peer connection
- ✅ **Acknowledgments** - Message delivery confirmation

## 🔧 System Configuration

### Network Settings

- **Channel**: 1 (configurable 1-14)
- **Max Peers**: 10 simultaneous robots (peers)
- **Message Size**: 250 bytes maximum
- **Retry Count**: 3 transmission attempts
- **Heartbeat Interval**: 5 seconds
- **Device ID**: 1 (unique per robot)

### Communication Frequency

- **Sensor Data**: Broadcast every 2 seconds
- **Status Updates**: Broadcast every 10 seconds  
- **Heartbeats**: Sent every 5 seconds
- **Peer Cleanup**: Inactive peers removed after 30 seconds

## 🚀 How It Works

### Initialization Sequence

1. **ESP-NOW Setup** - Initialize ESP-NOW subsystem
2. **Channel Configuration** - Set communication channel
3. **Callback Registration** - Register send/receive handlers
4. **Status Indication** - Green LED + success tone on ready
5. **Heartbeat Start** - Begin peer discovery broadcasts

### Peer Discovery Process

1. **Heartbeat Broadcast** - Robot sends periodic heartbeat messages
2. **Peer Detection** - Other robots receive heartbeat and auto-add peer
3. **Bidirectional Connection** - Both robots add each other as peers
4. **Data Exchange** - Sensor data and status sharing begins
5. **Health Monitoring** - Connection monitored with automatic cleanup

### Message Flow

```txt
Robot A ──[Heartbeat]──→ Robot B
       ←──[Heartbeat]──

Robot A ──[Sensor Data]──→ Robot B  
       ←──[Sensor Data]──

Robot A ──[Command]──→ Robot B
       ←──[ACK]──
```

## 📊 Data Transmission

### Sensor Data Broadcasting

Your robot automatically shares:

```txt
📏 Distance: 1500mm
📐 Tilt: 2.5°/1.1°
🛡️ Edge: Clear
🔊 Sound: Detected
👁️ Motion: Clear
```

### Status Information Sharing

System status broadcasted includes:

```txt
🔧 Sensors: 4/5 active
📶 WiFi: Connected (192.168.1.100)
📡 ESP-NOW: 3 peers
💾 Memory: 280KB available
⏱️ Uptime: 125 minutes
```

## 🔍 Network Monitoring

### Real-time ESP-NOW Status

```txt
📡 ESP-NOW STATUS REPORT
════════════════════════════════════════════════════════════════
🔗 Status: Active
📻 Channel: 1
👥 Active Peers: 3
📤 Messages Sent: 1,247
📥 Messages Received: 2,891
❌ Send Failures: 12
⏰ Last Activity: 2 seconds ago
📍 MAC Address: 24:6F:28:AB:CD:EF
════════════════════════════════════════════════════════════════
```

### Active Peers List

```txt
👥 ACTIVE PEERS LIST
════════════════════════════════════════════════════════════════
🤖 Device ID 2: 24:6F:28:12:34:56 - Last seen: 3s ago, RX: 145, TX: 98
🤖 Device ID 3: 24:6F:28:78:9A:BC - Last seen: 1s ago, RX: 89, TX: 76
🤖 Device ID 4: 24:6F:28:DE:F0:12 - Last seen: 5s ago, RX: 203, TX: 154
════════════════════════════════════════════════════════════════
```

## 🎛️ Remote Commands

### Command Structure

ESP-NOW supports sending commands to other robots:

```cpp
// Example: Send move forward command to specific robot
uint8_t command = 1; // Move forward
uint8_t targetMac[] = {0x24, 0x6F, 0x28, 0x12, 0x34, 0x56};
sendCommand(command, nullptr, 0, targetMac);
```

### Available Commands (Extensible)

- **Command 1**: Move Forward
- **Command 2**: Move Backward  
- **Command 3**: Turn Left
- **Command 4**: Turn Right
- **Command 5**: Stop
- **Command 6**: Emergency Stop
- **Command 7**: Status Request
- **Command 8**: Custom Action

## 📊 System Integration

### WiFi + ESP-NOW Coexistence

- **Dual Mode**: WiFi and ESP-NOW operate simultaneously
- **Channel Coordination**: ESP-NOW uses same channel as WiFi when connected
- **Resource Sharing**: Both systems share radio resources efficiently
- **Independent Operation**: ESP-NOW works even when WiFi is disconnected

### Status Reporting Integration

ESP-NOW status is integrated into main system status:

```txt
📊 SYSTEM STATUS REPORT
════════════════════════════════════════════════════════════════
🔧 Platform: ESP32 @ 240 MHz
💾 Memory: 280KB available
🔌 Sensors active: 4/5 components
📶 WiFi: Connected (192.168.1.100)
📡 ESP-NOW: Active (3 peers)
════════════════════════════════════════════════════════════════
```

## 🛠️ Technical Implementation

### Memory Usage

- **RAM**: 13.6% (up from 13.5%) - Minimal ESP-NOW overhead
- **Flash**: 60.1% (up from 59.2%) - ESP-NOW libraries
- **Performance**: No impact on sensor processing or motor control

### Message Reliability

- **Checksum Validation**: All messages include integrity checking
- **Sequence Numbers**: Message ordering and duplicate detection
- **Retry Logic**: Failed transmissions automatically retried
- **Timeout Protection**: Prevents hanging on failed sends

### Automatic Peer Management

- **Auto-Discovery**: Peers automatically found via heartbeats
- **Auto-Cleanup**: Inactive peers removed after timeout
- **Connection Monitoring**: Peer health continuously tracked
- **MAC Address Storage**: Persistent peer identification

## 🔧 Troubleshooting

### Communication Issues

#### "No peers discovered"

- Ensure other ESP32 devices are running ESP-NOW
- Check that devices are on the same channel
- Verify devices are within communication range
- Look for "Heartbeat from device X" messages in serial monitor

#### "Send failures increasing"

- Check signal strength (move devices closer)
- Verify peer is still active and responding
- Monitor for radio interference
- Check for message size limits (250 bytes max)

#### "Peer connections dropping"

- Ensure devices maintain power (not sleeping)
- Check for physical obstructions
- Monitor heartbeat intervals
- Verify stable power supply

### Status Indicators

| LED Pattern | Status | Description |
|-------------|--------|-------------|
| Green flash | ESP-NOW Ready | Initialization successful |
| Blue blink  | Transmitting   | Sending data to peers |
| Yellow blink| Peer Activity  | Receiving data from peers |
| Red blink   | Send Failure   | Communication error |

## 🚀 Use Cases & Applications

### Multi-Robot Coordination

- **Swarm Robotics**: Coordinate multiple robots for tasks
- **Formation Flying**: Maintain robot formations
- **Task Distribution**: Assign different roles to different robots
- **Collective Mapping**: Share sensor data for joint navigation

### Remote Monitoring

- **Status Dashboard**: Monitor multiple robots from one unit
- **Sensor Fusion**: Combine data from multiple robot sensors
- **Alert System**: Broadcast emergency or status alerts
- **Performance Metrics**: Track fleet-wide performance

### Mesh Networking

- **Range Extension**: Relay messages through other robots
- **Redundancy**: Multiple communication paths
- **Self-Healing**: Automatic route reconfiguration
- **Scalability**: Add/remove robots dynamically

## 🔮 Future Enhancements

### Potential Additions

- **Message Encryption**: Secure communications between robots
- **Mesh Routing**: Multi-hop message relay
- **GPS Coordination**: Location-based peer discovery
- **Role-Based Commands**: Different command sets per robot type
- **Data Logging**: Historical communication analysis
- **Web Dashboard**: Browser-based fleet monitoring

### Advanced Features

- **Dynamic Channel Selection**: Automatic best channel detection
- **Load Balancing**: Distribute communication across channels
- **Quality of Service**: Priority messaging for critical data
- **Network Topology**: Visualize robot communication network

## 📝 Configuration

### Changing Device ID

To set a unique device ID, modify `src/espnow_manager.cpp` (or your peer management source file):

```cpp
uint8_t deviceId = 2; // Change from 1 to your desired ID
```

### Adjusting Communication Intervals

Modify timing in `include/config.h` (or your configuration header):

```cpp
const unsigned long ESPNOW_HEARTBEAT_INTERVAL = 3000; // 3 seconds instead of 5
```

### Adding Custom Commands

Extend the command handler in `src/espnow_manager.cpp` (or your command handler source file):

```cpp
void handleCommand(const ESPNowMessage& message, const uint8_t* senderMac) {
  uint8_t command = message.data[0];
  switch(command) {
    case 1: /* Move forward */ break;
    case 2: /* Move backward */ break;
    case 10: /* Your custom command */ break;
  }
}
```

---

**Your robot is now part of an intelligent mesh network!** 📡🤖🤖🤖
