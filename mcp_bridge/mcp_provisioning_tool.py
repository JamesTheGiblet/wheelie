"""
MCP Provisioning Tool (PC Side)
- Communicates with ESP32 bridge over Serial
- Listens for MSG_DISCOVERY_REPORT (0x10) packets
- Displays robot info and prompts for config
- Sends MSG_PROVISION_CONFIG (0x11) with ProvisioningConfig struct
"""
import serial
import struct
import time

# Serial port for ESP32 bridge (update as needed)
SERIAL_PORT = 'COM3'  # Change to your port
BAUD_RATE = 115200

# Message types
MSG_DISCOVERY_REPORT = 0x10
MSG_PROVISION_CONFIG = 0x11

# Data structures (must match C++ layout)
HARDWARE_MANIFEST_FMT = '<IBBBB B'  # uint32, 4x bool, uint8
PROVISIONING_CONFIG_FMT = '<HfffB'   # uint16, 3x float, uint8

class HardwareManifest:
    def __init__(self, data):
        (self.chipId, self.hasToF, self.hasMPU, self.hasDisplay, self.hasServoDriver, self.sensorCount) = struct.unpack(HARDWARE_MANIFEST_FMT, data)
    def __str__(self):
        return f"ChipID: {self.chipId:08X}, ToF: {bool(self.hasToF)}, MPU: {bool(self.hasMPU)}, Display: {bool(self.hasDisplay)}, Servo: {bool(self.hasServoDriver)}, Sensors: {self.sensorCount}"

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    print("MCP Provisioning Tool Started. Waiting for robots...")
    while True:
        # Wait for a discovery report
        header = ser.read(1)
        if not header:
            continue
        msg_type = header[0]
        if msg_type == MSG_DISCOVERY_REPORT:
            payload = ser.read(struct.calcsize(HARDWARE_MANIFEST_FMT))
            if len(payload) != struct.calcsize(HARDWARE_MANIFEST_FMT):
                print("Incomplete manifest received.")
                continue
            manifest = HardwareManifest(payload)
            print(f"\nNew Robot Found: {manifest}")
            # Prompt for config
            robotId = int(input("Assign Robot ID: "))
            attraction = float(input("Attraction Constant: "))
            repulsion = float(input("Repulsion Constant: "))
            maxSpeed = float(input("Max Speed: "))
            swarmGroup = int(input("Swarm Group ID: "))
            # Build and send config
            config = struct.pack(PROVISIONING_CONFIG_FMT, robotId, attraction, repulsion, maxSpeed, swarmGroup)
            msg = bytes([MSG_PROVISION_CONFIG]) + config
            ser.write(msg)
            print("Provisioning config sent!\n")
        else:
            # Ignore other messages
            continue

if __name__ == '__main__':
    main()
