# 📡 Over-the-Air (OTA) Firmware Updates Guide

## 1. Overview

This guide is the single source of truth for updating the Wheelie robot's firmware wirelessly. It contains a simple guide for all users and a technical reference for developers.

Over-the-Air (OTA) updates allow you to upload new firmware to the robot over your WiFi network, eliminating the need for a USB cable after the initial setup.

---

## 2. User Guide

### Requirements

- The robot must be flashed with OTA-enabled firmware via USB at least once.
- Your computer and the robot must be on the **same WiFi network**.
- You need the robot's IP address.

### Visual Feedback (RGB LED)

The robot's RGB LED indicates the OTA status:

- **🟢 Solid Green**: Ready to receive an OTA update.
- **🔵 Solid Blue**: An OTA update is in progress.
- **🟢 Flashing Green**: The update was successful, and the robot is rebooting.
- **🔴 Solid Red**: An error occurred during the update.

### Step-by-Step Update Process

1. **Initial USB Flash**: The very first time, you must upload the firmware using a USB cable. This installs the necessary code to enable future wireless updates.

2. **Find the Robot's IP Address**:
    - Connect the robot to your computer via USB and open the Serial Monitor in VS Code/PlatformIO (`pio device monitor`).
    - Power on the robot and wait for it to connect to WiFi.
    - Look for a message like this and copy the IP address:

        ```txt
        WiFi connected! IP address: 192.168.1.123
        ```

3. **Upload Wirelessly**:
    - **Configure the IP Address**: Open the `platformio.ini` file and find the `[env:ota]` section. Update the `upload_port` with the IP address you copied.

      ```ini
      [env:ota]
      ...
      upload_port = 192.168.1.123
      ```

    - Open a terminal in VS Code.
    - Run the following command. This specifically uses the `ota` environment, which is pre-configured for wireless updates.

        ```sh
        pio run -e ota --target upload
        ```

4. **Confirm the Update**:
    - The terminal will show the upload progress.
    - The robot's LED will turn blue during the update.
    - After the upload is complete, the robot will reboot, and the LED should turn green.

### Basic Troubleshooting

| Issue | Solution |
| :--- | :--- |
| **Device Not Found** | Verify your computer and the robot are on the same WiFi network. Check the IP address. |
| **Upload Fails** | Ensure the robot is powered on and running OTA-enabled firmware. If you previously uploaded code using a different environment (like `debug`), you must re-flash it via USB first. |
| **Authentication Error** | If you have set an OTA password, ensure it is correctly configured in your `platformio.ini` file. |

---

## 3. Developer & Technical Guide

### PlatformIO Configuration

Your `platformio.ini` file should have a dedicated environment for OTA uploads. This keeps your debug and release configurations separate from your wireless deployment configuration.

```ini
[env:ota]
; Inherit common settings
extends = env:debug

; --- OTA Upload Settings ---
; Use the OTA protocol instead of serial
upload_protocol = espota

; Set the robot's network IP address here
upload_port = 192.168.1.240

; Provide the password defined in your ota_manager
upload_flags = --auth=your_ota_password
```

### Securing OTA Updates

For production or secure environments, you should always protect OTA updates with a password.

1. **In your firmware (e.g., `ota_manager.cpp`):**

    ```cpp
    // Set a password for OTA updates
    ArduinoOTA.setPassword("your_secure_password");
    ```

2. **In `platformio.ini`:**

    ```ini
    upload_flags = --auth=your_secure_password
    ```

### Pre-Update Safety Checks

Before initiating an update, the firmware can perform safety checks to ensure a stable process.

- **Battery Level**: Check if the battery is above a safe threshold (e.g., >30%).
- **System State**: Ensure the robot is in an `IDLE` state and not performing a critical task.
- **Connection Stability**: Verify the WiFi signal strength (`WiFi.RSSI()`) is sufficient (e.g., > -70 dBm).

### Recovery Mechanisms

The ESP32's OTA implementation includes robust recovery features:

- **Partition Rollback**: If an update fails or the new firmware crashes on boot, the bootloader will automatically roll back to the last known good firmware version. This prevents the robot from being "bricked" by a bad update.
- **Watchdog Timer**: An internal watchdog timer will reset the device if the firmware becomes unresponsive, which can trigger a rollback if it happens after an update.

### Network Discovery

If you don't know the robot's IP address, you can use network tools to find it.

- **mDNS/Bonjour**: If `ArduinoOTA.setHostname("wheelie-robot")` is set, you can use `wheelie-robot.local` as the hostname instead of the IP address.

  ```sh
  # This works on most systems with mDNS support (macOS, Linux Avahi)
  pio run --target upload --upload-port wheelie-robot.local
  ```

- **Network Scanning**: Use tools like `nmap` to scan for devices on your network.

  ```sh
  # Scan the 192.168.1.x subnet for devices
  nmap -sn 192.168.1.0/24
  ```

### Automated Update Script

You can automate the update process using Python and the `espota.py` script that comes with the Arduino-ESP32 core.

```bash
#!/bin/bash
# Example script to automate OTA updates

ROBOT_IP="192.168.1.123"
FIRMWARE_BIN=".pio/build/esp32dev/firmware.bin"
OTA_PASSWORD="your_secure_password"

# First, build the project
pio run

# Then, upload using the espota script
echo "Starting OTA update for robot at $ROBOT_IP..."
python ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py -i $ROBOT_IP -p 3232 -a "$OTA_PASSWORD" -f "$FIRMWARE_BIN"

echo "Update complete!"
```

---
*For more technical details, refer to the official PlatformIO documentation on Over-the-Air Updates.*
