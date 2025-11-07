#ifndef CONFIG_H
#define CONFIG_H

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURATION - System constants and settings
// ═══════════════════════════════════════════════════════════════════════════

// PWM Channels - ESP32 LED Controller assignments
#define BUZZER_CHANNEL   4      // PWM channel for buzzer (moved from 0 to avoid motor conflict)
// Note: Motor control uses PWM channels 0-3 directly in motors.cpp

// PWM Configuration
const int PWM_FREQ = 5000;          // PWM frequency (Hz)
const int PWM_RESOLUTION = 8;       // PWM resolution (8-bit = 0-255)
const int BUZZER_FREQ = 2000;       // Default buzzer frequency (Hz)

// Motor Control
const int TEST_SPEED = 200;         // Default motor test speed (0-255)
const int MAX_SPEED = 255;          // Maximum motor speed
const int SLOW_SPEED = 120;         // Slow movement speed
const int TURN_SPEED = 180;         // Turning speed
const int MOTOR_DEADZONE_TEST_PWM = 50; // Initial PWM to test for deadzone movement

// Calibration Settings
const unsigned long CALIBRATION_TIMEOUT = 300000; // 5 minutes maximum calibration time (moved from calibration.h)

// Sensor Thresholds
const int OBSTACLE_DISTANCE = 200;   // Obstacle detection threshold (mm)
const int WARNING_DISTANCE = 350;    // Warning distance threshold (mm)
const float TILT_THRESHOLD = 45.0;   // Tilt detection threshold (degrees) - increased tolerance
const float TILT_TOLERANCE = 65.0;   // Tolerance for serious tilts only (flipped/on side) - was 30°
const int EDGE_THRESHOLD = 500;      // Edge sensor threshold (analog value)

// Timing Intervals (milliseconds)
const unsigned long TOF_INTERVAL = 50;         // Time-of-Flight sensor read interval
const unsigned long MPU_INTERVAL = 100;        // IMU sensor read interval
const unsigned long SOUND_COOLDOWN = 3000;     // Sound trigger cooldown period
const unsigned long MOTION_COOLDOWN = 5000;    // Motion trigger cooldown period
const unsigned long EDGE_COOLDOWN = 2000;      // Edge trigger cooldown period
const int DEBOUNCE_DELAY = 50;                  // Button/sensor debounce delay

// Communication
const unsigned long SERIAL_BAUD = 115200;      // Serial communication speed
const unsigned long I2C_CLOCK = 400000;        // I2C bus speed (400kHz)

// WiFi Configuration (credentials defined in credentials.h for security)
// #include "credentials.h" in your main.cpp to access WIFI_SSID and WIFI_PASSWORD
const unsigned long WIFI_TIMEOUT = 20000;      // WiFi connection timeout (ms)
const unsigned long WIFI_RETRY_INTERVAL = 5000; // WiFi retry interval (ms)

// ESP-NOW Configuration
const int ESPNOW_CHANNEL = 1;                   // ESP-NOW communication channel (1-14)
const int ESPNOW_MAX_PEERS = 10;                // Maximum number of peers
const int ESPNOW_RETRY_COUNT = 3;               // Number of transmission retries
const unsigned long ESPNOW_TIMEOUT = 1000;     // ESP-NOW transmission timeout (ms)
const unsigned long ESPNOW_HEARTBEAT_INTERVAL = 5000; // Heartbeat interval (ms)

// Sensor Configuration
const int TOF_TIMEOUT = 500;                    // VL53L0X timeout (ms)
const unsigned long TOF_TIMING_BUDGET = 33000;  // VL53L0X measurement timing budget (μs) - FIXED: Must be < TOF_INTERVAL

#endif // CONFIG_H