// ═══════════════════════════════════════════════════════════════════════════
// 🤖 WHEELIE AUTONOMOUS ROBOT - Refactored Main File with Auto-Calibration
// ═══════════════════════════════════════════════════════════════════════════
// Features: Multi-sensor fusion, obstacle avoidance, motion detection,
//           tilt protection, comprehensive diagnostics, and autonomous calibration
// Author: Your Name | Date: November 2025
// ═══════════════════════════════════════════════════════════════════════════

#include "robot.h"
#include "calibration.h"
#include <ArduinoOTA.h>
#include "indicators.h" // This line is required
#include <Update.h>
#include <SPIFFS.h>
#include <FS.h>

// ═══════════════════════════════════════════════════════════════════════════
// ROBOT STATE MANAGEMENT SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATION FAILURE HANDLER
// ═══════════════════════════════════════════════════════════════════════════

void handleCalibrationFailure() {
  Serial.println("🔄 Attempting calibration recovery...");
  
  // Try limited calibration without movement
  if (attemptStaticCalibration()) {
    Serial.println("✅ Limited calibration successful - basic operation enabled");
    isCalibrated = true;
    return;
  }
  
  // Final fallback - factory defaults
  loadFactoryCalibration();
  Serial.println("⚠️  Using factory defaults - recalibrate when possible");
}

// ═══════════════════════════════════════════════════════════════════════════
// SYSTEM HEALTH MONITORING
// ═══════════════════════════════════════════════════════════════════════════

// Add watchdog and resource monitoring
void monitorSystemHealth() {
  static unsigned long lastHealthCheck = 0;
  
  // Parameter validation - check for millis() overflow
  unsigned long currentTime = millis();
  if (currentTime < lastHealthCheck) {
    Serial.println("⚠️  Timer overflow detected - resetting health check");
    lastHealthCheck = currentTime;
    return;
  }
  
  if (currentTime - lastHealthCheck > 5000) {
    // Check memory with validation
    uint32_t freeHeap = esp_get_free_heap_size();
    if (freeHeap == 0) {
      Serial.println("❌ Invalid heap size - system error");
      ESP.restart();
    }
    
    if (freeHeap < 10000) {
      Serial.println("⚠️  Low memory - restarting");
      ESP.restart();
    }
    
    // Check task stack
    checkStackUsage();
    
    lastHealthCheck = currentTime;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP PERFORMANCE MONITORING
// ═══════════════════════════════════════════════════════════════════════════

// Track loop timing for performance analysis
void monitorLoopPerformance() {
  static unsigned long lastLoopTime = 0;
  static unsigned long maxLoopTime = 0;
  static unsigned long loopCount = 0;
  
  unsigned long currentTime = millis();
  
  // Parameter validation - handle timer overflow
  if (currentTime < lastLoopTime) {
    Serial.println("⚠️  Timer overflow in performance monitor - resetting");
    lastLoopTime = currentTime;
    maxLoopTime = 0;
    loopCount = 0;
    return;
  }
  
  // Validate first run
  if (lastLoopTime == 0) {
    lastLoopTime = currentTime;
    return;
  }
  
  unsigned long loopDuration = currentTime - lastLoopTime;
  
  // Validate loop duration is reasonable (< 1 second)
  if (loopDuration > 1000) {
    Serial.printf("⚠️  Excessive loop time detected: %lums\n", loopDuration);
  }
  
  if (loopDuration > maxLoopTime) {
    maxLoopTime = loopDuration;
  }
  
  loopCount++;
  
  // Prevent overflow of loop counter
  if (loopCount == ULONG_MAX) {
    loopCount = 0;
    maxLoopTime = 0;
  }
  
  // Report every 1000 loops
  if (loopCount % 1000 == 0) {
    Serial.printf("📊 Loop performance - Current: %lums, Max: %lums\n", 
                  loopDuration, maxLoopTime);
    maxLoopTime = 0; // Reset max
  }
  
  lastLoopTime = currentTime;
}

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURATION VALIDATION
// ═══════════════════════════════════════════════════════════════════════════

bool validateConfiguration() {
  bool valid = true;
  
  Serial.println("🔍 Starting configuration validation...");
  
  // Verify critical sensors with null checks
  Serial.print("Testing mpu... ");
  if (!mpu.begin()) {
    Serial.println("❌ IMU initialization failed");
    valid = false;
  } else {
    Serial.println("✅ IMU OK");
  }
  
  // Verify motor controllers with safety checks
  Serial.print("Testing motors... ");
  if (!testMotors()) {
    Serial.println("❌ Motor test failed");
    valid = false;
  } else {
    Serial.println("✅ Motors OK");
  }
  
  // Verify communication with timeout
  Serial.print("Testing communications... ");
  if (!testCommunications()) {
    Serial.println("❌ Communication test failed");
    valid = false;
  } else {
    Serial.println("✅ Communications OK");
  }
  
  // Additional validation checks
  Serial.print("Checking power levels... ");
  if (getBatteryVoltage() < 6.0) {  // Minimum safe voltage
    Serial.println("⚠️  Low battery voltage");
    valid = false;
  } else {
    Serial.println("✅ Power OK");
  }
  
  Serial.printf("🔍 Configuration validation %s\n", valid ? "PASSED" : "FAILED");
  return valid;
}

// ═══════════════════════════════════════════════════════════════════════════
// GRACEFUL DEGRADATION SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

// Sensor health tracking
typedef struct {
  bool imu_healthy;
  bool distance_sensor_healthy;
  bool encoders_healthy;
  bool motors_healthy;
  bool wifi_healthy;
  bool espnow_healthy;
  unsigned long last_health_check;
} SensorHealth_t;

SensorHealth_t sensorHealth = {
  .imu_healthy = true,
  .distance_sensor_healthy = true,
  .encoders_healthy = true,
  .motors_healthy = true,
  .wifi_healthy = true,
  .espnow_healthy = true,
  .last_health_check = 0
};

void checkSensorHealth() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck > 2000) { // Check every 2 seconds
    // Test IMU
    bool imu_test = testIMU();
    if (sensorHealth.imu_healthy && !imu_test) {
      Serial.println("⚠️  IMU failure detected - switching to encoder-only navigation");
      setRobotState(ROBOT_SAFE_MODE);
    }
    sensorHealth.imu_healthy = imu_test;
    
    // Test distance sensor
    bool distance_test = testDistanceSensor();
    if (sensorHealth.distance_sensor_healthy && !distance_test) {
      Serial.println("⚠️  Distance sensor failure - using reduced obstacle avoidance");
    }
    sensorHealth.distance_sensor_healthy = distance_test;
    
    // Test encoders
    bool encoder_test = testEncoders();
    if (sensorHealth.encoders_healthy && !encoder_test) {
      Serial.println("⚠️  Encoder failure - switching to time-based movement");
    }
    sensorHealth.encoders_healthy = encoder_test;
    
    // Test motors
    bool motor_test = testMotors();
    if (sensorHealth.motors_healthy && !motor_test) {
      Serial.println("❌ Motor failure detected - entering error mode");
      setRobotState(ROBOT_ERROR);
    }
    sensorHealth.motors_healthy = motor_test;
    
    // Test WiFi
    sensorHealth.wifi_healthy = WiFi.status() == WL_CONNECTED;
    
    // Test ESP-NOW
    sensorHealth.espnow_healthy = testESPNOW();
    
    lastCheck = millis();
  }
}

void adaptToSensorFailures() {
  // Adapt navigation based on available sensors
  if (!sensorHealth.imu_healthy && !sensorHealth.encoders_healthy) {
    // No position feedback - emergency stop
    Serial.println("❌ Critical navigation sensors failed - emergency stop");
    emergencyStop();
    setRobotState(ROBOT_ERROR);
    return;
  }
  
  // Adapt obstacle avoidance
  if (!sensorHealth.distance_sensor_healthy) {
    // Use slower, more cautious movement
    setMaxSpeed(0.3f); // Reduce to 30% speed
    setObstacleAvoidanceMode(CONSERVATIVE_MODE);
  }
  
  // Adapt navigation method
  if (!sensorHealth.imu_healthy) {
    // Use encoder-only navigation
    setNavigationMode(ENCODER_ONLY);
    Serial.println("🔄 Switched to encoder-only navigation");
  } else if (!sensorHealth.encoders_healthy) {
    // Use IMU-only navigation with time estimation
    setNavigationMode(IMU_ONLY);
    Serial.println("🔄 Switched to IMU-only navigation");
  }
  
  // Handle communication failures
  if (!sensorHealth.wifi_healthy && !sensorHealth.espnow_healthy) {
    Serial.println("⚠️  All communication failed - operating in isolation mode");
    setCommunicationMode(OFFLINE_MODE);
  } else if (!sensorHealth.wifi_healthy) {
    Serial.println("🔄 WiFi failed - using ESP-NOW only");
    setCommunicationMode(ESPNOW_ONLY);
  } else if (!sensorHealth.espnow_healthy) {
    Serial.println("🔄 ESP-NOW failed - using WiFi only");
    setCommunicationMode(WIFI_ONLY);
  }
}

void reportSystemCapabilities() {
  static unsigned long lastReport = 0;
  
  if (millis() - lastReport > 30000) { // Report every 30 seconds
    Serial.println("\n📊 SYSTEM CAPABILITIES STATUS:");
    Serial.printf("🧭 IMU: %s\n", sensorHealth.imu_healthy ? "✅ ACTIVE" : "❌ DEGRADED");
    Serial.printf("📏 Distance: %s\n", sensorHealth.distance_sensor_healthy ? "✅ ACTIVE" : "❌ DEGRADED");
    Serial.printf("⚙️  Encoders: %s\n", sensorHealth.encoders_healthy ? "✅ ACTIVE" : "❌ DEGRADED");
    Serial.printf("🔋 Motors: %s\n", sensorHealth.motors_healthy ? "✅ ACTIVE" : "❌ FAILED");
    Serial.printf("📶 WiFi: %s\n", sensorHealth.wifi_healthy ? "✅ CONNECTED" : "❌ OFFLINE");
    Serial.printf("📡 ESP-NOW: %s\n", sensorHealth.espnow_healthy ? "✅ ACTIVE" : "❌ OFFLINE");
    
    // Calculate overall system health percentage
    int healthyCount = 0;
    if (sensorHealth.imu_healthy) healthyCount++;
    if (sensorHealth.distance_sensor_healthy) healthyCount++;
    if (sensorHealth.encoders_healthy) healthyCount++;
    if (sensorHealth.motors_healthy) healthyCount++;
    if (sensorHealth.wifi_healthy) healthyCount++;
    if (sensorHealth.espnow_healthy) healthyCount++;
    
    int healthPercent = (healthyCount * 100) / 6;
    Serial.printf("🤖 Overall System Health: %d%%\n\n", healthPercent);
    
    lastReport = millis();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// DATA LOGGING SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

// Logging configuration
typedef struct {
  bool enabled;
  bool log_sensors;
  bool log_performance;
  bool log_states;
  bool log_errors;
  bool log_communications;
  unsigned long log_interval;
  unsigned long last_log_time;
  char log_buffer[2048]; // Static buffer to avoid heap fragmentation
  int buffer_current_size; // Tracks used bytes in buffer
  int buffer_size; // Entry count
} DataLogger_t;

DataLogger_t dataLogger = {
  true,   // enabled
  true,   // log_sensors
  true,   // log_performance
  true,   // log_states
  true,   // log_errors
  false,  // log_communications
  1000,   // log_interval
  0,      // last_log_time
  "",     // log_buffer
  0,      // buffer_current_size
  0       // buffer_size
};

void initializeLogging() {
  Serial.println("📝 Initializing data logging system...");
  
  // Initialize SPIFFS for log storage
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS initialization failed");
    dataLogger.enabled = false;
    return;
  }
  
  // Create log file with timestamp
  String filename = "/logs/robot_" + String(millis()) + ".log";
  File logFile = SPIFFS.open(filename, "w");
  if (logFile) {
    logFile.println("# Wheelie Robot Data Log");
    logFile.println("# Timestamp,Uptime,State,IMU_X,IMU_Y,IMU_Z,Distance,LeftEncoder,RightEncoder,BatteryV,FreeHeap,LoopTime,Event");
    logFile.close();
    Serial.printf("✅ Log file created: %s\n", filename.c_str());
  } else {
    Serial.println("❌ Failed to create log file");
    dataLogger.enabled = false;
  }
}

void logSensorData() {
  if (!dataLogger.enabled || !dataLogger.log_sensors) return;
  
  String logEntry = "";
  logEntry += String(millis()) + ",";
  logEntry += String(sysStatus.uptime) + ",";
  logEntry += String(getCurrentState()) + ",";
  
  // IMU data
  if (sensorHealth.imu_healthy) {
    logEntry += String(mpu.getAccX(), 3) + ",";
    logEntry += String(mpu.getAccY(), 3) + ",";
    logEntry += String(mpu.getAccZ(), 3) + ",";
  } else {
    logEntry += "NaN,NaN,NaN,";
  }
  
  // Distance sensor
  if (sensorHealth.distance_sensor_healthy) {
    logEntry += String(getDistanceCm()) + ",";
  } else {
    logEntry += "NaN,";
  }
  
  // Encoder data
  if (sensorHealth.encoders_healthy) {
    logEntry += String(getLeftEncoderCount()) + ",";
    logEntry += String(getRightEncoderCount()) + ",";
  } else {
    logEntry += "NaN,NaN,";
  }
  
  // System data
  logEntry += String(getBatteryVoltage(), 2) + ",";
  logEntry += String(esp_get_free_heap_size()) + ",";
  logEntry += ",SENSOR_DATA";
  
  appendToLogBuffer(logEntry);
}

void logPerformanceData() {
  if (!dataLogger.enabled || !dataLogger.log_performance) return;
  
  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();
  unsigned long loopDuration = currentTime - lastLoopTime;
  
  String logEntry = "";
  logEntry += String(currentTime) + ",";
  logEntry += String(sysStatus.uptime) + ",";
  logEntry += String(getCurrentState()) + ",";
  logEntry += ",,,,,,"; // Skip sensor columns
  logEntry += String(getBatteryVoltage(), 2) + ",";
  logEntry += String(esp_get_free_heap_size()) + ",";
  logEntry += String(loopDuration) + ",";
  logEntry += "PERFORMANCE";
  
  appendToLogBuffer(logEntry);
  lastLoopTime = currentTime;
}

void logStateChange(RobotState oldState, RobotState newState) {
  if (!dataLogger.enabled || !dataLogger.log_states) return;
  
  String logEntry = "";
  logEntry += String(millis()) + ",";
  logEntry += String(sysStatus.uptime) + ",";
  logEntry += String(newState) + ",";
  logEntry += ",,,,,,"; // Skip sensor columns
  logEntry += String(getBatteryVoltage(), 2) + ",";
  logEntry += String(esp_get_free_heap_size()) + ",";
  logEntry += ",STATE_CHANGE_" + String(oldState) + "_TO_" + String(newState);
  
  appendToLogBuffer(logEntry);
}

void logError(String errorType, String errorMessage) {
  if (!dataLogger.enabled || !dataLogger.log_errors) return;
  
  String logEntry = "";
  logEntry += String(millis()) + ",";
  logEntry += String(sysStatus.uptime) + ",";
  logEntry += String(getCurrentState()) + ",";
  logEntry += ",,,,,,"; // Skip sensor columns
  logEntry += String(getBatteryVoltage(), 2) + ",";
  logEntry += String(esp_get_free_heap_size()) + ",";
  logEntry += ",ERROR_" + errorType + "_" + errorMessage;
  
  appendToLogBuffer(logEntry);
  
  // Also log to serial for immediate debugging
  Serial.printf("🔴 ERROR LOG: %s - %s\n", errorType.c_str(), errorMessage.c_str());
}

void logEvent(String eventType, String eventData = "") {
  if (!dataLogger.enabled) return;
  
  String logEntry = "";
  logEntry += String(millis()) + ",";
  logEntry += String(sysStatus.uptime) + ",";
  logEntry += String(getCurrentState()) + ",";
  logEntry += ",,,,,,"; // Skip sensor columns
  logEntry += String(getBatteryVoltage(), 2) + ",";
  logEntry += String(esp_get_free_heap_size()) + ",";
  logEntry += ",EVENT_" + eventType;
  if (eventData.length() > 0) {
    logEntry += "_" + eventData;
  }
  
  appendToLogBuffer(logEntry);
}

void appendToLogBuffer(String entry) {
  String entryWithNewline = entry + "\n";
  int entryLen = entryWithNewline.length();
  // Check if there's enough space in the char[] buffer
  if (dataLogger.buffer_current_size + entryLen < 2048) {
    // Append the new entry safely
    strncat(dataLogger.log_buffer, entryWithNewline.c_str(), entryLen);
    dataLogger.buffer_current_size += entryLen;
    dataLogger.buffer_size++; // Increment entry count
  } else {
    // Buffer is full, flush it first
    flushLogBuffer();
    // Then add the new entry to the now-empty buffer
    strncpy(dataLogger.log_buffer, entryWithNewline.c_str(), entryLen);
    dataLogger.log_buffer[entryLen] = '\0'; // Ensure null termination
    dataLogger.buffer_current_size = entryLen;
    dataLogger.buffer_size = 1;
  }
  // Flush buffer when it gets large or periodically
  if (dataLogger.buffer_size >= 10 || 
      (millis() - dataLogger.last_log_time > 5000)) {
    flushLogBuffer();
  }
}

void flushLogBuffer() {
  if (!dataLogger.enabled || dataLogger.buffer_current_size == 0) return;
  // Find the most recent log file
  File dir = SPIFFS.open("/logs");
  String latestFile = "";
  while (File file = dir.openNextFile()) {
    String fileName = String(file.name());
    if (fileName > latestFile) {
      latestFile = fileName;
    }
    file.close();
  }
  dir.close();
  if (latestFile.length() > 0) {
    File logFile = SPIFFS.open("/logs/" + latestFile, "a");
    if (logFile) {
      logFile.print(dataLogger.log_buffer); // Write the char buffer
      logFile.close();
    }
  }
  // Clear buffer
  dataLogger.log_buffer[0] = '\0'; // Fast way to clear a C-string
  dataLogger.buffer_current_size = 0;
  dataLogger.buffer_size = 0;
  dataLogger.last_log_time = millis();
}

void periodicDataLogging() {
  static unsigned long lastLog = 0;
  
  if (millis() - lastLog > dataLogger.log_interval) {
    logSensorData();
    logPerformanceData();
    lastLog = millis();
  }
}

void printLogSummary() {
  if (!dataLogger.enabled) {
    Serial.println("📝 Data logging is disabled");
    return;
  }
  
  Serial.println("\n📊 DATA LOGGING SUMMARY:");
  Serial.printf("📝 Status: %s\n", dataLogger.enabled ? "✅ ACTIVE" : "❌ DISABLED");
  Serial.printf("🔍 Sensors: %s\n", dataLogger.log_sensors ? "✅ ON" : "❌ OFF");
  Serial.printf("⚡ Performance: %s\n", dataLogger.log_performance ? "✅ ON" : "❌ OFF");
  Serial.printf("🤖 States: %s\n", dataLogger.log_states ? "✅ ON" : "❌ OFF");
  Serial.printf("🔴 Errors: %s\n", dataLogger.log_errors ? "✅ ON" : "❌ OFF");
  Serial.printf("📡 Communications: %s\n", dataLogger.log_communications ? "✅ ON" : "❌ OFF");
  Serial.printf("⏱️  Interval: %lu ms\n", dataLogger.log_interval);
  Serial.printf("💾 Buffer size: %d entries\n", dataLogger.buffer_size);
  
  // Show available space
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  Serial.printf("💿 Storage: %d/%d bytes used (%.1f%%)\n", 
                usedBytes, totalBytes, (float)usedBytes/totalBytes*100);
}

// Debug logging functions
void debugLog(String message) {
  logEvent("DEBUG", message);
  Serial.printf("🔍 DEBUG: %s\n", message.c_str());
}

void warningLog(String message) {
  logEvent("WARNING", message);
  Serial.printf("⚠️  WARNING: %s\n", message.c_str());
}

void infoLog(String message) {
  logEvent("INFO", message);
  Serial.printf("ℹ️  INFO: %s\n", message.c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// OTA UPDATE SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

// OTA configuration
typedef struct {
  bool enabled;
  bool update_in_progress;
  String hostname;
  String password;
  unsigned long last_check;
  int update_progress;
  bool require_auth;
  unsigned long update_timeout;
} OTAManager_t;

OTAManager_t otaManager = {
  .enabled = true,
  .update_in_progress = false,
  .hostname = "wheelie-robot",
  .password = "wheelie123", // Change this for security
  .last_check = 0,
  .update_progress = 0,
  .require_auth = true,
  .update_timeout = 300000 // 5 minutes
};

void initializeOTA() {
  if (!otaManager.enabled) {
    Serial.println("🔄 OTA updates disabled");
    return;
  }
  
  Serial.println("🔄 Initializing OTA update system...");
  
  // Set hostname
  ArduinoOTA.setHostname(otaManager.hostname.c_str());
  
  // Set authentication password
  if (otaManager.require_auth) {
    ArduinoOTA.setPassword(otaManager.password.c_str());
  }
  
  // Configure OTA callbacks
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    
    // Stop all robot operations during update
    emergencyStop();
    setRobotState(ROBOT_ERROR); // Prevent normal operation
    otaManager.update_in_progress = true;
    
    Serial.println("🔄 OTA Update Started: " + type);
    logEvent("OTA_START", type);
    
    // Visual indication
    setLEDColor(LEDColors::BLUE);
    buzz(1000, 100);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\n✅ OTA Update Complete!");
    logEvent("OTA_COMPLETE", "Success");
    
    // Victory indication
    for (int i = 0; i < 3; i++) {
      setLEDColor(LEDColors::GREEN);
      buzz(800, 200);
      delay(300);
      setLEDColor(LEDColors::OFF);
      delay(200);
    }
    
    otaManager.update_in_progress = false;
    Serial.println("🔄 Restarting in 3 seconds...");
    delay(3000);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percent = (progress / (total / 100));
    otaManager.update_progress = percent;
    
    Serial.printf("🔄 Progress: %u%%\r", percent);
    
    // Update LED with blue color during OTA update
    setLEDColor(LEDColors::BLUE);
    
    // Log progress every 10%
    if (percent % 10 == 0 && percent > 0) {
      logEvent("OTA_PROGRESS", String(percent) + "%");
    }
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("❌ OTA Error[%u]: ", error);
    String errorMsg = "";
    
    if (error == OTA_AUTH_ERROR) {
      errorMsg = "Auth_Failed";
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      errorMsg = "Begin_Failed";
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      errorMsg = "Connect_Failed";
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      errorMsg = "Receive_Failed";
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      errorMsg = "End_Failed";
      Serial.println("End Failed");
    }
    
    logError("OTA_ERROR", errorMsg);
    
    // Error indication
    for (int i = 0; i < 5; i++) {
      setLEDColor(LEDColors::RED);
      buzz(400, 100);
      delay(200);
      setLEDColor(LEDColors::OFF);
      delay(200);
    }
    
    otaManager.update_in_progress = false;
    // Return to normal operation
    setRobotState(ROBOT_IDLE);
  });
  
  ArduinoOTA.begin();
  Serial.printf("✅ OTA Ready - Hostname: %s\n", otaManager.hostname.c_str());
  Serial.printf("🔐 Authentication: %s\n", otaManager.require_auth ? "ENABLED" : "DISABLED");
  
  logEvent("OTA_INITIALIZED", otaManager.hostname);
}

void handleOTA() {
  if (!otaManager.enabled) return;
  
  // Handle OTA during update
  if (otaManager.update_in_progress) {
    ArduinoOTA.handle();
    
    // Check for update timeout
    if (millis() - otaManager.last_check > otaManager.update_timeout) {
      Serial.println("⚠️  OTA Update timeout - resuming normal operation");
      otaManager.update_in_progress = false;
      setRobotState(ROBOT_IDLE);
    }
    return;
  }
  
  // Normal OTA handling (check for incoming updates)
  ArduinoOTA.handle();
}

void checkForOTAUpdate() {
  static unsigned long lastCheck = 0;
  
  // Check every 30 seconds when idle
  if (millis() - lastCheck > 30000 && getCurrentState() == ROBOT_IDLE) {
    // Only check if WiFi is connected
    if (WiFi.status() == WL_CONNECTED) {
      handleOTA();
    }
    lastCheck = millis();
  }
}

void printOTAStatus() {
  Serial.println("\n🔄 OTA UPDATE STATUS:");
  Serial.printf("📡 Status: %s\n", otaManager.enabled ? "✅ ENABLED" : "❌ DISABLED");
  Serial.printf("🏠 Hostname: %s\n", otaManager.hostname.c_str());
  Serial.printf("🔐 Auth Required: %s\n", otaManager.require_auth ? "✅ YES" : "❌ NO");
  Serial.printf("📶 WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "✅ CONNECTED" : "❌ DISCONNECTED");
  
  if (otaManager.update_in_progress) {
    Serial.printf("🔄 Update Progress: %d%%\n", otaManager.update_progress);
    Serial.printf("⏱️  Timeout: %lu ms remaining\n", 
                  otaManager.update_timeout - (millis() - otaManager.last_check));
  } else {
    Serial.println("⏱️  Status: Ready for updates");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("🌐 IP Address: %s\n", WiFi.localIP().toString().c_str());
      Serial.println("💡 Arduino IDE: Tools -> Port -> Network Ports");
    }
  }
}

void enableOTA(bool enable) {
  otaManager.enabled = enable;
  if (enable) {
    Serial.println("✅ OTA updates enabled");
    logEvent("OTA_ENABLED");
  } else {
    Serial.println("❌ OTA updates disabled");
    logEvent("OTA_DISABLED");
  }
}

void setOTAPassword(String newPassword) {
  otaManager.password = newPassword;
  ArduinoOTA.setPassword(newPassword.c_str());
  Serial.println("🔐 OTA password updated");
  logEvent("OTA_PASSWORD_CHANGED");
}

void setOTAHostname(String newHostname) {
  otaManager.hostname = newHostname;
  ArduinoOTA.setHostname(newHostname.c_str());
  Serial.printf("🏠 OTA hostname set to: %s\n", newHostname.c_str());
  logEvent("OTA_HOSTNAME_CHANGED", newHostname);
}

// Emergency OTA functions
void forceOTAMode() {
  Serial.println("🚨 ENTERING EMERGENCY OTA MODE");
  emergencyStop();
  setRobotState(ROBOT_ERROR);
  
  // Stay in OTA mode for 10 minutes
  unsigned long startTime = millis();
  while (millis() - startTime < 600000) { // 10 minutes
    ArduinoOTA.handle();
    
    // Blink LED to indicate OTA mode
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      static bool ledState = false;
      setLEDColor(ledState ? LEDColors::BLUE : LEDColors::OFF);
      ledState = !ledState;
      lastBlink = millis();
    }
    
    delay(100);
  }
  
  Serial.println("⏰ Emergency OTA mode timeout - resuming normal operation");
  setRobotState(ROBOT_IDLE);
}

bool isOTAInProgress() {
  return otaManager.update_in_progress;
}

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY MONITORING & POWER MANAGEMENT SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

// Battery monitoring configuration
typedef struct {
  float voltage;
  float percentage;
  float voltage_min;          // 6.0V for 2S LiPo minimum
  float voltage_max;          // 8.4V for 2S LiPo maximum
  float voltage_low;          // 6.6V warning level
  float voltage_critical;     // 6.2V critical level
  bool low_power_mode;
  bool critical_power_mode;
  bool charging;
  unsigned long last_check;
  unsigned long check_interval;
  float consumption_rate;     // mAh per hour estimate
  unsigned long estimated_runtime; // minutes remaining
} BatteryMonitor_t;

BatteryMonitor_t battery = {
  .voltage = 0.0,
  .percentage = 100.0,
  .voltage_min = 6.0,      // 2S LiPo minimum safe voltage
  .voltage_max = 8.4,      // 2S LiPo maximum voltage
  .voltage_low = 6.6,      // Low battery warning
  .voltage_critical = 6.2, // Critical battery level
  .low_power_mode = false,
  .critical_power_mode = false,
  .charging = false,
  .last_check = 0,
  .check_interval = 2000,  // Check every 2 seconds
  .consumption_rate = 0.0,
  .estimated_runtime = 0
};

// Power management modes
typedef enum {
  POWER_NORMAL = 0,
  POWER_ECONOMY,
  POWER_LOW,
  POWER_CRITICAL,
  POWER_SHUTDOWN
} PowerMode_t;

PowerMode_t currentPowerMode = POWER_NORMAL;

void initializeBatteryMonitoring() {
  Serial.println("🔋 Initializing battery monitoring system...");
  
  // Read initial battery voltage
  updateBatteryVoltage();
  
  Serial.printf("🔋 Initial battery voltage: %.2fV (%.1f%%)\n", 
                battery.voltage, battery.percentage);
  
  // Log initial battery state
  logEvent("BATTERY_INIT", String(battery.voltage, 2) + "V");
  
  Serial.println("✅ Battery monitoring initialized");
}

void updateBatteryVoltage() {
  // Read battery voltage (this function should be implemented in sensors.cpp)
  battery.voltage = getBatteryVoltage();
  
  // Calculate percentage based on voltage range
  if (battery.voltage >= battery.voltage_max) {
    battery.percentage = 100.0;
  } else if (battery.voltage <= battery.voltage_min) {
    battery.percentage = 0.0;
  } else {
    battery.percentage = ((battery.voltage - battery.voltage_min) / 
                         (battery.voltage_max - battery.voltage_min)) * 100.0;
  }
  
  // Detect charging (voltage increasing)
  static float lastVoltage = 0.0;
  if (lastVoltage > 0.0 && battery.voltage > lastVoltage + 0.1) {
    battery.charging = true;
  } else if (battery.voltage < lastVoltage - 0.05) {
    battery.charging = false;
  }
  lastVoltage = battery.voltage;
}

void monitorBatteryHealth() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck > battery.check_interval) {
    updateBatteryVoltage();
    
    // Determine power mode based on voltage
    PowerMode_t newPowerMode = POWER_NORMAL;
    
    if (battery.voltage <= battery.voltage_min) {
      newPowerMode = POWER_SHUTDOWN;
    } else if (battery.voltage <= battery.voltage_critical) {
      newPowerMode = POWER_CRITICAL;
    } else if (battery.voltage <= battery.voltage_low) {
      newPowerMode = POWER_LOW;
    } else if (battery.percentage < 30.0) {
      newPowerMode = POWER_ECONOMY;
    }
    
    // Handle power mode changes
    if (newPowerMode != currentPowerMode) {
      handlePowerModeChange(currentPowerMode, newPowerMode);
      currentPowerMode = newPowerMode;
    }
    
    // Estimate runtime
    estimateRemainingRuntime();
    
    lastCheck = millis();
  }
}

void handlePowerModeChange(PowerMode_t oldMode, PowerMode_t newMode) {
  Serial.printf("🔋 Power mode change: %d -> %d\n", oldMode, newMode);
  logEvent("POWER_MODE_CHANGE", String(oldMode) + "_to_" + String(newMode));
  
  switch (newMode) {
    case POWER_NORMAL:
      exitLowPowerMode();
      Serial.println("🔋 Normal power mode - full operation");
      break;
      
    case POWER_ECONOMY:
      enterEconomyMode();
      Serial.println("🔋 Economy mode - reduced performance");
      break;
      
    case POWER_LOW:
      enterLowPowerMode();
      Serial.println("⚠️  Low power mode - limited operation");
      break;
      
    case POWER_CRITICAL:
      enterCriticalPowerMode();
      Serial.println("🚨 Critical power mode - survival operation only");
      break;
      
    case POWER_SHUTDOWN:
      initiateEmergencyShutdown();
      Serial.println("❌ Emergency shutdown - battery depleted");
      break;
  }
}

void enterEconomyMode() {
  // Reduce performance to extend battery life
  setMaxSpeed(0.7f);           // 70% speed
  setLEDBrightness(128);       // 50% LED brightness
  battery.check_interval = 5000; // Check less frequently
  dataLogger.log_interval = 2000; // Reduce logging frequency
  
  // Reduce sensor polling rates
  setSensorUpdateRate(0.8f);   // 80% of normal rate
  
  logEvent("ECONOMY_MODE_ENABLED");
}

void enterLowPowerMode() {
  battery.low_power_mode = true;
  
  // Significantly reduce power consumption
  setMaxSpeed(0.4f);           // 40% speed
  setLEDBrightness(64);        // 25% LED brightness
  battery.check_interval = 10000; // Check every 10 seconds
  dataLogger.log_interval = 5000;  // Reduce logging
  
  // Reduce sensor update rates
  setSensorUpdateRate(0.5f);   // 50% of normal rate
  
  // Disable non-essential features
  dataLogger.log_communications = false;
  
  // Set robot to safe mode for conservative operation
  setRobotState(ROBOT_SAFE_MODE);
  
  Serial.println("⚠️  Entering low power mode");
  logEvent("LOW_POWER_MODE_ENABLED");
  
  // Audio/visual warning
  for (int i = 0; i < 3; i++) {
    setLEDColor(LEDColors::YELLOW);
    buzz(600, 200);
    delay(300);
    setLEDColor(LEDColors::OFF);
    delay(200);
  }
}

void enterCriticalPowerMode() {
  battery.critical_power_mode = true;
  
  // Minimal power consumption
  setMaxSpeed(0.2f);           // 20% speed
  setLEDBrightness(32);        // 12.5% LED brightness
  battery.check_interval = 30000; // Check every 30 seconds
  
  // Disable most logging
  dataLogger.log_sensors = false;
  dataLogger.log_performance = false;
  dataLogger.log_communications = false;
  
  // Minimal sensor updates
  setSensorUpdateRate(0.25f);  // 25% of normal rate
  
  // Disable OTA to save power
  enableOTA(false);
  
  // Force robot to safe mode
  setRobotState(ROBOT_SAFE_MODE);
  
  Serial.println("🚨 Entering critical power mode - survival operation only");
  logEvent("CRITICAL_POWER_MODE_ENABLED");
  
  // Critical warning
  for (int i = 0; i < 5; i++) {
    setLEDColor(LEDColors::RED);
    buzz(400, 100);
    delay(150);
    setLEDColor(LEDColors::OFF);
    delay(150);
  }
}

void initiateEmergencyShutdown() {
  Serial.println("🚨 EMERGENCY SHUTDOWN - BATTERY DEPLETED");
  logEvent("EMERGENCY_SHUTDOWN", "Battery_depleted");
  
  // Emergency stop all motors
  emergencyStop();
  
  // Save critical data
  flushLogBuffer();
  
  // Final warning sequence
  for (int i = 0; i < 10; i++) {
    setLEDColor(LEDColors::RED);
    buzz(300, 50);
    delay(100);
    setLEDColor(LEDColors::OFF);
    delay(100);
  }
  
  // Set to error state and disable most systems
  setRobotState(ROBOT_ERROR);
  
  // Enter deep sleep to preserve remaining power
  Serial.println("💤 Entering deep sleep mode");
  delay(1000);
  
  // Wake up every 60 seconds to check if charging
  esp_sleep_enable_timer_wakeup(60 * 1000000); // 60 seconds in microseconds
  esp_deep_sleep_start();
}

void exitLowPowerMode() {
  if (battery.low_power_mode || battery.critical_power_mode) {
    Serial.println("🔋 Exiting low power mode - normal operation restored");
    
    // Restore normal operation
    battery.low_power_mode = false;
    battery.critical_power_mode = false;
    
    setMaxSpeed(1.0f);           // Full speed
    setLEDBrightness(255);       // Full LED brightness
    battery.check_interval = 2000; // Normal check interval
    dataLogger.log_interval = 1000; // Normal logging
    
    // Restore sensor update rates
    setSensorUpdateRate(1.0f);   // Full rate
    
    // Re-enable features
    dataLogger.log_sensors = true;
    dataLogger.log_performance = true;
    dataLogger.log_communications = false; // Keep this off by default
    
    // Re-enable OTA
    enableOTA(true);
    
    // Return to idle state
    setRobotState(ROBOT_IDLE);
    
    logEvent("LOW_POWER_MODE_DISABLED");
    
    // Success indication
    for (int i = 0; i < 3; i++) {
      setLEDColor(LEDColors::GREEN);
      buzz(800, 200);
      delay(300);
      setLEDColor(LEDColors::OFF);
      delay(200);
    }
  }
}

void estimateRemainingRuntime() {
  // Simple runtime estimation based on current consumption
  static float lastVoltage = 0.0;
  static unsigned long lastTime = 0;
  
  if (lastTime > 0 && lastVoltage > 0.0) {
    unsigned long timeDiff = millis() - lastTime;
    float voltageDiff = lastVoltage - battery.voltage;
    
    if (timeDiff > 60000 && voltageDiff > 0) { // At least 1 minute and voltage dropping
      // Calculate consumption rate (V/hour)
      float consumptionRate = (voltageDiff / timeDiff) * 3600000; // V/hour
      
      // Estimate remaining time based on voltage to critical level
      float remainingVoltage = battery.voltage - battery.voltage_critical;
      if (consumptionRate > 0) {
        battery.estimated_runtime = (remainingVoltage / consumptionRate) * 60; // minutes
      }
    }
  }
  
  // Update for next calculation
  if (millis() - lastTime > 60000) { // Update every minute
    lastVoltage = battery.voltage;
    lastTime = millis();
  }
}

void printBatteryStatus() {
  Serial.println("\n🔋 BATTERY STATUS:");
  Serial.printf("⚡ Voltage: %.2fV (%.1f%%)\n", battery.voltage, battery.percentage);
  Serial.printf("🔌 Charging: %s\n", battery.charging ? "✅ YES" : "❌ NO");
  Serial.printf("⚠️  Low Battery: %.2fV\n", battery.voltage_low);
  Serial.printf("🚨 Critical: %.2fV\n", battery.voltage_critical);
  Serial.printf("🔋 Power Mode: ");
  
  switch (currentPowerMode) {
    case POWER_NORMAL:    Serial.println("✅ NORMAL"); break;
    case POWER_ECONOMY:   Serial.println("💡 ECONOMY"); break;
    case POWER_LOW:       Serial.println("⚠️  LOW POWER"); break;
    case POWER_CRITICAL:  Serial.println("🚨 CRITICAL"); break;
    case POWER_SHUTDOWN:  Serial.println("❌ SHUTDOWN"); break;
  }
  
  if (battery.estimated_runtime > 0) {
    Serial.printf("⏱️  Estimated runtime: %lu minutes\n", battery.estimated_runtime);
  }
}

bool isBatteryLow() {
  return battery.voltage <= battery.voltage_low;
}

bool isBatteryCritical() {
  return battery.voltage <= battery.voltage_critical;
}

bool isCharging() {
  return battery.charging;
}

float getBatteryPercentage() {
  return battery.percentage;
}

PowerMode_t getCurrentPowerMode() {
  return currentPowerMode;
}

void setBatteryThresholds(float low, float critical, float minimum) {
  battery.voltage_low = low;
  battery.voltage_critical = critical;
  battery.voltage_min = minimum;
  
  Serial.printf("🔋 Battery thresholds updated: Low=%.2fV, Critical=%.2fV, Min=%.2fV\n",
                low, critical, minimum);
  logEvent("BATTERY_THRESHOLDS_UPDATED");
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP - System initialization with calibration check
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  
  // Set initial robot state to BOOTING
  setRobotState(ROBOT_BOOTING);
  Serial.println("🤖 Robot state set to: BOOTING");
  
  printBanner();
  startupAnimation();
  setupSystem();
  initializeSensors();
  
  // Initialize logger buffer fields (C++ does not guarantee zero-init for char[])
  dataLogger.log_buffer[0] = '\0';
  dataLogger.buffer_current_size = 0;
  dataLogger.buffer_size = 0;
  // Initialize data logging system
  initializeLogging();
  logEvent("STARTUP", "System_initialization_started");
  
  // Initialize OTA update system
  initializeOTA();
  
  // Initialize battery monitoring system
  initializeBatteryMonitoring();
  
  // ═══════════════════════════════════════════════════════════════════════════
  // 🎯 AUTONOMOUS CALIBRATION SYSTEM - Run once, store forever
  // ═══════════════════════════════════════════════════════════════════════════
  
  Serial.println("\n🔍 CHECKING CALIBRATION STATUS...");
  
  // Initialize encoder system first
  setupEncoders();
  
  // Check if user wants to force recalibration (hold BOOT button during startup)
  bool forceRecal = shouldForceRecalibration();
  
  // Check if robot is already calibrated
  bool alreadyCalibrated = checkCalibrationStatus();
  
  if (forceRecal || !alreadyCalibrated) {
    // Robot needs calibration - run full sequence
    Serial.println("\n🤖 ROBOT REQUIRES CALIBRATION");
    Serial.println("Please ensure:");
    Serial.println("• Robot has at least 1 meter of clear space");
    Serial.println("• A wall or obstacle is within 2 meters");
    Serial.println("• Robot is on a flat, stable surface");
    Serial.println("\nStarting calibration in 5 seconds...");
    Serial.println("(Power off now if conditions are not suitable)");
    
    // Countdown with warning animation
    for (int i = 5; i > 0; i--) {
      Serial.printf("⏰ Starting in %d...\n", i);
      setLEDColor(LEDColors::YELLOW);
      buzz(800, 200);
      delay(300);
      setLEDColor(LEDColors::OFF);
      delay(700);
    }
    
    // Run the full calibration sequence
    runFullCalibrationSequence();
    // Note: This function will reboot the ESP32 upon completion
    
  } else {
    // Robot is already calibrated - load saved data
    if (loadCalibrationData()) {
      Serial.println("✅ Calibration data loaded successfully!");
      Serial.println("🤖 Robot is ready for precise autonomous operation");
    } else {
      Serial.println("❌ Error loading calibration data");
      Serial.println("🔄 Please restart with BOOT button held to recalibrate");
      
      // -----------------------------------------------------------------
      // 🔴 DELETING THIS INFINITE LOOP 🔴
      // The "if (!isCalibrated)" block below needs to run.
      // while (true) {
      //     errorAnimation();
      //     delay(1000);
      // }
      // -----------------------------------------------------------------
    }
  }
  
  // Check if calibration was successful (important!)
  if (!isCalibrated) {
    Serial.println("\n⚠️  CALIBRATION FAILED - ENTERING SAFE MODE");
    Serial.println("Robot will not operate autonomously without calibration.");
    Serial.println("However, robot state is set to IDLE (0) for basic control.");
    Serial.println("Possible solutions:");
    Serial.println("• Check robot is on flat, level surface");
    Serial.println("• Ensure wheels can move freely");
    Serial.println("• Verify motor connections");
    Serial.println("• Check battery charge level");
    Serial.println("• Hold BOOT button during restart to retry calibration");
    
    // Set state to IDLE (0) even without calibration for basic control
    setRobotState(ROBOT_IDLE);
    Serial.println("🤖 Robot state maintained as: IDLE (0)");
    
    // Continue with limited functionality instead of infinite loop
    Serial.println("⚠️  Limited functionality mode - basic sensors and communication available");
  }
  
  // Initialize navigation system now that calibration is known
  initializeNavigation();
  
  // Continue with normal startup
  printSystemInfo();
  runDiagnostics();
  
  Serial.println("\n🚀 System initialization complete!");
  Serial.println("🤖 Robot is ready for autonomous operation\n");
  
  // Log system startup completion
  logEvent("STARTUP_COMPLETE", "Robot_ready_for_operation");
  printLogSummary();
  printOTAStatus();
  printBatteryStatus();
  
  // Victory fanfare
  victoryAnimation();
  
  setRobotState(ROBOT_IDLE); // Set IDLE state at the very end
  Serial.println("🤖 Robot state set to: IDLE (0)");
  delay(1000);
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP - Core robot operation
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  sysStatus.uptime = millis();
  
  // 1. Handle high-priority system tasks (OTA, Power)
  monitorSystemHealth();
  monitorBatteryHealth();
  handleOTA();
  
  // Skip normal operations if OTA update is in progress
  if (isOTAInProgress()) {
    return;
  }
  
  // Handle critical power modes
  if (currentPowerMode == POWER_CRITICAL || currentPowerMode == POWER_SHUTDOWN) {
    checkAllSafety();
    delay(1000); // Reduce CPU usage
    return;
  }
  
  // 2. SENSE: Update all inputs from the environment
  checkSensorHealth();
  updateAllSensors();
  updateCommunications(); // This handles both WiFi and ESP-NOW
  
  // 3. THINK: Process data and make decisions
  
  // Highest priority thought: Check for immediate danger
  if (checkAllSafety()) {
    logSafetyEvent();
    return;
  }
  
  // Adapt behavior based on long-term sensor health
  adaptToSensorFailures();
  
  // Main "brain" logic: decide what to do based on the current state
  executeStateBehavior();
  
  // 4. ACT: Execute decisions and update outputs
  indicators_update();
  
  // 5. BACKGROUND TASKS: Logging, performance monitoring, etc.
  if (currentPowerMode == POWER_NORMAL || currentPowerMode == POWER_ECONOMY) {
    monitorLoopPerformance();
    checkForOTAUpdate(); // Only check for OTA when not in low power
  }
  if (currentPowerMode != POWER_CRITICAL) {
    reportSystemCapabilities();
  }
  periodicDataLogging();
}
