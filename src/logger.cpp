#include "logger.h"
#include <LittleFS.h> // Use LittleFS instead of SPIFFS
#include <FS.h>       // FS header remains the same
#include "main.h"          // For getCurrentState()
#include "calibration.h"   // For get...EncoderCount() functions
#include "SwarmCommunicator.h" // For broadcasting logs
#include <WheelieHAL.h>

// ═══════════════════════════════════════════════════════════════════════════
// DATA LOGGING SYSTEM IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

// External global data (declarations are in the included headers)
extern SystemStatus sysStatus;
extern SensorData sensors;
extern SensorHealth_t sensorHealth;
extern WheelieHAL hal; // For getting battery voltage

// Global logger configuration
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
  
  // Initialize buffer fields
  dataLogger.log_buffer[0] = '\0';
  dataLogger.buffer_current_size = 0;
  dataLogger.buffer_size = 0;

  if (!LittleFS.begin(true)) { // Use LittleFS
    Serial.println("❌ LittleFS initialization failed");
    dataLogger.enabled = false;
    return;
  }
  
  // Ensure /logs directory exists
  if (!LittleFS.exists("/logs")) {
    Serial.println("ℹ️  /logs directory not found. Creating...");
    LittleFS.mkdir("/logs");
  }

  String filename = "/logs/robot_" + String(millis()) + ".log";
  File logFile = LittleFS.open(filename, "w"); // Use LittleFS
  if (logFile) {
    logFile.println("# Wheelie Robot Data Log");
    logFile.println("# Timestamp,Uptime,State,IMU_X,IMU_Y,IMU_Z,FrontDist(cm),LeftEncoder,RightEncoder,BatteryV,FreeHeap,LoopTime,Event");
    logFile.close();
    Serial.printf("✅ Log file created: %s\n", filename.c_str());
  } else {
    Serial.println("❌ Failed to create log file");
    dataLogger.enabled = false;
  }
}

void appendToLogBuffer(String entry) {
  int entryLen = entry.length() + 1; // +1 for newline
  
  // Check if the new entry will fit in the buffer
  if (dataLogger.buffer_current_size + entryLen < sizeof(dataLogger.log_buffer)) {
    // Append the new entry with a newline
    snprintf(dataLogger.log_buffer + dataLogger.buffer_current_size, sizeof(dataLogger.log_buffer) - dataLogger.buffer_current_size, "%s\n", entry.c_str());
    dataLogger.buffer_current_size += entryLen; // Update the size
    dataLogger.buffer_size++;
  } else {
    // Buffer is full, flush it first
    flushLogBuffer();
    // Now add the new entry to the (now empty) buffer
    snprintf(dataLogger.log_buffer, sizeof(dataLogger.log_buffer), "%s\n", entry.c_str());
    dataLogger.buffer_current_size = entryLen;
    dataLogger.buffer_size = 1;
  }
  // Flush if buffer is getting full or after a timeout
  if (dataLogger.buffer_size >= 10 || (millis() - dataLogger.last_log_time > 5000)) {
    flushLogBuffer();
  }

  // Also broadcast the log entry over ESP-NOW for wireless monitoring
  SwarmCommunicator::getInstance().broadcastLogMessage(entry);
}

void flushLogBuffer() {
  if (!dataLogger.enabled || dataLogger.buffer_current_size == 0) return;
  File dir = LittleFS.open("/logs"); // Use LittleFS
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
    File logFile = LittleFS.open("/logs/" + latestFile, "a"); // Use LittleFS
    if (logFile) {
      logFile.print(dataLogger.log_buffer);
      logFile.close();
    }
  }
  dataLogger.log_buffer[0] = '\0';
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

void logSensorData() {
  if (!dataLogger.enabled || !dataLogger.log_sensors) return;
  String logEntry = String(millis()) + "," + String(sysStatus.uptime) + "," + String(getCurrentState()) + ",";
  if (sensorHealth.mpuHealthy) {
    logEntry += String(sensors.tiltX, 3) + "," + String(sensors.tiltY, 3) + "," + String(sensors.headingAngle, 3) + ",";
  } else { logEntry += "NaN,NaN,NaN,"; }
  if (sensorHealth.tofHealthy) {
    logEntry += String(sensors.frontDistanceCm) + ",";
  } else { logEntry += "NaN,"; }
  if (sensorHealth.edgeHealthy) {
    logEntry += String(getLeftEncoderCount()) + "," + String(getRightEncoderCount()) + ",";
  } else { logEntry += "NaN,NaN,"; }
  logEntry += String(hal.getBatteryVoltage(), 2) + "," + String(esp_get_free_heap_size()) + ",,SENSOR_DATA";
  appendToLogBuffer(logEntry);
}

void logPerformanceData() {
  if (!dataLogger.enabled || !dataLogger.log_performance) return;
  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();
  unsigned long loopDuration = currentTime - lastLoopTime;
  String logEntry = String(currentTime) + "," + String(sysStatus.uptime) + "," + String(getCurrentState()) + ",,,,,,,";
  logEntry += String(hal.getBatteryVoltage(), 2) + "," + String(esp_get_free_heap_size()) + "," + String(loopDuration) + ",PERFORMANCE";
  appendToLogBuffer(logEntry);
  lastLoopTime = currentTime;
}

void logStateChange(RobotStateEnum oldState, RobotStateEnum newState) {
  if (!dataLogger.enabled || !dataLogger.log_states) return;
  String logEntry = String(millis()) + "," + String(sysStatus.uptime) + "," + String((int)newState) + ",,,,,,,";
  logEntry += String(hal.getBatteryVoltage(), 2) + "," + String(esp_get_free_heap_size()) + ",,STATE_CHANGE_" + String((int)oldState) + "_TO_" + String((int)newState);
  appendToLogBuffer(logEntry);
}

void logEvent(String eventType, String eventData) {
  if (!dataLogger.enabled) return;
  String logEntry = String(millis()) + "," + String(sysStatus.uptime) + "," + String(getCurrentState()) + ",,,,,,,";
  logEntry += String(hal.getBatteryVoltage(), 2) + "," + String(esp_get_free_heap_size()) + ",,EVENT_" + eventType;
  if (eventData.length() > 0) { logEntry += "_" + eventData; }
  appendToLogBuffer(logEntry);
}

void logError(String errorType, String errorMessage) {
  if (!dataLogger.enabled || !dataLogger.log_errors) return;
  logEvent("ERROR_" + errorType, errorMessage);
  Serial.printf("🔴 ERROR LOG: %s - %s\n", errorType.c_str(), errorMessage.c_str());
}

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
  size_t totalBytes = LittleFS.totalBytes(); // Use LittleFS
  size_t usedBytes = LittleFS.usedBytes();   // Use LittleFS
  Serial.printf("💿 Storage: %d/%d bytes used (%.1f%%)\n", usedBytes, totalBytes, (float)usedBytes/totalBytes*100);
}