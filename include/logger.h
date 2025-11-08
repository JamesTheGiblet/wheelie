#ifndef LOGGER_H
#define LOGGER_H

#include "robot.h"

// ═══════════════════════════════════════════════════════════════════════════
// DATA LOGGING SYSTEM - SPIFFS-based event and data logging
// ═══════════════════════════════════════════════════════════════════════════

// Logging configuration structure
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

extern DataLogger_t dataLogger;

// Core Functions
void initializeLogging();
void periodicDataLogging();
void flushLogBuffer();

// Specific Log Entry Functions
void logSensorData();
void logPerformanceData();
void logStateChange(RobotState oldState, RobotState newState);

/**
 * @brief Logs a generic event with a type and optional data.
 * This is the primary function for logging custom events.
 * @param eventType A string describing the event category (e.g., "STARTUP").
 * @param eventData Optional string with more details.
 */
void logEvent(String eventType, String eventData = "");

// Helper Logging Functions (wrappers around logEvent)
void logError(String errorType, String errorMessage);
void debugLog(String message);
void warningLog(String message);
void infoLog(String message);

// Internal buffer management
void appendToLogBuffer(String entry);

// Status Reporting
void printLogSummary();

#endif // LOGGER_H