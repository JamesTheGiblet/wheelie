#include <Arduino.h>
#include "robot.h"

// ═══════════════════════════════════════════════════════════════════════════
// MAIN APPLICATION ENTRY POINT
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial to initialize

  // --- Core System Initialization ---
  printBanner();
  setupSystem(); // Initializes motors, indicators, WiFi, ESP-NOW

  // --- Sensor & Calibration Initialization ---
  initializeSensors();
  if (!loadCalibrationData()) { // Tries to load from EEPROM
    // If loading fails, run the full autonomous calibration
    setRobotState(ROBOT_CALIBRATING);
    if (runAutonomousCalibration()) {
      saveCalibrationData();
      Serial.println("✅ Calibration successful and saved. Rebooting...");
      delay(1000);
      ESP.restart();
    } else {
      Serial.println("❌ CRITICAL: Calibration failed. Robot halted.");
      setRobotState(ROBOT_ERROR);
      while (true) { delay(1000); } // Halt
    }
  }
  
  // --- High-Level Systems Initialization ---
  initializeNavigation();
  initializePowerManagement();
  initializeLogging();
  initializeWebServer();
  initializeOTA();
  initializeCLI();

  // --- Final Diagnostics and Startup ---
  printSystemInfo();
  runDiagnostics();

  setRobotState(ROBOT_IDLE);
  Serial.println("✅ System initialization complete. Robot is IDLE.");
}

void loop() {
  // --- Handle Inputs & Communications ---
  handleCLI();
  handleOTA();
  handleWebServer();
  updateCommunications(); // Manages WiFi and ESP-NOW

  // --- Main Robot Logic ---
  if (getCurrentState() != ROBOT_CALIBRATING && getCurrentState() != ROBOT_ERROR) {
    updateAllSensors();
    monitorPower();
    if (!checkAllSafety()) { // If no safety issue was detected...
      executeStateBehavior(); // Run the main state machine
    }
  }
  manageEmergencyBrake(); // Manages the non-blocking brake sequence
}
