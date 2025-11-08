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
#include "ota_manager.h"
#include "cli_manager.h"
#include "indicators.h" // This line is required
#include "logger.h"
#include <Update.h>
#include "power_manager.h"
#include <SPIFFS.h>
#include <FS.h>
#include "web_server.h" // <-- Include web server header

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
  if (battery.voltage < 6.0) {  // Minimum safe voltage
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


void checkSensorHealth() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck > 2000) { // Check every 2 seconds
    // Test IMU
    bool imu_test = testIMU();
    if (sensorHealth.mpuHealthy && !imu_test) {
      Serial.println("⚠️  IMU failure detected - switching to encoder-only navigation");
      setRobotState(ROBOT_SAFE_MODE);
    }
    sensorHealth.mpuHealthy = imu_test;

    // Test distance sensor (ToF)
    bool distance_test = testDistanceSensor();
    if (sensorHealth.tofHealthy && !distance_test) {
      Serial.println("⚠️  Distance sensor failure - using reduced obstacle avoidance");
    }
    sensorHealth.tofHealthy = distance_test;

    // Test encoders (use edgeHealthy as closest match)
    bool encoder_test = testEncoders();
    if (sensorHealth.edgeHealthy && !encoder_test) {
      Serial.println("⚠️  Encoder failure - switching to time-based movement");
    }
    sensorHealth.edgeHealthy = encoder_test;

    // Test WiFi (no direct field, use tofHealthy as placeholder if needed)
    // sensorHealth.wifiHealthy = WiFi.status() == WL_CONNECTED; // Uncomment if you add wifiHealthy to struct

    // Test ESP-NOW (no direct field, use tofHealthy as placeholder if needed)
    // sensorHealth.espnowHealthy = testESPNOW(); // Uncomment if you add espnowHealthy to struct
    
    lastCheck = millis();
  }
}

void adaptToSensorFailures() {
  // Adapt navigation based on available sensors
  if (!sensorHealth.mpuHealthy && !sensorHealth.edgeHealthy) {
    // No position feedback - emergency stop
    Serial.println("❌ Critical navigation sensors failed - emergency stop");
    emergencyStop();
    setRobotState(ROBOT_ERROR);
    return;
  }
  
  // Adapt obstacle avoidance
  if (!sensorHealth.tofHealthy) {
    // Use slower, more cautious movement
    setMaxSpeed(0.3f); // Reduce to 30% speed
    setObstacleAvoidanceMode(CONSERVATIVE_MODE);
  }
  
  // Adapt navigation method
  if (!sensorHealth.mpuHealthy) {
    // Use encoder-only navigation
    setNavigationMode(ENCODER_ONLY);
    Serial.println("🔄 Switched to encoder-only navigation");
  } else if (!sensorHealth.edgeHealthy) {
    // Use IMU-only navigation with time estimation
    setNavigationMode(IMU_ONLY);
    Serial.println("🔄 Switched to IMU-only navigation");
  }
  
  // Handle communication failures
  // Communication health adaptation not implemented (add fields to SensorHealth_t if needed)
}

void reportSystemCapabilities() {
  static unsigned long lastReport = 0;
  
  if (millis() - lastReport > 30000) { // Report every 30 seconds
    Serial.println("\n📊 SYSTEM CAPABILITIES STATUS:");
  Serial.printf("🧭 IMU: %s\n", sensorHealth.mpuHealthy ? "✅ ACTIVE" : "❌ DEGRADED");
  Serial.printf("📏 Distance: %s\n", sensorHealth.tofHealthy ? "✅ ACTIVE" : "❌ DEGRADED");
  Serial.printf("⚙️  Encoders: %s\n", sensorHealth.edgeHealthy ? "✅ ACTIVE" : "❌ DEGRADED");
    
    // Calculate overall system health percentage
    int healthyCount = 0;
  if (sensorHealth.mpuHealthy) healthyCount++;
  if (sensorHealth.tofHealthy) healthyCount++;
  if (sensorHealth.edgeHealthy) healthyCount++;
    
    int healthPercent = (healthyCount * 100) / 5;
    Serial.printf("🤖 Overall System Health: %d%%\n\n", healthPercent);
    
    lastReport = millis();
  }
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
  // Initialize data logging system
  initializeLogging();
  
  // Initialize OTA update system
  initializeOTA();
  
  // Initialize battery monitoring system
  initializePowerManagement();
  
  // Initialize Web Server (if WiFi is connected)
  initializeWebServer();
  
  // Initialize Command Line Interface
  initializeCLI();
  
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
  }
  
  // Initialize navigation system now that calibration is known
  initializeNavigation();
  
  Serial.println("\n🚀 System initialization complete!");
  Serial.println("🤖 Robot is ready for autonomous operation\n");
  
  // Log system startup completion
  logEvent("STARTUP_COMPLETE", "Robot_ready_for_operation");
  
  // Print final status reports
  printSystemInfo();
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
  monitorPower();
  handleOTA();
  handleWebServer(); // <-- Web server handler

  // Handle the non-blocking emergency brake sequence if it's active
  manageEmergencyBrake();
  
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

  // Handle networking (WiFi, ESP-NOW, Web Server)
  updateCommunications(); // This handles both WiFi and ESP-NOW
  handleWebServer();

  // Handle serial command line interface
  handleCLI();
  
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
