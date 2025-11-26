#include "main.h"
#include "power_manager.h"
#include "logger.h"

// ADC Pin for battery voltage measurement
#define BATTERY_ADC_PIN 36 // ADC1_CH0 - A common pin for voltage sensing
const float ADC_VOLTAGE_DIVIDER = 2.0; // Adjust based on your voltage divider circuit (e.g., 2.0 for two equal resistors)

// ═══════════════════════════════════════════════════════════════════════════
// POWER MANAGEMENT IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

// --- Forward Declarations for internal functions ---
void handlePowerModeChange(PowerMode_t oldMode, PowerMode_t newMode);
void estimateRemainingRuntime();
void enterEconomyMode();
void enterLowPowerMode();
void exitLowPowerMode();
void enterCriticalPowerMode();
void initiateEmergencyShutdown();

// Global power management data
BatteryMonitor_t battery = {
    .voltage = 7.4,
    .percentage = 100.0,
    .voltage_min = 6.0,
    .voltage_max = 8.4,
    .voltage_low = 6.8,
    .voltage_critical = 6.4,
    .low_power_mode = false,
    .critical_power_mode = false,
    .charging = false,
    .last_check = 0,
    .check_interval = 2000,
    .consumption_rate = 0.0,
    .estimated_runtime = 0
};

PowerMode_t currentPowerMode = POWER_NORMAL;


void initializePowerManagement() {
  Serial.println("🔋 Initializing battery monitoring system...");
  pinMode(BATTERY_ADC_PIN, INPUT); // Set up the ADC pin
  updateBatteryVoltage();
  Serial.printf("🔋 Initial battery voltage: %.2fV (%.1f%%)\n", 
                battery.voltage, battery.percentage);
  logEvent("BATTERY_INIT", String(battery.voltage, 2) + "V");
  Serial.println("✅ Battery monitoring initialized");
}

void updateBatteryVoltage() {
  // Read the raw ADC value from the specified pin
  int rawValue = analogRead(BATTERY_ADC_PIN);
  // Convert the 12-bit ADC value (0-4095) to a voltage, accounting for the 3.3V reference and the voltage divider circuit
  float voltage = (rawValue / 4095.0) * 3.3 * ADC_VOLTAGE_DIVIDER;
  battery.voltage = voltage; // Update the global battery struct with the real voltage
  if (battery.voltage >= battery.voltage_max) {
    battery.percentage = 100.0;
  } else if (battery.voltage <= battery.voltage_min) {
    battery.percentage = 0.0;
  } else {
    battery.percentage = ((battery.voltage - battery.voltage_min) / 
                         (battery.voltage_max - battery.voltage_min)) * 100.0;
  }

  static float lastVoltage = 0.0;
  if (lastVoltage > 0.0 && battery.voltage > lastVoltage + 0.1) {
    battery.charging = true;
  } else if (battery.voltage < lastVoltage - 0.05) {
    battery.charging = false;
  }
  lastVoltage = battery.voltage;
}

void monitorPower() {
  if (millis() - battery.last_check > battery.check_interval) {
    updateBatteryVoltage();
    
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
    
    if (newPowerMode != currentPowerMode) {
      handlePowerModeChange(currentPowerMode, newPowerMode);
      currentPowerMode = newPowerMode;
    }
    
    estimateRemainingRuntime();
    battery.last_check = millis();
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
  hal.setMaxSpeed(0.7f);
  hal.setLEDBrightness(128);
  battery.check_interval = 5000;
  logEvent("ECONOMY_MODE_ENABLED");
}

void enterLowPowerMode() {
  battery.low_power_mode = true;
  hal.setMaxSpeed(0.4f);
  hal.setLEDBrightness(64);
  battery.check_interval = 10000;
  setRobotState(ROBOT_SAFE_MODE);
  Serial.println("⚠️  Entering low power mode");
  logEvent("LOW_POWER_MODE_ENABLED");
}

void enterCriticalPowerMode() {
  battery.critical_power_mode = true;
  hal.setMaxSpeed(0.2f);
  hal.setLEDBrightness(32);
  battery.check_interval = 30000;
  // OTA removed
  setRobotState(ROBOT_SAFE_MODE);
  Serial.println("🚨 Entering critical power mode - survival operation only");
  logEvent("CRITICAL_POWER_MODE_ENABLED");
}

void initiateEmergencyShutdown() {
  Serial.println("🚨 EMERGENCY SHUTDOWN - BATTERY DEPLETED");
  logEvent("EMERGENCY_SHUTDOWN", "Battery_depleted");
  hal.emergencyStop();
  flushLogBuffer();
  setRobotState(ROBOT_ERROR);
  Serial.println("💤 Entering deep sleep mode");
  delay(1000);
  esp_sleep_enable_timer_wakeup(60 * 1000000);
  esp_deep_sleep_start();
}

void exitLowPowerMode() {
  if (battery.low_power_mode || battery.critical_power_mode) {
    Serial.println("🔋 Exiting low power mode - normal operation restored");
    battery.low_power_mode = false;
    battery.critical_power_mode = false;
    hal.setMaxSpeed(1.0f);
    hal.setLEDBrightness(255);
    battery.check_interval = 2000;
  // OTA removed
    setRobotState(ROBOT_IDLE);
    logEvent("LOW_POWER_MODE_DISABLED");
  }
}

void estimateRemainingRuntime() {
  static float lastVoltage = 0.0;
  static unsigned long lastTime = 0;
  
  if (lastTime > 0 && lastVoltage > 0.0) {
    unsigned long timeDiff = millis() - lastTime;
    float voltageDiff = lastVoltage - battery.voltage;
    
    if (timeDiff > 60000 && voltageDiff > 0) {
      float consumptionRate = (voltageDiff / timeDiff) * 3600000;
      if (consumptionRate > 0) {
        float remainingVoltage = battery.voltage - battery.voltage_critical;
        battery.estimated_runtime = (remainingVoltage / consumptionRate) * 60;
      }
    }
  }
  
  if (millis() - lastTime > 60000) {
    lastVoltage = battery.voltage;
    lastTime = millis();
  }
}

void printBatteryStatus() {
  Serial.println("\n🔋 BATTERY STATUS:");
  Serial.printf("⚡ Voltage: %.2fV (%.1f%%)\n", battery.voltage, battery.percentage);
  Serial.printf("🔌 Charging: %s\n", battery.charging ? "✅ YES" : "❌ NO");
  Serial.printf("🔋 Power Mode: ");
  switch (currentPowerMode) {
    case POWER_NORMAL: Serial.println("✅ NORMAL"); break;
    case POWER_ECONOMY: Serial.println("💡 ECONOMY"); break;
    case POWER_LOW: Serial.println("⚠️  LOW POWER"); break;
    case POWER_CRITICAL: Serial.println("🚨 CRITICAL"); break;
    case POWER_SHUTDOWN: Serial.println("❌ SHUTDOWN"); break;
  }
  if (battery.estimated_runtime > 0) {
    Serial.printf("⏱️  Estimated runtime: %lu minutes\n", battery.estimated_runtime);
  }
}

String getBatteryStatusString() {
    char buffer[256];
    String status = "🔋 BATTERY STATUS:\n";
    snprintf(buffer, sizeof(buffer), "⚡ Voltage: %.2fV (%.1f%%)\n", battery.voltage, battery.percentage);
    status += buffer;
    snprintf(buffer, sizeof(buffer), "🔌 Charging: %s\n", battery.charging ? "✅ YES" : "❌ NO");
    status += buffer;
    status += "🔋 Power Mode: ";
    switch (currentPowerMode) {
        case POWER_NORMAL:   status += "✅ NORMAL\n"; break;
        case POWER_ECONOMY:  status += "💡 ECONOMY\n"; break;
        case POWER_LOW:      status += "⚠️  LOW POWER\n"; break;
        case POWER_CRITICAL: status += "🚨 CRITICAL\n"; break;
        case POWER_SHUTDOWN: status += "❌ SHUTDOWN\n"; break;
    }
    if (battery.estimated_runtime > 0) {
        snprintf(buffer, sizeof(buffer), "⏱️  Estimated runtime: %lu minutes\n", battery.estimated_runtime);
        status += buffer;
    }
    return status;
}

bool isBatteryLow() { return battery.voltage <= battery.voltage_low; }
bool isBatteryCritical() { return battery.voltage <= battery.voltage_critical; }
bool isCharging() { return battery.charging; }
float getBatteryPercentage() { return battery.percentage; }
PowerMode_t getCurrentPowerMode() { return currentPowerMode; }

void setBatteryThresholds(float low, float critical, float minimum) {
  battery.voltage_low = low;
  battery.voltage_critical = critical;
  battery.voltage_min = minimum;
  Serial.printf("🔋 Battery thresholds updated: Low=%.2fV, Critical=%.2fV, Min=%.2fV\n", low, critical, minimum);
  logEvent("BATTERY_THRESHOLDS_UPDATED");
}