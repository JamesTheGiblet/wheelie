#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "types.h" // For RobotState and other enums
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// POWER MANAGEMENT SYSTEM - Battery monitoring and power mode control
// ═══════════════════════════════════════════════════════════════════════════

// Power management modes
enum PowerMode_t {
  POWER_NORMAL = 0,
  POWER_ECONOMY,
  POWER_LOW,
  POWER_CRITICAL,
  POWER_SHUTDOWN
};

// Battery monitoring configuration
struct BatteryMonitor_t {
  float voltage;
  float percentage;
  float voltage_min;
  float voltage_max;
  float voltage_low;
  float voltage_critical;
  bool low_power_mode;
  bool critical_power_mode;
  bool charging;
  unsigned long last_check;
  unsigned long check_interval;
  float consumption_rate;
  unsigned long estimated_runtime;
};

// Public access to power data
extern BatteryMonitor_t battery;
extern PowerMode_t currentPowerMode;

// Core Functions
void initializePowerManagement();
void monitorPower();

// State Changers
void handlePowerModeChange(PowerMode_t oldMode, PowerMode_t newMode);
void enterEconomyMode();
void enterLowPowerMode();
void enterCriticalPowerMode();
void initiateEmergencyShutdown();
void exitLowPowerMode();

// Utility Functions
void updateBatteryVoltage();
void estimateRemainingRuntime();
void printBatteryStatus();
bool isBatteryLow();
bool isBatteryCritical();
bool isCharging();
float getBatteryPercentage();
PowerMode_t getCurrentPowerMode();
void setBatteryThresholds(float low, float critical, float minimum);

#endif // POWER_MANAGER_H