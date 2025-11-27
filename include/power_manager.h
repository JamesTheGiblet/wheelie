#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

typedef enum {
    POWER_NORMAL,
    POWER_ECONOMY,
    POWER_LOW,
    POWER_CRITICAL,
    POWER_SHUTDOWN
} PowerMode_t;

typedef struct {
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
} BatteryMonitor_t;

extern BatteryMonitor_t battery;
extern PowerMode_t currentPowerMode;

void initializePowerManagement();
void updateBatteryVoltage();
void monitorPower();
void printBatteryStatus();
String getBatteryStatusString(); // <-- ADD THIS
bool isBatteryLow();
bool isBatteryCritical();
bool isCharging();
float getBatteryPercentage();
PowerMode_t getCurrentPowerMode();

void setBatteryThresholds(float low, float critical, float minimum);

void calibrateADC(float actual_voltage);

#endif // POWER_MANAGER_H