#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050_light.h>
#include "pins.h"
#include "config.h"
#include "types.h"

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR MANAGEMENT - Auto-Discovery & Data Polling
// ═══════════════════════════════════════════════════════════════════════════

// Global sensor objects (defined in sensors.cpp)
extern VL53L0X tofSensor;
extern MPU6050 mpu;

/**
 * @brief Scans the I2C bus to find connected sensors.
 * This is the core of the Layer 1 Auto-Discovery.
 * It updates the global `sysStatus` flags (tofAvailable, mpuAvailable).
 */
void autoDetectSensors();

/**
 * @brief Initializes the sensors that were found during auto-detection.
 * This function will only initialize sensors that are flagged as 'available'.
 */
void initializeSensors();

/**
 * @brief Polls all available sensors and updates the global `sensors` struct.
 * This is called once per main loop.
 */
void updateAllSensors();

#endif // SENSORS_H