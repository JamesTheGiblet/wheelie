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
// SENSOR MANAGEMENT - Functions for sensor initialization and data reading
// ═══════════════════════════════════════════════════════════════════════════

// Global sensor objects (declared in sensors.cpp)
extern VL53L0X tofSensor;
extern MPU6050 mpu;

// Sensor Setup
void initializeSensors();

// Sensor Reading Functions
void updateAllSensors();
int readToFDistance();
void readIMUData();
bool readEdgeSensor();
bool readSoundSensor();
bool readMotionSensor();

// MPU Calibration Support
void getMPUBaseline(float* baselineX, float* baselineY);

// Sensor Status Functions
bool isToFAvailable();
bool isIMUAvailable();
bool isPIRAvailable();

#endif // SENSORS_H