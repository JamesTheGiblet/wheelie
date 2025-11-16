#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include <VL53L0X.h>

VL53L0X tof;

int16_t tof_offset = 0; // Calibration offset
const int AVERAGE_SAMPLES = 8;

// State for non-blocking calibration
enum CalibState { NOT_CALIBRATING, WAITING_FOR_DIST, SAMPLING };
CalibState calibState = NOT_CALIBRATING;
int refDist = 0;
long calibSum = 0;
int calibSampleCount = 0;
unsigned long nextCalibSampleTime = 0;

void calibrateToF() {
  if (calibState == NOT_CALIBRATING) {
    Serial.println("\n--- ToF Calibration ---");
    Serial.println("1. Place sensor at a known reference distance.");
    Serial.println("2. Type the true distance in mm, then press Enter.");
    calibState = WAITING_FOR_DIST;
  }
  if (calibState == WAITING_FOR_DIST && Serial.available()) {
    refDist = Serial.parseInt();
    Serial.printf("Reference distance entered: %d mm. Now sampling...\n", refDist);
    calibSum = 0;
    calibSampleCount = 0;
    calibState = SAMPLING;
    nextCalibSampleTime = millis();
  }
  if (calibState == SAMPLING && millis() >= nextCalibSampleTime) {
    if (calibSampleCount < AVERAGE_SAMPLES) {
      calibSum += tof.readRangeSingleMillimeters();
      calibSampleCount++;
      nextCalibSampleTime = millis() + 50; // 50ms between samples
    } else {
      int measured = calibSum / AVERAGE_SAMPLES;
      tof_offset = measured - refDist;
      Serial.printf("Calibration complete. Measured avg: %d mm. Offset set to: %d\n", measured, tof_offset);
      calibState = NOT_CALIBRATING;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C with correct pins from pins.h
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("VL53L0X ToF Test - Only ToF enabled");

  // Initialize ToF sensor
  tof.setTimeout(500);
  if (tof.init()) {
    Serial.println("✅ ToF sensor initialized successfully.");
  } else {
    Serial.println("❌ ToF sensor initialization failed!");
    while (1) delay(1000); // Halt if ToF fails
  }
  Serial.println("Type 'c' and Enter to calibrate ToF offset at any time.");
}

unsigned long lastReadingTime = 0;
const unsigned long readingInterval = 500;

void loop() {
  // Serial-triggered calibration
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      calibState = NOT_CALIBRATING; // Reset and start calibration process
      // Clear any extra input
      while (Serial.available()) Serial.read();
    }
  }

  if (millis() - lastReadingTime >= readingInterval) {
    // Non-blocking averaging
    static int sampleCount = 0;
    static long sampleSum = 0;
    static int validCount = 0;

    uint16_t d = tof.readRangeSingleMillimeters();
    if (!tof.timeoutOccurred() && d > 0 && d < 2000) { // 2m max reasonable
      sampleSum += d;
      validCount++;
    }
    sampleCount++;

    if (sampleCount >= AVERAGE_SAMPLES) {
      Serial.print("ToF: ");
      if (validCount == 0) {
        Serial.println("No valid readings");
      } else {
        int avg = sampleSum / validCount;
        int calibrated = avg - tof_offset;
        Serial.printf("avg=%d mm, calibrated=%d mm (%d/%d valid samples)\n", avg, calibrated, validCount, AVERAGE_SAMPLES);
      }
      // Reset for next burst
      sampleCount = 0;
      sampleSum = 0;
      validCount = 0;
      lastReadingTime = millis();
    }
  }

  // Handle the interactive calibration state machine
  if (calibState != NOT_CALIBRATING) {
    calibrateToF();
  }
}
