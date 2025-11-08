#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include <VL53L0X.h>

VL53L0X tof;

int16_t tof_offset = 0; // Calibration offset
const int AVERAGE_SAMPLES = 8;

void calibrateToF() {
  Serial.println("Calibration: Place sensor at reference distance and type the true distance in mm, then press Enter.");
  while (!Serial.available()) {
    delay(10);
  }
  int refDist = Serial.parseInt();
  Serial.print("Reference distance entered: ");
  Serial.println(refDist);
  // Take average of several readings
  long sum = 0;
  for (int i = 0; i < AVERAGE_SAMPLES; i++) {
    sum += tof.readRangeSingleMillimeters();
    delay(50);
  }
  int measured = sum / AVERAGE_SAMPLES;
  tof_offset = measured - refDist;
  Serial.print("Calibration complete. Offset set to: ");
  Serial.println(tof_offset);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

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

void loop() {
  // Serial-triggered calibration
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      calibrateToF();
      // Clear any extra input
      while (Serial.available()) Serial.read();
    }
  }

  // Averaging
  long sum = 0;
  int valid = 0;
  for (int i = 0; i < AVERAGE_SAMPLES; i++) {
    uint16_t d = tof.readRangeSingleMillimeters();
    if (!tof.timeoutOccurred() && d != 0 && d < 2000) { // 2m max reasonable
      sum += d;
      valid++;
    }
    delay(20);
  }
  Serial.print("ToF: ");
  if (valid == 0) {
    Serial.println("No valid readings");
  } else {
    int avg = sum / valid;
    int calibrated = avg - tof_offset;
    Serial.print("avg=");
    Serial.print(avg);
    Serial.print(" mm, calibrated=");
    Serial.print(calibrated);
    Serial.println(" mm");
  }
  delay(500);
}
