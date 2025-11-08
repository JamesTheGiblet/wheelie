#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include <MPU6050_light.h>
#include <VL53L0X.h>

VL53L0X tof;

void setup() {
  // Initialize ToF sensor
  Serial.println("Initializing VL53L0X ToF sensor...");
  tof.setTimeout(500);
  if (tof.init()) {
    Serial.println("✅ ToF sensor initialized successfully.");
  } else {
    Serial.println("❌ ToF sensor initialization failed!");
  }
  Serial.begin(115200);
  delay(1000);

  // Initialize I2C with correct pins from pins.h
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("MPU6050 IMU Test - Only IMU enabled");

  // Initialize MPU6050
  byte status = mpu.begin();
  if (status == 0) {
    Serial.println("✅ IMU initialized successfully.");
  } else {
    Serial.print("❌ IMU initialization failed! Status: ");
    Serial.println(status);
    while (1) delay(1000); // Halt if IMU fails
  }

  Serial.println("Calibrating IMU...");
  mpu.calcOffsets(true, true); // gyro and accel
  Serial.println("IMU calibration complete.");
  Serial.println("Type 'c' and press Enter to recalibrate IMU at any time.");
}

void loop() {
  mpu.update();
  Serial.print("IMU: TiltX=");
  Serial.print(mpu.getAngleX());
  Serial.print("°, TiltY=");
  Serial.print(mpu.getAngleY());
  Serial.print("°, Heading=");
  Serial.print(mpu.getAngleZ());
  Serial.print("°");

  // ToF sensor reading
  Serial.print(" | ToF: ");
  uint16_t distance = tof.readRangeSingleMillimeters();
  if (tof.timeoutOccurred()) {
    Serial.print("Timeout");
  } else {
    Serial.print(distance);
    Serial.print(" mm");
  }
  Serial.println("");

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      Serial.println("\nRecalibrating IMU... Keep device still.");
      mpu.calcOffsets(true, true);
      Serial.println("IMU recalibration complete.");
    }
  }
  delay(500);
}
