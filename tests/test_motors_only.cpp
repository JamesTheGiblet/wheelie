#include <Arduino.h>
#include "pins.h"

// Minimal test for dual motor driver (FASIZI L298N or MOSFET H-Bridge)
// Cycles each motor forward, reverse, and stop at different speeds

void setLeftMotor(int pwmA, int pwmB) {
  analogWrite(IN1_PIN, pwmA);
  analogWrite(IN2_PIN, pwmB);
}

void setRightMotor(int pwmA, int pwmB) {
  analogWrite(IN3_PIN, pwmA);
  analogWrite(IN4_PIN, pwmB);
}

void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  stopMotors();
  Serial.println("Motor Driver Test");
  Serial.println("Each motor will run forward, reverse, and stop. Observe wheels.");
}

void loop() {
  // Left motor forward
  Serial.println("Left motor forward");
  setLeftMotor(200, 0);
  setRightMotor(0, 0);
  delay(1500);
  stopMotors();
  delay(500);

  // Left motor reverse
  Serial.println("Left motor reverse");
  setLeftMotor(0, 200);
  setRightMotor(0, 0);
  delay(1500);
  stopMotors();
  delay(500);

  // Right motor forward
  Serial.println("Right motor forward");
  setLeftMotor(0, 0);
  setRightMotor(200, 0);
  delay(1500);
  stopMotors();
  delay(500);

  // Right motor reverse
  Serial.println("Right motor reverse");
  setLeftMotor(0, 0);
  setRightMotor(0, 200);
  delay(1500);
  stopMotors();
  delay(500);

  // Both motors forward
  Serial.println("Both motors forward");
  setLeftMotor(200, 0);
  setRightMotor(200, 0);
  delay(1500);
  stopMotors();
  delay(1000);

  // Both motors reverse
  Serial.println("Both motors reverse");
  setLeftMotor(0, 200);
  setRightMotor(0, 200);
  delay(1500);
  stopMotors();
  delay(2000);
}
