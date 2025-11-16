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

enum MotorTestStep {
  LEFT_FWD, LEFT_REV, RIGHT_FWD, RIGHT_REV, BOTH_FWD, BOTH_REV, PAUSE, DONE
};
MotorTestStep currentStep = LEFT_FWD;
unsigned long nextActionTime = 0;
const unsigned long runDuration = 1500;
const unsigned long pauseDuration = 500;

void loop() {
  if (millis() < nextActionTime) {
    return; // Not time yet
  }

  stopMotors();
  nextActionTime = millis() + (currentStep == PAUSE ? pauseDuration : runDuration);

  switch (currentStep) {
    case LEFT_FWD:
      Serial.println("Left motor forward");
      setLeftMotor(200, 0);
      currentStep = PAUSE;
      break;
    case LEFT_REV:
      Serial.println("Left motor reverse");
      setLeftMotor(0, 200);
      currentStep = PAUSE;
      break;
    case RIGHT_FWD:
      Serial.println("Right motor forward");
      setRightMotor(200, 0);
      currentStep = PAUSE;
      break;
    case RIGHT_REV:
      Serial.println("Right motor reverse");
      setRightMotor(0, 200);
      currentStep = PAUSE;
      break;
    case BOTH_FWD:
      Serial.println("Both motors forward");
      setLeftMotor(200, 0); setRightMotor(200, 0);
      currentStep = PAUSE;
      break;
    case BOTH_REV:
      Serial.println("Both motors reverse");
      setLeftMotor(0, 200); setRightMotor(0, 200);
      currentStep = PAUSE;
      break;
    case PAUSE:
      currentStep = (MotorTestStep)((int)currentStep + 1);
      break;
    case DONE:
      Serial.println("Test complete. Restarting...");
      currentStep = LEFT_FWD;
      nextActionTime = millis() + 2000; // Longer pause before restart
      break;
  }
}
