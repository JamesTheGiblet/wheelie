#include "motors.h"
#include <calibration.h>

// Make calibration data available to the motor controller
extern CalibrationData calibData;
extern bool isCalibrated;

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL IMPLEMENTATION - MOS-FET H-Bridge Driver
// ═══════════════════════════════════════════════════════════════════════════
// Motor Driver: Dual H-Bridge with built-in MOSFETs
// - Input voltage: 2V-10V
// - Operating current: 1.5A per channel, peak 2.5A
// - Built-in thermal protection
// - Speed control via PWM on IN pins (no separate enable pins)
// ═══════════════════════════════════════════════════════════════════════════

// PWM Channel Assignments (ESP32 LEDC)
#define LEFT_MOTOR_PWM_CH  0
#define RIGHT_MOTOR_PWM_CH 1

void setupMotors() {
  Serial.println("🔧 Initializing MOS-FET motor driver...");

  // Initialize PWM channels for speed control
  ledcSetup(LEFT_MOTOR_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN1_PIN, LEFT_MOTOR_PWM_CH);
  ledcSetup(RIGHT_MOTOR_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN3_PIN, RIGHT_MOTOR_PWM_CH);

  // Configure the remaining direction pins as simple digital outputs
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Initialize motors to stopped state
  allStop();
  Serial.println("✅ MOS-FET motor driver initialized");
}

void setMotorPWM(int pwmLeft, int pwmRight) {
  // Clamp PWM values to the allowed range [-255, 255]
  pwmLeft = constrain(pwmLeft, -255, 255);
  pwmRight = constrain(pwmRight, -255, 255);

    // --- Left Motor (M1) ---
    if (pwmLeft > 0) { // Forward
        digitalWrite(IN2_PIN, LOW);
        ledcWrite(LEFT_MOTOR_PWM_CH, pwmLeft);
    } else if (pwmLeft < 0) { // Reverse
        digitalWrite(IN2_PIN, HIGH);
        ledcWrite(LEFT_MOTOR_PWM_CH, 255 + pwmLeft); // e.g., -100 becomes 155
    } else { // Stop
        digitalWrite(IN2_PIN, LOW);
        ledcWrite(LEFT_MOTOR_PWM_CH, 0);
    }

    // --- Right Motor (M2) ---
    if (pwmRight > 0) { // Forward
        digitalWrite(IN4_PIN, LOW);
        ledcWrite(RIGHT_MOTOR_PWM_CH, pwmRight);
    } else if (pwmRight < 0) { // Reverse
        digitalWrite(IN4_PIN, HIGH);
        ledcWrite(RIGHT_MOTOR_PWM_CH, 255 + pwmRight);
    } else { // Stop
        digitalWrite(IN4_PIN, LOW);
        ledcWrite(RIGHT_MOTOR_PWM_CH, 0);
    }
}

void allStop() {
  setMotorPWM(0, 0);
}

void stopWithBrake() {
    // Applies a dynamic brake by setting both inputs for each motor HIGH.
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, HIGH);
}

void setMotorsFromVector(float magnitude, float angleDeg) {
  float angleRad = angleDeg * M_PI / 180.0f;

  // Simple mixing for differential drive
  float forwardComponent = magnitude * cos(angleRad);
  float turnComponent = magnitude * sin(angleRad) * 2.0f; // Amplify turn component

  int pwmLeft = round(forwardComponent - turnComponent);
  int pwmRight = round(forwardComponent + turnComponent);

  setMotorPWM(pwmLeft, pwmRight);
}