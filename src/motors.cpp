#include "motors.h"

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
#define LEFT_MOTOR_CH1  0
#define LEFT_MOTOR_CH2  1
#define RIGHT_MOTOR_CH1 2
#define RIGHT_MOTOR_CH2 3

void setupMotors() {
  Serial.println("🔧 Initializing MOS-FET motor driver...");
  
  // Configure motor control pins as PWM outputs
  // Setup PWM channels for all motor control pins
  ledcSetup(LEFT_MOTOR_CH1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN1_PIN, LEFT_MOTOR_CH1);

  ledcSetup(LEFT_MOTOR_CH2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN2_PIN, LEFT_MOTOR_CH2);

  ledcSetup(RIGHT_MOTOR_CH1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN3_PIN, RIGHT_MOTOR_CH1);

  ledcSetup(RIGHT_MOTOR_CH2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN4_PIN, RIGHT_MOTOR_CH2);
  
  // Initialize motors to stopped state
  allStop();
  Serial.println("✅ MOS-FET motor driver initialized");
}

void setMotorPWM(int pwmLeft, int pwmRight) {
  // Clamp PWM values to the allowed range
  pwmLeft = constrain(pwmLeft, -255, 255);
  pwmRight = constrain(pwmRight, -255, 255);
  
  // Control left motor (A)
  if (pwmLeft > 0) { // Forward
    ledcWrite(LEFT_MOTOR_CH1, pwmLeft);
    ledcWrite(LEFT_MOTOR_CH2, 0);
  } else if (pwmLeft < 0) { // Reverse
    ledcWrite(LEFT_MOTOR_CH1, 0);
    ledcWrite(LEFT_MOTOR_CH2, -pwmLeft);
  } else { // Stop (coast)
    ledcWrite(LEFT_MOTOR_CH1, 0);
    ledcWrite(LEFT_MOTOR_CH2, 0);
  }
  
  // Control right motor (B)
  if (pwmRight > 0) { // Forward
    ledcWrite(RIGHT_MOTOR_CH1, pwmRight);
    ledcWrite(RIGHT_MOTOR_CH2, 0);
  } else if (pwmRight < 0) { // Reverse
    ledcWrite(RIGHT_MOTOR_CH1, 0);
    ledcWrite(RIGHT_MOTOR_CH2, -pwmRight);
  } else { // Stop (coast)
    ledcWrite(RIGHT_MOTOR_CH1, 0);
    ledcWrite(RIGHT_MOTOR_CH2, 0);
  }
}

void allStop() {
  setMotorPWM(0, 0);
}

void stopWithBrake() {
    // Applies a dynamic brake by setting both inputs for each motor HIGH.
    ledcWrite(LEFT_MOTOR_CH1, 255);
    ledcWrite(LEFT_MOTOR_CH2, 255);
    ledcWrite(RIGHT_MOTOR_CH1, 255);
    ledcWrite(RIGHT_MOTOR_CH2, 255);
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

void setMotorsFromVector(const Vector2D& v) {
    float magnitude = v.magnitude();
    float angleRad = v.angle();
    float angleDeg = angleRad * 180.0f / M_PI;

    // The navigator's velocity is in cm/s. We need to map this to PWM.
    // For now, we'll use a simple scaling factor, assuming MAX_SPEED_CM_S is defined.
    // Let's assume a simple direct mapping for now, capped at 255.
    float scaledMagnitude = constrain(magnitude, 0, 255);

    setMotorsFromVector(scaledMagnitude, angleDeg);
}