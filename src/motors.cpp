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
  // Clamp speed values to the allowed range
  int speedLeft = constrain(abs(pwmLeft), 0, 255);
  int speedRight = constrain(abs(pwmRight), 0, 255);
  
  // Control left motor (A)
  if (pwmLeft < 0) { // Forward (Reversed)
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (pwmLeft > 0) { // Reverse (Reversed)
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else { // Stop (coast)
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
  ledcWrite(LEFT_MOTOR_PWM_CH, speedLeft);
  
  // Control right motor (B)
  if (pwmRight < 0) { // Forward (Reversed)
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else if (pwmRight > 0) { // Reverse (Reversed)
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  } else { // Stop (coast)
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }
  ledcWrite(RIGHT_MOTOR_PWM_CH, speedRight);
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