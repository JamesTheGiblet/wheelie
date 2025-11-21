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
  
  // Control left motor (A)
  if (pwmLeft > 0) { // Forward
    ledcWrite(LEFT_MOTOR_PWM_CH, pwmLeft);
    digitalWrite(IN2_PIN, LOW);
  } else if (pwmLeft < 0) { // Reverse
    ledcWrite(LEFT_MOTOR_PWM_CH, 0); // Turn off PWM on IN1
    digitalWrite(IN2_PIN, HIGH);
    // This assumes IN2 is not a PWM pin. If it were, you'd PWM it.
  } else { // Stop
    ledcWrite(LEFT_MOTOR_PWM_CH, 0);
    digitalWrite(IN2_PIN, LOW);
  }
  
  // Control right motor (B)
  if (pwmRight > 0) { // Forward
    ledcWrite(RIGHT_MOTOR_PWM_CH, pwmRight);
    digitalWrite(IN4_PIN, LOW);
  } else if (pwmRight < 0) { // Reverse
    ledcWrite(RIGHT_MOTOR_PWM_CH, 0); // Turn off PWM on IN3
    digitalWrite(IN4_PIN, HIGH);
  } else { // Stop
    ledcWrite(RIGHT_MOTOR_PWM_CH, 0);
    digitalWrite(IN4_PIN, LOW);
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

void setMotorsFromVector(const Vector2D& v) {
    float magnitude = v.magnitude();
    float angleRad = v.angle();
    float angleDeg = angleRad * 180.0f / M_PI;

    // ═══════════════════════════════════════════════════════════════════
    // CRITICAL FIX: Scale velocity from mm/s to PWM (0-255)
    // ═══════════════════════════════════════════════════════════════════

    // Navigator outputs velocity in mm/s (typically 0-40 mm/s)
    // Motors need PWM values (0-255)

    // Use the calibrated minimum PWM value if available, otherwise use a safe default.
    const float minPwmValue = isCalibrated ? (float)calibData.minMotorSpeedPWM : 50.0f;
    const float MAX_NAVIGATOR_VELOCITY_MM_S = 40.0f; // Maximum expected velocity from navigator
    const float MAX_PWM = 255.0f;          // Maximum PWM

    // Scale magnitude from [0, MAX_NAVIGATOR_VELOCITY_MM_S] to [minPwmValue, MAX_PWM]
    float scaledMagnitude = 0.0f;
    if (magnitude > 0.1f) {  // Deadzone
      // Linear scaling with offset for minimum motor speed
      scaledMagnitude = minPwmValue + (magnitude / MAX_NAVIGATOR_VELOCITY_MM_S) * (MAX_PWM - minPwmValue);
      scaledMagnitude = constrain(scaledMagnitude, minPwmValue, MAX_PWM);
    }

    // Debug output (remove after testing)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      Serial.printf("🎮 Motor: Input=%.1fmm/s -> PWM=%.0f\n", magnitude, scaledMagnitude);
      lastPrint = millis();
    }

    setMotorsFromVector(scaledMagnitude, angleDeg);
}