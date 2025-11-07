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

void setupMotors() {
  Serial.println("🔧 Initializing MOS-FET motor driver...");
  
  // Configure motor control pins as PWM outputs
  // Setup PWM channels for all motor control pins
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);  // IN1 PWM channel
  ledcAttachPin(IN1_PIN, 0);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);  // IN2 PWM channel  
  ledcAttachPin(IN2_PIN, 1);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);  // IN3 PWM channel
  ledcAttachPin(IN3_PIN, 2);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);  // IN4 PWM channel
  ledcAttachPin(IN4_PIN, 3);
  
  // Initialize motors to stopped state
  allStop();
  Serial.println("✅ MOS-FET motor driver initialized");
}

void setMotorPWM(int pwmLeft, int pwmRight) {
  // Enhanced motor control with direction support
  // Positive speed = forward, negative = reverse, 0 = stop
  pwmLeft = constrain(pwmLeft, -255, 255);
  pwmRight = constrain(pwmRight, -255, 255);
  
  // Control left motor (A)
  if (pwmLeft > 0) {
    // Forward: IN1=PWM, IN2=0
    ledcWrite(0, pwmLeft);  // IN1 PWM
    ledcWrite(1, 0);        // IN2 off
  } else if (pwmLeft < 0) {
    // Reverse: IN1=0, IN2=PWM
    ledcWrite(0, 0);        // IN1 off
    ledcWrite(1, -pwmLeft); // IN2 PWM (negative speed)
  } else {
    // Stop: both pins off
    ledcWrite(0, 0);        // IN1 off
    ledcWrite(1, 0);        // IN2 off
  }
  
  // Control right motor (B)
  if (pwmRight > 0) {
    // Forward: IN3=PWM, IN4=0
    ledcWrite(2, pwmRight);  // IN3 PWM
    ledcWrite(3, 0);         // IN4 off
  } else if (pwmRight < 0) {
    // Reverse: IN3=0, IN4=PWM
    ledcWrite(2, 0);         // IN3 off
    ledcWrite(3, -pwmRight); // IN4 PWM (negative speed)
  } else {
    // Stop: both pins off
    ledcWrite(2, 0);         // IN3 off
    ledcWrite(3, 0);         // IN4 off
  }
}

void allStop() {
  // Stop both motors immediately (coast)
  setMotorPWM(0, 0);
}

void stopWithBrake() {
    // Emergency stop with brake (short both motor terminals)
    // This function ONLY applies the brake. It does NOT coast.
    // The calling function must call allStop() later to release the brake.
    ledcWrite(0, 255); // IN1 full
    ledcWrite(1, 255); // IN2 full
    ledcWrite(2, 255); // IN3 full
    ledcWrite(3, 255); // IN4 full
}

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR TEST FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void testMotorA() {
  Serial.println("   Testing Left Motor A...");
  Serial.println("     Forward 2 seconds");
  setMotorPWM(TEST_SPEED, 0);
  delay(2000);
  Serial.println("     Backward 2 seconds");
  setMotorPWM(-TEST_SPEED, 0);
  delay(2000);
  Serial.println("     Stop");
  allStop();
  delay(500);
}

void testMotorB() {
  Serial.println("   Testing Right Motor B...");
  Serial.println("     Forward 2 seconds");
  setMotorPWM(0, TEST_SPEED);
  delay(2000);
  Serial.println("     Backward 2 seconds");
  setMotorPWM(0, -TEST_SPEED);
  delay(2000);
  Serial.println("     Stop");
  allStop();
  delay(500);
}