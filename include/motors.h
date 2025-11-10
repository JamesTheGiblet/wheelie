#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "pins.h"
#include "config.h"

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL - Low-level hardware driver for motors
// ═══════════════════════════════════════════════════════════════════════════
//
// This driver uses a signed PWM model, common for modern MOSFET H-bridges.
// Positive values are forward, negative values are reverse.
//
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Initializes the motor driver pins and PWM channels.
 */
void setupMotors();

/**
 * @brief Sets the speed and direction for both motors.
 * @param pwmLeft Speed for the left motor. [-255, +255].
 * - Negative values = reverse
 * - Positive values = forward
 * - 0 = coast (or brake, depending on stopMode)
 * @param pwmRight Speed for the right motor. [-255, +255].
 */
void setMotorPWM(int pwmLeft, int pwmRight);

/**
 * @brief Stops both motors immediately (coast).
 * Alias for setMotorPWM(0, 0).
 */
void allStop();

/**
 * @brief Stops both motors using a dynamic brake (if supported).
 * (e.g., sets both IN pins for a motor to HIGH).
 */
void stopWithBrake();


// ═══════════════════════════════════════════════════════════════════════════
// ALIAS FUNCTIONS (Optional, but convenient)
// ═══════════════════════════════════════════════════════════════════════════
// These functions just provide a simpler wrapper for setMotorPWM.

/** @brief Moves robot forward at a given speed. */
inline void moveForward(int speed) {
    setMotorPWM(speed, speed);
}

/** @brief Moves robot backward at a given speed. */
inline void moveBackward(int speed) {
    setMotorPWM(-speed, -speed);
}

/** @brief Rotates robot left (on the spot) at a given speed. */
inline void rotateLeft(int speed) {
    setMotorPWM(-speed, speed);
}

/** @brief Rotates robot right (on the spot) at a given speed. */
inline void rotateRight(int speed) {
    setMotorPWM(speed, -speed);
}

#endif // MOTORS_H

/**
 * @brief Sets the motors based on a velocity vector (magnitude, angle in degrees).
 * @param magnitude The speed (0-255, or -255 to 255 for reverse).
 * @param angleDeg The direction of movement, in degrees (0 = forward, 90 = left, -90 = right, 180/-180 = backward).
 */
void setMotorsFromVector(float magnitude, float angleDeg);