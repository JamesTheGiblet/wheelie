// ═══════════════════════════════════════════════════════════════════════════
// ENCODER IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════
// This file contains the low-level implementation for reading motor encoders
// using hardware interrupts for maximum accuracy.
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include "globals.h" // For pin definitions
#include "pins.h"

// --- Volatile variables for Interrupt Service Routines (ISRs) ---
// 'volatile' tells the compiler that these values can change at any time,
// preventing optimizations that might cause incorrect readings.
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

/**
 * @brief Interrupt Service Routine for the left encoder.
 * This function is called automatically on every rising edge of the encoder signal.
 * It should be as fast as possible.
 */
void IRAM_ATTR isr_left_encoder() {
    leftEncoderTicks++;
}

/**
 * @brief Interrupt Service Routine for the right encoder.
 * This function is called automatically on every rising edge of the encoder signal.
 */
void IRAM_ATTR isr_right_encoder() {
    rightEncoderTicks++;
}

/**
 * @brief Sets up the encoder pins and attaches the hardware interrupts.
 * This should be called once during initialization.
 */
void setupEncoders() {
    Serial.println("🔧 Initializing Encoders with Interrupts...");
    
    // Set encoder pins as inputs with pull-up resistors to prevent floating
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);

    // Attach the interrupt handlers to the pins.
    // They will trigger on the RISING edge of the signal (when the signal goes from LOW to HIGH).
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), isr_left_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), isr_right_encoder, RISING);

    Serial.println("✅ Encoders initialized.");
}

/**
 * @brief Resets the tick counts for both encoders to zero.
 */
void resetEncoders() {
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
}

/**
 * @brief Safely reads the current tick count for the left encoder.
 */
long getLeftEncoderCount() {
    return leftEncoderTicks;
}

/**
 * @brief Safely reads the current tick count for the right encoder.
 */
long getRightEncoderCount() {
    return rightEncoderTicks;
}

/**
 * @brief Calculates the average tick count of both encoders.
 */
long getAverageEncoderCount() {
    return (leftEncoderTicks + rightEncoderTicks) / 2;
}