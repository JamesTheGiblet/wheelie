#ifndef PINS_H
#define PINS_H

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - Hardware pin assignments for ESP32
// ═══════════════════════════════════════════════════════════════════════════

// Motors - Dual H-Bridge Motor Driver (MOS-FET based)
#define IN1_PIN 23      // Left motor control pin 1 (PWM capable)
#define IN2_PIN 22      // Left motor control pin 2 (PWM capable)
#define IN3_PIN 19      // Right motor control pin 1 (PWM capable)
#define IN4_PIN 18      // Right motor control pin 2 (PWM capable)
// Note: No separate enable pins - speed control via PWM on IN pins

// Encoders - Motor feedback
#define ENCODER_A_PIN 5     // Left encoder pin (Blue wire)
#define ENCODER_B_PIN 33    // Right encoder pin (Purple wire)

// Indicators - Visual and Audio Feedback
#define LED_RED_PIN   14    // RGB LED - Red channel
#define LED_GREEN_PIN 25    // RGB LED - Green channel (Safe pin, moved from GPIO12)
#define LED_BLUE_PIN  13    // RGB LED - Blue channel
#define BUZZER_PIN    21    // Piezoelectric buzzer

// Sensors - Digital and Analog Inputs
#define EDGE_SENSOR_PIN  15     // Edge detection sensor (not installed)
#define SOUND_SENSOR_PIN 17     // Sound sensor
#define PIR_SENSOR_PIN   39     // PIR motion sensor (not installed)

// I2C Bus - Shared by VL53L0X and MPU6050
#define I2C_SDA          27     // I2C Data line
#define I2C_SCL          26     // I2C Clock line

// Ultrasonic Sensor (Front) - HC-SR04
#define FRONT_ULTRASONIC_TRIG_PIN 16 // Trigger pin for the front sensor
#define FRONT_ULTRASONIC_ECHO_PIN 32 // Echo pin for the front sensor

#endif // PINS_H