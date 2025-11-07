#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <EEPROM.h>
#include "pins.h"
#include "config.h"
#include "types.h"
#include "motors.h"
#include "sensors.h"
#include "indicators.h"

// ═══════════════════════════════════════════════════════════════════════════
// AUTONOMOUS CALIBRATION SYSTEM - One-time self-calibration with EEPROM storage
// ═══════════════════════════════════════════════════════════════════════════

// EEPROM Memory Layout (512 bytes total available - increased for future expansion)
#define EEPROM_SIZE 512
// Base address for storing the entire calibration data struct
#define EEPROM_CALIB_DATA_ADDR 0

// Magic Numbers and Versions
#define CALIBRATION_MAGIC 0x7B                 // Magic number (123 decimal)
#define CALIBRATION_VERSION 2                  // Current calibration version (incremented for checksum addition)

// Calibration Result Codes (enhanced error reporting)
enum CalibrationResult {
    CALIB_SUCCESS = 0,
    CALIB_ERR_TIMEOUT,
    CALIB_ERR_NO_MOVEMENT,
    CALIB_ERR_SENSOR_INVALID,
    CALIB_ERR_UNSTABLE,
    CALIB_ERR_CHECKSUM_FAILED,
    CALIB_ERR_DEADZONE_NOT_FOUND,
    CALIB_ERR_INSUFFICIENT_SPACE,
    CALIB_ERR_MEMORY_CORRUPTION,
    CALIB_ERR_SENSOR_TIMEOUT,
    CALIB_ERR_MOTOR_ERROR,
    CALIB_ERR_RANGE_ERROR,
    CALIB_ERR_NO_TARGET,
    CALIB_ERR_SENSOR_ERROR,
    CALIB_ERROR_MPU_INIT
};

// Calibration Control
#define FORCE_RECALIBRATION_PIN 0              // GPIO0 (BOOT button) - hold during power-on
// CALIBRATION_TIMEOUT moved to config.h for centralized configuration

// Motor Direction Commands (stored as bit flags)
struct MotorDirections {
    bool leftFwd_M1Fwd : 1;     // Left turn: Motor 1 forward?
    bool leftFwd_M1Rev : 1;     // Left turn: Motor 1 reverse?
    bool leftFwd_M2Fwd : 1;     // Left turn: Motor 2 forward?
    bool leftFwd_M2Rev : 1;     // Left turn: Motor 2 reverse?
    bool fwdMove_M1Fwd : 1;     // Forward: Motor 1 forward?
    bool fwdMove_M1Rev : 1;     // Forward: Motor 1 reverse?
    bool fwdMove_M2Fwd : 1;     // Forward: Motor 2 forward?
    bool fwdMove_M2Rev : 1;     // Forward: Motor 2 reverse?
};

// MPU Orientation Flags
struct MPUFlags {
    bool xAxisInverted : 1;     // X-axis reads backwards
    bool yAxisInverted : 1;     // Y-axis reads backwards
    bool zAxisInverted : 1;     // Z-axis reads backwards
    bool gyroXInverted : 1;     // Gyro X-axis inverted
    bool gyroYInverted : 1;     // Gyro Y-axis inverted
    bool gyroZInverted : 1;     // Gyro Z-axis inverted
    bool reserved1 : 1;         // Reserved for future use
    bool reserved2 : 1;         // Reserved for future use
};

// MPU6050 Calibration Offsets (full 6-axis calibration)
struct MPUOffsets {
    int16_t accelX, accelY, accelZ;   // Accelerometer offset corrections
    int16_t gyroX, gyroY, gyroZ;      // Gyroscope offset corrections
    float baselineX, baselineY;       // Baseline angles when robot is level
};

// Calibration Data Structure (packed for consistent EEPROM layout)
struct __attribute__((packed)) CalibrationData {
    uint8_t magic;              // Magic number for validation
    uint8_t version;            // Calibration version
    MotorDirections motorDirs;  // Motor direction mappings
    float ticksPer90Degrees;    // Encoder ticks for 90° turn
    float ticksPerMillimeter;   // Encoder ticks per millimeter movement
    float tofOffsetMM;          // ToF sensor physical offset (mm)
    MPUFlags mpuFlags;          // MPU sensor orientation flags
    MPUOffsets mpuOffsets;      // MPU calibration offsets (6-axis)
    uint8_t minMotorSpeedPWM;   // Minimum PWM value to overcome static friction
    uint16_t checksum;          // CRC16 checksum for data integrity
};

// Global calibration data (available after loading)
extern CalibrationData calibData;
extern bool isCalibrated;

// Encoder Variables (interrupt-based counting)
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;

// ═══════════════════════════════════════════════════════════════════════════
// MAIN CALIBRATION FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// System Boot Functions
bool checkCalibrationStatus();           // Check if robot is already calibrated
CalibrationResult loadCalibrationData(); // Load saved calibration from EEPROM
CalibrationResult saveCalibrationData(); // Save calibration data to EEPROM
bool shouldForceRecalibration();         // Check if user wants to force recal

// Master Calibration Sequence
CalibrationResult runFullCalibrationSequence(); // Run all calibration phases

// Phase 1: Directional Mapping
CalibrationResult calibrateDirectionalMapping(); // Determine left/right motor commands

// Phase 2: Turn Distance Calibration  
CalibrationResult calibrateTurnDistance(); // Find ticks per 90° turn

// Phase 3: Forward/Backward Detection
CalibrationResult calibrateForwardBackward(); // Determine forward/backward commands

// Phase 4: Distance & ToF Calibration
CalibrationResult calibrateDistanceAndToF(); // Calibrate movement distance and ToF offset

// Phase 5: Motor Deadzone Detection (NEW)
CalibrationResult calibrateMotorDeadzone(); // Find minimum PWM for reliable movement

// Phase 6: MPU6050 Offset Calibration (NEW)
CalibrationResult calibrateMPU(); // Calibrate 6-axis MPU offsets for zero drift

// ═══════════════════════════════════════════════════════════════════════════
// ENCODER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Encoder Setup and Control
void setupEncoders();               // Initialize encoder interrupts

// Thread-safe (atomic) functions for accessing encoder counts
void resetEncoders();               // Reset encoder counts to zero
long getLeftEncoderCount();         // Get left encoder count
long getRightEncoderCount();        // Get right encoder count
long getAverageEncoderCount();      // Get average of both encoders

// Encoder Interrupt Handlers (called by hardware interrupts)
void IRAM_ATTR leftEncoderISR();    // Left encoder interrupt service routine
void IRAM_ATTR rightEncoderISR();   // Right encoder interrupt service routine

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATED MOVEMENT FUNCTIONS (use calibration data)
// ═══════════════════════════════════════════════════════════════════════════

// Calibrated Directional Movement
void calibratedTurnLeft(int speed = TURN_SPEED);     // Turn left using calibrated commands
void calibratedTurnRight(int speed = TURN_SPEED);    // Turn right using calibrated commands
void calibratedMoveForward(int speed = TEST_SPEED);  // Move forward using calibrated commands
void calibratedMoveBackward(int speed = TEST_SPEED); // Move backward using calibrated commands

// Precise Movement Functions
void calibratedTurn90Left();             // Turn exactly 90° left
void calibratedTurn90Right();            // Turn exactly 90° right
void calibratedMoveDistance(float mm);   // Move exact distance in millimeters
void calibratedMoveToTarget(float targetDistanceMM); // Move to specific ToF distance

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY AND DEBUG FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Calibration Utilities
void printCalibrationData();             // Print current calibration values
CalibrationResult eraseCalibrationData(); // Completely erase EEPROM calibration
CalibrationResult validateCalibrationData(); // Validate loaded calibration data
void calibrationProgressUpdate(const char* phase, int progress); // Progress feedback

// Data Integrity Functions (NEW)
uint16_t calculateCRC16(const uint8_t* data, size_t length); // Calculate CRC16 checksum
bool verifyDataIntegrity(const CalibrationData* data); // Verify checksum matches data
CalibrationResult updateChecksum(CalibrationData* data); // Calculate and update checksum

// Motor Command Utilities
void executeMotorCommand(bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed);
void stopMotorsGently();                 // Gradual motor stop for calibration
float getStableToFReading(int samples = 10); // Get stable ToF distance reading
float getStableMPUHeading(int samples = 5);  // Get stable MPU heading reading

// Motor Deadzone Utilities (NEW)
uint8_t findMotorDeadzone(int maxPWM = 255, int stepSize = 5); // Find minimum PWM for movement
bool testMotorMovement(uint8_t pwmValue, int durationMs = 500); // Test if motor moves at given PWM

// Safety and Validation
bool isCalibrationSafe();                // Check if it's safe to calibrate
void calibrationEmergencyStop();         // Emergency stop during calibration
bool waitForStableConditions();          // Wait for sensors to stabilize

// Error Handling and Safety (NEW)
const char* getCalibrationErrorString(CalibrationResult result); // Get human-readable error message
CalibrationResult handleCalibrationFailure(CalibrationResult result, const char* phase); // Handle calibration failures with fail-safe halt

#endif // CALIBRATION_H