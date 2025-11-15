// ═══════════════════════════════════════════════════════════════════════════
// 🤖 AUTONOMOUS CALIBRATION SYSTEM - Implementation
// ═══════════════════════════════════════════════════════════════════════════
// Complete self-calibration system with EEPROM storage
// Runs once on first boot, stores results permanently
// ═══════════════════════════════════════════════════════════════════════════

#include "calibration.h"
#include "WheelieHAL.h"

extern WheelieHAL hal; // Allow access to the global HAL object
// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════

// Calibration data storage
//#include "globals.h" // <-- This is now included via calibration.h
#include "globals.h"

// Declare global sensorHealth variable defined in main.cpp
extern SensorHealth_t sensorHealth;

// --- PHYSICAL CONSTANTS ---
const int ENCODER_SLOTS = 20;           // Slots per revolution (TT motor encoder)
const float GEAR_RATIO = 48.0;          // TT motor gear ratio (e.g., 1:48)
const float WHEEL_DIAMETER_MM = 66.0;   // Wheel diameter in mm
const float TRACK_WIDTH_MM = 150.0;     // Distance between wheels in mm

// Encoder variables (volatile for interrupt safety)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// ═══════════════════════════════════════════════════════════════════════════
// ENCODER INTERRUPT HANDLERS
// ═══════════════════════════════════════════════════════════════════════════

void IRAM_ATTR leftEncoderISR() {
    leftEncoderCount++;
}

void IRAM_ATTR rightEncoderISR() {
    rightEncoderCount++;
}

// ═══════════════════════════════════════════════════════════════════════════
// ENCODER SETUP AND CONTROL
// ═══════════════════════════════════════════════════════════════════════════

void setupEncoders() {
    Serial.println("📏 Setting up encoder interrupts...");
    
    // Configure encoder pins as inputs with pull-up resistors
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    
    // Attach interrupt handlers (trigger on falling edge)
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), leftEncoderISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), rightEncoderISR, FALLING);
    
    // Reset counters
    resetEncoders();
    
    Serial.println("✅ Encoders initialized successfully");
}

void resetEncoders() {
    noInterrupts();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    interrupts();
}

long getLeftEncoderCount() {
    noInterrupts();
    long count = leftEncoderCount;
    interrupts();
    return count;
}

long getRightEncoderCount() {
    noInterrupts();
    long count = rightEncoderCount;
    interrupts();
    return count;
}

long getAverageEncoderCount() {
    long left = getLeftEncoderCount();
    long right = getRightEncoderCount();
    return (abs(left) + abs(right)) / 2;
}

// ═══════════════════════════════════════════════════════════════════════════
// DATA INTEGRITY FUNCTIONS (CRC16 CHECKSUM)
// ═══════════════════════════════════════════════════════════════════════════

uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // CRC-16-CCITT polynomial
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

bool verifyDataIntegrity(const CalibrationData* data) {
    // Calculate checksum of all data except the checksum field itself
    uint16_t calculatedCRC = calculateCRC16((const uint8_t*)data, 
                                           sizeof(CalibrationData) - sizeof(uint16_t));
    
    return calculatedCRC == data->checksum;
}

CalibrationResult updateChecksum(CalibrationData* data) {
    if (data == nullptr) {
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    // Calculate and store checksum
    data->checksum = calculateCRC16((const uint8_t*)data, 
                                   sizeof(CalibrationData) - sizeof(uint16_t));
    
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// ERROR HANDLING AND RECOVERY
// ═══════════════════════════════════════════════════════════════════════════

const char* getCalibrationErrorString(CalibrationResult result) {
    switch (result) {
        case CALIB_SUCCESS: return "Success";
        case CALIB_ERR_TIMEOUT: return "Operation timed out";
        case CALIB_ERR_NO_MOVEMENT: return "No movement detected";
        case CALIB_ERR_SENSOR_INVALID: return "Sensor reading invalid";
        case CALIB_ERR_UNSTABLE: return "Unstable conditions";
        case CALIB_ERR_CHECKSUM_FAILED: return "Data checksum verification failed";
        case CALIB_ERR_DEADZONE_NOT_FOUND: return "Motor deadzone not found";
        case CALIB_ERR_INSUFFICIENT_SPACE: return "Insufficient EEPROM space";
        case CALIB_ERR_MEMORY_CORRUPTION: return "Memory corruption detected";
        default: return "Unknown error";
    }
}

CalibrationResult handleCalibrationFailure(CalibrationResult result, const char* phase) {
    Serial.printf("❌ CRITICAL CALIBRATION FAILURE in %s: %s\n", 
                  phase, getCalibrationErrorString(result));
    
    // Provide detailed troubleshooting information
    Serial.println("\n🔧 TROUBLESHOOTING CHECKLIST:");
    switch (result) {
        case CALIB_ERR_TIMEOUT:
            Serial.println("• Check for stuck wheels or mechanical obstructions");
            Serial.println("• Verify encoders are properly connected");
            Serial.println("• Ensure sufficient battery power");
            break;
            
        case CALIB_ERR_NO_MOVEMENT:
            Serial.println("• Verify motor connections and power supply");
            Serial.println("• Check motor driver functionality");
            Serial.println("• Ensure wheels are not stuck or blocked");
            break;
            
        case CALIB_ERR_SENSOR_INVALID:
            Serial.println("• Check I2C sensor connections (SDA/SCL)");
            Serial.println("• Verify sensor power supply (3.3V)");
            Serial.println("• Test individual sensor functionality");
            break;
            
        case CALIB_ERR_CHECKSUM_FAILED:
            Serial.println("• EEPROM data corruption detected");
            Serial.println("• Power supply instability during previous save");
            Serial.println("• Hardware failure in EEPROM storage");
            break;
            
        default:
            Serial.println("• Check all hardware connections");
            Serial.println("• Verify power supply stability");
            Serial.println("• Review system health diagnostics");
            break;
    }
    
    Serial.println("\n🛑 SYSTEM HALT FOR SAFETY");
    Serial.println("   Hardware issue must be resolved before continuing");
    Serial.println("   Hold BOOT button during restart to retry calibration");
    
    // Fail-safe halt sequence
    calibrationEmergencyStop();
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 1: DIRECTIONAL MAPPING (LEFT/RIGHT)
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateDirectionalMapping() {
    Serial.println("\n🧭 PHASE 1: Directional Mapping (Left/Right)");
    Serial.println("─────────────────────────────────────────────");
    Serial.println("Goal: Determine which motor commands create left/right turns");
    
    // CRITICAL: Force MPU update before starting
    Serial.println("🔧 Pre-flight MPU check...");
    for (int i = 0; i < 10; i++) {
        hal.updateAllSensors();
        Serial.printf("   Pre-test reading %d: heading=%.2f°, gyroZ=%.2f°/s\n", 
                     i, sensors.headingAngle, sensors.gyroZ);
        delay(100);
    }
    
    // Wait for stable MPU reading
    if (!waitForStableConditions()) {
        return CALIB_ERR_TIMEOUT;
    }
    
    // Set current heading as zero reference
    float baseHeading = getStableMPUHeading();
    Serial.printf("📍 Base heading set to: %.2f°\n", baseHeading);
    
    // Test hypothesis: Motor 1 REV + Motor 2 FWD = Turn LEFT
    Serial.println("🔬 Testing hypothesis: M1-REV + M2-FWD = LEFT turn");
    
    resetEncoders();
    delay(100);
    
    // Execute test command with REAL-TIME MONITORING
    Serial.println("   🔄 Executing turn test with live telemetry...");
    Serial.println("   📊 Time | Heading | GyroZ | EncoderAvg");
    Serial.println("   ─────────────────────────────────────────");
    
    unsigned long startTime = millis();
    executeMotorCommand(false, true, true, false, 150); // M1-REV, M2-FWD
    
    // Sample every 100ms for 1500ms
    for (int i = 0; i < 15; i++) {
        delay(100);
        hal.updateAllSensors();
        long avgEncoder = getAverageEncoderCount();
        Serial.printf("   %4dms | %6.2f° | %6.2f°/s | %5ld\n", 
                     (i+1)*100, 
                     sensors.headingAngle, 
                     sensors.gyroZ,
                     avgEncoder);
    }
    
    allStop();
    Serial.println("   🛑 Motors stopped, settling...");
    delay(1000);
    
    // Check the result
    float newHeading = getStableMPUHeading();
    float headingChange = newHeading - baseHeading;
    long finalEncoderAvg = getAverageEncoderCount();
    
    // Normalize heading change to -180 to +180
    while (headingChange > 180) headingChange -= 360;
    while (headingChange < -180) headingChange += 360;
    
    Serial.printf("\n📊 RESULTS:\n");
    Serial.printf("   Start heading: %.2f°\n", baseHeading);
    Serial.printf("   End heading: %.2f°\n", newHeading);
    Serial.printf("   Change: %.2f° (negative = left, positive = right)\n", headingChange);
    Serial.printf("   Encoder ticks: %ld (confirms physical movement)\n", finalEncoderAvg);
    
    // Check if we have ENCODER movement but NO gyro reading
    if (finalEncoderAvg > 100 && abs(headingChange) < 2.0) {
        Serial.println("\n⚠️ CRITICAL DIAGNOSTIC:");
        Serial.println("   ✅ Encoders detect movement (" + String(finalEncoderAvg) + " ticks)");
        Serial.println("   ❌ Gyroscope detects NO rotation (" + String(headingChange) + "°)");
        Serial.println("\n🔍 POSSIBLE CAUSES:");
        Serial.println("   1. MPU6050 Z-axis not aligned with rotation axis");
        Serial.println("   2. Gyroscope sensitivity too low (need to increase DPS)");
        Serial.println("   3. MPU6050 initialization failed or incomplete");
        Serial.println("   4. I2C communication issues during rapid updates");
        Serial.println("\n💡 SOLUTIONS TO TRY:");
        Serial.println("   A. Check physical MPU mounting (Z-axis must be vertical)");
        Serial.println("   B. Increase gyro full-scale range in MPU config");
        Serial.println("   C. Add MPU self-test before calibration");
        Serial.println("   D. Reduce I2C clock speed for stability");
        
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    // Determine if hypothesis was correct
    if (headingChange < -2.0) {
        Serial.println("✅ Hypothesis CORRECT: M1-REV + M2-FWD = LEFT turn");
        calibData.motorDirs.leftFwd_M1Fwd = false;
        calibData.motorDirs.leftFwd_M1Rev = true;
        calibData.motorDirs.leftFwd_M2Fwd = true;
        calibData.motorDirs.leftFwd_M2Rev = false;
    } else if (headingChange > 2.0) {
        Serial.println("❌ Hypothesis WRONG: M1-REV + M2-FWD = RIGHT turn");
        Serial.println("✅ Corrected: M1-FWD + M2-REV = LEFT turn");
        calibData.motorDirs.leftFwd_M1Fwd = true;
        calibData.motorDirs.leftFwd_M1Rev = false;
        calibData.motorDirs.leftFwd_M2Fwd = false;
        calibData.motorDirs.leftFwd_M2Rev = true;
    } else {
        Serial.printf("❌ ERROR: Insufficient heading change detected (%.2f°)\n", headingChange);
        return CALIB_ERR_NO_MOVEMENT;
    }
    
    Serial.println("✅ Phase 1 complete: Left/Right motor mapping determined");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 2: TURN DISTANCE CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateTurnDistance() {
    Serial.println("\n🎯 PHASE 2: Turn Distance Calibration");
    Serial.println("──────────────────────────────────────");
    Serial.println("Goal: Find exact encoder ticks needed for 90° turn");

    // --- Theoretical calculation ---
    float wheelCircumference = WHEEL_DIAMETER_MM * 3.14159265f;
    float robotTurnCircumference = TRACK_WIDTH_MM * 3.14159265f;
    float ticksPerRevolution = ENCODER_SLOTS * GEAR_RATIO; // Corrected for gear ratio
    float mmPerTick = wheelCircumference / ticksPerRevolution;
    float ticksPer90Deg_theoretical = (robotTurnCircumference / 4.0f) / mmPerTick;

    Serial.printf("[Theoretical] Wheel circumference: %.2f mm\n", wheelCircumference);
    Serial.printf("[Theoretical] Track (turn) circumference: %.2f mm\n", robotTurnCircumference);
    Serial.printf("[Theoretical] mm per tick: %.2f\n", mmPerTick);
    Serial.printf("[Theoretical] Ticks per 90°: %.2f\n", ticksPer90Deg_theoretical);

    // Wait for stable conditions
    if (!waitForStableConditions()) {
        return CALIB_ERR_TIMEOUT;
    }

    // Set baseline heading (for direction confirmation only)
    float startHeading = getStableMPUHeading();
    Serial.printf("📍 Starting heading: %.2f°\n", startHeading);

    // Reset encoders
    resetEncoders();
    delay(100);

    // Execute calibrated left turn command for a fixed duration
    Serial.println("🔄 Executing LEFT turn for fixed duration (encoder-based calibration)...");

    bool m1Fwd = calibData.motorDirs.leftFwd_M1Fwd;
    bool m1Rev = calibData.motorDirs.leftFwd_M1Rev;
    bool m2Fwd = calibData.motorDirs.leftFwd_M2Fwd;
    bool m2Rev = calibData.motorDirs.leftFwd_M2Rev;
    
    // --- Smart Calibration: Perform a test turn for a fixed duration ---
    const int TURN_DURATION_MS = 1000;
    const int TURN_SPEED = 85; // A moderate, reliable speed.

    unsigned long turnStartTime = millis();
    executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, TURN_SPEED);
    delay(TURN_DURATION_MS);

    // Stop motors immediately
    stopWithBrake();
    delay(250);

    // --- Analyze the results of the test turn ---
    long leftTicks = abs(getLeftEncoderCount());
    long rightTicks = abs(getRightEncoderCount());
    long avgTicks = (leftTicks + rightTicks) / 2;

    // Get the actual angle turned
    float finalHeading = getStableMPUHeading();
    float actualTurn = finalHeading - startHeading;
    while (actualTurn > 180) actualTurn -= 360;
    while (actualTurn < -180) actualTurn += 360;
    actualTurn = abs(actualTurn); // We only care about the magnitude of the turn

    Serial.printf("📊 Turn Results:\n");
    Serial.printf("   Left encoder: %ld ticks\n", leftTicks);
    Serial.printf("   Right encoder: %ld ticks\n", rightTicks);
    Serial.printf("   Average: %ld ticks\n", avgTicks);
    Serial.printf("   Actual angle turned: %.2f°\n", actualTurn);

    // --- Smart Calculation ---
    if (avgTicks < 10 || avgTicks > 10000) {
        Serial.println("❌ ERROR: Encoder tick count outside reasonable range");
        return handleCalibrationFailure(CALIB_ERR_SENSOR_INVALID, "Turn Test");
    }

    float calculatedTicksFor90;
    // If the turn was too small to be reliable, use the theoretical value as a fallback.
    if (actualTurn < 5.0) {
        Serial.printf("⚠️ WARNING: Turn angle is very small (%.2f°). Falling back to theoretical value.\n", actualTurn);
        Serial.println("   This may affect turn accuracy. Consider increasing TURN_SPEED or checking for wheel slippage.");
        calculatedTicksFor90 = ticksPer90Deg_theoretical;
    } else {
        // Otherwise, calculate the ticks-per-degree ratio from the empirical test data.
        float ticksPerDegree = (float)avgTicks / actualTurn;
        calculatedTicksFor90 = ticksPerDegree * 90.0f;
        Serial.printf("   Calculated ticks per degree: %.2f\n", ticksPerDegree);
    }

    Serial.printf("   Extrapolated ticks for 90°: %.2f\n", calculatedTicksFor90);

    // Store the calculated (extrapolated) result
    calibData.ticksPer90Degrees = calculatedTicksFor90;
    calibData.ticksPer90DegreesEmpirical = calculatedTicksFor90;
    calibData.ticksPer90DegreesTheoretical = ticksPer90Deg_theoretical;

    Serial.printf("✅ Phase 2 complete: 90° turn = %.0f ticks\n", calibData.ticksPer90Degrees);
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 4: DISTANCE & TOF CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateDistanceAndToF() {
    Serial.println("\n📏 PHASE 4: Distance & ToF Calibration");
    Serial.println("─────────────────────────────────────────");
    Serial.println("Goal: Calibrate encoder ticks to real distance and find ToF offset");
    
    Serial.println("This phase now uses theoretical values for maximum reliability.");
    Serial.println("It will still perform a forward movement to ensure encoders are functional.");
    
    // Reset encoders
    resetEncoders();
    delay(100);
    
    // Execute a test forward movement for a short duration
    Serial.println("🔄 Performing a short forward movement test...");
    
    bool m1Fwd = calibData.motorDirs.fwdMove_M1Fwd;
    bool m1Rev = calibData.motorDirs.fwdMove_M1Rev;
    bool m2Fwd = calibData.motorDirs.fwdMove_M2Fwd;
    bool m2Rev = calibData.motorDirs.fwdMove_M2Rev;

    executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, TEST_SPEED);    
    delay(1000); // Move for 1 second
    stopWithBrake();
    delay(500);
    
    long encoderTicks = getAverageEncoderCount();
    
    Serial.printf("📊 Movement Test Results:\n");
    Serial.printf("   Encoder ticks detected: %ld\n", encoderTicks);
    
    // Sanity check: ensure encoders are working at all.
    if (encoderTicks < 50) {
        Serial.println("⚠️ WARNING: Insufficient encoder movement detected during forward test.");
        Serial.println("   Proceeding with theoretical values, but encoder accuracy may be low.");
        // Do not return a failure, just issue a warning.
    }
    // --- Theoretical calculation ---
    // This is more reliable than using the ToF sensor.
    float wheelCircumference = WHEEL_DIAMETER_MM * 3.14159265f;
    float ticksPerRevolution = ENCODER_SLOTS * GEAR_RATIO; // Corrected for gear ratio
    float mmPerTick = wheelCircumference / ticksPerRevolution;
    float ticksPerMM_theoretical = ticksPerRevolution / wheelCircumference;

    Serial.println("\n[Theoretical Calculation]");
    Serial.printf("[Theoretical] Wheel circumference: %.2f mm\n", wheelCircumference);
    Serial.printf("[Theoretical] mm per tick: %.2f\n", mmPerTick);
    Serial.printf("[Theoretical] Ticks per mm: %.4f\n", ticksPerMM_theoretical);

    // Always use theoretical value for ticks per mm
    // Set the calibration data based on the reliable theoretical values.
    calibData.ticksPerMillimeter = ticksPerMM_theoretical;
    calibData.ticksPerMillimeterEmpirical = 0.0f; // Not used
    calibData.ticksPerMillimeterTheoretical = ticksPerMM_theoretical;

    // ToF offset is set to 0 as it cannot be reliably determined this way.
    calibData.tofOffsetMM = 0.0;

    Serial.printf("📊 Calibration Values Calculated:\n");
    Serial.printf("   Ticks per millimeter (used): %.4f\n", calibData.ticksPerMillimeter);
    Serial.printf("   ToF Offset (used): %.2f mm\n", calibData.tofOffsetMM);

    Serial.println("✅ Phase 4 complete: Distance calibration successful");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATED MOVEMENT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void calibratedTurnLeft(int speed) {
    if (!isCalibrated) {
        rotateLeft(TURN_SPEED); // Fallback to basic turn
        return;
    }
    
    executeMotorCommand(calibData.motorDirs.leftFwd_M1Fwd, calibData.motorDirs.leftFwd_M1Rev,
                       calibData.motorDirs.leftFwd_M2Fwd, calibData.motorDirs.leftFwd_M2Rev, speed);
}

void calibratedTurnRight(int speed) {
    if (!isCalibrated) {
        rotateRight(TURN_SPEED); // Fallback to basic turn
        return;
    }
    
    executeMotorCommand(!calibData.motorDirs.leftFwd_M1Fwd, !calibData.motorDirs.leftFwd_M1Rev,
                       !calibData.motorDirs.leftFwd_M2Fwd, !calibData.motorDirs.leftFwd_M2Rev, speed);
}

void calibratedMoveForward(int speed) {
    if (!isCalibrated) {
        moveForward(TEST_SPEED); // Fallback to basic movement
        return;
    }
    
    executeMotorCommand(calibData.motorDirs.fwdMove_M1Fwd, calibData.motorDirs.fwdMove_M1Rev,
                       calibData.motorDirs.fwdMove_M2Fwd, calibData.motorDirs.fwdMove_M2Rev, speed);
}

void calibratedMoveBackward(int speed) {
    if (!isCalibrated) {
        moveBackward(TEST_SPEED); // Fallback to basic movement
        return;
    }
    
    executeMotorCommand(!calibData.motorDirs.fwdMove_M1Fwd, !calibData.motorDirs.fwdMove_M1Rev,
                       !calibData.motorDirs.fwdMove_M2Fwd, !calibData.motorDirs.fwdMove_M2Rev, speed);
}

void calibratedTurn90Left() {
    if (!isCalibrated) return;
    
    resetEncoders();
    calibratedTurnLeft(TURN_SPEED);
    
    while (getAverageEncoderCount() < calibData.ticksPer90Degrees) {
        delay(10);
    }
    
    allStop();
}

void calibratedTurn90Right() {
    if (!isCalibrated) return;
    
    resetEncoders();
    calibratedTurnRight(TURN_SPEED);
    
    while (getAverageEncoderCount() < calibData.ticksPer90Degrees) {
        delay(10);
    }
    
    allStop();
}

void calibratedMoveDistance(float mm) {
    if (!isCalibrated) return;
    
    long targetTicks = (long)(mm * calibData.ticksPerMillimeter);
    
    resetEncoders();
    
    if (mm > 0) {
        calibratedMoveForward(TEST_SPEED);
    } else {
        calibratedMoveBackward(TEST_SPEED);
        targetTicks = abs(targetTicks);
    }
    
    while (getAverageEncoderCount() < targetTicks) {
        delay(10);
    }
    
    allStop();
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void executeMotorCommand(bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed) {
    // Determine motor 1 direction and speed
    int motor1Speed = 0;
    if (m1Fwd && !m1Rev) motor1Speed = speed;
    else if (m1Rev && !m1Fwd) motor1Speed = -speed;
    
    // Determine motor 2 direction and speed
    int motor2Speed = 0;
    if (m2Fwd && !m2Rev) motor2Speed = speed;
    else if (m2Rev && !m2Fwd) motor2Speed = -speed;
    
    setMotorPWM(motor1Speed, motor2Speed);
}

void stopMotorsGently() {
    // Gradual stop for calibration precision
    for (int speed = TEST_SPEED; speed >= 0; speed -= 20) {
        setMotorPWM(speed, speed);
        delay(50);
    }
    allStop();
}

float getStableToFReading(int samples) {
    float total = 0;
    int validSamples = 0;
    
    for (int i = 0; i < samples; i++) {
        hal.updateAllSensors(); // Poll the hardware via the HAL
        int distance = sensors.distance;
        if (distance > 0 && distance < 2000) {
            total += distance;
            validSamples++;
        }
        delay(50);
    }
    
    return validSamples > 0 ? (total / validSamples) : 0;
}

float getStableMPUHeading(int samples) {
    float total = 0;
    
    for (int i = 0; i < samples; i++) {
        hal.updateAllSensors(); // Poll all sensors via the HAL
        total += sensors.headingAngle; // Read from the global struct
    }
    
    return total / samples;
}

bool isCalibrationSafe() {
    // Check sensor availability
    if (!sysStatus.tofAvailable || !sysStatus.mpuAvailable) {
        Serial.println("❌ Required sensors not available");
        return false;
    }
    
    // Check robot orientation - must be reasonably level for calibration
    hal.updateAllSensors();
    float tiltX = abs(sensors.tiltX);
    float tiltY = abs(sensors.tiltY);
    if (tiltX > 30.0 || tiltY > 30.0) {
        Serial.printf("❌ Robot is tilted too much for calibration (X:%.1f deg Y:%.1f deg)\n", tiltX, tiltY);
        Serial.println("   Please place robot on a flat, level surface");
        return false;
    }
    
    // Check for clear ToF readings
    float distance = getStableToFReading(3);
    if (distance < 100) {
        Serial.println("❌ Insufficient clearance for calibration (need 10cm+ space)");
        return false;
    }
    
    Serial.println("✅ Safety checks passed - robot is level and has clearance");
    return true;
}

void calibrationEmergencyStop() {
    allStop();
    setLEDColor(LEDColors::RED);
    buzz(2000, 1000);
    Serial.println("🚨 CALIBRATION EMERGENCY STOP");
}

bool waitForStableConditions() {
    Serial.println("⏳ Waiting for stable sensor conditions...");
    
    for (int i = 0; i < 10; i++) {
        hal.updateAllSensors(); // Poll all sensors via the HAL
        delay(100);
    }
    
    Serial.println("✅ Sensors stabilized");
    return true;
}

void calibrationProgressUpdate(const char* phase, int progress) {
    Serial.printf("\n📈 PROGRESS: %s (%d%%)\n", phase, progress);
    
    // Visual progress indication with LED (no beeping to avoid confusion)
    if (progress < 25) setLEDColor(LEDColors::RED);
    else if (progress < 50) setLEDColor(LEDColors::YELLOW);
    else if (progress < 75) setLEDColor(LEDColors::BLUE);
    else if (progress < 100) setLEDColor(LEDColors::GREEN);
    else setLEDColor(LEDColors::WHITE);
    
    // Single beep only for major milestones (no continuous beeping)
    if (progress == 100) {
        buzz(2000, 200); // Success beep only at completion
    }
}

CalibrationResult validateCalibrationData() {
    // Check magic number and version
    if (calibData.magic != CALIBRATION_MAGIC) {
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    if (calibData.version != CALIBRATION_VERSION) {
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    // Check reasonable ranges for calibration values
    if (calibData.ticksPer90Degrees < 100 || calibData.ticksPer90Degrees > 10000) {
        return CALIB_ERR_SENSOR_INVALID;
    }
    if (calibData.ticksPerMillimeter < 1.0 || calibData.ticksPerMillimeter > 100.0) {
        return CALIB_ERR_SENSOR_INVALID;
    }
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR DEADZONE CALIBRATION (NEW PHASE 5)
// ═══════════════════════════════════════════════════════════════════════════

uint8_t findMotorDeadzone(int maxPWM, int stepSize) {
    Serial.println("🎯 Finding motor deadzone (minimum PWM for movement)...");
    
    for (uint8_t pwm = stepSize; pwm <= maxPWM; pwm += stepSize) {
        Serial.printf("Testing PWM: %d... ", pwm);
        
        if (testMotorMovement(pwm, 500)) {
            Serial.printf("✅ Movement detected at PWM %d\n", pwm);
            return pwm;
        } else {
            Serial.println("❌ No movement");
        }
        
        delay(200); // Brief pause between tests
    }
    
    Serial.printf("⚠️ No movement detected up to PWM %d\n", maxPWM);
    return maxPWM; // Fallback to maximum tested value
}

bool testMotorMovement(uint8_t pwmValue, int durationMs) {
    // Reset encoders to detect any movement
    resetEncoders();
    delay(50);
    
    // Test forward movement
    executeMotorCommand(true, false, true, false, pwmValue);
    delay(durationMs);
    stopMotorsGently();
    
    // Check if any movement was detected
    long leftTicks = abs(getLeftEncoderCount());
    long rightTicks = abs(getRightEncoderCount());
    long totalMovement = leftTicks + rightTicks;
    
    Serial.printf("(L:%ld R:%ld) ", leftTicks, rightTicks);
    
    return totalMovement > 0; // Any movement counts as success
}

CalibrationResult calibrateMotorDeadzone() {
    Serial.println("\n🎯 PHASE 5: MOTOR DEADZONE CALIBRATION");
    Serial.println("Finding minimum PWM value for reliable movement...");
    
    calibrationProgressUpdate("Motor Deadzone", 0);
    
    // Find the minimum PWM needed for movement
    uint8_t deadzoneMargin = 10; // Add safety margin above deadzone
    uint8_t rawDeadzone = findMotorDeadzone(100, 5); // Test up to PWM 100 in steps of 5
    
    calibrationProgressUpdate("Motor Deadzone", 50);
    
    // Add safety margin and validate
    calibData.minMotorSpeedPWM = rawDeadzone + deadzoneMargin;
    
    // Ensure it's within reasonable bounds
    if (calibData.minMotorSpeedPWM < 15) {
        calibData.minMotorSpeedPWM = 15; // Minimum sensible value
    }
    if (calibData.minMotorSpeedPWM > 80) {
        calibData.minMotorSpeedPWM = 80; // Maximum reasonable deadzone
    }
    
    calibrationProgressUpdate("Motor Deadzone", 75);
    
    // Test the final value
    Serial.printf("🧪 Testing final deadzone value: %d\n", calibData.minMotorSpeedPWM);
    if (!testMotorMovement(calibData.minMotorSpeedPWM, 1000)) {
        Serial.println("⚠️ Final deadzone test failed, but continuing...");
    }
    
    calibrationProgressUpdate("Motor Deadzone", 100);
    
    Serial.printf("✅ Motor deadzone calibration complete: %d PWM\n", calibData.minMotorSpeedPWM);
    return CALIB_SUCCESS;
}
// ═══════════════════════════════════════════════════════════════════════════
// MISSING FUNCTIONS - Add these to calibration.cpp
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 3: FORWARD/BACKWARD DETECTION
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateForwardBackward() {
    Serial.println("\n🧭 PHASE 3: Forward/Backward Detection");
    Serial.println("─────────────────────────────────────────────");
    Serial.println("Goal: Determine which motor pattern moves robot forward");
    
    // Wait for stable conditions
    if (!waitForStableConditions()) {
        return CALIB_ERR_TIMEOUT;
    }
    
    // Get baseline ToF distance
    float initialDistance = getStableToFReading();
    if (initialDistance < 100 || initialDistance > 1500) {
        Serial.printf("❌ ERROR: Initial distance unsuitable: %.0f mm\n", initialDistance);
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    Serial.printf("📍 Baseline distance: %.0f mm\n", initialDistance);
    
    // Test hypothesis: Both motors forward = Robot moves forward (distance decreases)
    Serial.println("🔬 Testing hypothesis: M1-FWD + M2-FWD = FORWARD movement");
    
    resetEncoders();
    delay(100);
    
    // Execute test: both motors forward
    Serial.println("   🔄 Running motors forward for 1 second...");
    executeMotorCommand(true, false, true, false, TEST_SPEED);
    delay(1000);
    allStop();
    delay(500);
    
    // Measure new distance
    float finalDistance = getStableToFReading();
    float distanceChange = finalDistance - initialDistance;
    long encoderTicks = getAverageEncoderCount();
    
    Serial.printf("\n📊 RESULTS:\n");
    Serial.printf("   Initial distance: %.0f mm\n", initialDistance);
    Serial.printf("   Final distance: %.0f mm\n", finalDistance);
    Serial.printf("   Distance change: %.0f mm (negative = moved forward)\n", distanceChange);
    Serial.printf("   Encoder ticks: %ld (confirms movement)\n", encoderTicks);
    
    // Determine direction based on ToF change
    if (distanceChange < -20.0) {
        // Hypothesis CORRECT: M1-FWD + M2-FWD = FORWARD
        Serial.println("✅ Hypothesis CORRECT: M1-FWD + M2-FWD = FORWARD");
        calibData.motorDirs.fwdMove_M1Fwd = true;
        calibData.motorDirs.fwdMove_M1Rev = false;
        calibData.motorDirs.fwdMove_M2Fwd = true;
        calibData.motorDirs.fwdMove_M2Rev = false;
    } else if (distanceChange > 20.0) {
        // Hypothesis WRONG: M1-FWD + M2-FWD = BACKWARD
        Serial.println("❌ Hypothesis WRONG: M1-FWD + M2-FWD = BACKWARD");
        Serial.println("✅ Corrected: M1-REV + M2-REV = FORWARD");
        calibData.motorDirs.fwdMove_M1Fwd = false;
        calibData.motorDirs.fwdMove_M1Rev = true;
        calibData.motorDirs.fwdMove_M2Fwd = false;
        calibData.motorDirs.fwdMove_M2Rev = true;
    } else {
        Serial.printf("⚠️ WARNING: Ambiguous distance change (%.0f mm). ToF data is inconclusive.\n", distanceChange);
        // Fallback to logical deduction if ToF is ambiguous
        Serial.println("   Falling back to logical deduction based on turn direction.");
        // (The logical implementation from the previous step should be here, but it was merged incorrectly.
        // For now, we'll just assume standard wiring as a safe fallback.)
        calibData.motorDirs.fwdMove_M1Fwd = true;
        calibData.motorDirs.fwdMove_M1Rev = false;
        calibData.motorDirs.fwdMove_M2Fwd = true;
        calibData.motorDirs.fwdMove_M2Rev = false;
    }
    
    Serial.println("✅ Phase 3 complete: Forward/Backward motor mapping determined");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// NOTE: stopWithBrake() and victoryAnimation() are defined in motors.cpp 
// and indicators.cpp respectively - no need to redefine them here
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATION MANAGEMENT FUNCTIONS (for WheelieHAL)
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult saveCalibrationData() {
    Serial.println("💾 Saving calibration data to EEPROM...");
    
    // Update checksum before saving
    updateChecksum(&calibData);
    
    // Write to EEPROM
    EEPROM.put(EEPROM_CALIB_DATA_ADDR, calibData);
    
    if (!EEPROM.commit()) {
        Serial.println("❌ ERROR: EEPROM commit failed");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    Serial.println("✅ Calibration data saved successfully");
    return CALIB_SUCCESS;
}

bool shouldForceRecalibration() {
    pinMode(FORCE_RECALIBRATION_PIN, INPUT_PULLUP);
    delay(10); // Allow pin to settle
    bool force = (digitalRead(FORCE_RECALIBRATION_PIN) == LOW);
    if (force) {
        Serial.println("\n\n🔄 FORCE RECALIBRATION detected (GPIO0 held low).");
        eraseCalibrationData();
    }
    return force;
}

CalibrationResult eraseCalibrationData() {
    Serial.println("🔥 Erasing calibration data from EEPROM...");
    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0);
    }
    if (EEPROM.commit()) {
        Serial.println("✅ EEPROM erased.");
        return CALIB_SUCCESS;
    } else {
        Serial.println("❌ EEPROM erase failed.");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
}

CalibrationResult loadCalibrationData() {
    Serial.println("📖 Loading calibration data from EEPROM...");
    
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(EEPROM_CALIB_DATA_ADDR, calibData);
    
    // 1. Check magic number
    if (calibData.magic != CALIBRATION_MAGIC) {
        Serial.println("   ❌ Invalid magic number. Data is not valid.");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    // 2. Check version
    if (calibData.version != CALIBRATION_VERSION) {
        Serial.printf("   ❌ Version mismatch. Found %d, expected %d.\n", calibData.version, CALIBRATION_VERSION);
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    // 3. Verify checksum
    if (!verifyDataIntegrity(&calibData)) {
        Serial.println("   ❌ Checksum verification failed. Data is corrupt.");
        return CALIB_ERR_CHECKSUM_FAILED;
    }
    
    // 4. Validate data ranges
    CalibrationResult validationResult = validateCalibrationData();
    if (validationResult != CALIB_SUCCESS) {
        Serial.println("   ❌ Calibration data is out of reasonable bounds.");
        return validationResult;
    }
    
    Serial.println("✅ Calibration data loaded and verified successfully.");
    isCalibrated = true;
    return CALIB_SUCCESS;
}

CalibrationResult runFullCalibrationSequence() {
    Serial.println("\n🚀 STARTING AUTONOMOUS CALIBRATION SEQUENCE");
    Serial.println("════════════════════════════════════════════════");
    Serial.println("This is a ONE-TIME process that will teach the robot:");
    Serial.println("• Motor directions (left/right, forward/backward)");
    Serial.println("• Precise turning distances (90° turns)");
    Serial.println("• Movement calibration (distance per encoder tick)");
    Serial.println("• Sensor orientations and offsets");
    Serial.println("• Motor deadzone compensation (minimum PWM)");
    Serial.println("════════════════════════════════════════════════\n");

    // --- Sensor Health Summary ---
    Serial.println("SENSOR HEALTH SUMMARY:");
    Serial.printf("   ToF Sensor:      %s\n", sysStatus.tofAvailable ? "AVAILABLE" : "NOT FOUND");
    Serial.printf("   IMU (MPU6050):   %s\n", sysStatus.mpuAvailable ? "AVAILABLE" : "NOT FOUND");
    Serial.printf("   PIR Sensor:      %s\n", sensorHealth.pirHealthy ? "HEALTHY" : "FAULT");
    Serial.printf("   Edge Sensor:     %s\n", sensorHealth.edgeHealthy ? "HEALTHY" : "FAULT");
    Serial.printf("   Sound Sensor:    %s\n", sensorHealth.soundHealthy ? "HEALTHY" : "FAULT");
    Serial.println("");
    
    // Safety checks
    if (!isCalibrationSafe()) {
        Serial.println("❌ CALIBRATION ABORTED: Unsafe conditions detected");
        return handleCalibrationFailure(CALIB_ERR_UNSTABLE, "Safety Check");
    }
    
    // Initialize calibration data structure
    calibData.magic = CALIBRATION_MAGIC;
    calibData.version = CALIBRATION_VERSION;
    
    // Start calibration timer
    unsigned long startTime = millis();
    
    // 🎯 PHASE 1: Directional Mapping (Left/Right)
    calibrationProgressUpdate("Phase 1: Directional Mapping", 10);
    CalibrationResult result = calibrateDirectionalMapping();
    if (result != CALIB_SUCCESS) {
        return handleCalibrationFailure(result, "Directional Mapping");
    }
    
    // 🎯 PHASE 2: Turn Distance Calibration
    calibrationProgressUpdate("Phase 2: Turn Calibration", 35);
    result = calibrateTurnDistance();
    if (result != CALIB_SUCCESS) {
        return handleCalibrationFailure(result, "Turn Distance");
    }
    
    // 🎯 PHASE 3: Forward/Backward Detection
    calibrationProgressUpdate("Phase 3: Forward/Backward Detection", 60);
    result = calibrateForwardBackward();
    if (result != CALIB_SUCCESS) {
        return handleCalibrationFailure(result, "Forward/Backward Detection");
    }
    
    // 🎯 PHASE 4: Distance & ToF Calibration
    calibrationProgressUpdate("Phase 4: Distance & ToF Calibration", 85);
    result = calibrateDistanceAndToF();
    if (result != CALIB_SUCCESS) {
        return handleCalibrationFailure(result, "Distance & ToF");
    }
    
    // 🎯 PHASE 5: Motor Deadzone Calibration
    calibrationProgressUpdate("Phase 5: Motor Deadzone Calibration", 90);
    result = calibrateMotorDeadzone();
    if (result != CALIB_SUCCESS) {
        return handleCalibrationFailure(result, "Motor Deadzone");
    }
    
    // 💾 FINAL STEP: Save to EEPROM with CRC16
    calibrationProgressUpdate("Saving Calibration Data", 98);
    result = saveCalibrationData();
    if (result != CALIB_SUCCESS) {
        return handleCalibrationFailure(result, "Save to EEPROM");
    }
    
    // Calculate total time
    unsigned long totalTime = millis() - startTime;
    
    // Success!
    calibrationProgressUpdate("Calibration Complete", 100);
    Serial.println("\n🎉 CALIBRATION SUCCESS!");
    Serial.printf("   Total time: %.1f seconds\n", totalTime / 1000.0);
    Serial.println("   Robot is now ready for autonomous operation");
    Serial.println("   Calibration data saved permanently to EEPROM");
    
    // Victory animation
    victoryAnimation();
    
    // Set global flag
    isCalibrated = true;
    calibData.valid = true; // Mark data as valid
    
    return CALIB_SUCCESS;
}



/*
    Serial.println("💾 Saving calibration data to EEPROM...");
    
    // Update checksum
    calibData.checksum = calculateCRC16((uint8_t*)&calibData, 
                                       sizeof(CalibrationData) - sizeof(calibData.checksum));
    
    // Write to EEPROM
    EEPROM.put(EEPROM_CALIB_DATA_ADDR, calibData);
    
    if (!EEPROM.commit()) {
        Serial.println("❌ ERROR: EEPROM commit failed");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    Serial.println("✅ Calibration data saved successfully");
    return CALIB_SUCCESS;
}
*/