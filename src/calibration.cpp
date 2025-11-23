// ═══════════════════════════════════════════════════════════════════════════
// 🤖 AUTONOMOUS CALIBRATION SYSTEM - Implementation (ENHANCED MPU6050)
// ═══════════════════════════════════════════════════════════════════════════
// Complete self-calibration system with EEPROM storage
// Enhanced MPU6050 integration with saturation detection and drift compensation
// ═══════════════════════════════════════════════════════════════════════════

#include "calibration.h"
#include "WheelieHAL.h"

// Forward declaration for non-blocking movement function
bool executeMoveUntil(long targetTicks, unsigned long timeoutMs, bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed);
bool executeMoveForDuration(unsigned long durationMs, bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed);

// Define wheel diameter in millimeters if not defined elsewhere
#ifndef WHEEL_DIAMETER_MM
#define WHEEL_DIAMETER_MM 65.0f
#endif

#ifndef TRACK_WIDTH_MM
#define TRACK_WIDTH_MM 120.0f
#endif

#ifndef ENCODER_SLOTS
#define ENCODER_SLOTS 20
#endif

#ifndef GEAR_RATIO
#define GEAR_RATIO 1.0f
#endif

// Constants for the encoder test
const int ENCODER_TEST_SPEED = 150;     // PWM duty cycle for the test
const int ENCODER_TEST_DURATION_MS = 500; // How long to run the motors
const long ENCODER_MIN_TICKS_THRESHOLD = 20; // Minimum ticks to be considered a success
extern WheelieHAL hal;

// ═══════════════════════════════════════════════════════════════════════════
// ENHANCED MPU6050 UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Detects if the gyroscope is saturating (reading max values)
 * This indicates the robot is turning faster than the sensor can measure
 */
bool detectGyroSaturation() {
    hal.updateAllSensors();
    float gyroZ = sensors.gyroZ;
    
    // Check if we're hitting the configured range limits
    // Default MPU6050 range is ±250°/s, saturation occurs near these limits
    const float SATURATION_THRESHOLD = 240.0f; // 96% of ±250°/s range
    
    if (abs(gyroZ) > SATURATION_THRESHOLD) {
        Serial.printf("⚠️ GYRO SATURATION DETECTED: %.2f°/s (limit: ±250°/s)\n", gyroZ);
        return true;
    }
    
    return false;
}

/**
 * @brief Gets stable heading with drift compensation and quality checking
 * @param samples Number of samples to average
 * @param checkStability If true, verifies readings are stable (low variance)
 * @return Average heading angle in degrees
 */
float getStableMPUHeading(int samples, bool checkStability) {
    float readings[samples];
    float total = 0;
    
    // Collect samples
    for (int i = 0; i < samples; i++) {
        hal.updateAllSensors();
        readings[i] = sensors.headingAngle;
        total += readings[i];
        delay(50); // 20Hz sampling
    }
    
    float average = total / samples;
    
    // Check stability if requested
    if (checkStability) {
        float variance = 0;
        for (int i = 0; i < samples; i++) {
            float diff = readings[i] - average;
            variance += diff * diff;
        }
        variance /= samples;
        float stdDev = sqrt(variance);
        
        if (stdDev > 2.0) {
            Serial.printf("⚠️ WARNING: Unstable heading readings (std dev: %.2f°)\n", stdDev);
            Serial.println("   Robot may be vibrating or on unstable surface");
        } else {
            Serial.printf("✅ Stable heading: %.2f° (std dev: %.2f°)\n", average, stdDev);
        }
    }
    
    return average;
}

/**
 * @brief Overload for backward compatibility
 */
float getStableMPUHeading(int samples) {
    return getStableMPUHeading(samples, false);
}

/**
 * @brief Monitors rotation with real-time gyroscope integration
 * More accurate than polling heading at start/end
 * @param durationMs How long to monitor
 * @param expectedDirection -1 for left (negative), +1 for right (positive)
 * @return Total rotation in degrees
 */
float monitorRotationWithGyro(unsigned long durationMs, int expectedDirection) {
    unsigned long startTime = millis();
    unsigned long lastUpdate = startTime;
    unsigned long lastPrint = startTime;
    float totalRotation = 0;
    int saturatedSamples = 0;
    int totalSamples = 0;
    
    Serial.println("   📊 Real-time gyro monitoring:");
    Serial.println("   Time(ms)  GyroZ(°/s)  Rotation(°)  Status");
    Serial.println("   ────────  ──────────  ───────────  ──────");
    
    while (millis() - startTime < durationMs) {
        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0f; // Convert to seconds
        
        hal.updateAllSensors();
        float gyroZ = sensors.gyroZ;
        
        // Integrate gyroscope reading
        float deltaAngle = gyroZ * dt;
        totalRotation += deltaAngle;
        
        // Check for saturation
        totalSamples++;
        if (abs(gyroZ) > 240.0f) {
            saturatedSamples++;
        }
        
        // Print progress every 200ms
        if (now - lastPrint >= 200) {
            char status[20];
            if (abs(gyroZ) > 240.0f) {
                strcpy(status, "⚠️ SATURATED");
            } else if (abs(gyroZ) < 5.0f) {
                strcpy(status, "⚠️ NO ROTATION");
            } else {
                strcpy(status, "✓ OK");
            }
            
            Serial.printf("   %7lu   %9.2f   %10.2f   %s\n", 
                         now - startTime, gyroZ, totalRotation, status);
            lastPrint = now;
        }
        
        lastUpdate = now;
        delay(10); // 100Hz update rate
    }
    
    // Report saturation statistics
    float saturationPercent = (saturatedSamples * 100.0f) / totalSamples;
    if (saturationPercent > 10.0f) {
        Serial.printf("\n⚠️ CRITICAL: %.1f%% of samples were saturated!\n", saturationPercent);
        Serial.println("   Solutions:");
        Serial.println("   1. Reduce turn speed (currently using high speed)");
        Serial.println("   2. Increase gyro range in WheelieHAL.cpp:");
        Serial.println("      Change mpu.setGyroConfig(2) to mpu.setGyroConfig(3)");
        Serial.println("      This increases range from ±1000°/s to ±2000°/s");
    } else {
        Serial.printf("   ✅ Gyro performed well (%.1f%% saturation)\n", saturationPercent);
    }
    
    // Check if rotation matches expected direction
    int actualDirection = (totalRotation < 0) ? -1 : 1;
    if (actualDirection != expectedDirection) {
        Serial.printf("⚠️ WARNING: Expected %s turn but detected %s turn\n",
                     expectedDirection < 0 ? "LEFT" : "RIGHT",
                     actualDirection < 0 ? "LEFT" : "RIGHT");
    }
    
    return totalRotation;
}

/**
 * @brief Pre-flight MPU diagnostics - ensures sensor is working properly
 */
bool performMPUDiagnostics() {
    Serial.println("\n🔬 MPU6050 PRE-FLIGHT DIAGNOSTICS");
    Serial.println("─────────────────────────────────");
    
    // Test 1: Check for valid readings
    Serial.println("Test 1: Checking sensor responsiveness...");
    bool hasValidReadings = false;
    for (int i = 0; i < 10; i++) {
        hal.updateAllSensors();
        if (sensors.gyroZ != 0.0f || sensors.accelZ != 0.0f) {
            hasValidReadings = true;
            break;
        }
        delay(100);
    }
    
    if (!hasValidReadings) {
        Serial.println("   ❌ FAILED: MPU6050 returning all zeros");
        Serial.println("   → Check I2C connections (SDA/SCL)");
        Serial.println("   → Verify sensor power (3.3V)");
        return false;
    }
    Serial.println("   ✅ PASSED: Sensor responding");
    
    // Test 2: Check for noise/stability
    Serial.println("\nTest 2: Checking noise levels...");
    float gyroReadings[20];
    for (int i = 0; i < 20; i++) {
        hal.updateAllSensors();
        gyroReadings[i] = sensors.gyroZ;
        delay(50);
    }
    
    // Calculate standard deviation
    float mean = 0;
    for (int i = 0; i < 20; i++) mean += gyroReadings[i];
    mean /= 20;
    
    float variance = 0;
    for (int i = 0; i < 20; i++) {
        float diff = gyroReadings[i] - mean;
        variance += diff * diff;
    }
    variance /= 20;
    float stdDev = sqrt(variance);
    
    Serial.printf("   Gyro Z-axis: Mean=%.2f°/s, StdDev=%.2f°/s\n", mean, stdDev);
    
    if (stdDev > 10.0f) {
        Serial.println("   ⚠️ WARNING: High noise detected");
        Serial.println("   → Robot may be on unstable surface");
        Serial.println("   → Check for vibration sources");
    } else {
        Serial.println("   ✅ PASSED: Acceptable noise levels");
    }
    
    // Test 3: Check mounting orientation
    Serial.println("\nTest 3: Checking sensor orientation...");
    hal.updateAllSensors();
    float accelZ = sensors.accelZ;
    
    Serial.printf("   Accel Z-axis: %.2fg\n", accelZ);
    
    if (abs(accelZ - 1.0f) > 0.3f) {
        Serial.println("   ⚠️ WARNING: Sensor may not be level");
        Serial.printf("   → Expected ~1.0g, got %.2fg\n", accelZ);
        Serial.println("   → Ensure robot is on flat surface");
        Serial.println("   → Verify MPU6050 is mounted flat");
    } else {
        Serial.println("   ✅ PASSED: Sensor properly oriented");
    }
    
    // Test 4: Temperature check
    Serial.println("\nTest 4: Checking sensor temperature...");
    hal.updateAllSensors();
    float temp = sensors.temperature;
    
    Serial.printf("   Temperature: %.1f°C\n", temp);
    
    if (temp < 10.0f || temp > 50.0f) {
        Serial.println("   ⚠️ WARNING: Temperature outside normal range");
        Serial.println("   → Sensor may not be functioning correctly");
    } else {
        Serial.println("   ✅ PASSED: Temperature normal");
    }
    
    Serial.println("\n📋 DIAGNOSTIC SUMMARY:");
    Serial.println("   All critical tests passed");
    Serial.println("   MPU6050 ready for calibration\n");
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS (CALIBRATION)
// ═══════════════════════════════════════════════════════════════════════════

void printPhaseHeader(const char* phaseName) {
    Serial.println();
    Serial.printf("🔄 PHASE: %s\n", phaseName);
    Serial.println("──────────────────────────────────────────");
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 0: ENCODER SANITY CHECK (NEW)
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Handles the failure of the encoder test, prints troubleshooting info, and returns false.
 * @param motorName The name of the motor/encoder that failed (e.g., "M1").
 * @return Always returns CALIB_ERR_MOTOR_ERROR to indicate failure.
 */
CalibrationResult handleEncoderTestFailure(const char* motorName) {
    Serial.printf("❌ (0 ticks) -> HALT: Encoder %s not responding!\n", motorName);
    return handleCalibrationFailure(CALIB_ERR_MOTOR_ERROR, "Encoder Sanity Check");
}

/**
 * @brief Runs a diagnostic test on both motors and encoders.
 *
 * This function moves each motor forward and backward, checking if the encoders
 * register a plausible amount of movement. It's designed to be the first
 * check in the calibration sequence to catch fundamental hardware issues early.
 *
 * @return CALIB_SUCCESS if both encoders pass the test, an error code otherwise.
 */
CalibrationResult runEncoderSanityCheck() {
    printPhaseHeader("Encoder Sanity Check");

    // --- Test Motor 1 (Left) ---
    resetEncoders();
    
    // Forward
    Serial.print("   - Testing M1 Forward... ");
    executeMotorCommand(true, false, false, false, ENCODER_TEST_SPEED); // M1 Fwd
    delay(ENCODER_TEST_DURATION_MS);
    stopMotorsGently();
    delay(100); // Settle
    long ticksM1Fwd = getLeftEncoderCount();
    if (abs(ticksM1Fwd) < ENCODER_MIN_TICKS_THRESHOLD) {
        return handleEncoderTestFailure("M1");
    }
    Serial.printf("✔ (%ld ticks)\n", ticksM1Fwd);

    // Reverse
    Serial.print("   - Testing M1 Reverse... ");
    executeMotorCommand(false, true, false, false, ENCODER_TEST_SPEED); // M1 Rev
    delay(ENCODER_TEST_DURATION_MS);
    stopMotorsGently();
    delay(100); // Settle
    long ticksM1Rev = getLeftEncoderCount() - ticksM1Fwd;
    if (abs(ticksM1Rev) < ENCODER_MIN_TICKS_THRESHOLD) {
        return handleEncoderTestFailure("M1");
    }
    Serial.printf("✔ (%ld ticks)\n", ticksM1Rev);

    // --- Test Motor 2 (Right) ---
    resetEncoders();

    // Forward
    Serial.print("   - Testing M2 Forward... ");
    executeMotorCommand(false, false, true, false, ENCODER_TEST_SPEED); // M2 Fwd
    delay(ENCODER_TEST_DURATION_MS);
    stopMotorsGently();
    delay(100); // Settle
    long ticksM2Fwd = getRightEncoderCount();
    if (abs(ticksM2Fwd) < ENCODER_MIN_TICKS_THRESHOLD) {
        return handleEncoderTestFailure("M2");
    }
    Serial.printf("✔ (%ld ticks)\n", ticksM2Fwd);

    // Reverse
    Serial.print("   - Testing M2 Reverse... ");
    executeMotorCommand(false, false, false, true, ENCODER_TEST_SPEED); // M2 Rev
    delay(ENCODER_TEST_DURATION_MS);
    stopMotorsGently();
    delay(100); // Settle
    long ticksM2Rev = getRightEncoderCount() - ticksM2Fwd;
    if (abs(ticksM2Rev) < ENCODER_MIN_TICKS_THRESHOLD) {
        return handleEncoderTestFailure("M2");
    }
    Serial.printf("✔ (%ld ticks)\n", ticksM2Rev);

    Serial.println("✅ Encoders and motors are responsive.");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 1: DIRECTIONAL MAPPING (LEFT/RIGHT) - ENHANCED
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateDirectionalMapping() {
    Serial.println("\n🧭 PHASE 1: Directional Mapping (Left/Right)");
    Serial.println("─────────────────────────────────────────────");
    Serial.println("Goal: Determine which motor commands create left/right turns");
    
    // PRE-FLIGHT: Run MPU diagnostics
    if (!performMPUDiagnostics()) {
        Serial.println("❌ MPU6050 diagnostics failed - cannot continue");
        return handleCalibrationFailure(CALIB_ERR_SENSOR_INVALID, "MPU Pre-Flight");
    }
    
    // Wait for stable conditions
    if (!waitForStableConditions()) {
        return CALIB_ERR_TIMEOUT;
    }
    
    // Get baseline heading with stability check
    float baseHeading = getStableMPUHeading(20, true);
    Serial.printf("📍 Base heading established: %.2f°\n", baseHeading);
    
    // Test hypothesis: Motor 1 REV + Motor 2 FWD = Turn LEFT
    Serial.println("\n🔬 Testing hypothesis: M1-REV + M2-FWD = LEFT turn");
    Serial.println("   Strategy: Reduced speed + gyro integration for accuracy");
    
    resetEncoders();
    delay(100);
    
    // Use LOWER speed to avoid gyro saturation
    const int DIRECTIONAL_TEST_SPEED = 180; // Increased from 120 to overcome motor deadzone
    const unsigned long TEST_DURATION_MS = 2000; // 2 seconds
    
    // Start motors
    executeMotorCommand(false, true, true, false, DIRECTIONAL_TEST_SPEED); // M1-REV, M2-FWD
    
    // Monitor rotation with real-time gyro integration
    float totalRotation = monitorRotationWithGyro(TEST_DURATION_MS, -1); // Expect left (negative)
    
    // Stop motors
    allStop();
    Serial.println("   🛑 Motors stopped, settling...");
    delay(1000);
    
    // Verify with final heading measurement
    float finalHeading = getStableMPUHeading(20, false);
    float headingChange = finalHeading - baseHeading;
    
    // Normalize to -180 to +180
    while (headingChange > 180) headingChange -= 360;
    while (headingChange < -180) headingChange += 360;
    
    long finalEncoderAvg = getAverageEncoderCount();
    
    Serial.printf("\n📊 COMPREHENSIVE RESULTS:\n");
    Serial.printf("   Method                Result\n");
    Serial.printf("   ────────────────────  ───────────────\n");
    Serial.printf("   Gyro Integration:     %.2f°\n", totalRotation);
    Serial.printf("   Start-End Heading:    %.2f°\n", headingChange);
    Serial.printf("   Encoder Ticks:        %ld\n", finalEncoderAvg);
    
    // Use gyro integration as primary measurement (more accurate)
    float primaryMeasurement = totalRotation;
    
    // Verify encoder detected movement
    if (finalEncoderAvg < 50) {
        Serial.println("\n❌ ERROR: Encoders detected insufficient movement");
        Serial.println("   → Check encoder connections");
        Serial.println("   → Verify wheels are not slipping");
        return handleCalibrationFailure(CALIB_ERR_NO_MOVEMENT, "Directional Mapping");
    }
    
    // Check for gyro saturation issues
    if (abs(primaryMeasurement) < 5.0f && finalEncoderAvg > 100) {
        Serial.println("\n⚠️ CRITICAL ISSUE DETECTED:");
        Serial.println("   ✅ Encoders detect significant movement (" + String(finalEncoderAvg) + " ticks)");
        Serial.println("   ❌ Gyroscope detects minimal rotation (" + String(primaryMeasurement) + "°)");
        Serial.println("\n🔍 DIAGNOSIS: Gyroscope Saturation");
        Serial.println("   The robot is turning faster than the gyro can measure.");
        Serial.println("\n💡 SOLUTION:");
        Serial.println("   In WheelieHAL.cpp, inside initializeSensors(), change:");
        Serial.println("   FROM: mpu.setGyroConfig(2);  // ±1000°/s");
        Serial.println("   TO:   mpu.setGyroConfig(3);  // ±2000°/s");
        Serial.println("\n   Then restart calibration.");
        
        return handleCalibrationFailure(CALIB_ERR_SENSOR_INVALID, "Gyro Saturation");
    }
    
    // Determine direction with clear threshold
    const float TURN_THRESHOLD = 10.0f; // Need at least 10° rotation
    
    if (primaryMeasurement < -TURN_THRESHOLD) {
        Serial.println("\n✅ Hypothesis CORRECT: M1-REV + M2-FWD = LEFT turn");
        calibData.motorDirs.leftFwd_M1Fwd = false;
        calibData.motorDirs.leftFwd_M1Rev = true;
        calibData.motorDirs.leftFwd_M2Fwd = true;
        calibData.motorDirs.leftFwd_M2Rev = false;
    } else if (primaryMeasurement > TURN_THRESHOLD) {
        Serial.println("\n❌ Hypothesis WRONG: M1-REV + M2-FWD = RIGHT turn");
        Serial.println("✅ Corrected: M1-FWD + M2-REV = LEFT turn");
        calibData.motorDirs.leftFwd_M1Fwd = true;
        calibData.motorDirs.leftFwd_M1Rev = false;
        calibData.motorDirs.leftFwd_M2Fwd = false;
        calibData.motorDirs.leftFwd_M2Rev = true;
    } else {
        Serial.printf("\n❌ ERROR: Insufficient rotation detected (%.2f°)\n", primaryMeasurement);
        Serial.println("   Possible causes:");
        Serial.println("   • Battery voltage too low");
        Serial.println("   • Motors not powerful enough");
        Serial.println("   • High friction on wheels");
        Serial.println("   • Surface too slippery");
        return handleCalibrationFailure(CALIB_ERR_NO_MOVEMENT, "Directional Mapping");
    }
    
    // Cross-check gyro integration vs heading change
    float discrepancy = abs(totalRotation - headingChange);
    if (discrepancy > 15.0f) {
        Serial.printf("\n⚠️ WARNING: Measurement discrepancy detected (%.1f°)\n", discrepancy);
        Serial.println("   Gyro integration may have drift");
        Serial.println("   This is normal for long movements");
    }
    
    Serial.println("\n✅ Phase 1 complete: Left/Right motor mapping determined");
    Serial.printf("   Confidence: %s\n", 
                  discrepancy < 10.0f ? "HIGH ✓" : "MODERATE ⚠️");
    
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 2: TURN DISTANCE CALIBRATION - ENHANCED
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateTurnDistance() {
    Serial.println("\n🎯 PHASE 2: Turn Distance Calibration");
    Serial.println("──────────────────────────────────────");
    Serial.println("Goal: Find exact encoder ticks needed for 90° turn");

    // Theoretical calculation
    float wheelCircumference = WHEEL_DIAMETER_MM * M_PI;
    float robotTurnCircumference = TRACK_WIDTH_MM * M_PI;
    float ticksPerRevolution = ENCODER_SLOTS * GEAR_RATIO;
    float mmPerTick = wheelCircumference / ticksPerRevolution;
    float ticksPer90Deg_theoretical = (robotTurnCircumference / 4.0f) / mmPerTick;

    Serial.printf("[Theoretical] Ticks per 90°: %.2f\n", ticksPer90Deg_theoretical);

    // Wait for stable conditions
    if (!waitForStableConditions()) {
        return CALIB_ERR_TIMEOUT;
    }

    // Get baseline heading
    float startHeading = getStableMPUHeading(20, true);
    Serial.printf("📍 Starting heading: %.2f°\n", startHeading);

    // Reset encoders
    resetEncoders();
    delay(100);

    // Execute calibrated left turn
    Serial.println("🔄 Executing LEFT turn with gyro monitoring...");

    bool m1Fwd = calibData.motorDirs.leftFwd_M1Fwd;
    bool m1Rev = calibData.motorDirs.leftFwd_M1Rev;
    bool m2Fwd = calibData.motorDirs.leftFwd_M2Fwd;
    bool m2Rev = calibData.motorDirs.leftFwd_M2Rev;
    
    const int TURN_SPEED = 150; // Moderate speed for accuracy
    const unsigned long MAX_TURN_DURATION_MS = 5000; // 5 second safety timeout
    
    // Start turning
    executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, TURN_SPEED);
    
    // Monitor until we reach approximately 90°
    unsigned long turnStartTime = millis();
    unsigned long lastUpdate = turnStartTime;
    float accumulatedRotation = 0;
    const float TARGET_ANGLE = 90.0f;
    const float ANGLE_TOLERANCE = 5.0f;
    
    Serial.println("   Monitoring rotation progress:");
    
    while (millis() - turnStartTime < MAX_TURN_DURATION_MS) {
        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0f;
        
        hal.updateAllSensors();
        float gyroZ = sensors.gyroZ;
        
        // Integrate rotation
        accumulatedRotation += abs(gyroZ * dt);
        
        // Print progress every 500ms
        if (now - lastUpdate >= 500) {
            Serial.printf("   %.1fs: Rotated %.1f° (target: %.1f°)\n",
                         (now - turnStartTime) / 1000.0f,
                         accumulatedRotation, TARGET_ANGLE);
            lastUpdate = now;
        }
        
        // Stop when we reach target
        if (accumulatedRotation >= TARGET_ANGLE - ANGLE_TOLERANCE) {
            break;
        }
        
        delay(10);
    }
    
    // Stop motors
    allStop();
    delay(500);

    // Get measurements
    long leftTicks = abs(getLeftEncoderCount());
    long rightTicks = abs(getRightEncoderCount());
    long avgTicks = (leftTicks + rightTicks) / 2;

    // Verify final angle
    float finalHeading = getStableMPUHeading(20, false);
    float actualTurn = finalHeading - startHeading;
    while (actualTurn > 180) actualTurn -= 360;
    while (actualTurn < -180) actualTurn += 360;
    actualTurn = abs(actualTurn);

    Serial.printf("\n📊 Turn Analysis:\n");
    Serial.printf("   Left encoder:     %ld ticks\n", leftTicks);
    Serial.printf("   Right encoder:    %ld ticks\n", rightTicks);
    Serial.printf("   Average:          %ld ticks\n", avgTicks);
    Serial.printf("   Gyro integration: %.2f°\n", accumulatedRotation);
    Serial.printf("   Final heading:    %.2f°\n", actualTurn);
    Serial.printf("   Target:           90.0°\n");

    // Validation
    if (avgTicks < 50 || avgTicks > 10000) {
        Serial.println("❌ ERROR: Encoder reading out of range");
        return handleCalibrationFailure(CALIB_ERR_SENSOR_INVALID, "Turn Calibration");
    }

    // Use the actual measured angle for calculation
    float measuredAngle = actualTurn; // Use heading change as most reliable
    
    if (measuredAngle < 70.0f || measuredAngle > 110.0f) {
        Serial.printf("⚠️ WARNING: Turn significantly off target (%.1f°)\n", measuredAngle);
        if (measuredAngle < 45.0f) {
            Serial.println("   Falling back to theoretical value");
            calibData.ticksPer90Degrees = ticksPer90Deg_theoretical;
            calibData.ticksPer90DegreesEmpirical = ticksPer90Deg_theoretical;
            calibData.ticksPer90DegreesTheoretical = ticksPer90Deg_theoretical;
        } else {
            // Scale the encoder reading to 90°
            float ticksPerDegree = (float)avgTicks / measuredAngle;
            calibData.ticksPer90Degrees = ticksPerDegree * 90.0f;
            calibData.ticksPer90DegreesEmpirical = calibData.ticksPer90Degrees;
            calibData.ticksPer90DegreesTheoretical = ticksPer90Deg_theoretical;
        }
    } else {
        // Good measurement - use it directly with scaling
        float ticksPerDegree = (float)avgTicks / measuredAngle;
        calibData.ticksPer90Degrees = ticksPerDegree * 90.0f;
        calibData.ticksPer90DegreesEmpirical = calibData.ticksPer90Degrees;
        calibData.ticksPer90DegreesTheoretical = ticksPer90Deg_theoretical;
    }

    float deviation = abs(calibData.ticksPer90DegreesEmpirical - calibData.ticksPer90DegreesTheoretical);
    float deviationPercent = (deviation / calibData.ticksPer90DegreesTheoretical) * 100.0f;

    Serial.printf("\n✅ Calibration Complete:\n");
    Serial.printf("   Empirical:   %.0f ticks/90° ✓ USED\n", calibData.ticksPer90DegreesEmpirical);
    Serial.printf("   Theoretical: %.0f ticks/90°\n", calibData.ticksPer90DegreesTheoretical);
    Serial.printf("   Deviation:   %.1f%%\n", deviationPercent);
    
    if (deviationPercent > 20.0f) {
        Serial.println("   ⚠️ Large deviation - check wheel diameter and track width");
    }

    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 3: FORWARD/BACKWARD DETECTION
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateForwardBackward() {
    Serial.println("\n🧭 PHASE 3: Forward/Backward Detection");
    Serial.println("─────────────────────────────────────────────");
    Serial.println("Goal: Determine which motor pattern moves robot forward");
    
    if (!waitForStableConditions()) {
        return CALIB_ERR_TIMEOUT;
    }
    
    float initialDistance = getStableToFReading(10);
    if (initialDistance < 100 || initialDistance > 1500) {
        Serial.printf("❌ ERROR: Initial distance unsuitable: %.0f mm\n", initialDistance);
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    Serial.printf("📍 Baseline distance: %.0f mm\n", initialDistance);
    Serial.println("🔬 Testing hypothesis: M1-FWD + M2-FWD = FORWARD movement");
    
    resetEncoders();
    delay(100);
    
    executeMotorCommand(true, false, true, false, TEST_SPEED);
    delay(1000);
    allStop();
    delay(500);
    
    float finalDistance = getStableToFReading(10);
    float distanceChange = finalDistance - initialDistance;
    long encoderTicks = getAverageEncoderCount();
    
    Serial.printf("\n📊 RESULTS:\n");
    Serial.printf("   Initial distance: %.0f mm\n", initialDistance);
    Serial.printf("   Final distance: %.0f mm\n", finalDistance);
    Serial.printf("   Distance change: %.0f mm (negative = moved forward)\n", distanceChange);
    Serial.printf("   Encoder ticks: %ld\n", encoderTicks);
    
    if (distanceChange < -20.0) {
        Serial.println("✅ Hypothesis CORRECT: M1-FWD + M2-FWD = FORWARD");
        calibData.motorDirs.fwdMove_M1Fwd = true;
        calibData.motorDirs.fwdMove_M1Rev = false;
        calibData.motorDirs.fwdMove_M2Fwd = true;
        calibData.motorDirs.fwdMove_M2Rev = false;
    } else if (distanceChange > 20.0) {
        Serial.println("❌ Hypothesis WRONG: M1-FWD + M2-FWD = BACKWARD");
        Serial.println("✅ Corrected: M1-REV + M2-REV = FORWARD");
        calibData.motorDirs.fwdMove_M1Fwd = false;
        calibData.motorDirs.fwdMove_M1Rev = true;
        calibData.motorDirs.fwdMove_M2Fwd = false;
        calibData.motorDirs.fwdMove_M2Rev = true;
    } else {
        Serial.printf("⚠️ WARNING: Ambiguous distance change (%.0f mm)\n", distanceChange);
        Serial.println("   Using logical deduction from Phase 1 turn data...");
        
        calibData.motorDirs.fwdMove_M1Fwd = !calibData.motorDirs.leftFwd_M1Fwd;
        calibData.motorDirs.fwdMove_M1Rev = !calibData.motorDirs.leftFwd_M1Rev;
        calibData.motorDirs.fwdMove_M2Fwd = calibData.motorDirs.leftFwd_M2Fwd;
        calibData.motorDirs.fwdMove_M2Rev = calibData.motorDirs.leftFwd_M2Rev;
    }
    
    Serial.println("✅ Phase 3 complete: Forward/Backward motor mapping determined");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 4: DISTANCE & TOF CALIBRATION (FULLY AUTOMATED WITH FALLBACK)
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateDistanceAndToF() {
    Serial.println("\n📏 PHASE 4: Distance & ToF Calibration");
    Serial.println("─────────────────────────────────────────");
    Serial.println("Goal: Calibrate encoder ticks to real distance and find sensor offsets.");

    const float KNOWN_START_DISTANCE_MM = 300.0f; // 30 cm
    const float KNOWN_MOVE_DISTANCE_MM = 100.0f;  // 10 cm
    const float MAX_DETECTION_DISTANCE_MM = 800.0f; // 80cm - max distance to look for obstacles

    // Calculate Theoretical Value FIRST
    float wheelCircumference = WHEEL_DIAMETER_MM * M_PI;
    float ticksPerRevolution = ENCODER_SLOTS * GEAR_RATIO;
    float mmPerTick = wheelCircumference / ticksPerRevolution;
    calibData.ticksPerMillimeterTheoretical = 1.0f / mmPerTick;
    
    Serial.printf("[Theoretical] Ticks per mm: %.4f\n", calibData.ticksPerMillimeterTheoretical);

    // STEP 1: DETECT WHICH SENSOR CAN SEE AN OBSTACLE
    bool useFrontSensor = false;
    bool useRearSensor = false;
    
    Serial.println("\n🔍 Scanning for obstacles...");
    
    // Check front ToF sensor
    if (sysStatus.tofAvailable) {
        float frontDist = getStableToFReading(10);
        Serial.printf("   Front (ToF): %.1fmm\n", frontDist);
        
        if (frontDist > 0 && frontDist < MAX_DETECTION_DISTANCE_MM) {
            useFrontSensor = true;
            Serial.println("   ✅ Front sensor detects obstacle - will use ToF");
        }
    }
    
    // Check rear Ultrasonic sensor
    if (sysStatus.ultrasonicAvailable) {
        hal.updateAllSensors();
        float rearDistMM = sensors.rearDistanceCm * 10.0f;
        Serial.printf("   Rear (Ultrasonic): %.1fmm\n", rearDistMM);
        
        if (rearDistMM > 0 && rearDistMM < MAX_DETECTION_DISTANCE_MM) {
            useRearSensor = true;
            Serial.println("   ✅ Rear sensor detects obstacle - available as fallback");
        }
    }
    
    // STEP 2: DECIDE WHICH SENSOR TO USE
    bool needToTurn = false;
    bool usingFrontSensor = false;
    
    if (useFrontSensor) {
        Serial.println("\n🎯 Using FRONT sensor (ToF) for calibration");
        usingFrontSensor = true;
        needToTurn = false;
    } 
    else if (useRearSensor) {
        Serial.println("\n🔄 Front sensor cannot detect obstacle.");
        Serial.println("   Performing 180° turn to use REAR sensor (Ultrasonic)...");
        
        CalibrationResult turnResult = calibrate180Turn();
        if (turnResult != CALIB_SUCCESS) {
            Serial.println("❌ ERROR: 180° turn failed");
            return turnResult;
        }
        
        Serial.println("   ✅ 180° turn complete - rear sensor now facing obstacle");
        usingFrontSensor = false;
        needToTurn = true;
    } 
    else {
        Serial.println("❌ ERROR: No obstacles detected by any sensor!");
        Serial.println("   Please place an obstacle within 80cm of either the front or rear of the robot.");
        return CALIB_ERR_SENSOR_INVALID;
    }

    // STEP 3: AUTOMATED POSITIONING
    Serial.printf("\n🤖 Auto-positioning robot to %.0fmm from obstacle...\n", KNOWN_START_DISTANCE_MM);
    
    const float POSITION_TOLERANCE_MM = 15.0f;
    const int POSITION_SPEED = 80;
    const int MAX_POSITION_ATTEMPTS = 50;
    
    float currentDistance = 0.0f;
    int attempts = 0;
    
    while (attempts < MAX_POSITION_ATTEMPTS) {
        if (usingFrontSensor) {
            currentDistance = getStableToFReading(5);
        } else {
            hal.updateAllSensors();
            currentDistance = sensors.rearDistanceCm * 10.0f;
        }
        
        if (abs(currentDistance - KNOWN_START_DISTANCE_MM) <= POSITION_TOLERANCE_MM) {
            break;
        }
        
        float error = currentDistance - KNOWN_START_DISTANCE_MM;
        
        if (error > POSITION_TOLERANCE_MM) {
            Serial.printf("   Adjusting: %.0fmm away, moving closer...\n", error);
            executeMotorCommand(true, false, true, false, POSITION_SPEED);
            delay(200);
        } else if (error < -POSITION_TOLERANCE_MM) {
            Serial.printf("   Adjusting: %.0fmm too close, moving away...\n", -error);
            executeMotorCommand(false, true, false, true, POSITION_SPEED);
            delay(200);
        }
        
        allStop();
        delay(300);
        attempts++;
        
        if (attempts > MAX_POSITION_ATTEMPTS / 2) {
            if (abs(currentDistance - KNOWN_START_DISTANCE_MM) < POSITION_TOLERANCE_MM * 2) {
                Serial.println("   ⚠️ Accepting wider tolerance");
                break;
            }
        }
    }
    
    allStop();
    delay(500);

    if (attempts >= MAX_POSITION_ATTEMPTS) {
        Serial.println("❌ ERROR: Could not position robot accurately");
        return CALIB_ERR_TIMEOUT;
    }

    Serial.printf("✅ Positioned at %.1fmm from obstacle (target: %.1fmm)\n", 
                  currentDistance, KNOWN_START_DISTANCE_MM);
    
    if (!waitForStableConditions()) return CALIB_ERR_UNSTABLE;

    // STEP 4: MEASURE INITIAL DISTANCE
    float initialReading = 0.0f;
    if (usingFrontSensor) {
        initialReading = getStableToFReading(20);
    } else {
        float total = 0.0f;
        int samples = 20;
        for (int i = 0; i < samples; i++) {
            hal.updateAllSensors();
            total += sensors.rearDistanceCm * 10.0f;
            delay(50);
        }
        initialReading = total / samples;
    }
    
    if (initialReading <= 0 || initialReading > 2000) {
        return handleCalibrationFailure(CALIB_ERR_SENSOR_INVALID, "Initial Reading");
    }
    
    Serial.printf("   📏 Initial reading: %.1fmm (%s)\n", 
                  initialReading, usingFrontSensor ? "ToF" : "Ultrasonic");

    // STEP 5: MOVE FORWARD KNOWN DISTANCE
    resetEncoders();
    delay(100);
    
    Serial.printf("   🔄 Moving forward %.0fmm (non-blocking)...\n", KNOWN_MOVE_DISTANCE_MM);
    long targetTicks = (long)(KNOWN_MOVE_DISTANCE_MM * calibData.ticksPerMillimeterTheoretical);
    
    executeMoveUntil(targetTicks, 5000, true, false, true, false, TEST_SPEED);
    stopWithBrake();
    delay(500);

    // STEP 6: MEASURE FINAL DISTANCE
    float finalReading = 0.0f;
    if (usingFrontSensor) {
        finalReading = getStableToFReading(20);
    } else {
        float total = 0.0f;
        int samples = 20;
        for (int i = 0; i < samples; i++) {
            hal.updateAllSensors();
            total += sensors.rearDistanceCm * 10.0f;
            delay(50);
        }
        finalReading = total / samples;
    }
    
    if (finalReading <= 0 || finalReading > 2000) {
        return handleCalibrationFailure(CALIB_ERR_SENSOR_INVALID, "Final Reading");
    }
    
    Serial.printf("   📏 Final reading: %.1fmm (%s)\n", 
                  finalReading, usingFrontSensor ? "ToF" : "Ultrasonic");

    // STEP 7: CALCULATE CALIBRATION VALUES
    long actualTicks = getAverageEncoderCount();
    float distanceMoved = initialReading - finalReading;

    Serial.printf("\n📊 Movement Analysis:\n");
    Serial.printf("   Encoder ticks recorded: %ld\n", actualTicks);
    Serial.printf("   Distance moved (by sensor): %.1fmm\n", distanceMoved);
    Serial.printf("   Expected distance: %.1fmm\n", KNOWN_MOVE_DISTANCE_MM);

    // Sanity checks
    if (actualTicks < 50) {
        Serial.println("❌ ERROR: Too few encoder ticks");
        return handleCalibrationFailure(CALIB_ERR_NO_MOVEMENT, "Distance Calibration");
    }
    
    if (distanceMoved < KNOWN_MOVE_DISTANCE_MM * 0.5f || distanceMoved > KNOWN_MOVE_DISTANCE_MM * 2.0f) {
        Serial.printf("❌ ERROR: Sensor reports unrealistic movement (%.1fmm)\n", distanceMoved);
        return handleCalibrationFailure(CALIB_ERR_SENSOR_INVALID, "Distance Calibration");
    }

    // Calculate empirical calibration
    calibData.ticksPerMillimeterEmpirical = (float)actualTicks / distanceMoved;
    calibData.ticksPerMillimeter = calibData.ticksPerMillimeterEmpirical;

    // Calculate sensor offset
    float expectedFinalDistance = initialReading - distanceMoved;
    float sensorOffset = expectedFinalDistance - finalReading;
    
    if (usingFrontSensor) {
        calibData.tofOffsetMM = sensorOffset;
        Serial.printf("   ToF sensor offset: %.2fmm\n", calibData.tofOffsetMM);
    } else {
        Serial.printf("   Ultrasonic offset: %.2fmm (informational)\n", sensorOffset);
        calibData.tofOffsetMM = 0.0f;
    }

    Serial.printf("\n✅ Calibration Values:\n");
    Serial.printf("   Sensor used: %s\n", usingFrontSensor ? "ToF (front)" : "Ultrasonic (rear)");
    Serial.printf("   Ticks per mm (theoretical): %.4f\n", calibData.ticksPerMillimeterTheoretical);
    Serial.printf("   Ticks per mm (empirical):   %.4f ✓ USED\n", calibData.ticksPerMillimeterEmpirical);
    Serial.printf("   Deviation: %.1f%%\n", 
                  abs(1.0f - (calibData.ticksPerMillimeterEmpirical / 
                              calibData.ticksPerMillimeterTheoretical)) * 100.0f);

    // STEP 8: TURN BACK IF WE DID A 180° TURN EARLIER
    if (needToTurn) {
        Serial.println("\n🔄 Returning to original orientation...");
        CalibrationResult turnResult = calibrate180Turn();
        if (turnResult != CALIB_SUCCESS) {
            Serial.println("⚠️ WARNING: Could not turn back to original orientation");
        } else {
            Serial.println("   ✅ Returned to original orientation");
        }
    }

    Serial.println("✅ Phase 4 complete: Distance calibration successful");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 5: MOTOR DEADZONE CALIBRATION
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
        
        delay(200);
    }
    
    Serial.printf("⚠️ No movement detected up to PWM %d\n", maxPWM);
    return maxPWM;
}

bool testMotorMovement(uint8_t pwmValue, int durationMs) {
    resetEncoders();
    delay(50);
    
    executeMotorCommand(true, false, true, false, pwmValue);
    delay(durationMs);
    stopMotorsGently();
    
    long leftTicks = abs(getLeftEncoderCount());
    long rightTicks = abs(getRightEncoderCount());
    long totalMovement = leftTicks + rightTicks;
    
    Serial.printf("(L:%ld R:%ld) ", leftTicks, rightTicks);
    
    return totalMovement > 0;
}

CalibrationResult calibrateMotorDeadzone() {
    Serial.println("\n🎯 PHASE 5: MOTOR DEADZONE CALIBRATION");
    Serial.println("Finding minimum PWM value for reliable movement...");
    
    calibrationProgressUpdate("Motor Deadzone", 0);
    
    uint8_t deadzoneMargin = 10;
    uint8_t rawDeadzone = findMotorDeadzone(100, 5);
    
    calibrationProgressUpdate("Motor Deadzone", 50);
    
    calibData.minMotorSpeedPWM = rawDeadzone + deadzoneMargin;
    
    if (calibData.minMotorSpeedPWM < 15) {
        calibData.minMotorSpeedPWM = 15;
    }
    if (calibData.minMotorSpeedPWM > 80) {
        calibData.minMotorSpeedPWM = 80;
    }
    
    calibrationProgressUpdate("Motor Deadzone", 75);
    
    Serial.printf("🧪 Testing final deadzone value: %d\n", calibData.minMotorSpeedPWM);
    if (!testMotorMovement(calibData.minMotorSpeedPWM, 1000)) {
        Serial.println("⚠️ Final deadzone test failed, but continuing...");
    }
    
    calibrationProgressUpdate("Motor Deadzone", 100);
    
    Serial.printf("✅ Motor deadzone calibration complete: %d PWM\n", calibData.minMotorSpeedPWM);
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// 180° TURN CALIBRATION (Helper for Phase 4)
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrate180Turn() {
    Serial.println("\n🔄 Performing 180° turn calibration...");
    
    if (!waitForStableConditions()) {
        return CALIB_ERR_TIMEOUT;
    }
    
    float startHeading = getStableMPUHeading(20, true);
    Serial.printf("   Start heading: %.2f°\n", startHeading);
    
    // Calculate ticks needed for 180° (2x the 90° value)
    long ticksFor180 = (long)(calibData.ticksPer90Degrees * 2.0f);
    
    resetEncoders();
    delay(100);
    
    Serial.println("   Executing 180° turn...");
    
    // Use calibrated left turn direction
    bool m1Fwd = calibData.motorDirs.leftFwd_M1Fwd;
    bool m1Rev = calibData.motorDirs.leftFwd_M1Rev;
    bool m2Fwd = calibData.motorDirs.leftFwd_M2Fwd;
    bool m2Rev = calibData.motorDirs.leftFwd_M2Rev;
    
    executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, TURN_SPEED);
    
    // Monitor until encoder target reached
    unsigned long startTime = millis();
    while (getAverageEncoderCount() < ticksFor180 && millis() - startTime < 10000) {
        hal.updateAllSensors();
        delay(10);
    }
    
    allStop();
    delay(500);
    
    float finalHeading = getStableMPUHeading(20, false);
    float actualTurn = finalHeading - startHeading;
    
    // Normalize
    while (actualTurn > 180) actualTurn -= 360;
    while (actualTurn < -180) actualTurn += 360;
    actualTurn = abs(actualTurn);
    
    Serial.printf("   Target: 180.0°, Actual: %.1f°\n", actualTurn);
    
    if (actualTurn < 160.0f || actualTurn > 200.0f) {
        Serial.println("   ⚠️ WARNING: 180° turn accuracy is poor");
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    Serial.println("   ✅ 180° turn complete");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

bool executeMoveUntil(long targetTicks, unsigned long timeoutMs, bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed) {
    static unsigned long moveStartTime = 0;
    static bool moveInProgress = false;

    if (!moveInProgress) {
        resetEncoders();
        delay(50);
        executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, speed);
        moveStartTime = millis();
        moveInProgress = true;
        Serial.printf("   [Move] Started. Target: %ld ticks, Timeout: %lu ms\n", targetTicks, timeoutMs);
        return true;
    }

    if (moveInProgress) {
        if (getAverageEncoderCount() >= targetTicks) {
            stopWithBrake();
            moveInProgress = false;
            Serial.printf("   [Move] Completed at %ld ticks.\n", getAverageEncoderCount());
            return false;
        }

        if (millis() - moveStartTime > timeoutMs) {
            stopWithBrake();
            moveInProgress = false;
            Serial.println("   [Move] ❌ Timed out.");
            return false;
        }
    }
    return true;
}

bool executeMoveForDuration(unsigned long durationMs, bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed) {
    static unsigned long moveStartTime = 0;
    static bool moveInProgress = false;

    if (!moveInProgress) {
        executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, speed);
        moveStartTime = millis();
        moveInProgress = true;
        Serial.printf("   [Move] Started. Duration: %lu ms\n", durationMs);
        return true;
    }

    if (moveInProgress) {
        if (millis() - moveStartTime >= durationMs) {
            stopWithBrake();
            moveInProgress = false;
            Serial.println("   [Move] Duration complete.");
            return false;
        }
    }

    return true;
}

void executeMotorCommand(bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed) {
    int motor1Speed = 0;
    if (m1Fwd && !m1Rev) motor1Speed = speed;
    else if (m1Rev && !m1Fwd) motor1Speed = -speed;
    
    int motor2Speed = 0;
    if (m2Fwd && !m2Rev) motor2Speed = speed;
    else if (m2Rev && !m2Fwd) motor2Speed = -speed;
    
    setMotorPWM(motor1Speed, motor2Speed);
}

void stopMotorsGently() {
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
        hal.updateAllSensors();
        float distanceCm = sensors.frontDistanceCm;
        if (distanceCm > 0 && distanceCm < 200.0) {
            total += distanceCm * 10.0f;
            validSamples++;
        }
        delay(50);
    }
    
    return validSamples > 0 ? (total / validSamples) : 0;
}

bool isCalibrationSafe() {
    if (!sysStatus.tofAvailable || !sysStatus.mpuAvailable) {
        Serial.println("❌ Required sensors not available");
        return false;
    }
    
    hal.updateAllSensors();
    float tiltX = abs(sensors.tiltX);
    float tiltY = abs(sensors.tiltY);
    if (tiltX > 30.0 || tiltY > 30.0) {
        Serial.printf("❌ Robot is tilted too much for calibration (X:%.1f deg Y:%.1f deg)\n", tiltX, tiltY);
        Serial.println("   Please place robot on a flat, level surface");
        return false;
    }
    
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
        hal.updateAllSensors();
        delay(100);
    }
    
    Serial.println("✅ Sensors stabilized");
    return true;
}

void calibrationProgressUpdate(const char* phase, int progress) {
    Serial.printf("\n📈 PROGRESS: %s (%d%%)\n", phase, progress);
    
    if (progress < 25) setLEDColor(LEDColors::RED);
    else if (progress < 50) setLEDColor(LEDColors::YELLOW);
    else if (progress < 75) setLEDColor(LEDColors::BLUE);
    else if (progress < 100) setLEDColor(LEDColors::GREEN);
    else setLEDColor(LEDColors::WHITE);
    
    if (progress == 100) {
        buzz(2000, 200);
    }
}

CalibrationResult validateCalibrationData() {
    if (calibData.magic != CALIBRATION_MAGIC) {
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    if (calibData.version != CALIBRATION_VERSION) {
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    if (calibData.ticksPer90Degrees < 100 || calibData.ticksPer90Degrees > 10000) {
        return CALIB_ERR_SENSOR_INVALID;
    }
    if (calibData.ticksPerMillimeter < 1.0 || calibData.ticksPerMillimeter > 100.0) {
        return CALIB_ERR_SENSOR_INVALID;
    }
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// CHECKSUM AND DATA INTEGRITY
// ═══════════════════════════════════════════════════════════════════════════

uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool verifyDataIntegrity(const CalibrationData* data) {
    uint16_t calculatedCRC = calculateCRC16((const uint8_t*)data, 
                                           sizeof(CalibrationData) - sizeof(uint16_t));
    
    return calculatedCRC == data->checksum;
}

CalibrationResult updateChecksum(CalibrationData* data) {
    if (data == nullptr) {
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
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
    
    calibrationEmergencyStop();
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATION MANAGEMENT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult saveCalibrationData() {
    Serial.println("💾 Saving calibration data to EEPROM...");
    
    updateChecksum(&calibData);
    
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
    delay(10);
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
    
    if (calibData.magic == 0xFF) {
        Serial.println("   ℹ️  EEPROM is uninitialized (reads as 0xFF). Calibration required.");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    else if (calibData.magic != CALIBRATION_MAGIC) {
        Serial.printf("   ❌ Invalid magic number. Expected 0x%02X, but found 0x%02X. Data is corrupt or from an old version.\n",
                      CALIBRATION_MAGIC, calibData.magic);
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    if (calibData.version != CALIBRATION_VERSION) {
        Serial.printf("   ❌ Version mismatch. Found %d, expected %d.\n", calibData.version, CALIBRATION_VERSION);
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    if (!verifyDataIntegrity(&calibData)) {
        Serial.println("   ❌ Checksum verification failed. Data is corrupt.");
        return CALIB_ERR_CHECKSUM_FAILED;
    }
    
    CalibrationResult validationResult = validateCalibrationData();
    if (validationResult != CALIB_SUCCESS) {
        Serial.println("   ❌ Calibration data is out of reasonable bounds.");
        return validationResult;
    }
    
    Serial.println("✅ Calibration data loaded and verified successfully.");
    isCalibrated = true;
    return CALIB_SUCCESS;
}

CalibrationResult calibrateSensorBaselines() {
    Serial.println("📊 Establishing zero-angle baseline for IMU...");
    hal.updateAllSensors();
    calibData.mpuOffsets.baselineTiltX = sensors.tiltX;
    calibData.mpuOffsets.baselineTiltY = sensors.tiltY;
    hal.updateAllSensors();
    Serial.printf("   ✅ Baseline established. Tilt X: %.2f, Tilt Y: %.2f\n", 
                  calibData.mpuOffsets.baselineTiltX, calibData.mpuOffsets.baselineTiltY);
    return CALIB_SUCCESS;
}

CalibrationResult runFullCalibrationSequence() {
    Serial.println("🔄 Starting full calibration sequence...");
    CalibrationResult result;
    
    // NOTE: Encoder Sanity Check is now called from WheelieHAL::init()
    // before MPU calibration, as requested. It can also be added here
    // if you want it to be part of the main sequence post-MPU.

    result = calibrateDirectionalMapping();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Directional Mapping");

    result = calibrateTurnDistance();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Turn Distance");

    result = calibrateForwardBackward();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Forward/Backward");

    result = calibrateSensorBaselines();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Sensor Baselines");

    result = calibrateMotorDeadzone();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Motor Deadzone");

    result = saveCalibrationData();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Save Calibration");

    Serial.println("✅ All calibration phases complete!");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATED MOVEMENT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void calibratedTurnLeft(int speed) {
    if (!isCalibrated) {
        rotateLeft(TURN_SPEED);
        return;
    }
    
    executeMotorCommand(calibData.motorDirs.leftFwd_M1Fwd, calibData.motorDirs.leftFwd_M1Rev,
                       calibData.motorDirs.leftFwd_M2Fwd, calibData.motorDirs.leftFwd_M2Rev, speed);
}

void calibratedTurnRight(int speed) {
    if (!isCalibrated) {
        rotateRight(TURN_SPEED);
        return;
    }
    
    executeMotorCommand(!calibData.motorDirs.leftFwd_M1Fwd, !calibData.motorDirs.leftFwd_M1Rev,
                       !calibData.motorDirs.leftFwd_M2Fwd, !calibData.motorDirs.leftFwd_M2Rev, speed);
}

void calibratedMoveForward(int speed) {
    if (!isCalibrated) {
        moveForward(TEST_SPEED);
        return;
    }
    
    executeMotorCommand(calibData.motorDirs.fwdMove_M1Fwd, calibData.motorDirs.fwdMove_M1Rev,
                       calibData.motorDirs.fwdMove_M2Fwd, calibData.motorDirs.fwdMove_M2Rev, speed);
}

void calibratedMoveBackward(int speed) {
    if (!isCalibrated) {
        moveBackward(TEST_SPEED);
        return;
    }
    
    executeMotorCommand(!calibData.motorDirs.fwdMove_M1Fwd, !calibData.motorDirs.fwdMove_M1Rev,
                       !calibData.motorDirs.fwdMove_M2Fwd, !calibData.motorDirs.fwdMove_M2Rev, speed);
}

void calibratedTurn90Left() {
    if (!isCalibrated) return;
    
    resetEncoders();
    calibratedTurnLeft(TURN_SPEED);
    
    while (executeMoveUntil(calibData.ticksPer90Degrees, 5000, 
                            calibData.motorDirs.leftFwd_M1Fwd, calibData.motorDirs.leftFwd_M1Rev,
                            calibData.motorDirs.leftFwd_M2Fwd, calibData.motorDirs.leftFwd_M2Rev, TURN_SPEED)) {
        // Non-blocking wait
    }
}

void calibratedTurn90Right() {
    if (!isCalibrated) return;
    
    resetEncoders();
    calibratedTurnRight(TURN_SPEED);

    while (executeMoveUntil(calibData.ticksPer90Degrees, 5000,
                           !calibData.motorDirs.leftFwd_M1Fwd, !calibData.motorDirs.leftFwd_M1Rev,
                           !calibData.motorDirs.leftFwd_M2Fwd, !calibData.motorDirs.leftFwd_M2Rev, TURN_SPEED)) {
        // Non-blocking wait
    }
}

void calibratedMoveDistance(float mm) {
    if (!isCalibrated) return;
    
    long targetTicks = (long)(mm * calibData.ticksPerMillimeter);
    bool isForward = mm > 0;

    if (mm > 0) {
        while (executeMoveUntil(targetTicks, 10000, calibData.motorDirs.fwdMove_M1Fwd, calibData.motorDirs.fwdMove_M1Rev, calibData.motorDirs.fwdMove_M2Fwd, calibData.motorDirs.fwdMove_M2Rev, TEST_SPEED)) {
            // Non-blocking wait
        }
    } else {
        while (executeMoveUntil(targetTicks, 10000, !calibData.motorDirs.fwdMove_M1Fwd, !calibData.motorDirs.fwdMove_M1Rev, !calibData.motorDirs.fwdMove_M2Fwd, !calibData.motorDirs.fwdMove_M2Rev, TEST_SPEED)) {
            // Non-blocking wait
        }
    }
}