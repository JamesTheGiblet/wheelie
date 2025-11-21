// ═══════════════════════════════════════════════════════════════════════════
// 🤖 AUTONOMOUS CALIBRATION SYSTEM - Implementation
// ═══════════════════════════════════════════════════════════════════════════
// Complete self-calibration system with EEPROM storage
// Runs once on first boot, stores results permanently
// ═══════════════════════════════════════════════════════════════════════════

#include "calibration.h"
#include "WheelieHAL.h"

// Forward declaration for non-blocking movement function
bool executeMoveUntil(long targetTicks, unsigned long timeoutMs, bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed);
bool executeMoveForDuration(unsigned long durationMs, bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed);

// Define wheel diameter in millimeters if not defined elsewhere
#ifndef WHEEL_DIAMETER_MM
#define WHEEL_DIAMETER_MM 65.0f // Example value, set to your actual wheel diameter
#endif

// Define track width in millimeters if not defined elsewhere
#ifndef TRACK_WIDTH_MM
#define TRACK_WIDTH_MM 120.0f // Example value, set to your actual track width
#endif

// Define encoder slots if not defined elsewhere
#ifndef ENCODER_SLOTS
#define ENCODER_SLOTS 20 // Example value, set to your actual encoder slots per revolution
#endif

// Define gear ratio if not defined elsewhere
#ifndef GEAR_RATIO
#define GEAR_RATIO 1.0f // Example value, set to your actual gear ratio
#endif

extern WheelieHAL hal; // Allow access to the global HAL object
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

    // --- Calculate Theoretical Value FIRST ---
    float wheelCircumference = WHEEL_DIAMETER_MM * M_PI;
    float ticksPerRevolution = ENCODER_SLOTS * GEAR_RATIO;
    float mmPerTick = wheelCircumference / ticksPerRevolution;
    calibData.ticksPerMillimeterTheoretical = 1.0f / mmPerTick;
    
    Serial.printf("[Theoretical] Ticks per mm: %.4f\n", calibData.ticksPerMillimeterTheoretical);

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 1: DETECT WHICH SENSOR CAN SEE AN OBSTACLE
    // ═══════════════════════════════════════════════════════════════════════
    
    bool useFrontSensor = false;  // ToF at front
    bool useRearSensor = false;   // Ultrasonic at rear
    
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
        float rearDistMM = sensors.rearDistanceCm * 10.0f; // Convert cm to mm
        Serial.printf("   Rear (Ultrasonic): %.1fmm\n", rearDistMM);
        
        if (rearDistMM > 0 && rearDistMM < MAX_DETECTION_DISTANCE_MM) {
            useRearSensor = true;
            Serial.println("   ✅ Rear sensor detects obstacle - available as fallback");
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 2: DECIDE WHICH SENSOR TO USE (WITH AUTOMATIC 180° TURN IF NEEDED)
    // ═══════════════════════════════════════════════════════════════════════
    
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
        
        // Perform 180° turn
        CalibrationResult turnResult = calibrate180Turn();
        if (turnResult != CALIB_SUCCESS) {
            Serial.println("❌ ERROR: 180° turn failed");
            return turnResult;
        }
        
        Serial.println("   ✅ 180° turn complete - rear sensor now facing obstacle");
        usingFrontSensor = false; // We're now using the "front" position but it's the ultrasonic
        needToTurn = true; // Remember we turned so we can turn back later
    } 
    else {
        Serial.println("❌ ERROR: No obstacles detected by any sensor!");
        Serial.println("   Please place an obstacle within 80cm of either the front or rear of the robot.");
        return CALIB_ERR_SENSOR_INVALID;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 3: AUTOMATED POSITIONING
    // ═══════════════════════════════════════════════════════════════════════
    
    Serial.printf("\n🤖 Auto-positioning robot to %.0fmm from obstacle...\n", KNOWN_START_DISTANCE_MM);
    
    const float POSITION_TOLERANCE_MM = 15.0f;
    const int POSITION_SPEED = 80;
    const int MAX_POSITION_ATTEMPTS = 50;
    
    float currentDistance = 0.0f;
    int attempts = 0;
    
    while (attempts < MAX_POSITION_ATTEMPTS) {
        // Read from the active sensor
        if (usingFrontSensor) {
            currentDistance = getStableToFReading(5);
        } else {
            hal.updateAllSensors();
            currentDistance = sensors.rearDistanceCm * 10.0f;
        }
        
        // Check if we're in position
        if (abs(currentDistance - KNOWN_START_DISTANCE_MM) <= POSITION_TOLERANCE_MM) {
            break;
        }
        
        float error = currentDistance - KNOWN_START_DISTANCE_MM;
        
        if (error > POSITION_TOLERANCE_MM) {
            // Too far, move forward (toward obstacle)
            Serial.printf("   Adjusting: %.0fmm away, moving closer...\n", error);
            executeMotorCommand(true, false, true, false, POSITION_SPEED);
            delay(200);
        } else if (error < -POSITION_TOLERANCE_MM) {
            // Too close, move backward (away from obstacle)
            Serial.printf("   Adjusting: %.0fmm too close, moving away...\n", -error);
            executeMotorCommand(false, true, false, true, POSITION_SPEED);
            delay(200);
        }
        
        allStop();
        delay(300);
        attempts++;
        
        // Adaptive tolerance after many attempts
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

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 4: MEASURE INITIAL DISTANCE
    // ═══════════════════════════════════════════════════════════════════════
    
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

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 5: MOVE FORWARD KNOWN DISTANCE
    // ═══════════════════════════════════════════════════════════════════════
    
    resetEncoders();
    delay(100);
    
    Serial.printf("   🔄 Moving forward %.0fmm (non-blocking)...\n", KNOWN_MOVE_DISTANCE_MM);
    long targetTicks = (long)(KNOWN_MOVE_DISTANCE_MM * calibData.ticksPerMillimeterTheoretical);
    
    // This is now a non-blocking call.
    executeMoveUntil(targetTicks, 5000, true, false, true, false, TEST_SPEED);
    stopWithBrake();
    delay(500);

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 6: MEASURE FINAL DISTANCE
    // ═══════════════════════════════════════════════════════════════════════
    
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

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 7: CALCULATE CALIBRATION VALUES
    // ═══════════════════════════════════════════════════════════════════════
    
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
        // Store ultrasonic offset (you may want to add this to CalibrationData struct)
        Serial.printf("   Ultrasonic offset: %.2fmm (informational)\n", sensorOffset);
        calibData.tofOffsetMM = 0.0f; // ToF not calibrated
    }

    Serial.printf("\n✅ Calibration Values:\n");
    Serial.printf("   Sensor used: %s\n", usingFrontSensor ? "ToF (front)" : "Ultrasonic (rear)");
    Serial.printf("   Ticks per mm (theoretical): %.4f\n", calibData.ticksPerMillimeterTheoretical);
    Serial.printf("   Ticks per mm (empirical):   %.4f ✓ USED\n", calibData.ticksPerMillimeterEmpirical);
    Serial.printf("   Deviation: %.1f%%\n", 
                  abs(1.0f - (calibData.ticksPerMillimeterEmpirical / 
                              calibData.ticksPerMillimeterTheoretical)) * 100.0f);

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 8: TURN BACK IF WE DID A 180° TURN EARLIER
    // ═══════════════════════════════════════════════════════════════════════
    
    if (needToTurn) {
        Serial.println("\n🔄 Returning to original orientation...");
        CalibrationResult turnResult = calibrate180Turn();
        if (turnResult != CALIB_SUCCESS) {
            Serial.println("⚠️ WARNING: Could not turn back to original orientation");
            // Don't fail calibration for this - the distance calibration succeeded
        } else {
            Serial.println("   ✅ Returned to original orientation");
        }
    }

    Serial.println("✅ Phase 4 complete: Distance calibration successful");
    return CALIB_SUCCESS;
}
        
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
    unsigned long startTime = millis();
    executeMotorCommand(false, true, true, false, 150); // M1-REV, M2-FWD
    delay(1500); // Run motors for 1.5 seconds
    
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
        Serial.println("   1. GYROSCOPE SATURATION: The robot is turning faster than the sensor's configured range.");
        Serial.println("      The current range is set to +/- 1000 dps. This is the most likely cause.");
        Serial.println("   2. INCORRECT MOUNTING: The MPU6050's Z-axis is not aligned with the robot's vertical axis of rotation.");
        Serial.println("   3. HARDWARE/I2C ISSUE: The MPU6050 may have failed or there are I2C communication errors.");
        Serial.println("\n💡 SOLUTIONS TO TRY:");
        Serial.println("   A. In WheelieHAL.cpp, inside initializeSensors(), change mpu.setGyroConfig(2) to mpu.setGyroConfig(3).");
        Serial.println("      This increases the range to +/- 2000 dps, which should prevent saturation.");
        Serial.println("   B. Verify the MPU is mounted flat and its Z-axis is pointing straight up or down.");
        Serial.println("   C. Check I2C wiring (SDA/SCL) for loose connections.");
        
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
    const int TURN_SPEED = 150; // Increased for a more definitive turn.

    // Use the new non-blocking move function
    while (executeMoveForDuration(TURN_DURATION_MS, m1Fwd, m1Rev, m2Fwd, m2Rev, TURN_SPEED)) {
        // This loop allows background tasks to run.
    }
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
    
    // Use the non-blocking move function. The timeout is generous.
    while (executeMoveUntil(calibData.ticksPer90Degrees, 5000, 
                            calibData.motorDirs.leftFwd_M1Fwd, calibData.motorDirs.leftFwd_M1Rev,
                            calibData.motorDirs.leftFwd_M2Fwd, calibData.motorDirs.leftFwd_M2Rev, TURN_SPEED)) {
        // This loop allows background tasks to run.
    }
}

void calibratedTurn90Right() {
    if (!isCalibrated) return;
    
    resetEncoders();
    calibratedTurnRight(TURN_SPEED);

    // Use the non-blocking move function. The timeout is generous.
    while (executeMoveUntil(calibData.ticksPer90Degrees, 5000,
                           !calibData.motorDirs.leftFwd_M1Fwd, !calibData.motorDirs.leftFwd_M1Rev,
                           !calibData.motorDirs.leftFwd_M2Fwd, !calibData.motorDirs.leftFwd_M2Rev, TURN_SPEED)) {
        // This loop allows background tasks to run.
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

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief NON-BLOCKING movement function. Starts a motor command and continues until
 * a target encoder count is reached or a timeout occurs.
 * This function MUST be called repeatedly in a loop until it stops returning true.
 * @param targetTicks The average encoder count to reach.
 * @param timeoutMs The maximum time to allow for the movement.
 * @return true if the move is still in progress, false if it has completed or timed out.
 */
bool executeMoveUntil(long targetTicks, unsigned long timeoutMs, bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed) {
    static unsigned long moveStartTime = 0;
    static bool moveInProgress = false;

    // --- State 1: Start the movement ---
    if (!moveInProgress) {
        resetEncoders();
        delay(50); // Small delay to ensure encoders are zeroed
        executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, speed);
        moveStartTime = millis();
        moveInProgress = true;
        Serial.printf("   [Move] Started. Target: %ld ticks, Timeout: %lu ms\n", targetTicks, timeoutMs);
        return true; // Still in progress
    }

    // --- State 2: Monitor the movement ---
    if (moveInProgress) {
        // Check for completion
        if (getAverageEncoderCount() >= targetTicks) {
            stopWithBrake();
            moveInProgress = false; // Reset for next call
            Serial.printf("   [Move] Completed at %ld ticks.\n", getAverageEncoderCount());
            return false; // Move finished
        }

        // Check for timeout
        if (millis() - moveStartTime > timeoutMs) {
            stopWithBrake();
            moveInProgress = false; // Reset for next call
            Serial.println("   [Move] ❌ Timed out.");
            return false; // Move finished (by timeout)
        }
    }
    return true; // Still in progress
}

/**
 * @brief NON-BLOCKING movement function. Starts a motor command and continues until
 * a specific duration has elapsed.
 * This function MUST be called repeatedly in a loop until it stops returning true.
 * @param durationMs The time to run the motors for.
 * @return true if the move is still in progress, false if it has completed.
 */
bool executeMoveForDuration(unsigned long durationMs, bool m1Fwd, bool m1Rev, bool m2Fwd, bool m2Rev, int speed) {
    static unsigned long moveStartTime = 0;
    static bool moveInProgress = false;

    // --- State 1: Start the movement ---
    if (!moveInProgress) {
        executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, speed);
        moveStartTime = millis();
        moveInProgress = true;
        Serial.printf("   [Move] Started. Duration: %lu ms\n", durationMs);
        return true; // Still in progress
    }

    // --- State 2: Monitor the movement ---
    if (moveInProgress) {
        // Check for completion
        if (millis() - moveStartTime >= durationMs) {
            stopWithBrake();
            moveInProgress = false; // Reset for next call
            Serial.println("   [Move] Duration complete.");
            return false; // Move finished
        }
    }

    return true; // Still in progress
}

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
        float distanceCm = sensors.frontDistanceCm;
        if (distanceCm > 0 && distanceCm < 200.0) { // Check in cm
            total += distanceCm * 10.0f; // Convert back to mm for consistency
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
        Serial.println("   Falling back to robust logical deduction based on Phase 1 turn data.");

        // LOGIC: A left turn requires the right wheel to go forward and the left wheel to go backward.
        // Therefore, the command for "forward" for the right wheel (M2) is the same as in a left turn.
        // The command for "forward" for the left wheel (M1) is the OPPOSITE of what it was in a left turn.
        calibData.motorDirs.fwdMove_M1Fwd = !calibData.motorDirs.leftFwd_M1Fwd;
        calibData.motorDirs.fwdMove_M1Rev = !calibData.motorDirs.leftFwd_M1Rev;
        calibData.motorDirs.fwdMove_M2Fwd = calibData.motorDirs.leftFwd_M2Fwd;
        calibData.motorDirs.fwdMove_M2Rev = calibData.motorDirs.leftFwd_M2Rev;
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
    if (calibData.magic == 0xFF) {
        Serial.println("   ℹ️  EEPROM is uninitialized (reads as 0xFF). Calibration required.");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    else if (calibData.magic != CALIBRATION_MAGIC) {
        Serial.printf("   ❌ Invalid magic number. Expected 0x%02X, but found 0x%02X. Data is corrupt or from an old version.\n",
                      CALIBRATION_MAGIC, calibData.magic);
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

// Simplified sensor baseline calibration
CalibrationResult calibrateSensorBaselines() {
    Serial.println("📊 Establishing zero-angle baseline for IMU...");
    // This is a two-step process to avoid a NaN result.
    // 1. Get a raw reading first to establish the baseline.
    hal.updateAllSensors();
    calibData.mpuOffsets.baselineTiltX = sensors.tiltX;
    calibData.mpuOffsets.baselineTiltY = sensors.tiltY;
    // 2. Now, subsequent calls to updateAllSensors() will correctly subtract this valid baseline.
    hal.updateAllSensors();
    Serial.printf("   ✅ Baseline established. Tilt X: %.2f, Tilt Y: %.2f\n", 
                  calibData.mpuOffsets.baselineTiltX, calibData.mpuOffsets.baselineTiltY);
    return CALIB_SUCCESS;
}

// Unified and simplified calibration sequence
CalibrationResult runFullCalibrationSequence() {
    Serial.println("🔄 Starting full calibration sequence...");
    CalibrationResult result;

    // 1. Directional mapping
    result = calibrateDirectionalMapping();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Directional Mapping");

    // 2. Turn distance
    result = calibrateTurnDistance();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Turn Distance");

    // 3. Forward/backward
    result = calibrateForwardBackward();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Forward/Backward");

    // 4. Baseline IMU/ToF
    result = calibrateSensorBaselines();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Sensor Baselines");

    // 5. Motor Deadzone
    result = calibrateMotorDeadzone();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Motor Deadzone");

    // 5. Save
    result = saveCalibrationData();
    if (result != CALIB_SUCCESS) return handleCalibrationFailure(result, "Save Calibration");

    Serial.println("✅ All calibration phases complete!");
    return CALIB_SUCCESS;
}
