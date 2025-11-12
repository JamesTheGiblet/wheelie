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
    
    // SOS pattern for critical failure indication
    for (int i = 0; i < 10; i++) {
        // SOS: ... --- ...
        // Three short
        for (int j = 0; j < 3; j++) {
            setLEDColor(true, false, false); // Red
            buzz(1500, 200);
            delay(200);
            setLEDColor(false, false, false);
            delay(200);
        }
        delay(500);
        
        // Three long
        for (int j = 0; j < 3; j++) {
            setLEDColor(true, false, false); // Red
            buzz(1500, 600);
            delay(600);
            setLEDColor(false, false, false);
            delay(200);
        }
        delay(500);
        
        // Three short
        for (int j = 0; j < 3; j++) {
            setLEDColor(true, false, false); // Red
            buzz(1500, 200);
            delay(200);
            setLEDColor(false, false, false);
            delay(200);
        }
        
        delay(2000); // Pause between SOS sequences
    }
    
    // CRITICAL: Halt system to prevent unsafe operation
    Serial.println("💀 SYSTEM HALTED - Hardware reset required");
    
    // Return the failure result for API consistency (though unreachable)
    return result;
    
    while (true) {
        // Infinite loop - robot cannot continue with failed calibration
        setLEDColor(true, false, false); // Keep red LED on
        delay(1000);
        setLEDColor(false, false, false);
        delay(1000);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// EEPROM MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

bool checkCalibrationStatus() {
    EEPROM.begin(EEPROM_SIZE);
    
    // Read the entire calibration data structure
    CalibrationData tempData;
    EEPROM.get(EEPROM_CALIB_DATA_ADDR, tempData);
    
    Serial.printf("🔍 Checking calibration status...\n");
    Serial.printf("   Magic number: 0x%02X (expected: 0x%02X)\n", tempData.magic, CALIBRATION_MAGIC);
    Serial.printf("   Version: %d (expected: %d)\n", tempData.version, CALIBRATION_VERSION);
    
    if (tempData.magic == CALIBRATION_MAGIC && tempData.version == CALIBRATION_VERSION) {
        // Verify data integrity with CRC16
        uint16_t calculatedCRC = calculateCRC16((uint8_t*)&tempData, sizeof(CalibrationData) - sizeof(tempData.checksum));
        if (calculatedCRC == tempData.checksum) {
            Serial.println("✅ Robot is already calibrated!");
            return true;
        } else {
            Serial.println("❌ Calibration data corrupted (CRC mismatch)");
            return false;
        }
    } else {
        Serial.println("❌ Robot needs calibration");
        return false;
    }
}

CalibrationResult loadCalibrationData() {
    if (!checkCalibrationStatus()) {
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    Serial.println("📖 Loading calibration data from EEPROM...");
    
    // Read the entire calibration data structure in one operation
    EEPROM.get(EEPROM_CALIB_DATA_ADDR, calibData);
    
    // Validate the data integrity first
    if (!verifyDataIntegrity(&calibData)) {
        Serial.println("❌ Calibration data checksum verification failed!");
        return CALIB_ERR_CHECKSUM_FAILED;
    }
    
    // Validate the data ranges
    CalibrationResult validation = validateCalibrationData();
    if (validation == CALIB_SUCCESS) {
        isCalibrated = true;
        printCalibrationData();
        Serial.println("✅ Calibration data loaded successfully!");
        return CALIB_SUCCESS;
    } else {
        Serial.printf("❌ Calibration data validation failed: %s\n", 
                     getCalibrationErrorString(validation));
        return validation;
    }
}

CalibrationResult saveCalibrationData() {
    Serial.println("💾 Saving calibration data to EEPROM...");
    
    // Update checksum before saving
    CalibrationResult checksumResult = updateChecksum(&calibData);
    if (checksumResult != CALIB_SUCCESS) {
        return checksumResult;
    }
    
    // Write the entire calibration data structure in one operation
    EEPROM.put(EEPROM_CALIB_DATA_ADDR, calibData);
    
    // Commit to EEPROM
    if (!EEPROM.commit()) {
        Serial.println("❌ Failed to commit data to EEPROM!");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    // Verify the save by reading back and checking
    CalibrationData verifyData;
    EEPROM.get(EEPROM_CALIB_DATA_ADDR, verifyData);
    
    if (!verifyDataIntegrity(&verifyData)) {
        Serial.println("❌ EEPROM verification failed after save!");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    Serial.println("✅ Calibration data saved and verified successfully!");
    printCalibrationData();
    return CALIB_SUCCESS;
}

bool shouldForceRecalibration() {
    // Check if the boot button (GPIO0) is held during startup
    pinMode(FORCE_RECALIBRATION_PIN, INPUT_PULLUP);
    bool buttonPressed = (digitalRead(FORCE_RECALIBRATION_PIN) == LOW);
    
    if (buttonPressed) {
        Serial.println("🔄 FORCE RECALIBRATION detected (BOOT button held)");
        Serial.println("   Erasing existing calibration data...");
        eraseCalibrationData();
        return true;
    }
    
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// MASTER CALIBRATION SEQUENCE
// ═══════════════════════════════════════════════════════════════════════════

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
    calibData.checksum = calculateCRC16((uint8_t*)&calibData, sizeof(CalibrationData) - sizeof(calibData.checksum));
    EEPROM.put(EEPROM_CALIB_DATA_ADDR, calibData);
    EEPROM.commit();
    
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
    
    Serial.println("\n🔄 Rebooting in 3 seconds for normal operation...");
    delay(3000);
    ESP.restart();
    
    return CALIB_SUCCESS;  // This line will never be reached due to restart
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 1: DIRECTIONAL MAPPING (LEFT/RIGHT)
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateDirectionalMapping() {
    Serial.println("\n🧭 PHASE 1: Directional Mapping (Left/Right)");
    Serial.println("─────────────────────────────────────────────");
    Serial.println("Goal: Determine which motor commands create left/right turns");
    
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
    
    // Execute test command for LONGER duration (1500ms) at SLOWER speed
    Serial.println("   🔄 Executing turn test...");
    executeMotorCommand(false, true, true, false, 80); // M1-REV, M2-FWD at very slow speed
    delay(1500); // Increased from 500ms to 1500ms
    allStop();
    delay(1000); // Longer settling time
    
    // Check the result
    float newHeading = getStableMPUHeading();
    float headingChange = newHeading - baseHeading;
    
    // Normalize heading change to -180 to +180
    while (headingChange > 180) headingChange -= 360;
    while (headingChange < -180) headingChange += 360;
    
    Serial.printf("📊 Heading change: %.2f° (negative = left, positive = right)\n", headingChange);
    
    // Determine if hypothesis was correct (reduced threshold from 5.0° to 2.0°)
    if (headingChange < -2.0) {
        // Hypothesis CORRECT: negative change means left turn
        Serial.println("✅ Hypothesis CORRECT: M1-REV + M2-FWD = LEFT turn");
        calibData.motorDirs.leftFwd_M1Fwd = false;
        calibData.motorDirs.leftFwd_M1Rev = true;
        calibData.motorDirs.leftFwd_M2Fwd = true;
        calibData.motorDirs.leftFwd_M2Rev = false;
    } else if (headingChange > 2.0) {
        // Hypothesis WRONG: positive change means right turn
        Serial.println("❌ Hypothesis WRONG: M1-REV + M2-FWD = RIGHT turn");
        Serial.println("✅ Corrected: M1-FWD + M2-REV = LEFT turn");
        calibData.motorDirs.leftFwd_M1Fwd = true;
        calibData.motorDirs.leftFwd_M1Rev = false;
        calibData.motorDirs.leftFwd_M2Fwd = false;
        calibData.motorDirs.leftFwd_M2Rev = true;
    } else {
        Serial.printf("❌ ERROR: Insufficient heading change detected (%.2f°)\n", headingChange);
        Serial.println("   Possible causes:");
        Serial.println("   • Robot wheels not making contact with ground");
        Serial.println("   • Motor connections loose or incorrect");
        Serial.println("   • Robot is stuck or constrained");
        Serial.println("   • Insufficient battery power");
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
    
    // Wait for stable conditions
    if (!waitForStableConditions()) {
        return CALIB_ERR_TIMEOUT;
    }
    
    // Set baseline heading
    float startHeading = getStableMPUHeading();
    Serial.printf("📍 Starting heading: %.2f°\n", startHeading);
    
    // Reset encoders
    resetEncoders();
    delay(100);
    
    // Execute calibrated left turn command
    Serial.println("🔄 Executing LEFT turn until MPU reads -90°...");
    
    bool m1Fwd = calibData.motorDirs.leftFwd_M1Fwd;
    bool m1Rev = calibData.motorDirs.leftFwd_M1Rev;
    bool m2Fwd = calibData.motorDirs.leftFwd_M2Fwd;
    bool m2Rev = calibData.motorDirs.leftFwd_M2Rev;
    
    executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, TURN_SPEED);
    
    // Monitor heading until we reach -90 degrees
    float currentHeading;
    unsigned long turnStartTime = millis();
    
    do {
        currentHeading = getStableMPUHeading();
        float headingChange = currentHeading - startHeading;
        
        // Normalize heading change
        while (headingChange > 180) headingChange -= 360;
        while (headingChange < -180) headingChange += 360;
        
        // Safety timeout
        if (millis() - turnStartTime > 10000) {
            Serial.println("❌ ERROR: Turn timeout exceeded");
            allStop();
            return CALIB_ERR_TIMEOUT;
        }
        
        delay(10); // Small delay for stability
        
    } while (currentHeading - startHeading > -85.0); // Stop when we reach approximately -90°
    
    // Stop motors immediately
    allStop();
    delay(100);
    
    // Read final encoder values
    long leftTicks = abs(getLeftEncoderCount());
    long rightTicks = abs(getRightEncoderCount());
    long avgTicks = (leftTicks + rightTicks) / 2;
    
    // Verify the turn
    float finalHeading = getStableMPUHeading();
    float actualTurn = finalHeading - startHeading;
    while (actualTurn > 180) actualTurn -= 360;
    while (actualTurn < -180) actualTurn += 360;
    
    Serial.printf("📊 Turn Results:\n");
    Serial.printf("   Left encoder: %ld ticks\n", leftTicks);
    Serial.printf("   Right encoder: %ld ticks\n", rightTicks);
    Serial.printf("   Average: %ld ticks\n", avgTicks);
    Serial.printf("   Actual turn: %.2f°\n", actualTurn);
    
    // Validate the results
    if (abs(actualTurn + 90.0) > 10.0) {
        Serial.println("❌ ERROR: Turn accuracy outside acceptable range");
        return CALIB_ERR_UNSTABLE;
    }
    
    if (avgTicks < 100 || avgTicks > 10000) {
        Serial.println("❌ ERROR: Encoder tick count outside reasonable range");
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    // Store the calibration value
    calibData.ticksPer90Degrees = avgTicks;
    
    Serial.printf("✅ Phase 2 complete: 90° turn = %.0f ticks\n", calibData.ticksPer90Degrees);
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 3: FORWARD/BACKWARD DETECTION & SANITY CHECK
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateForwardBackward() {
    Serial.println("\n⬆️ PHASE 3: Forward/Backward Detection");
    Serial.println("────────────────────────────────────────");
    Serial.println("Goal: Determine forward/backward commands and verify sensor orientation");
    
    // Step 1: Find a target using ToF sensor
    Serial.println("🔍 Step 1: Scanning for ToF target...");
    
    float bestDistance = 0;
    int scanAttempts = 0;
    
    // Slowly turn right while scanning for a stable target
    while (bestDistance < 200 && scanAttempts < 8) { // Look for target at least 20cm away
        resetEncoders();
        
        // Turn right a bit
        executeMotorCommand(!calibData.motorDirs.leftFwd_M1Fwd, !calibData.motorDirs.leftFwd_M1Rev,
                           !calibData.motorDirs.leftFwd_M2Fwd, !calibData.motorDirs.leftFwd_M2Rev, 
                           SLOW_SPEED);
        delay(200);
        allStop();
        delay(500);
        
        // Check ToF reading
        bestDistance = getStableToFReading();
        Serial.printf("   Scan %d: Distance = %.0f mm\n", scanAttempts + 1, bestDistance);
        scanAttempts++;
    }
    
    if (bestDistance < 200 || bestDistance > 2000) {
        Serial.printf("❌ ERROR: No suitable target found (distance: %.0f mm)\n", bestDistance);
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    Serial.printf("✅ Target found at %.0f mm\n", bestDistance);
    
    // Step 2: Test forward movement hypothesis
    Serial.println("🔬 Step 2: Testing forward movement hypothesis...");
    Serial.println("   Hypothesis: M1-FWD + M2-FWD = Move FORWARD");
    
    float initialDistance = bestDistance;
    resetEncoders();
    delay(100);
    
    // Execute test move: 500 encoder ticks forward
    executeMotorCommand(true, false, true, false, TEST_SPEED); // M1-FWD + M2-FWD
    
    // Wait until we've moved 500 ticks
    while (getAverageEncoderCount() < 500) {
        delay(10);
    }
    
    allStop();
    delay(500);
    
    // Check results
    float finalDistance = getStableToFReading();
    float distanceChange = finalDistance - initialDistance;
    
    Serial.printf("📊 Movement Test Results:\n");
    Serial.printf("   Initial ToF distance: %.0f mm\n", initialDistance);
    Serial.printf("   Final ToF distance: %.0f mm\n", finalDistance);
    Serial.printf("   Distance change: %.0f mm\n", distanceChange);
    Serial.printf("   Encoder ticks: %ld\n", getAverageEncoderCount());
    
    // Analyze results
    if (distanceChange < -20) {
        // Distance decreased = moved closer = FORWARD is correct
        Serial.println("✅ Hypothesis CORRECT: M1-FWD + M2-FWD = FORWARD");
        calibData.motorDirs.fwdMove_M1Fwd = true;
        calibData.motorDirs.fwdMove_M1Rev = false;
        calibData.motorDirs.fwdMove_M2Fwd = true;
        calibData.motorDirs.fwdMove_M2Rev = false;
    } else if (distanceChange > 20) {
        // Distance increased = moved away = BACKWARD is correct
        Serial.println("❌ Hypothesis WRONG: M1-FWD + M2-FWD = BACKWARD");
        Serial.println("✅ Corrected: M1-REV + M2-REV = FORWARD");
        calibData.motorDirs.fwdMove_M1Fwd = false;
        calibData.motorDirs.fwdMove_M1Rev = true;
        calibData.motorDirs.fwdMove_M2Fwd = false;
        calibData.motorDirs.fwdMove_M2Rev = true;
    } else {
        Serial.println("❌ ERROR: No significant distance change detected");
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    // Initialize MPU flags (assume normal orientation for now)
    calibData.mpuFlags.xAxisInverted = false;
    calibData.mpuFlags.yAxisInverted = false;
    calibData.mpuFlags.zAxisInverted = false;
    calibData.mpuFlags.gyroXInverted = false;
    calibData.mpuFlags.gyroYInverted = false;
    calibData.mpuFlags.gyroZInverted = false;
    
    Serial.println("✅ Phase 3 complete: Forward/Backward motor mapping determined");
    return CALIB_SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// PHASE 4: DISTANCE & TOF CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════

CalibrationResult calibrateDistanceAndToF() {
    Serial.println("\n📏 PHASE 4: Distance & ToF Calibration");
    Serial.println("─────────────────────────────────────────");
    Serial.println("Goal: Calibrate encoder ticks to real distance and find ToF offset");
    
    // Get stable baseline distance
    float initialDistance = getStableToFReading();
    if (initialDistance < 100 || initialDistance > 1500) {
        Serial.printf("❌ ERROR: Initial distance unsuitable: %.0f mm\n", initialDistance);
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    Serial.printf("📍 Baseline distance: %.0f mm\n", initialDistance);
    
    // Reset encoders
    resetEncoders();
    delay(100);
    
    // Execute precise forward movement: 2000 ticks
    Serial.println("🔄 Moving forward 2000 encoder ticks...");
    
    bool m1Fwd = calibData.motorDirs.fwdMove_M1Fwd;
    bool m1Rev = calibData.motorDirs.fwdMove_M1Rev;
    bool m2Fwd = calibData.motorDirs.fwdMove_M2Fwd;
    bool m2Rev = calibData.motorDirs.fwdMove_M2Rev;
    
    executeMotorCommand(m1Fwd, m1Rev, m2Fwd, m2Rev, TEST_SPEED);
    
    // Wait until exactly 2000 ticks
    while (getAverageEncoderCount() < 2000) {
        delay(10);
    }
    
    allStop();
    delay(500);
    
    // Measure final distance
    float finalDistance = getStableToFReading();
    float distanceMoved = initialDistance - finalDistance; // Should be positive for forward movement
    
    Serial.printf("📊 Distance Calibration Results:\n");
    Serial.printf("   Initial distance: %.0f mm\n", initialDistance);
    Serial.printf("   Final distance: %.0f mm\n", finalDistance);
    Serial.printf("   Distance moved (ToF): %.0f mm\n", distanceMoved);
    Serial.printf("   Encoder ticks: %ld\n", getAverageEncoderCount());
    
    // Validate movement
    if (distanceMoved < 50 || distanceMoved > 500) {
        Serial.printf("❌ ERROR: Distance moved outside reasonable range: %.0f mm\n", distanceMoved);
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    // Calculate calibration factor
    calibData.ticksPerMillimeter = 2000.0 / distanceMoved;
    
    // ToF offset is set to 0 as it cannot be reliably determined this way.
    calibData.tofOffsetMM = 0.0;
    
    Serial.printf("📊 Calibration Values Calculated:\n");
    Serial.printf("   Ticks per millimeter: %.2f\n", calibData.ticksPerMillimeter);
    
    // Validate calibration values
    if (calibData.ticksPerMillimeter < 5.0 || calibData.ticksPerMillimeter > 50.0) {
        Serial.println("❌ ERROR: Ticks per millimeter outside reasonable range");
        return CALIB_ERR_SENSOR_INVALID;
    }
    
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
    if (calibData.minMotorSpeedPWM < 10 || calibData.minMotorSpeedPWM > 200) {
        return CALIB_ERR_SENSOR_INVALID;
    }
    
    return CALIB_SUCCESS;
}

void printCalibrationData() {
    Serial.println("\n📊 CALIBRATION DATA SUMMARY");
    Serial.println("════════════════════════════");
    Serial.printf("Magic: 0x%02X, Version: %d\n", calibData.magic, calibData.version);
    Serial.printf("Ticks per 90°: %.0f\n", calibData.ticksPer90Degrees);
    Serial.printf("Ticks per mm: %.2f\n", calibData.ticksPerMillimeter);
    Serial.printf("Min motor PWM: %d\n", calibData.minMotorSpeedPWM);
    Serial.printf("Checksum: 0x%04X\n", calibData.checksum);
    Serial.printf("Motor directions:\n");
    Serial.printf("  Left: M1=%s, M2=%s\n", 
                  calibData.motorDirs.leftFwd_M1Fwd ? "FWD" : "REV",
                  calibData.motorDirs.leftFwd_M2Fwd ? "FWD" : "REV");
    Serial.printf("  Forward: M1=%s, M2=%s\n",
                  calibData.motorDirs.fwdMove_M1Fwd ? "FWD" : "REV",
                  calibData.motorDirs.fwdMove_M2Fwd ? "FWD" : "REV");
    Serial.println("════════════════════════════\n");
}

CalibrationResult eraseCalibrationData() {
    EEPROM.begin(EEPROM_SIZE);
    
    // Write zeros to all calibration addresses
    for (int addr = 0; addr < sizeof(CalibrationData); addr++) {
        EEPROM.write(addr, 0x00);
    }
    
    if (!EEPROM.commit()) {
        Serial.println("❌ Failed to erase EEPROM data!");
        return CALIB_ERR_MEMORY_CORRUPTION;
    }
    
    Serial.println("🗑️ Calibration data erased from EEPROM");
    isCalibrated = false;
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