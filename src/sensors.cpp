#include "sensors.h"
#include "indicators.h" // For visual feedback during scan
#include "calibration.h"

// -----------------------------------------------------------------------------
// This file is the "Layer 1" implementation for the robot's sensors.
// 1. autoDetectSensors(): Scans hardware to see what's connected.
// 2. initializeSensors(): Initializes only the hardware that was found.
// 3. updateAllSensors(): Polls all active sensors and fills the global `sensors` struct.
// -----------------------------------------------------------------------------

// Global sensor objects
VL53L0X tofSensor;
MPU6050 mpu(Wire);

// Externs (data structs populated by this file)
extern SystemStatus sysStatus;
extern SensorData sensors;
extern CalibrationData calibData;
extern bool isCalibrated;

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR AUTO-DISCOVERY (Called by main.cpp during setup)
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Scans the I2C bus to determine connected sensors.
 * Updates global sysStatus flags (tofAvailable, mpuAvailable) automatically.
 */
void autoDetectSensors() {
    Serial.println("🔎 AUTO-DISCOVERY: Scanning I2C Bus...");
    Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK);
    
    byte count = 0;
    sysStatus.sensorsActive = 0;

    // Scan all possible I2C addresses
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("   ✓ Found I2C Device at 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);

            // Identify known devices by their address
            switch (address) {
                case 0x29: // VL53L0X
                    Serial.println(" -> ToF Sensor (VL53L0X)");
                    sysStatus.tofAvailable = true;
                    break;
                case 0x68: // MPU6050
                    Serial.println(" -> IMU (MPU6050)");
                    sysStatus.mpuAvailable = true;
                    break;
                case 0x3C: 
                    Serial.println(" -> OLED Display");
                    // sysStatus.oledAvailable = true; // Future
                    break;
                case 0x40: 
                    Serial.println(" -> Servo Driver (PCA9685)");
                    // sysStatus.servoAvailable = true; // Future
                    break;
                default:
                    Serial.println(" -> Unknown Device");
                    break;
            }
            count++;
        }
    }
    
    if (count == 0) {
        Serial.println("⚠️ No I2C devices found.");
    } else {
        Serial.printf("   📊 Scan Complete. Found %d I2C devices.\n", count);
    }
    
    // Setup digital pins
    pinMode(SOUND_SENSOR_PIN, INPUT);
    pinMode(EDGE_SENSOR_PIN, INPUT_PULLUP);
    pinMode(PIR_SENSOR_PIN, INPUT);
    
    // Setup Encoder Pins
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    // TODO: Attach encoder interrupts here
    
    Serial.println("✅ Auto-Discovery Complete.");
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR INITIALIZATION (Called by main.cpp during setup)
// ═══════════════════════════════════════════════════════════════════════════

void initializeSensors() {
    // 1. Run Auto-Discovery first
    autoDetectSensors();

    // 2. Initialize *only* the sensors that were found
    if (sysStatus.tofAvailable) {
        Serial.print("   🔧 Init ToF... ");
        tofSensor.setTimeout(500);
        if (tofSensor.init()) {
            tofSensor.startContinuous();
            // Check if the sensor is actually returning valid data
            tofSensor.readRangeContinuousMillimeters(); 
            if(tofSensor.timeoutOccurred()) {
                 Serial.println("⚠️ Timeout (Warning)");
            } else {
                 Serial.println("✅ Ready");
                 sysStatus.sensorsActive++;
            }
        } else {
            Serial.println("❌ Init Failed");
            sysStatus.tofAvailable = false;
        }
    }

    if (sysStatus.mpuAvailable) {
        Serial.print("   🔧 Init IMU... ");
        if (mpu.begin() == 0) {
            // Apply calibration data if it's loaded and valid
            if (isCalibrated && calibData.valid) {
                Serial.println("✅ Ready (Applying Saved Calibration)");
                mpu.setAccOffsets(calibData.mpuOffsets.accelX, calibData.mpuOffsets.accelY, calibData.mpuOffsets.accelZ);
                mpu.setGyroOffsets(calibData.mpuOffsets.gyroX, calibData.mpuOffsets.gyroY, calibData.mpuOffsets.gyroZ);
            } else {
                 Serial.println("⚠️ Ready (Uncalibrated, using defaults)");
                 // Don't run full calibration here, main.cpp will handle it.
            }
            sysStatus.sensorsActive++;
        } else {
            Serial.println("❌ Init Failed");
            sysStatus.mpuAvailable = false;
        }
    }
    
    Serial.printf("   📊 %d active sensors initialized.\n", sysStatus.sensorsActive);
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR POLLING (Called by main.cpp in loop)
// ═══════════════════════════════════════════════════════════════════════════

void updateAllSensors() {
    // Only read from sensors that were successfully initialized
    if (sysStatus.tofAvailable) {
        sensors.distance = tofSensor.readRangeContinuousMillimeters();
        if (tofSensor.timeoutOccurred()) {
            sensors.distance = 8190; // Max range on timeout
        }
    }

    if (sysStatus.mpuAvailable) {
        mpu.update();
        sensors.tiltX = mpu.getAngleX();
        sensors.tiltY = mpu.getAngleY();
        sensors.headingAngle = mpu.getAngleZ();
    }

    // Encoder counts are assumed to be updated by interrupts
    // So we just read the latest values into the struct
    // sensors.leftEncoderCount = getLeftEncoderCount();  // Assumes ISRs update globals
    // sensors.rightEncoderCount = getRightEncoderCount(); // Assumes ISRs update globals

    // Read Digital Sensors
    sensors.soundDetected = (digitalRead(SOUND_SENSOR_PIN) == HIGH);
    sensors.edgeDetected = (digitalRead(EDGE_SENSOR_PIN) == HIGH);
    sensors.motionDetected = (digitalRead(PIR_SENSOR_PIN) == HIGH);
}