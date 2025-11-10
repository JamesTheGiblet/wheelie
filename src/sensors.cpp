#include "sensors.h"
#include "motors.h"
#include "indicators.h"
#include "calibration.h"  // For MPU calibration data

// ═══════════════════════════════════════════════════════════════════════════
// SENSORS IMPLEMENTATION - Sensor management and data processing
// ═══════════════════════════════════════════════════════════════════════════

// Global sensor objects
VL53L0X tofSensor;
MPU6050 mpu(Wire);

// Private timing variables (encapsulated within this module)
static unsigned long lastToFRead = 0;
static unsigned long lastMPURead = 0;
static unsigned long lastSoundTrigger = 0;
static unsigned long lastMotionTrigger = 0;
static unsigned long lastEdgeTrigger = 0;

// Private debouncing variables (encapsulated within this module)
static int lastSoundState = LOW;
static int lastMotionState = LOW;
static int lastEdgeState = LOW;
static unsigned long lastSoundDebounce = 0;
static unsigned long lastMotionDebounce = 0;
static unsigned long lastEdgeDebounce = 0;

// External global data interfaces
extern SystemStatus sysStatus;
extern SensorData sensors;

void setupI2C() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK);
  Serial.println("   ✓ I2C bus initialized (400kHz)");
}

void initializeSensors() {
  Serial.println("🔧 Initializing sensor array...\n");
  
  // I2C Bus
  setupI2C();
  
  // VL53L0X ToF Sensor
  Serial.print("   📏 VL53L0X ToF sensor... ");
  tofSensor.setTimeout(TOF_TIMEOUT);
  if (tofSensor.init()) {
    Serial.println("✅ ONLINE");
    
    // VL53L0X calibration and setup
    Serial.println("      🔧 Configuring VL53L0X parameters...");
    tofSensor.setMeasurementTimingBudget(TOF_TIMING_BUDGET);
    
    // Perform initial readings to stabilize sensor
    Serial.print("      📊 Warming up sensor... ");
    setLEDColor(LEDColors::CYAN);
    for (int i = 0; i < 5; i++) {
      tofSensor.readRangeSingleMillimeters();
      delay(100);
    }
    
    tofSensor.startContinuous();
    
    // Take baseline reading
    delay(200);
    int baselineReading = tofSensor.readRangeContinuousMillimeters();
    Serial.println("Done!");
    Serial.println("      📈 Baseline reading: " + String(baselineReading) + "mm");
    
    if (baselineReading > 50 && baselineReading < 2000) {
      Serial.println("      ✅ Sensor ready for operation");
      setLEDColor(LEDColors::GREEN);
      playTone(1000, 100);
    } else {
      Serial.println("      ⚠️  Check sensor alignment or obstacles");
      setLEDColor(LEDColors::YELLOW);
      playTone(600, 200);
    }
    
    clearLEDs();
    sysStatus.tofAvailable = true;
    sysStatus.sensorsActive++;
  } else {
    Serial.println("❌ OFFLINE");
  }
  
  // MPU6050 IMU
  Serial.print("   🔄 MPU6050 IMU... ");
    if (mpu.begin() == 0) {
        Serial.println("✅ ONLINE");
        extern CalibrationData calibData; // Get the global calib data
        extern bool isCalibrated;

        if (isCalibrated) {
            Serial.println("   🔧 Applying saved MPU offsets from EEPROM...");
            // Apply the 6-axis offsets we saved during calibration
            mpu.setAccOffsets(calibData.mpuOffsets.accelX, calibData.mpuOffsets.accelY, calibData.mpuOffsets.accelZ);
            mpu.setGyroOffsets(calibData.mpuOffsets.gyroX, calibData.mpuOffsets.gyroY, calibData.mpuOffsets.gyroZ);
            
            Serial.println("   ✅ MPU offsets applied.");
        } else {
            Serial.println("   ⚠️ WARNING: MPU is uncalibrated. Run calibration.");
            // We don't run calcOffsets() here, as it's a long, blocking
            // process that belongs in the calibration module.
        }
        
        mpu.update();
        sysStatus.mpuAvailable = true;
        sysStatus.sensorsActive++;
    } else {
        Serial.println("❌ OFFLINE");
        sysStatus.mpuAvailable = false;
    }
  
  // PIR Status
  Serial.print("   👁️  PIR motion sensor... ");
  if (sysStatus.pirAvailable) {
    Serial.println("✅ ONLINE");
    sysStatus.sensorsActive++;
  } else {
    Serial.println("⚠️  DISCONNECTED (will add later)");
  }
  
  // Sensor calibration and baseline establishment
  Serial.println("   🔧 Calibrating analog sensors...");
  setLEDColor(LEDColors::MAGENTA);
  
  // Edge sensor baseline
  Serial.print("      🛡️  Edge sensor baseline... ");
  int edgeSum = 0;
  for (int i = 0; i < 10; i++) {
    edgeSum += digitalRead(EDGE_SENSOR_PIN);
    delay(50);
  }
  int edgeBaseline = edgeSum / 10;
  Serial.println("Done! (baseline: " + String(edgeBaseline) + ")");
  
  // Sound sensor baseline
  Serial.print("      🔊 Sound sensor baseline... ");
  int soundSum = 0;
  for (int i = 0; i < 10; i++) {
    soundSum += digitalRead(SOUND_SENSOR_PIN);
    delay(50);
  }
  int soundBaseline = soundSum / 10;
  Serial.println("Done! (baseline: " + String(soundBaseline) + ")");
  
  clearLEDs();
  
  // Other sensors
  Serial.println("   🛡️  Edge sensor... ✅ ONLINE");
  Serial.println("   🔊 Sound sensor... ✅ ONLINE");
  sysStatus.sensorsActive += 2;
  
  // Calibration complete
  Serial.println();
  Serial.println("🎯 SENSOR CALIBRATION COMPLETE!");
  Serial.println("   📊 All sensors calibrated and ready");
  Serial.println("   🤖 Robot ready for autonomous operation");
  
  // Success indication
  setLEDColor(LEDColors::GREEN);
  playTone(1500, 200);
  delay(200);
  playTone(1800, 200);
  delay(200);
  clearLEDs();
  
  Serial.println();
}

void updateAllSensors() {
  unsigned long currentTime = millis(); // Get time once

  // Update ToF
  if (sysStatus.tofAvailable && currentTime - lastToFRead >= TOF_INTERVAL) {
    sensors.distance = readToFDistance();
    lastToFRead = currentTime;
  }

  // Update Encoders (always, no timing restriction)
  sensors.leftEncoderCount = getLeftEncoderCount();
  sensors.rightEncoderCount = getRightEncoderCount();
  
  // Update MPU
  if (sysStatus.mpuAvailable && currentTime - lastMPURead >= MPU_INTERVAL) {
    readIMUData(); // This function already saves to the 'sensors' struct
    lastMPURead = currentTime;
  }

    // --- ADD THESE ---
    
    // Update Edge Sensor (with debouncing)
    sensors.edgeDetected = readEdgeSensor();
    
    // Update Sound Sensor (with debouncing)
    sensors.soundDetected = readSoundSensor();
    
    // Update Motion Sensor (with debouncing)
    sensors.motionDetected = readMotionSensor();
}

int readToFDistance() {
  int distance = tofSensor.readRangeContinuousMillimeters();
  if (tofSensor.timeoutOccurred() || distance > 2000) {
    distance = 2000;
  }
  return distance;
}

void readIMUData() {
    mpu.update();

    // The MPU library is now pre-calibrated with our saved offsets.
    // The getAngleX/Y functions now return the *true* tilt.
    sensors.tiltX = mpu.getAngleX();
    sensors.tiltY = mpu.getAngleY();
    
    // The Z angle is our absolute heading
    sensors.headingAngle = mpu.getAngleZ();

    // Debug output (optional, but good)
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 3000) {
        Serial.printf("🔍 MPU: TiltX=%.1f°, TiltY=%.1f°, Heading=%.1f°\n",
            sensors.tiltX, sensors.tiltY, sensors.headingAngle);
        lastDebug = millis();
    }
}

bool readEdgeSensor() {
    bool reading = (digitalRead(EDGE_SENSOR_PIN) == HIGH);
    if (reading != lastEdgeState) {
        lastEdgeDebounce = millis();
    }
    lastEdgeState = reading;

    if ((millis() - lastEdgeDebounce) > DEBOUNCE_DELAY) {
        return reading;
    }
    return sensors.edgeDetected; // Return last known good value
}

bool readSoundSensor() {
    bool reading = (digitalRead(SOUND_SENSOR_PIN) == HIGH);
    if (reading != lastSoundState) {
        lastSoundDebounce = millis();
    }
    lastSoundState = reading;

    if ((millis() - lastSoundDebounce) > DEBOUNCE_DELAY) {
        return reading;
    }
    return sensors.soundDetected; // Return last known good value
}

bool readMotionSensor() {
    if (!sysStatus.pirAvailable) return false;

    bool reading = (digitalRead(PIR_SENSOR_PIN) == HIGH);
    if (reading != lastMotionState) {
        lastMotionDebounce = millis();
    }
    lastMotionState = reading;

    if ((millis() - lastMotionDebounce) > DEBOUNCE_DELAY) {
        return reading;
    }
    return sensors.motionDetected; // Return last known good value
}

bool isToFAvailable() {
  return sysStatus.tofAvailable;
}

bool isIMUAvailable() {
  return sysStatus.mpuAvailable;
}

bool isPIRAvailable() {
  return sysStatus.pirAvailable;
}

/*
// ═══════════════════════════════════════════════════════════════════════════
// SAFETY CHECK FUNCTIONS - DISABLED FOR ARCHITECTURAL CLEAN-UP
// ═══════════════════════════════════════════════════════════════════════════
// These functions violated separation of concerns - sensors should only collect data
// Safety logic has been moved to navigation/robot modules

bool checkAndHandleTilt() {
  if (!sysStatus.mpuAvailable) return false;
  
  static int consecutiveTiltAlerts = 0;
  static unsigned long lastTiltTime = 0;
  static bool inTiltRecovery = false;
  
  // Check if either axis exceeds the tolerance
  if (abs(sensors.tiltX) > TILT_TOLERANCE || abs(sensors.tiltY) > TILT_TOLERANCE) {
    
    // First tilt detection - stop and assess
    if (!inTiltRecovery) {
      Serial.println("\n⚠️  � TILT DETECTED - Starting smart recovery");
      Serial.print("   Tilt: X=");
      Serial.print(sensors.tiltX, 1);
      Serial.print("° Y=");
      Serial.print(sensors.tiltY, 1);
      Serial.print("° (limit: ±");
      Serial.print(TILT_TOLERANCE, 0);
      Serial.println("°)");
      
      allStop();
      setLEDColor(LEDColors::YELLOW);
      playTone(1500, 200);
      
      inTiltRecovery = true;
      lastTiltTime = millis();
      consecutiveTiltAlerts++;
    }
    
    // Give robot time to stabilize
    if (millis() - lastTiltTime < 1000) {
      return true; // Still in assessment period
    }
    
    // Attempt intelligent recovery
    Serial.println("   🤖 Attempting tilt recovery...");
    
    if (consecutiveTiltAlerts <= 3) {
      // Try gentle recovery moves
      Serial.print("   📐 Recovery attempt ");
      Serial.print(consecutiveTiltAlerts);
      Serial.println("/3");
      
      if (sensors.tiltX > TILT_TOLERANCE) {
        Serial.println("   ⬅️  Gentle reverse to reduce forward tilt");
        goBackward();
        delay(300);
        allStop();
      } else if (sensors.tiltX < -TILT_TOLERANCE) {
        Serial.println("   ➡️  Gentle forward to reduce backward tilt");
        goForward();
        delay(300);
        allStop();
      }
      
      if (abs(sensors.tiltY) > TILT_TOLERANCE) {
        Serial.println("   🔄 Slight adjustment turn");
        if (sensors.tiltY > 0) {
          turnRight();
        } else {
          turnLeft();
        }
        delay(200);
        allStop();
      }
      
      // Wait and retest
      Serial.println("   ⏳ Waiting for stabilization...");
      delay(500);
      updateAllSensors(); // Get fresh readings
      
      // Check if recovery was successful
      if (abs(sensors.tiltX) <= TILT_TOLERANCE && abs(sensors.tiltY) <= TILT_TOLERANCE) {
        Serial.println("   ✅ Tilt recovery SUCCESSFUL!");
        Serial.print("   📐 New angles: X=");
        Serial.print(sensors.tiltX, 1);
        Serial.print("° Y=");
        Serial.print(sensors.tiltY, 1);
        Serial.println("°");
        
        setLEDColor(LEDColors::GREEN);
        playTone(1200, 100);
        delay(100);
        playTone(1500, 100);
        clearLEDs();
        
        // Reset recovery state
        inTiltRecovery = false;
        consecutiveTiltAlerts = 0;
        
        Serial.println("   🚀 Resuming navigation\n");
        return false; // Allow navigation to continue
      } else {
        Serial.println("   ⚠️  Recovery attempt failed, will retry...");
        lastTiltTime = millis();
        return true; // Stay in recovery mode
      }
      
    } else {
      // Multiple failed attempts - escalate to emergency stop
      Serial.println("\n🚨 CRITICAL TILT - Multiple recovery attempts failed!");
      Serial.print("   Final angles: X=");
      Serial.print(sensors.tiltX, 1);
      Serial.print("° Y=");
      Serial.print(sensors.tiltY, 1);
      Serial.println("°");
      Serial.println("   � EMERGENCY STOP - Manual intervention required");
      
      for (int i = 0; i < 5; i++) {
        setLEDColor(LEDColors::RED);
        playTone(3000, 100);
        delay(100);
        clearLEDs();
        delay(100);
      }
      
      delay(5000); // Longer delay for critical situations
      return true;
    }
    
  } else {
    // Robot is level - reset recovery state if needed
    if (inTiltRecovery) {
      Serial.println("   ✅ Robot naturally stabilized");
      inTiltRecovery = false;
      consecutiveTiltAlerts = 0;
      clearLEDs();
    }
    return false;
  }
  
  return false;
}

bool checkAndHandleEdge() {
  if (millis() - lastEdgeTrigger < EDGE_COOLDOWN) return false;
  
  int reading = digitalRead(EDGE_SENSOR_PIN);
  if (reading != lastEdgeState) lastEdgeDebounce = millis();
  
  if ((millis() - lastEdgeDebounce) > DEBOUNCE_DELAY) {
    if (reading == HIGH && lastEdgeState == LOW) {
      Serial.println("\n⚠️  🚨 EDGE DETECTED! 🚨");
      Serial.println("   🛑 EMERGENCY STOP ACTIVATED");
      
      allStop();
      
      for (int i = 0; i < 5; i++) {
        setLEDColor(LEDColors::RED);
        playTone(2500, 80);
        delay(80);
        clearLEDs();
        delay(80);
      }
      
      Serial.println("   ⬅️  Executing safety reversal...");
      setLEDColor(LEDColors::YELLOW);
      goBackward();
      delay(800);
      allStop();
      clearLEDs();
      
      Serial.println("   ✅ Safe from edge\n");
      lastEdgeTrigger = millis();
      lastEdgeState = reading;
      return true;
    }
  }
  
  lastEdgeState = reading;
  return false;
}

bool checkAndHandleObstacle() {
  if (!sysStatus.tofAvailable) return false;
  if (sensors.distance >= OBSTACLE_DISTANCE || sensors.distance <= 30) return false;
  
  Serial.println("\n🚧 OBSTACLE DETECTED!");
  Serial.print("   Distance: ");
  Serial.print(sensors.distance);
  Serial.println("mm");
  Serial.println("   🤖 Initiating avoidance maneuver...");
  
  allStop();
  
  for (int i = 0; i < 3; i++) {
    setLEDColor(LEDColors::RED);
    playTone(2000, 100);
    delay(100);
    clearLEDs();
    delay(100);
  }
  
  Serial.println("   ⬅️  Reversing...");
  setLEDColor(LEDColors::YELLOW);
  goBackward();
  delay(500);
  allStop();
  delay(300);
  
  if (random(0, 2) == 0) {
    Serial.println("   ↪️  Turning left...");
    turnLeft();
  } else {
    Serial.println("   ↩️  Turning right...");
    turnRight();
  }
  delay(800);
  allStop();
  clearLEDs();
  
  Serial.println("   ✅ Obstacle avoided\n");
  delay(1000);
  return true;
}

void checkAndHandleSound() {
  if (millis() - lastSoundTrigger < SOUND_COOLDOWN) return;
  
  int reading = digitalRead(SOUND_SENSOR_PIN);
  if (reading != lastSoundState) lastSoundDebounce = millis();
  
  if ((millis() - lastSoundDebounce) > DEBOUNCE_DELAY) {
    if (reading == HIGH && lastSoundState == LOW) {
      Serial.println("\n🔊 Sound detected - Acknowledging...");
      allStop();
      
      for (int i = 0; i < 2; i++) {
        setLEDColor(LEDColors::GREEN);
        playTone(1500, 80);
        delay(80);
        clearLEDs();
        delay(80);
      }
      
      Serial.println("   ✓ Acknowledged\n");
      lastSoundTrigger = millis();
      lastSoundState = reading;
      delay(500);
    }
  }
  
  lastSoundState = reading;
}

void checkAndHandleMotion() {
  if (!sysStatus.pirAvailable) return;
  if (millis() - lastMotionTrigger < MOTION_COOLDOWN) return;
  
  int reading = digitalRead(PIR_SENSOR_PIN);
  if (reading != lastMotionState) lastMotionDebounce = millis();
  
  if ((millis() - lastMotionDebounce) > DEBOUNCE_DELAY) {
    if (reading == HIGH && lastMotionState == LOW) {
      Serial.println("\n👁️  Motion detected - Acknowledging...");
      allStop();
      
      for (int i = 0; i < 3; i++) {
        setLEDColor(LEDColors::RED);
        playTone(1200, 100);
        delay(100);
        setLEDColor(LEDColors::BLUE);
        delay(100);
      }
      
      clearLEDs();
      Serial.println("   ✓ Acknowledged\n");
      lastMotionTrigger = millis();
      lastMotionState = reading;
      delay(500);
    }
  }
  
  lastMotionState = reading;
}

*/ // END OF COMMENTED OUT SAFETY FUNCTIONS