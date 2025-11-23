#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include <VL53L0X.h>

VL53L0X tof;

// ============================================================================
// CONFIGURATION
// ============================================================================

#define SERIAL_BAUD 115200
#define I2C_CLOCK 100000  // 100kHz for reliability

// Calibration tracking
struct CalibrationData {
  int16_t offsetMm = 0;
  float crossTalkMcps = 0;
  bool isOffsetCalibrated = false;
  bool isCrossTalkCalibrated = false;
  unsigned long timestamp = 0;
  
  // Calibration quality metrics
  float stdDevMm = 0;
  int sampleCount = 0;
  float minReading = 0;
  float maxReading = 0;
  float avgReading = 0;
};

CalibrationData calibration;

// Performance tracking
struct PerformanceMetrics {
  unsigned long totalMeasurements = 0;
  unsigned long successfulMeasurements = 0;
  unsigned long timeouts = 0;
  unsigned long outOfRange = 0;
  unsigned long startTime = 0;
  
  float minDistance = 9999;
  float maxDistance = 0;
  float totalDistance = 0;
  
  // Timing statistics
  unsigned long minMeasurementTime = 9999;
  unsigned long maxMeasurementTime = 0;
  unsigned long totalMeasurementTime = 0;
};

PerformanceMetrics metrics;

// Test modes
enum TestMode {
  MODE_LIVE_DISTANCE,
  MODE_HISTOGRAM,
  MODE_CONTINUOUS_FAST,
  MODE_HIGH_ACCURACY,
  MODE_RANGE_PROFILE
};

TestMode currentMode = MODE_LIVE_DISTANCE;

// Measurement timing budgets (in microseconds)
const uint32_t TIMING_BUDGET_FAST = 20000;      // 20ms - fast but less accurate
const uint32_t TIMING_BUDGET_BALANCED = 33000;  // 33ms - balanced (default)
const uint32_t TIMING_BUDGET_ACCURATE = 200000; // 200ms - slow but accurate

// Histogram for distance distribution
const int HISTOGRAM_BINS = 20;
const int HISTOGRAM_RANGE_MM = 2000; // 0-2000mm
int histogram[HISTOGRAM_BINS] = {0};

// ============================================================================
// SETUP
// ============================================================================

// Forward declaration for printMainMenu
void printMainMenu();

void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Wait for serial with timeout
  unsigned long startTime = millis();
  while (!Serial && millis() - startTime < 3000) {
    delay(10);
  }
  
  printWelcomeBanner();
  
  // Initialize I2C
  initializeI2C();
  
  // Scan I2C bus
  scanI2CBus();
  
  // Initialize VL53L0X
  initializeVL53L0X();
  
  // Configure VL53L0X
  configureVL53L0X();
  
  // Perform initial offset calibration
  Serial.println("\n⚠ Note: Offset calibration recommended for best accuracy");
  Serial.println("  Press 'c' in menu to perform calibration");
  
  // Initialize metrics
  metrics.startTime = millis();
  
  // Show menu
  printMainMenu();
  
  Serial.println("\n📊 Starting live distance monitoring...");
  Serial.println("Press 'h' for command menu\n");
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

String repeatChar(char c, int count) {
  String result = "";
  for (int i = 0; i < count; i++) {
    result += c;
  }
  return result;
}

void printWelcomeBanner() {
  Serial.println("\n" + repeatChar('═', 70));
  Serial.println("      VL53L0X COMPREHENSIVE TEST & CALIBRATION SYSTEM");
  Serial.println(repeatChar('═', 70));
  Serial.println("  Purpose: Complete testing suite for VL53L0X ToF sensor");
  Serial.println("  Features: Calibration, accuracy testing, performance analysis");
  Serial.println("  Project: WHEELIE Robot Obstacle Detection System");
  Serial.println(repeatChar('═', 70));
}

void initializeI2C() {
  Serial.println("\n[STEP 1] Initializing I2C Bus");
  Serial.println(repeatChar('─', 70));
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK);
  
  Serial.print("  SDA Pin: GPIO ");
  Serial.println(I2C_SDA);
  Serial.print("  SCL Pin: GPIO ");
  Serial.println(I2C_SCL);
  Serial.print("  Clock Speed: ");
  Serial.print(I2C_CLOCK / 1000);
  Serial.println(" kHz");
  
  delay(100); // Bus stabilization
  
  Serial.println("  ✓ I2C bus initialized successfully");
}

void scanI2CBus() {
  Serial.println("\n[STEP 2] Scanning I2C Bus");
  Serial.println(repeatChar('─', 70));
  
  bool vl53Found = false;
  int deviceCount = 0;
  
  Serial.println("  Scanning addresses 0x08 to 0x77...\n");
  
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  ✓ Device found at 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.print(addr, HEX);
      
      if (addr == 0x29) {
        Serial.println(" ← VL53L0X ToF Sensor");
        vl53Found = true;
      } else {
        Serial.println();
      }
      
      deviceCount++;
    }
  }
  
  Serial.println();
  Serial.print("  Total devices found: ");
  Serial.println(deviceCount);
  
  if (!vl53Found) {
    Serial.println("\n  ✗ ERROR: VL53L0X not detected!");
    Serial.println("  → Check power (VCC=2.6-3.5V, GND)");
    Serial.println("  → Check I2C connections (SDA, SCL)");
    Serial.println("  → Verify XSHUT pin is HIGH or floating");
    Serial.println("  → Check GPIO0/GPIO1 not interfering");
    while (1) delay(1000);
  }
  
  Serial.println("  ✓ VL53L0X detected successfully");
}

void initializeVL53L0X() {
  Serial.println("\n[STEP 3] Initializing VL53L0X");
  Serial.println(String('─') * 70);
  
  tof.setTimeout(500);
  
  if (!tof.init()) {
    Serial.println("  ✗ ERROR: VL53L0X initialization failed!");
    Serial.println("\n  Possible causes:");
    Serial.println("  → Device not responding properly");
    Serial.println("  → Incorrect I2C communication");
    Serial.println("  → Device in unexpected state");
    Serial.println("  → Power supply issues");
    while (1) delay(1000);
  }
  
  Serial.println("  ✓ VL53L0X initialized successfully");
  
  // Read device information
  Serial.println("\n  Device Information:");
  Serial.println("  " + String('─') * 66);
  
  // Read model ID
  uint8_t modelId = readRegister(0xC0);
  Serial.print("    Model ID: 0x");
  if (modelId < 0x10) Serial.print("0");
  Serial.print(modelId, HEX);
  if (modelId == 0xEE) {
    Serial.println(" ✓ (VL53L0X verified)");
  } else {
    Serial.println(" ⚠ (Unexpected value)");
  }
  
  // Read module type
  uint8_t moduleType = readRegister(0xC1);
  Serial.print("    Module Type: 0x");
  if (moduleType < 0x10) Serial.print("0");
  Serial.println(moduleType, HEX);
  
  // Read revision ID
  uint8_t revisionId = readRegister(0xC2);
  Serial.print("    Revision ID: 0x");
  if (revisionId < 0x10) Serial.print("0");
  Serial.println(revisionId, HEX);
  
  Serial.println("\n  Sensor Specifications:");
  Serial.println("  " + String('─') * 66);
  Serial.println("    Technology: Time-of-Flight (ToF) laser ranging");
  Serial.println("    Range: 30mm - 2000mm (3cm - 2m)");
  Serial.println("    Field of View: 25° cone");
  Serial.println("    Wavelength: 940nm (infrared)");
  Serial.println("    Accuracy: ±3% (typ), ±5% (max)");
}

void configureVL53L0X() {
  Serial.println("\n[STEP 4] Configuring VL53L0X");
  Serial.println(String('─') * 70);
  
  // Set timing budget (default: 33ms for balanced performance)
  tof.setMeasurementTimingBudget(TIMING_BUDGET_BALANCED);
  
  Serial.println("  Configuration Applied:");
  Serial.println("  " + String('─') * 66);
  Serial.println("    Timing Budget: 33ms (~30Hz)");
  Serial.println("    Mode: Single-shot ranging");
  Serial.println("    Signal Rate Limit: Default");
  Serial.println("    Sigma Estimator: Default");
  
  Serial.println("\n  Available Timing Budgets:");
  Serial.println("    • 20ms (Fast) - Less accurate, ~50Hz");
  Serial.println("    • 33ms (Balanced) - Good accuracy, ~30Hz ← Current");
  Serial.println("    • 200ms (Accurate) - Best accuracy, ~5Hz");
  
  Serial.println("\n  ✓ Configuration complete");
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(0x29);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x29, (uint8_t)1);
  
  if (Wire.available()) {
    return Wire.read();
  }
  Serial.println("\n" + repeatChar('═', 70));
  Serial.println("VL53L0X OFFSET CALIBRATION");
  Serial.println(repeatChar('═', 70));
// ============================================================================
// CALIBRATION FUNCTIONS
// ============================================================================

void performOffsetCalibration() {
  Serial.println("\n" + String('═') * 70);
  Serial.println("VL53L0X OFFSET CALIBRATION");
  Serial.println(String('═') * 70);

  Serial.println("\n🎯 SETUP REQUIREMENTS:");
  Serial.println("  1. WHITE, FLAT target (paper or cardboard)");
  Serial.println("  2. Target EXACTLY at known distance");
  Serial.println("  3. Target PERPENDICULAR to sensor");
  Serial.println("  4. No other objects in field of view (25° cone)");
  Serial.println("  5. Normal room lighting (not direct sunlight)");
  Serial.println("  6. Stable mounting - no vibration");

  Serial.println("\n📏 Recommended Calibration Distances:");
  Serial.println("  • 100mm - Best for close-range work (recommended)");
  Serial.println("  • 200mm - Good for medium-range");
  Serial.println("  • 400mm - For long-range applications");

  Serial.print("\nEnter actual target distance (mm, 50-1000) [100]: ");
  int actualDistanceMm = readIntFromSerial(50, 1000, 100);

  Serial.println("\n📝 Calibration Setup Checklist:");
  Serial.println("  [ ] White target positioned at " + String(actualDistanceMm) + "mm");
  Serial.println("  [ ] Target perpendicular to sensor");
  Serial.println("  [ ] Area behind target is clear");
  Serial.println("  [ ] Stable mounting (no movement)");
  Serial.println("  [ ] Normal indoor lighting");

  Serial.println("\nPress any key when ready to start calibration...");
  waitForSerialInput();

  // The rest of the function implementation should follow here.
  // Make sure readIntFromSerial and waitForSerialInput are implemented above this function.
}
  
  // Configuration for calibration
  const int numSamples = 100;
  int validSamples = 0;
  long sumDistances = 0;
  float distances[numSamples];
  
  Serial.println("\n⏱ Collecting " + String(numSamples) + " measurements...");
  Serial.println("(Keep target still and stable)\n");
  
  unsigned long startTime = millis();
  
  for (int i = 0; i < numSamples; i++) {
    uint16_t distance = tof.readRangeSingleMillimeters();
    
    if (!tof.timeoutOccurred() && distance < 8190) {
      distances[validSamples] = distance;
      sumDistances += distance;
      validSamples++;
      
      // Progress indicator
      if ((i + 1) % 10 == 0) {
        Serial.print("  Progress: ");
        Serial.print(((i + 1) * 100) / numSamples);
        Serial.print("% | Last reading: ");
        Serial.print(distance);
        Serial.println("mm");
      }
    } else {
      Serial.print("  Sample ");
      Serial.print(i + 1);
      Serial.println(": Failed (timeout or out of range)");
    }
    
    delay(50); // 20Hz sampling
  }
  
  unsigned long duration = millis() - startTime;
  
  // Check if we have enough valid samples
  if (validSamples < numSamples * 0.8) {
    Serial.println("\n✗ CALIBRATION FAILED");
    Serial.println("  Too many failed measurements (" + String(validSamples) + "/" + String(numSamples) + ")");
    Serial.println("\n  Possible causes:");
    Serial.println("  • Target too far or too close");
    Serial.println("  • Target not in field of view");
    Serial.println("  • Target material too dark/reflective");
    Serial.println("  • Ambient light too bright");
    Serial.println("  • Sensor malfunction");
    return;
  }
  
  // Calculate statistics
  float avgMeasured = (float)sumDistances / validSamples;
  
  // Calculate standard deviation
  float variance = 0;
  for (int i = 0; i < validSamples; i++) {
    float diff = distances[i] - avgMeasured;
    variance += diff * diff;
  }
  variance /= validSamples;
  float stdDev = sqrt(variance);
  
  // Find min/max
  float minDist = 9999;
  float maxDist = 0;
  for (int i = 0; i < validSamples; i++) {
  Serial.println("\n" + repeatChar('═', 70));
  Serial.println("OFFSET CALIBRATION RESULTS");
  Serial.println(repeatChar('═', 70));
  
  // Calculate offset
  int16_t calculatedOffset = (int16_t)(avgMeasured - actualDistanceMm);
  
  // Display results
  Serial.println("\n" + String('═') * 70);
  Serial.println("OFFSET CALIBRATION RESULTS");
  Serial.println(String('═') * 70);
  
  Serial.print("\nCalibration Duration: ");
  Serial.print(duration / 1000.0, 2);
  Serial.println(" seconds");
  
  Serial.print("Valid Samples: ");
  Serial.print(validSamples);
  Serial.print("/");
  Serial.print(numSamples);
  Serial.print(" (");
  Serial.print((validSamples * 100) / numSamples);
  Serial.println("%)");
  
  Serial.println("\n┌─ MEASUREMENTS ──────────────────────────────────────────────────┐");
  Serial.print("│  Actual Distance:     ");
  Serial.print(actualDistanceMm);
  Serial.println(" mm                                     │");
  
  Serial.print("│  Average Measured:    ");
  Serial.print(avgMeasured, 2);
  Serial.println(" mm                                  │");
  
  Serial.print("│  Standard Deviation:  ");
  Serial.print(stdDev, 2);
  Serial.println(" mm                                   │");
  
  Serial.print("│  Minimum Reading:     ");
  Serial.print(minDist, 0);
  Serial.println(" mm                                     │");
  Serial.println("\n[ERROR] VL53L0X register read failed");
  return 0;
  Serial.println(" mm                                     │");
// ============================================================================
// CALIBRATION FUNCTIONS
// ============================================================================

void performOffsetCalibration() {
  Serial.println("\n" + String('═') * 70);
  Serial.println("VL53L0X OFFSET CALIBRATION");
  Serial.println(String('═') * 70);

  Serial.println("\n🎯 SETUP REQUIREMENTS:");
  Serial.println("  1. WHITE, FLAT target (paper or cardboard)");
  Serial.println("  2. Target EXACTLY at known distance");
  Serial.println("  3. Target PERPENDICULAR to sensor");
  Serial.println("  4. No other objects in field of view (25° cone)");
  Serial.println("  5. Normal room lighting (not direct sunlight)");
  Serial.println("  6. Stable mounting - no vibration");

  Serial.println("\n📏 Recommended Calibration Distances:");
  Serial.println("  • 100mm - Best for close-range work (recommended)");
  Serial.println("  • 200mm - Good for medium-range");
  Serial.println("  • 400mm - For long-range applications");

  Serial.print("\nEnter actual target distance (mm, 50-1000) [100]: ");
  int actualDistanceMm = readIntFromSerial(50, 1000, 100);

  Serial.println("\n📝 Calibration Setup Checklist:");
  Serial.println("  [ ] White target positioned at " + String(actualDistanceMm) + "mm");
  Serial.println("  [ ] Target perpendicular to sensor");
  Serial.println("  [ ] Area behind target is clear");
  Serial.println("  [ ] Stable mounting (no movement)");
  Serial.println("  [ ] Normal indoor lighting");

  Serial.println("\nPress any key when ready to start calibration...");
  waitForSerialInput();

  // Configuration for calibration
  const int numSamples = 100;
  int validSamples = 0;
  long sumDistances = 0;
  float distances[numSamples];

  Serial.println("\n⏱ Collecting " + String(numSamples) + " measurements...");
  Serial.println("(Keep target still and stable)\n");

  unsigned long startTime = millis();

  for (int i = 0; i < numSamples; i++) {
    uint16_t distance = tof.readRangeSingleMillimeters();

    if (!tof.timeoutOccurred() && distance < 8190) {
      distances[validSamples] = distance;
      sumDistances += distance;
      validSamples++;

      // Progress indicator
      if ((i + 1) % 10 == 0) {
        Serial.print("  Progress: ");
        Serial.print(((i + 1) * 100) / numSamples);
        Serial.print("% | Last reading: ");
        Serial.print(distance);
        Serial.println("mm");
      }
    } else {
      Serial.print("  Sample ");
      Serial.print(i + 1);
      Serial.println(": Failed (timeout or out of range)");
    }

    delay(50); // 20Hz sampling
  }

  unsigned long duration = millis() - startTime;

  // Check if we have enough valid samples
  if (validSamples < numSamples * 0.8) {
    Serial.println("\n✗ CALIBRATION FAILED");
    Serial.println("  Too many failed measurements (" + String(validSamples) + "/" + String(numSamples) + ")");
    Serial.println("\n  Possible causes:");
    Serial.println("  • Target too far or too close");
    Serial.println("  • Target not in field of view");
    Serial.println("  • Target material too dark/reflective");
    Serial.println("  • Ambient light too bright");
    Serial.println("  • Sensor malfunction");
    return;
  }

  // Calculate statistics
  float avgMeasured = (float)sumDistances / validSamples;

  // Calculate standard deviation
  float variance = 0;
  for (int i = 0; i < validSamples; i++) {
    float diff = distances[i] - avgMeasured;
    variance += diff * diff;
  }
  variance /= validSamples;
  float stdDev = sqrt(variance);

  // Find min/max
  float minDist = 9999;
  float maxDist = 0;
  for (int i = 0; i < validSamples; i++) {
    if (distances[i] < minDist) minDist = distances[i];
    if (distances[i] > maxDist) maxDist = distances[i];
  }

  // Calculate offset
  int16_t calculatedOffset = (int16_t)(avgMeasured - actualDistanceMm);

  // Display results
  Serial.println("\n" + String('═') * 70);
  Serial.println("OFFSET CALIBRATION RESULTS");
  Serial.println(String('═') * 70);

  Serial.print("\nCalibration Duration: ");
  Serial.print(duration / 1000.0, 2);
  Serial.println(" seconds");

  Serial.print("Valid Samples: ");
  Serial.print(validSamples);
  Serial.print("/");
  Serial.print(numSamples);
  Serial.print(" (");
  Serial.print((validSamples * 100) / numSamples);
  Serial.println("%)");

  Serial.println("\n┌─ MEASUREMENTS ──────────────────────────────────────────────────┐");
  Serial.print("│  Actual Distance:     ");
  Serial.print(actualDistanceMm);
  Serial.println(" mm                                     │");

  Serial.print("│  Average Measured:    ");
  Serial.print(avgMeasured, 2);
  Serial.println(" mm                                  │");

  Serial.print("│  Standard Deviation:  ");
  Serial.print(stdDev, 2);
  Serial.println(" mm                                   │");

  Serial.print("│  Minimum Reading:     ");
  Serial.print(minDist, 0);
  Serial.println(" mm                                     │");

  Serial.print("│  Maximum Reading:     ");
  Serial.print(maxDist, 0);
  Serial.println(" mm                                     │");

  Serial.print("│  Range (max-min):     ");
  Serial.print(maxDist - minDist, 0);
  Serial.println(" mm                                      │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");

  Serial.println("\n┌─ CALIBRATION ───────────────────────────────────────────────────┐");
  Serial.print("│  Calculated Offset:   ");
  Serial.print(calculatedOffset);
  Serial.println(" mm                                       │");

  Serial.print("│  Error (uncalibrated): ");
  Serial.print(abs(calculatedOffset));
  Serial.print(" mm (");
  Serial.print((abs(calculatedOffset) / (float)actualDistanceMm) * 100.0, 1);
  Serial.println("%)                  │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");

  // Quality assessment
  Serial.println("\n┌─ QUALITY ASSESSMENT ────────────────────────────────────────────┐");
  Serial.println("│                                                                  │");

  bool qualityGood = true;

  if (stdDev < 2.0) {
    Serial.println("│  ✓ Excellent consistency (std dev < 2mm)                        │");
  } else if (stdDev < 5.0) {
    Serial.println("│  ✓ Good consistency (std dev < 5mm)                             │");
  } else if (stdDev < 10.0) {
    Serial.println("│  ⚠ Fair consistency (std dev < 10mm)                            │");
    Serial.println("│    Consider recalibrating with more stable setup                │");
    qualityGood = false;
  } else {
    Serial.println("│  ✗ Poor consistency (std dev > 10mm)                            │");
    Serial.println("│    Target likely moved or environment unstable                  │");
    Serial.println("│    Recalibration strongly recommended                           │");
    qualityGood = false;
  }

  if ((maxDist - minDist) < 20) {
    Serial.println("│  ✓ Low measurement spread (< 20mm range)                        │");
  } else if ((maxDist - minDist) < 50) {
    Serial.println("│  ⚠ Moderate measurement spread (20-50mm range)                  │");
    qualityGood = false;
  } else {
    Serial.println("│  ✗ High measurement spread (> 50mm range)                       │");
    qualityGood = false;
  }

  Serial.println("│                                                                  │");

  if (qualityGood) {
    Serial.println("│  Overall: ✓ CALIBRATION PASSED - High Quality                  │");
  } else {
    Serial.println("│  Overall: ⚠ CALIBRATION QUESTIONABLE - Consider Retry          │");
  }

  Serial.println("│                                                                  │
  Serial.println("└──────────────────────────────────────────────────────────────────┘");

  // Ask to apply calibration
  Serial.println("\n" + String('═') * 70);
  Serial.print("Apply this calibration? (y/n) [y]: ");

  char response = readCharFromSerial('y');

  if (response == 'y' || response == 'Y') {
    calibration.offsetMm = calculatedOffset;
    calibration.isOffsetCalibrated = true;
    calibration.timestamp = millis();
    calibration.stdDevMm = stdDev;
    calibration.sampleCount = validSamples;
    calibration.minReading = minDist;
    calibration.maxReading = maxDist;
    calibration.avgReading = avgMeasured;

    Serial.println("\n✓ Calibration applied successfully!");
    Serial.println("  All distance readings will now be offset-corrected.");
  } else {
    Serial.println("\n⚠ Calibration NOT applied");
    Serial.println("  Readings will remain uncalibrated.");
  }

  Serial.println(String('═') * 70);
}
    float avgAbsError = totalAbsError / validTests;
    
    Serial.println("\n┌─ SUMMARY ───────────────────────────────────────────────────────┐");
    Serial.print("│  Average Absolute Error: ");
    Serial.print(avgAbsError, 2);
    Serial.println(" mm                               │");
    
    if (avgAbsError < 10) {
      Serial.println("│  Overall Assessment: ✓ Excellent Calibration                   │");
    } else if (avgAbsError < 20) {
      Serial.println("│  Overall Assessment: ✓ Good Calibration                        │");
    } else if (avgAbsError < 40) {
      Serial.println("│  Overall Assessment: ⚠ Fair - Consider Recalibration           │");
    } else {
      Serial.println("│  Overall Assessment: ✗ Poor - Recalibration Recommended        │");
    }
    
    Serial.println("└──────────────────────────────────────────────────────────────────┘");
  }
  
  Serial.println("\n" + String('═') * 70);
}

// ============================================================================
// TEST MODE FUNCTIONS
// ============================================================================

void runRangeProfileTest() {
  Serial.println("\n" + String('═') * 70);
  Serial.println("RANGE PROFILE TEST");
  Serial.println(String('═') * 70);
  
  Serial.println("\nThis test measures accuracy across the full sensor range.");
  Serial.println("Move an object from near to far while monitoring readings.");
  
  Serial.print("\nTest duration (seconds, 10-120) [30]: ");
  int duration = readIntFromSerial(10, 120, 30);
  
  Serial.println("\nStarting range profile test...");
  Serial.println("Move object slowly from close to far distance");
  Serial.println();
  
  delay(2000);
  
  // Reset histogram
  for (int i = 0; i < HISTOGRAM_BINS; i++) {
    histogram[i] = 0;
  }
  
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;
  
  int measurementCount = 0;
  float minDist = 9999;
  float maxDist = 0;
  long sumDist = 0;
  
  while (millis() - startTime < duration * 1000UL) {
    uint16_t rawDist = tof.readRangeSingleMillimeters();
    
    if (!tof.timeoutOccurred() && rawDist < 8190) {
      int16_t distance = rawDist;
      
      if (calibration.isOffsetCalibrated) {
        distance -= calibration.offsetMm;
      }
      
      // Update histogram
      if (distance >= 0 && distance < HISTOGRAM_RANGE_MM) {
        int bin = (distance * HISTOGRAM_BINS) / HISTOGRAM_RANGE_MM;
        if (bin >= 0 && bin < HISTOGRAM_BINS) {
          histogram[bin]++;
        }
      }
      
      // Track statistics
      if (distance < minDist) minDist = distance;
      if (distance > maxDist) maxDist = distance;
      sumDist += distance;
      measurementCount++;
      
      // Print periodically
      if (millis() - lastPrint >= 500) {
        Serial.print("Distance: ");
        printPaddedInt(distance, 4);
        Serial.print("mm | Samples: ");
        printPaddedInt(measurementCount, 4);
        Serial.print(" | Range: ");
        printPaddedInt((int)minDist, 4);
        Serial.print("-");
        printPaddedInt((int)maxDist, 4);
        Serial.println("mm");
        
        lastPrint = millis();
      }
    }
    
    delay(20); // ~50Hz
  }
  
  // Display results
  float avgDist = (float)sumDist / measurementCount;
  
  Serial.println("\n" + String('═') * 70);
  Serial.println("RANGE PROFILE RESULTS");
  Serial.println(String('═') * 70);
  
  Serial.print("\nTotal Measurements: ");
  Serial.println(measurementCount);
  
  Serial.print("Minimum Distance: ");
  Serial.print(minDist, 0);
  Serial.println("mm");
  
  Serial.print("Maximum Distance: ");
  Serial.print(maxDist, 0);
  Serial.println("mm");
  
  Serial.print("Average Distance: ");
  Serial.print(avgDist, 1);
  Serial.println("mm");
  
  Serial.print("Range Covered: ");
  Serial.print(maxDist - minDist, 0);
  Serial.println("mm");
  
  // Display histogram
  Serial.println("\n┌─ DISTANCE DISTRIBUTION ─────────────────────────────────────────┐");
  Serial.println("│                                                                  │");
  
  int maxCount = 0;
  for (int i = 0; i < HISTOGRAM_BINS; i++) {
    if (histogram[i] > maxCount) maxCount = histogram[i];
  }
  
  for (int i = 0; i < HISTOGRAM_BINS; i++) {
    int rangeStart = (i * HISTOGRAM_RANGE_MM) / HISTOGRAM_BINS;
    int rangeEnd = ((i + 1) * HISTOGRAM_RANGE_MM) / HISTOGRAM_BINS;
    
    Serial.print("│  ");
    printPaddedInt(rangeStart, 4);
    Serial.print("-");
    printPaddedInt(rangeEnd, 4);
    Serial.print("mm: ");
    
    // Bar graph
    int barLength = (histogram[i] * 40) / (maxCount > 0 ? maxCount : 1);
    for (int b = 0; b < barLength; b++) {
      Serial.print("█");
    }
    for (int b = barLength; b < 40; b++) {
      Serial.print(" ");
    }
    
    Serial.print(" ");
    Serial.print(histogram[i]);
    Serial.println(" │");
  }
  
  Serial.println("│                                                                  │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n" + String('═') * 70);
}

void runMaterialResponseTest() {
  Serial.println("\n" + String('═') * 70);
  Serial.println("MATERIAL RESPONSE TEST");
  Serial.println(String('═') * 70);
  
  Serial.println("\nThis test measures sensor response to different materials.");
  Serial.println("You will test multiple surface types at a fixed distance.");
  
  Serial.print("\nTest distance (mm, 100-500) [200]: ");
  int testDistance = readIntFromSerial(100, 500, 200);
  
  Serial.print("Number of materials to test (1-10) [5]: ");
  int numMaterials = readIntFromSerial(1, 10, 5);
  
  struct MaterialResult {
    String name;
    float avgDistance;
    float stdDev;
    int failedSamples;
  };
  
  MaterialResult results[10];
  
  for (int m = 0; m < numMaterials; m++) {
    Serial.println("\n" + String('─') * 70);
    Serial.print("Material ");
    Serial.print(m + 1);
    Serial.print(" of ");
    Serial.println(numMaterials);
    Serial.println(String('─') * 70);
    
    Serial.print("\nEnter material name (e.g., white paper, black card): ");
    String materialName = readStringFromSerial();
    
    Serial.println("\n📝 Position " + materialName + " at " + String(testDistance) + "mm");
    Serial.println("Press any key when ready...");
    waitForSerialInput();
    
    // Take measurements
    const int samples = 50;
    float distances[samples];
    int validCount = 0;
    long sum = 0;
    
    Serial.println("Measuring... (50 samples)");
    
    for (int i = 0; i < samples; i++) {
      uint16_t rawDist = tof.readRangeSingleMillimeters();
      
      if (!tof.timeoutOccurred() && rawDist < 8190) {
        int16_t dist = rawDist;
        if (calibration.isOffsetCalibrated) {
          dist -= calibration.offsetMm;
        }
        
        distances[validCount] = dist;
        sum += dist;
        validCount++;
        
        if ((i + 1) % 10 == 0) {
          Serial.print("  ");
          Serial.print(validCount);
          Serial.print("/");
          Serial.print(i + 1);
          Serial.print(" valid");
          if (validCount > 0) {
            Serial.print(" - Current avg: ");
            Serial.print((float)sum / validCount, 1);
            Serial.print("mm");
          }
          Serial.println();
        }
      }
      
      delay(40);
    }
    
    // Calculate statistics
    if (validCount > 0) {
      float avg = (float)sum / validCount;
      
      // Standard deviation
      float variance = 0;
      for (int i = 0; i < validCount; i++) {
        float diff = distances[i] - avg;
        variance += diff * diff;
      }
      variance /= validCount;
      float stdDev = sqrt(variance);
      
      results[m].name = materialName;
      results[m].avgDistance = avg;
      results[m].stdDev = stdDev;
      results[m].failedSamples = samples - validCount;
      
      Serial.print("\n  Average: ");
      Serial.print(avg, 1);
      Serial.print("mm | Std Dev: ");
      Serial.print(stdDev, 2);
      Serial.print("mm | Failed: ");
      Serial.println(samples - validCount);
    } else {
      Serial.println("\n  ✗ All measurements failed!");
      results[m].name = materialName;
      results[m].avgDistance = -1;
      results[m].stdDev = 0;
      results[m].failedSamples = samples;
    }
  }
  
  // Display comparison
  Serial.println("\n" + String('═') * 70);
  Serial.println("MATERIAL RESPONSE COMPARISON");
  Serial.println(String('═') * 70);
  Serial.print("\nTest Distance: ");
  Serial.print(testDistance);
  Serial.println("mm");
  Serial.println();
  
  Serial.println("┌──────────────────────────────────────────────────────────────────┐");
  Serial.println("│  Material              Measured    Error    Std Dev    Failed    │");
  Serial.println("├──────────────────────────────────────────────────────────────────┤");
  
  for (int i = 0; i < numMaterials; i++) {
    if (results[i].avgDistance >= 0) {
      Serial.print("│  ");
      
      // Material name (truncate if too long)
      String name = results[i].name;
      if (name.length() > 20) name = name.substring(0, 17) + "...";
      Serial.print(name);
      for (int s = name.length(); s < 20; s++) Serial.print(" ");
      
      Serial.print("  ");
      printPaddedFloat(results[i].avgDistance, 7, 1);
      Serial.print("  ");
      
      float error = results[i].avgDistance - testDistance;
      printPaddedFloat(error, 6, 1);
      Serial.print("  ");
      
      printPaddedFloat(results[i].stdDev, 6, 2);
      Serial.print("  ");
      
      printPaddedInt(results[i].failedSamples, 5);
      Serial.println("  │");
    } else {
      Serial.print("│  ");
      String name = results[i].name;
      if (name.length() > 20) name = name.substring(0, 17) + "...";
      Serial.print(name);
      for (int s = name.length(); s < 20; s++) Serial.print(" ");
      Serial.println("  ALL FAILED                          │");
    }
  }
  
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n💡 Interpretation:");
  Serial.println("  • White/light materials: Most accurate (highest reflectance)");
  Serial.println("  • Dark materials: May under-read (low reflectance)");
  Serial.println("  • Shiny materials: May over-read (specular reflection)");
  Serial.println("  • Failed samples: Material absorbed/scattered too much light");
  
  Serial.println("\n" + String('═') * 70);
}

void runSpeedTest() {
  Serial.println("\n" + String('═') * 70);
  Serial.println("MEASUREMENT SPEED TEST");
  Serial.println(String('═') * 70);
  
  Serial.println("\nThis test measures sensor update rate and timing consistency.");
  
  Serial.print("\nNumber of measurements to collect (100-1000) [500]: ");
  int numMeasurements = readIntFromSerial(100, 1000, 500);
  
  Serial.println("\nTesting at different timing budgets...");
  
  uint32_t timingBudgets[] = {TIMING_BUDGET_FAST, TIMING_BUDGET_BALANCED, TIMING_BUDGET_ACCURATE};
  const char* budgetNames[] = {"Fast (20ms)", "Balanced (33ms)", "Accurate (200ms)"};
  
  for (int b = 0; b < 3; b++) {
    Serial.println("\n" + String('─') * 70);
    Serial.print("Testing: ");
    Serial.println(budgetNames[b]);
    Serial.println(String('─') * 70);
    
    tof.setMeasurementTimingBudget(timingBudgets[b]);
    delay(100);
    
    unsigned long measurements[1000];
    int validCount = 0;
    
    Serial.println("Measuring...");
    
    unsigned long startTime = millis();
    
    for (int i = 0; i < numMeasurements; i++) {
      unsigned long measureStart = micros();
      
      uint16_t distance = tof.readRangeSingleMillimeters();
      
      unsigned long measureTime = micros() - measureStart;
      
      if (!tof.timeoutOccurred() && distance < 8190) {
        measurements[validCount] = measureTime;
        validCount++;
      }
      
      if ((i + 1) % 100 == 0) {
        Serial.print("  ");
        Serial.print(i + 1);
        Serial.print("/");
        Serial.println(numMeasurements);
      }
    }
    
    unsigned long totalTime = millis() - startTime;
    
    if (validCount > 0) {
      // Calculate statistics
      unsigned long sum = 0;
      unsigned long minTime = 999999;
      unsigned long maxTime = 0;
      
      for (int i = 0; i < validCount; i++) {
        sum += measurements[i];
        if (measurements[i] < minTime) minTime = measurements[i];
        if (measurements[i] > maxTime) maxTime = measurements[i];
      }
      
      float avgTime = (float)sum / validCount / 1000.0; // Convert to ms
      float actualRate = (validCount * 1000.0) / totalTime; // Hz
      
      Serial.println("\n  Results:");
      Serial.print("    Valid measurements: ");
      Serial.print(validCount);
      Serial.print("/");
      Serial.println(numMeasurements);
      
      Serial.print("    Average measurement time: ");
      Serial.print(avgTime, 2);
      Serial.println("ms");
      
      Serial.print("    Min time: ");
      Serial.print(minTime / 1000.0, 2);
      Serial.println("ms");
      
      Serial.print("    Max time: ");
      Serial.print(maxTime / 1000.0, 2);
      Serial.println("ms");
      
      Serial.print("    Actual update rate: ");
      Serial.print(actualRate, 1);
      Serial.println("Hz");
      
      Serial.print("    Total test duration: ");
      Serial.print(totalTime / 1000.0, 2);
      Serial.println("s");
    } else {
      Serial.println("\n  ✗ All measurements failed!");
    }
  }
  
  // Reset to balanced mode
  tof.setMeasurementTimingBudget(TIMING_BUDGET_BALANCED);
  
  Serial.println("\n" + String('═') * 70);
  Serial.println("✓ Speed test complete");
  Serial.println("  Timing budget reset to Balanced (33ms)");
  Serial.println(String('═') * 70);
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

void displayLiveDistance() {
  unsigned long measureStart = micros();
  uint16_t rawDistance = tof.readRangeSingleMillimeters();
  unsigned long measureTime = micros() - measureStart;
  
  // Update metrics
  metrics.totalMeasurements++;
  metrics.totalMeasurementTime += measureTime;
  
  if (measureTime < metrics.minMeasurementTime) {
    metrics.minMeasurementTime = measureTime;
  }
  if (measureTime > metrics.maxMeasurementTime) {
    metrics.maxMeasurementTime = measureTime;
  }
  
  if (tof.timeoutOccurred()) {
    metrics.timeouts++;
    Serial.println("Distance: TIMEOUT                                    ");
    return;
  }
  
  if (rawDistance >= 8190) {
    metrics.outOfRange++;
    Serial.println("Distance: OUT OF RANGE (>2m)                         ");
    return;
  }
  
  // Apply calibration
  int16_t distance = rawDistance;
  if (calibration.isOffsetCalibrated) {
    distance -= calibration.offsetMm;
  }
  
  metrics.successfulMeasurements++;
  metrics.totalDistance += distance;
  
  if (distance < metrics.minDistance) metrics.minDistance = distance;
  if (distance > metrics.maxDistance) metrics.maxDistance = distance;
  
  // Display based on mode
  switch (currentMode) {
    case MODE_LIVE_DISTANCE:
      displayStandardDistance(distance, rawDistance, measureTime);
      break;
      
    case MODE_CONTINUOUS_FAST:
      displayFastDistance(distance);
      break;
      
    case MODE_HIGH_ACCURACY:
      displayHighAccuracyDistance(distance, measureTime);
      break;
      
    default:
      displayStandardDistance(distance, rawDistance, measureTime);
      break;
  }
}

void displayStandardDistance(int16_t distance, uint16_t raw, unsigned long timeUs) {
  Serial.print("Distance: ");
  printPaddedInt(distance, 4);
  Serial.print("mm");
  
  if (calibration.isOffsetCalibrated) {
    Serial.print(" (raw: ");
    printPaddedInt(raw, 4);
    Serial.print("mm, offset: ");
    Serial.print(calibration.offsetMm);
    Serial.print("mm)");
  }
  
  // Visual bar
  if (distance < 1000) {
    int barLength = map(distance, 0, 1000, 0, 30);
    barLength = constrain(barLength, 0, 30);
    
    Serial.print(" [");
    for (int i = 0; i < barLength; i++) Serial.print("█");
    for (int i = barLength; i < 30; i++) Serial.print(" ");
    Serial.print("]");
  }
  
  // Measurement time
  Serial.print(" ");
  Serial.print(timeUs / 1000.0, 1);
  Serial.print("ms");
  
  Serial.println();
}

void displayFastDistance(int16_t distance) {
  Serial.print(distance);
  Serial.print("mm ");
  
  // Simple bar
  int bars = distance / 100;
  for (int i = 0; i < bars && i < 20; i++) {
    Serial.print("█");
  }
  
  Serial.println();
}

void displayHighAccuracyDistance(int16_t distance, unsigned long timeUs) {
  // Take multiple samples for averaging
  static int sampleBuffer[10];
  static int sampleIndex = 0;
  static int sampleCount = 0;
  
  sampleBuffer[sampleIndex] = distance;
  sampleIndex = (sampleIndex + 1) % 10;
  if (sampleCount < 10) sampleCount++;
  
  // Calculate average
  long sum = 0;
  for (int i = 0; i < sampleCount; i++) {
    sum += sampleBuffer[i];
  }
  float avg = (float)sum / sampleCount;
  
  Serial.print("Distance: ");
  printPaddedFloat(avg, 7, 2);
  Serial.print("mm (");
  Serial.print(sampleCount);
  Serial.print("-sample avg) | Current: ");
  printPaddedInt(distance, 4);
  Serial.print("mm | Time: ");
  Serial.print(timeUs / 1000.0, 2);
  Serial.println("ms");
}

void displayStatistics() {
  unsigned long uptime = (millis() - metrics.startTime) / 1000;
  float successRate = (metrics.successfulMeasurements * 100.0) / 
                      (metrics.totalMeasurements > 0 ? metrics.totalMeasurements : 1);
  float avgDistance = metrics.totalDistance / 
                     (metrics.successfulMeasurements > 0 ? metrics.successfulMeasurements : 1);
  float avgMeasureTime = (float)metrics.totalMeasurementTime / 
                        (metrics.totalMeasurements > 0 ? metrics.totalMeasurements : 1) / 1000.0;
  
  Serial.println("\n" + String('═') * 70);
  Serial.println("VL53L0X PERFORMANCE STATISTICS");
  Serial.println(String('═') * 70);
  
  Serial.println("\n┌─ SYSTEM ────────────────────────────────────────────────────────┐");
  Serial.print("│  Uptime: ");
  Serial.print(uptime);
  Serial.print(" seconds (");
  Serial.print(uptime / 60);
  Serial.println(" minutes)                           │");
  
  Serial.print("│  Measurement Count: ");
  Serial.print(metrics.totalMeasurements);
  Serial.println("                                          │");
  
  Serial.print("│  Successful: ");
  Serial.print(metrics.successfulMeasurements);
  Serial.print(" (");
  Serial.print(successRate, 1);
  Serial.println("%)                                     │");
  
  Serial.print("│  Timeouts: ");
  Serial.print(metrics.timeouts);
  Serial.println("                                                   │");
  
  Serial.print("│  Out of Range: ");
  Serial.print(metrics.outOfRange);
  Serial.println("                                             │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n┌─ CALIBRATION ───────────────────────────────────────────────────┐");
  Serial.print("│  Offset Calibration: ");
  Serial.print(calibration.isOffsetCalibrated ? "✓ Active" : "✗ Not Calibrated");
  Serial.println("                                      │");
  
  if (calibration.isOffsetCalibrated) {
    Serial.print("│    Offset: ");
    Serial.print(calibration.offsetMm);
    Serial.println(" mm                                                 │");
    
    Serial.print("│    Quality Std Dev: ");
    Serial.print(calibration.stdDevMm, 2);
    Serial.println(" mm                                      │");
    
    unsigned long calAge = (millis() - calibration.timestamp) / 1000;
    Serial.print("│    Age: ");
    Serial.print(calAge / 60);
    Serial.print(" minutes ");
    Serial.print(calAge % 60);
    Serial.println(" seconds                                   │");
  }
  
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  if (metrics.successfulMeasurements > 0) {
    Serial.println("\n┌─ DISTANCE STATISTICS ───────────────────────────────────────────┐");
    Serial.print("│  Current: ");
    
    uint16_t current = tof.readRangeSingleMillimeters();
    if (!tof.timeoutOccurred() && current < 8190) {
      int16_t calibrated = current - (calibration.isOffsetCalibrated ? calibration.offsetMm : 0);
      Serial.print(calibrated);
      Serial.println(" mm                                                  │");
    } else {
      Serial.println("N/A                                                  │");
    }
    
    Serial.print("│  Average: ");
    Serial.print(avgDistance, 1);
    Serial.println(" mm                                               │");
    
    Serial.print("│  Minimum: ");
    Serial.print(metrics.minDistance, 0);
    Serial.println(" mm                                                │");
    
    Serial.print("│  Maximum: ");
    Serial.print(metrics.maxDistance, 0);
    Serial.println(" mm                                               │");
    
    Serial.print("│  Range: ");
    Serial.print(metrics.maxDistance - metrics.minDistance, 0);
    Serial.println(" mm                                                  │");
    Serial.println("└──────────────────────────────────────────────────────────────────┘");
  }
  
  Serial.println("\n┌─ TIMING STATISTICS ─────────────────────────────────────────────┐");
  Serial.print("│  Average Measurement Time: ");
  Serial.print(avgMeasureTime, 2);
  Serial.println(" ms                           │");
  
  Serial.print("│  Minimum: ");
  Serial.print(metrics.minMeasurementTime / 1000.0, 2);
  Serial.println(" ms                                            │");
  
  Serial.print("│  Maximum: ");
  Serial.print(metrics.maxMeasurementTime / 1000.0, 2);
  Serial.println(" ms                                            │");
  
  float theoreticalRate = 1000.0 / avgMeasureTime;
  Serial.print("│  Theoretical Max Rate: ");
  Serial.print(theoreticalRate, 1);
  Serial.println(" Hz                                 │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n" + String('═') * 70);
}

// ============================================================================
// MENU AND COMMAND HANDLING
// ============================================================================

void printMainMenu() {
  Serial.println("\n" + String('═') * 70);
  Serial.println("COMMAND MENU");
  Serial.println(String('═') * 70);
  
  Serial.println("\n┌─ CALIBRATION ───────────────────────────────────────────────────┐");
  Serial.println("│  c - Offset calibration (measures systematic error)             │");
  Serial.println("│  v - Verify calibration accuracy (multi-distance test)          │");
  Serial.println("│  x - Clear calibration (reset to uncalibrated)                  │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n┌─ DISPLAY MODES ─────────────────────────────────────────────────┐");
  Serial.println("│  1 - Standard distance display (with bar graph)                 │");
  Serial.println("│  2 - Fast mode (minimal output)                                 │");
  Serial.println("│  3 - High accuracy mode (10-sample averaging)                   │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n┌─ TESTS ─────────────────────────────────────────────────────────┐");
  Serial.println("│  r - Range profile test (scan full range)                       │");
  Serial.println("│  m - Material response test (compare surfaces)                  │");
  Serial.println("│  t - Speed test (measure update rates)                          │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n┌─ CONFIGURATION ─────────────────────────────────────────────────┐");
  Serial.println("│  f - Set timing budget FAST (20ms, ~50Hz)                       │");
  Serial.println("│  b - Set timing budget BALANCED (33ms, ~30Hz)                   │");
  Serial.println("│  a - Set timing budget ACCURATE (200ms, ~5Hz)                   │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n┌─ UTILITIES ─────────────────────────────────────────────────────┐");
  Serial.println("│  s - Show statistics                                             │");
  Serial.println("│  z - Reset metrics                                               │");
  Serial.println("│  h - Show this help menu                                         │");
  Serial.println("└──────────────────────────────────────────────────────────────────┘");
  
  Serial.println("\n" + String('═') * 70);
  Serial.println();
}

void handleSerialCommand(char cmd) {
  switch (cmd) {
    case 'c':
    case 'C':
      performOffsetCalibration();
      break;
      
    case 'v':
    case 'V':
      testCalibrationAccuracy();
      break;
      
    case 'x':
    case 'X':
      calibration.offsetMm = 0;
      calibration.isOffsetCalibrated = false;
      Serial.println("\n✓ Calibration cleared");
      Serial.println("  Sensor now operating without offset correction");
      break;
      
    case '1':
      currentMode = MODE_LIVE_DISTANCE;
      Serial.println("\n→ Mode: Standard Distance Display");
      break;
      
    case '2':
      currentMode = MODE_CONTINUOUS_FAST;
      Serial.println("\n→ Mode: Fast Display (minimal output)");
      break;
      
    case '3':
      currentMode = MODE_HIGH_ACCURACY;
      Serial.println("\n→ Mode: High Accuracy (10-sample averaging)");
      break;
      
    case 'r':
    case 'R':
      runRangeProfileTest();
      break;
      
    case 'm':
    case 'M':
      runMaterialResponseTest();
      break;
      
    case 't':
    case 'T':
      runSpeedTest();
      break;
      
    case 'f':
    case 'F':
      tof.setMeasurementTimingBudget(TIMING_BUDGET_FAST);
      Serial.println("\n✓ Timing budget: FAST (20ms, ~50Hz)");
      Serial.println("  Trade-off: Faster updates, less accurate");
      break;
      
    case 'b':
    case 'B':
      tof.setMeasurementTimingBudget(TIMING_BUDGET_BALANCED);
      Serial.println("\n✓ Timing budget: BALANCED (33ms, ~30Hz)");
      Serial.println("  Trade-off: Good balance of speed and accuracy");
      break;
      
    case 'a':
    case 'A':
      tof.setMeasurementTimingBudget(TIMING_BUDGET_ACCURATE);
      Serial.println("\n✓ Timing budget: ACCURATE (200ms, ~5Hz)");
      Serial.println("  Trade-off: Most accurate, slower updates");
      break;
      
    case 's':
    case 'S':
      displayStatistics();
      break;
      
    case 'z':
    case 'Z':
      metrics.totalMeasurements = 0;
      metrics.successfulMeasurements = 0;
      metrics.timeouts = 0;
      metrics.outOfRange = 0;
      metrics.startTime = millis();
      metrics.minDistance = 9999;
      metrics.maxDistance = 0;
      metrics.totalDistance = 0;
      metrics.minMeasurementTime = 9999;
      metrics.maxMeasurementTime = 0;
      metrics.totalMeasurementTime = 0;
      
      // Reset histogram
      for (int i = 0; i < HISTOGRAM_BINS; i++) {
        histogram[i] = 0;
      }
      
      Serial.println("\n✓ All metrics reset");
      break;
      
    case 'h':
    case 'H':
      printMainMenu();
      break;
      
    default:
      Serial.println("\n⚠ Unknown command. Press 'h' for help.");
      break;
  }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void printPaddedInt(int value, int width) {
  String str = String(value);
  for (int i = str.length(); i < width; i++) {
    Serial.print(" ");
  }
  Serial.print(str);
}

void printPaddedFloat(float value, int width, int decimals) {
  char buffer[20];
  dtostrf(value, width, decimals, buffer);
  Serial.print(buffer);
}

void waitForSerialInput() {
  while (!Serial.available()) {
    delay(10);
  }
  while (Serial.available()) {
    Serial.read(); // Clear buffer
  }
}

int readIntFromSerial(int minVal, int maxVal, int defaultVal) {
  unsigned long timeout = millis() + 10000;
  String input = "";
  
  while (millis() < timeout) {
    if (Serial.available()) {
      char c = Serial.read();
      
      if (c == '\n' || c == '\r') {
        if (input.length() == 0) {
          Serial.println(defaultVal);
          return defaultVal;
        }
        
        int value = input.toInt();
        
        if (value >= minVal && value <= maxVal) {
          Serial.println(value);
          return value;
        } else {
          Serial.println("\n⚠ Out of range. Using default: " + String(defaultVal));
          return defaultVal;
        }
      } else if (c >= '0' && c <= '9') {
        input += c;
        Serial.print(c);
      }
    }
    delay(10);
  }
  
  Serial.println("\n⏱ Timeout. Using default: " + String(defaultVal));
  return defaultVal;
}

char readCharFromSerial(char defaultChar) {
  unsigned long timeout = millis() + 10000;
  
  while (millis() < timeout) {
    if (Serial.available()) {
      char c = Serial.read();
      while (Serial.available()) Serial.read(); // Clear buffer
      
      Serial.println(c);
      return c;
    }
    delay(10);
  }
  
  Serial.println(defaultChar);
  return defaultChar;
}

String readStringFromSerial() {
  String input = "";
  unsigned long timeout = millis() + 30000; // 30 second timeout
  
  while (millis() < timeout) {
    if (Serial.available()) {
      char c = Serial.read();
      
      if (c == '\n' || c == '\r') {
        Serial.println();
        return input.length() > 0 ? input : "Unknown";
      } else if (c >= 32 && c <= 126) { // Printable characters
        input += c;
        Serial.print(c);
      }
    }
    delay(10);
  }
  
  Serial.println("\n⏱ Timeout");
  return input.length() > 0 ? input : "Unknown";
}

// ============================================================================
// MAIN LOOP
// ============================================================================

unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 100; // 10Hz default

void loop() {
  // Update display at controlled rate
  if (millis() - lastDisplayUpdate >= displayInterval) {
    displayLiveDistance();
    lastDisplayUpdate = millis();
  }
  
  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); // Clear buffer
    
    handleSerialCommand(cmd);
  }
}
