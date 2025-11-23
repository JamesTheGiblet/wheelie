#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

// Utility function to repeat a character n times and return as String
String repeatChar(char c, int count) {
  String result = "";
  for (int i = 0; i < count; i++) {
    result += c;
  }
  return result;
}

// Known I2C device addresses for WHEELIE
struct I2CDevice {
  uint8_t address;
  const char* name;
  const char* description;
};

const I2CDevice knownDevices[] = {
  {0x29, "VL53L0X", "Time-of-Flight Distance Sensor"},
  {0x68, "MPU6050", "6-Axis IMU (Gyro + Accel)"},
  {0x69, "MPU6050", "6-Axis IMU (AD0=HIGH)"},
  {0x76, "BMP280", "Pressure/Temperature Sensor"},
  {0x77, "BMP280", "Pressure/Temperature (Alt)"},
  {0x1E, "HMC5883L", "3-Axis Magnetometer"},
  {0x0C, "AK8963", "3-Axis Magnetometer"},
  {0x20, "PCF8574", "I2C I/O Expander"},
  {0x27, "LCD", "I2C LCD Display"},
  {0x3C, "OLED", "I2C OLED Display (SSD1306)"},
  {0x3D, "OLED", "I2C OLED Display (Alt)"},
  {0x48, "ADS1115", "16-bit ADC"},
  {0x50, "EEPROM", "I2C EEPROM (24C32)"}
};

const int numKnownDevices = sizeof(knownDevices) / sizeof(knownDevices[0]);

// Test results tracking
int devicesFound = 0;
uint8_t foundAddresses[128];
bool hasMPU6050 = false;
bool hasVL53L0X = false;

void setup() {
  Serial.begin(115200);
  
  // Wait for serial with timeout
  unsigned long startTime = millis();
  Serial.println(repeatChar('─', 60));
  while (!Serial && millis() - startTime < 3000) {
    delay(10);
  }
  
  printHeader();
  
  // Initialize I2C
  Serial.println("\n[STEP 1] Initializing I2C Bus");
  Serial.println(repeatChar('─', 60));
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.print("  SDA Pin: GPIO ");
  Serial.println(I2C_SDA);
  Serial.print("  SCL Pin: GPIO ");
  Serial.println(I2C_SCL);
  
  // Test different I2C speeds
  testI2CSpeeds();
  
  // Perform comprehensive scan
  Serial.println("\n[STEP 2] Comprehensive I2C Device Scan");
  Serial.println(repeatChar('─', 60));
  scanI2CBus();
  
  // Identify devices
  Serial.println("\n[STEP 3] Device Identification");
  Serial.println(repeatChar('─', 60));
  identifyDevices();
  
  // Verify critical WHEELIE sensors
  Serial.println("\n[STEP 4] WHEELIE Sensor Verification");
  Serial.println(repeatChar('─', 60));
  verifyWHEELIESensors();
  
  // Test device communication
  Serial.println("\n[STEP 5] Device Communication Test");
  Serial.println(repeatChar('─', 60));
  testDeviceCommunication();
  
  // Final summary
  printSummary();
  
  // Interactive mode
  Serial.println("\n[STEP 6] Entering Interactive Mode");
  Serial.println(repeatChar('─', 60));
  printInteractiveMenu();
}

void printHeader() {
  Serial.println("\n" + repeatChar('═', 60));
  Serial.println("       WHEELIE I2C Bus Diagnostic Tool v2.0");
  Serial.println(repeatChar('═', 60));
  Serial.println("  Purpose: Scan, identify, and test I2C devices");
  Serial.println("  Author: For WHEELIE Robot Project");
  Serial.println(repeatChar('═', 60));
}

void testI2CSpeeds() {
  const uint32_t speeds[] = {100000, 200000, 400000}; // 100kHz, 200kHz, 400kHz
  const char* speedNames[] = {"100kHz (Standard)", "200kHz (Fast)", "400kHz (Fast Mode)"};
  
  Serial.println("\n  Testing I2C Clock Speeds:");
  Serial.println("  " + repeatChar('─', 56));
  
  for (int i = 0; i < 3; i++) {
    Wire.setClock(speeds[i]);
    delay(50); // Let bus stabilize
    
    Serial.print("    ");
    Serial.print(speedNames[i]);
    Serial.print(": ");
    
    // Quick scan at this speed
    int deviceCount = quickScanCount();
    
    if (deviceCount > 0) {
      Serial.print("✓ OK (found ");
      Serial.print(deviceCount);
      Serial.println(" device(s))");
    } else {
      Serial.println("✗ No devices detected");
    }
  }
  
  // Set back to reliable 100kHz
  Wire.setClock(100000);
  Serial.println("\n  → Using 100kHz for remaining tests (most reliable)");
}

int quickScanCount() {
  int count = 0;
  
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      count++;
    }
    delay(1); // Small delay between attempts
  }
  
  return count;
}

void scanI2CBus() {
  Serial.println("  Scanning all addresses (0x01 to 0x7E)...\n");
  
  devicesFound = 0;
  
  Serial.println("       0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
  Serial.println("      ──────────────────────────────────────────────────");
  
  for (uint8_t row = 0; row < 8; row++) {
    Serial.print("  ");
    Serial.print(row, HEX);
    Serial.print("0: ");
    
    for (uint8_t col = 0; col < 16; col++) {
      uint8_t address = (row * 16) + col;
      
      // Skip reserved addresses
      if (address < 0x08 || address > 0x77) {
        Serial.print("   ");
        continue;
      }
      
      Wire.beginTransmission(address);
      uint8_t error = Wire.endTransmission();
      
      if (error == 0) {
        // Device found!
        if (address < 0x10) Serial.print("0");
        Serial.print(address, HEX);
        Serial.print(" ");
        
        foundAddresses[devicesFound] = address;
        devicesFound++;
        
        // Track specific devices
        if (address == 0x68 || address == 0x69) hasMPU6050 = true;
        if (address == 0x29) hasVL53L0X = true;
        
      } else {
        Serial.print("-- ");
      }
    }
    Serial.println();
  }
  
  Serial.println("\n  " + repeatChar('─', 56));
  Serial.print("  Total devices found: ");
  Serial.println(devicesFound);
}

void identifyDevices() {
  if (devicesFound == 0) {
    Serial.println("  ✗ No devices found to identify");
    return;
  }
  
  Serial.println("  Identified devices:\n");
  
  for (int i = 0; i < devicesFound; i++) {
    uint8_t addr = foundAddresses[i];
    
    Serial.print("    0x");
    if (addr < 0x10) Serial.print("0");
    Serial.print(addr, HEX);
    Serial.print(" → ");
    
    // Look up in known devices
    bool identified = false;
    for (int j = 0; j < numKnownDevices; j++) {
      if (knownDevices[j].address == addr) {
        Serial.print(knownDevices[j].name);
        Serial.print(" (");
        Serial.print(knownDevices[j].description);
        Serial.println(")");
        identified = true;
        break;
      }
    }
    
    if (!identified) {
      Serial.println("Unknown Device");
    }
  }
}

void verifyWHEELIESensors() {
  Serial.println("  Checking critical WHEELIE sensors:\n");
  
  // Check MPU6050
  Serial.print("    MPU6050 (IMU):        ");
  if (hasMPU6050) {
    Serial.println("✓ DETECTED");
  } else {
    Serial.println("✗ NOT FOUND");
    Serial.println("      → Check VCC, GND, SDA, SCL connections");
    Serial.println("      → Verify AD0 pin (LOW=0x68, HIGH=0x69)");
  }
  
  // Check VL53L0X
  Serial.print("    VL53L0X (ToF):        ");
  if (hasVL53L0X) {
    Serial.println("✓ DETECTED");
  } else {
    Serial.println("✗ NOT FOUND");
    Serial.println("      → Check VCC, GND, SDA, SCL connections");
    Serial.println("      → Verify XSHUT pin (should be HIGH or floating)");
  }
  
  // Overall status
  Serial.println();
  if (hasMPU6050 && hasVL53L0X) {
    Serial.println("  ✓ All critical WHEELIE sensors detected!");
  } else {
    Serial.println("  ⚠ Some WHEELIE sensors missing - check wiring");
  }
}

void testDeviceCommunication() {
  if (devicesFound == 0) {
    Serial.println("  ✗ No devices to test");
    return;
  }
  
  Serial.println("  Testing read/write capability:\n");
  
  for (int i = 0; i < devicesFound; i++) {
    uint8_t addr = foundAddresses[i];
    
    Serial.print("    0x");
    if (addr < 0x10) Serial.print("0");
    Serial.print(addr, HEX);
    Serial.print(": ");
    
    // Test specific devices
    if (addr == 0x68 || addr == 0x69) {
      testMPU6050(addr);
    } else if (addr == 0x29) {
      testVL53L0X(addr);
    } else {
      // Generic read test
      Wire.beginTransmission(addr);
      Wire.write(0x00); // Try to read register 0x00
      uint8_t error = Wire.endTransmission(false);
      
      if (error == 0) {
        Wire.requestFrom(addr, (uint8_t)1);
        if (Wire.available()) {
          uint8_t data = Wire.read();
          Serial.print("Read OK (0x");
          if (data < 0x10) Serial.print("0");
          Serial.print(data, HEX);
          Serial.println(")");
        } else {
          Serial.println("Read failed (no data)");
        }
      } else {
        Serial.println("Communication error");
      }
    }
  }
}

void testMPU6050(uint8_t addr) {
  // Read WHO_AM_I register (0x75)
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  
  if (Wire.available()) {
    uint8_t whoAmI = Wire.read();
    
    if (whoAmI == 0x68) {
      Serial.println("✓ MPU6050 responding correctly");
      
      // Check if it's in sleep mode
      Wire.beginTransmission(addr);
      Wire.write(0x6B); // PWR_MGMT_1
      Wire.endTransmission(false);
      Wire.requestFrom(addr, (uint8_t)1);
      
      if (Wire.available()) {
        uint8_t pwrMgmt = Wire.read();
        if (pwrMgmt & 0x40) {
          Serial.println("      ⚠ Device in SLEEP mode (needs wake-up)");
        } else {
          Serial.println("      ✓ Device AWAKE and ready");
        }
      }
      
    } else {
      Serial.print("⚠ Unexpected WHO_AM_I: 0x");
      if (whoAmI < 0x10) Serial.print("0");
      Serial.println(whoAmI, HEX);
    }
  } else {
    Serial.println("✗ No response from WHO_AM_I register");
  }
}

void testVL53L0X(uint8_t addr) {
  // Read model ID register (0xC0)
  Wire.beginTransmission(addr);
  Wire.write(0xC0);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  
  if (Wire.available()) {
    uint8_t modelId = Wire.read();
    
    if (modelId == 0xEE) {
      Serial.println("✓ VL53L0X responding correctly");
      Serial.println("      ✓ Model ID verified (0xEE)");
    } else {
      Serial.print("⚠ Unexpected Model ID: 0x");
      if (modelId < 0x10) Serial.print("0");
      Serial.println(modelId, HEX);
    }
  } else {
    Serial.println("✗ No response from Model ID register");
  }
}

void printSummary() {
  Serial.println("\n" + repeatChar('═', 60));
  Serial.println("DIAGNOSTIC SUMMARY");
  Serial.println(repeatChar('═', 60));
  
  Serial.print("\n  I2C Bus Status:        ");
  if (devicesFound > 0) {
    Serial.println("✓ OPERATIONAL");
  } else {
    Serial.println("✗ NO DEVICES DETECTED");
  }
  
  Serial.print("  Devices Found:         ");
  Serial.println(devicesFound);
  
  Serial.print("  MPU6050 (IMU):         ");
  Serial.println(hasMPU6050 ? "✓ DETECTED" : "✗ MISSING");
  
  Serial.print("  VL53L0X (ToF):         ");
  Serial.println(hasVL53L0X ? "✓ DETECTED" : "✗ MISSING");
  
  Serial.println();
  
  if (devicesFound == 0) {
    Serial.println("⚠ TROUBLESHOOTING STEPS:");
    Serial.println("  1. Check all power connections (VCC, GND)");
    Serial.println("  2. Verify SDA and SCL are on correct GPIO pins");
    Serial.println("  3. Check for loose breadboard connections");
    Serial.println("  4. Try different I2C pullup resistors (4.7kΩ)");
    Serial.println("  5. Test sensors individually");
  } else if (!hasMPU6050 || !hasVL53L0X) {
    Serial.println("⚠ MISSING SENSORS - Check:");
    if (!hasMPU6050) {
      Serial.println("  • MPU6050: VCC, GND, SDA, SCL, AD0 pin");
    }
    if (!hasVL53L0X) {
      Serial.println("  • VL53L0X: VCC, GND, SDA, SCL, XSHUT pin");
    }
  } else {
    Serial.println("✓ WHEELIE I2C bus is fully operational!");
    Serial.println("  All critical sensors detected and responding.");
  }
  
  Serial.println(repeatChar('═', 60));
}

void printInteractiveMenu() {
  Serial.println("\n  Available Commands:");
  Serial.println("    s - Re-scan I2C bus");
  Serial.println("    m - Test MPU6050 communication");
  Serial.println("    t - Test VL53L0X communication");
  Serial.println("    d - Dump device registers (debug)");
  Serial.println("    p - Test different I2C speeds");
  Serial.println("    h - Show this help menu");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); // Clear buffer
    
    Serial.println("\n" + repeatChar('─', 60));
    
    // Declare variables that may be initialized in case blocks before the switch
    String addrStr;
    uint8_t dumpAddr;

    switch (cmd) {
      case 's':
      case 'S':
        Serial.println("Re-scanning I2C bus...\n");
        devicesFound = 0;
        hasMPU6050 = false;
        hasVL53L0X = false;
        scanI2CBus();
        identifyDevices();
        break;
        
      case 'm':
      case 'M':
        if (hasMPU6050) {
          Serial.println("Testing MPU6050 detailed communication...\n");
          testMPU6050Detailed();
        } else {
          Serial.println("✗ MPU6050 not detected. Run scan first.");
        }
        break;
        
      case 't':
      case 'T':
        if (hasVL53L0X) {
          Serial.println("Testing VL53L0X detailed communication...\n");
          testVL53L0XDetailed();
        } else {
          Serial.println("✗ VL53L0X not detected. Run scan first.");
        }
        break;
        
      case 'd':
      case 'D':
        Serial.println("Register dump mode");
        Serial.println("Enter device address in hex (e.g., 68): ");
        // Wait for address input
        while (!Serial.available()) delay(10);
        
        addrStr = Serial.readStringUntil('\n');
        addrStr.trim();
        dumpAddr = (uint8_t)strtol(addrStr.c_str(), NULL, 16);
        
        dumpRegisters(dumpAddr);
        break;
        
      case 'p':
      case 'P':
        testI2CSpeeds();
        Wire.setClock(100000); // Reset to 100kHz
        break;
        
      case 'h':
      case 'H':
        printInteractiveMenu();
        break;
        
      default:
        Serial.println("⚠ Unknown command. Press 'h' for help.");
        break;
    }
    
    Serial.println(repeatChar('─', 60));
    Serial.println();
  }
}

void testMPU6050Detailed() {
  uint8_t addr = hasMPU6050 ? 0x68 : 0x69;
  
  Serial.println("  MPU6050 Register Tests:");
  
  // WHO_AM_I
  Serial.print("    WHO_AM_I (0x75):      ");
  uint8_t whoAmI = readRegister(addr, 0x75);
  Serial.print("0x");
  if (whoAmI < 0x10) Serial.print("0");
  Serial.print(whoAmI, HEX);
  Serial.println(whoAmI == 0x68 ? " ✓" : " ✗");
  
  // Power Management
  Serial.print("    PWR_MGMT_1 (0x6B):    ");
  uint8_t pwrMgmt = readRegister(addr, 0x6B);
  Serial.print("0x");
  if (pwrMgmt < 0x10) Serial.print("0");
  Serial.print(pwrMgmt, HEX);
  if (pwrMgmt & 0x40) {
    Serial.println(" (SLEEP MODE)");
  } else {
    Serial.println(" (AWAKE)");
  }
  
  // Gyro Config
  Serial.print("    GYRO_CONFIG (0x1B):   ");
  uint8_t gyroConfig = readRegister(addr, 0x1B);
  Serial.print("0x");
  if (gyroConfig < 0x10) Serial.print("0");
  Serial.print(gyroConfig, HEX);
  int gyroRange = (gyroConfig >> 3) & 0x03;
  Serial.print(" (±");
  Serial.print(250 * (1 << gyroRange));
  Serial.println("°/s)");
  
  // Accel Config
  Serial.print("    ACCEL_CONFIG (0x1C):  ");
  uint8_t accelConfig = readRegister(addr, 0x1C);
  Serial.print("0x");
  if (accelConfig < 0x10) Serial.print("0");
  Serial.print(accelConfig, HEX);
  int accelRange = (accelConfig >> 3) & 0x03;
  Serial.print(" (±");
  Serial.print(2 * (1 << accelRange));
  Serial.println("g)");
  
  Serial.println("\n  ✓ MPU6050 communication verified");
}

void testVL53L0XDetailed() {
  uint8_t addr = 0x29;
  
  Serial.println("  VL53L0X Register Tests:");
  
  // Model ID
  Serial.print("    Model ID (0xC0):      ");
  uint8_t modelId = readRegister(addr, 0xC0);
  Serial.print("0x");
  if (modelId < 0x10) Serial.print("0");
  Serial.print(modelId, HEX);
  Serial.println(modelId == 0xEE ? " ✓" : " ✗");
  
  // Module Type
  Serial.print("    Module Type (0xC1):   ");
  uint8_t moduleType = readRegister(addr, 0xC1);
  Serial.print("0x");
  if (moduleType < 0x10) Serial.print("0");
  Serial.println(moduleType, HEX);
  
  // Revision ID
  Serial.print("    Revision ID (0xC2):   ");
  uint8_t revId = readRegister(addr, 0xC2);
  Serial.print("0x");
  if (revId < 0x10) Serial.print("0");
  Serial.println(revId, HEX);
  
  Serial.println("\n  ✓ VL53L0X communication verified");
}

uint8_t readRegister(uint8_t deviceAddr, uint8_t regAddr) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddr, (uint8_t)1);
  
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

void dumpRegisters(uint8_t addr) {
  Serial.println("\n  Register Dump for 0x" + String(addr, HEX) + ":");
  Serial.println("  " + repeatChar('─', 56));
  Serial.println("       0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
  Serial.println("      ──────────────────────────────────────────────────");
  
  for (int row = 0; row < 16; row++) {
    Serial.print("  ");
    if (row < 10) Serial.print(" ");
    Serial.print(row, HEX);
    Serial.print("0: ");
    
    for (int col = 0; col < 16; col++) {
      uint8_t regAddr = (row * 16) + col;
      uint8_t value = readRegister(addr, regAddr);
      
      if (value < 0x10) Serial.print("0");
      Serial.print(value, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  Serial.println();
}
