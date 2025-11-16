#include <Arduino.h>
#include "pins.h"
#include "credentials.h" // For WiFi test
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
// Add other required sensor/actuator libraries here

// --- Test Step Enum ---
enum TestStep {
  TEST_BUZZER,
  TEST_RGB_LED,
  TEST_MOTORS,
  TEST_ENCODERS,
  TEST_SOUND_SENSOR,
  TEST_TOF,
  TEST_MPU6050,
  TEST_WIFI,
  TEST_ESPNOW,
  TEST_OTA,
  TEST_DONE
};

TestStep currentTest = TEST_BUZZER;
unsigned long nextStepTime = 0;
const unsigned long stepDelay = 2000; // 2 seconds between tests

// --- Buzzer Test ---
void runBuzzerTest() {
  Serial.println("[TEST] KY-006 Passive Buzzer");
  tone(BUZZER_PIN, 2000, 200);
  // Non-blocking melody would be better, but for a simple test this is okay.
  int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
  for (int i = 0; i < 4; i++) { // Shorten for non-blocking test
    tone(BUZZER_PIN, melody[i], 120);
    delay(150);
  }
  delay(1000);
}

// --- Motors Test (Stub) ---
void runMotorsTest() {
  Serial.println("\n[TEST] Motors");
  // Minimal test for dual motor driver (FASIZI L298N or MOSFET H-Bridge)
  // Cycles each motor forward, reverse, and stop at different speeds
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  auto setLeftMotor = [](int pwmA, int pwmB) {
    analogWrite(IN1_PIN, pwmA);
    analogWrite(IN2_PIN, pwmB);
  };
  auto setRightMotor = [](int pwmA, int pwmB) {
    analogWrite(IN3_PIN, pwmA);
    analogWrite(IN4_PIN, pwmB);
  };
  auto stopMotors = [&]() {
    setLeftMotor(0, 0);
    setRightMotor(0, 0);
  };
  stopMotors();
  // Left motor forward
  Serial.println("Left motor forward");
  setLeftMotor(200, 0);
  // The non-blocking loop will handle stopping it.
}

// --- Encoders Test (Stub) ---
void runEncodersTest() {
  Serial.println("\n[TEST] LM393 H2010 Encoders");
  // Minimal test for LM393 H2010 encoders (manual wheel movement)
  static volatile long leftCount = 0;
  static volatile long rightCount = 0;
  static volatile int8_t leftDir = 0;
  static volatile int8_t rightDir = 0;
  static volatile unsigned long leftLastTime = 0;
  static volatile unsigned long rightLastTime = 0;

  // For direction: sample B pin on A edge (simple, not quadrature)
  auto onLeftEncoder = []() {
    int b = digitalRead(ENCODER_B_PIN);
    leftDir = b ? 1 : -1;
    leftCount += leftDir;
    leftLastTime = micros();
  };
  auto onRightEncoder = []() {
    int a = digitalRead(ENCODER_A_PIN);
    rightDir = a ? 1 : -1;
    rightCount += rightDir;
    rightLastTime = micros();
  };

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  // Attach interrupts only once
  static bool attached = false;
  if (!attached) {
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onRightEncoder, RISING);
    attached = true;
  }
  Serial.println("Move wheels by hand. Counts, direction, and timing will be displayed.");
  Serial.println("Debug: Stuck detection every 2s, direction: +1=forward, -1=reverse");

  static long lastLeft = 0, lastRight = 0;
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Left: "); Serial.print(leftCount);
    Serial.print(" (dir "); Serial.print(leftDir); Serial.print(")  ");
    Serial.print("Right: "); Serial.print(rightCount);
    Serial.print(" (dir "); Serial.print(rightDir); Serial.print(")  ");
    Serial.println();
    lastPrint = millis();
  }
}

// --- RGB LED Test (Stub) ---
void runRGBLEDTest() {
  Serial.println("[TEST] KY-009 RGB LED");
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  auto setColor = [](int r, int g, int b) {
    analogWrite(LED_RED_PIN, r);
    analogWrite(LED_GREEN_PIN, g);
    analogWrite(LED_BLUE_PIN, b);
  };
  // Red
  setColor(255, 0, 0);
  Serial.println("  - Red");
}

// --- Sound Sensor Test (Stub) ---
void runSoundSensorTest() {
  Serial.println("\n[TEST] LM393 Sound Sensor");
  pinMode(SOUND_SENSOR_PIN, INPUT);
  Serial.println("Digital value: 0 = quiet, 1 = sound detected");
  int sound = digitalRead(SOUND_SENSOR_PIN);
  Serial.print("  - Sound Detected: ");
  Serial.println(sound);
}

// --- ToF Sensor Test (Stub) ---
void runToFTest() {
  Serial.println("\n[TEST] VL53L0X ToF Sensor");
  #include <Wire.h>
  #include <VL53L0X.h>
  static VL53L0X tof;
  static bool tofInitialized = false;
  if (!tofInitialized) {
    Wire.begin(I2C_SDA, I2C_SCL);
    tof.setTimeout(500);
    if (tof.init()) {
      Serial.println("✅ ToF sensor initialized successfully.");
      tofInitialized = true;
    } else {
      Serial.println("❌ ToF sensor initialization failed!");
      return;
    }
  }
  int dist = tof.readRangeSingleMillimeters();
  if (tof.timeoutOccurred()) {
    Serial.println("  - [ToF] Timeout!");
  } else {
    Serial.print("  - [ToF] Distance: ");
    Serial.print(dist);
    Serial.println(" mm");
  }
}

// --- MPU6050 Test (Stub) ---
#include <Wire.h>
#include <MPU6050_light.h>

void runMPU6050Test() {
  Serial.println("\n[TEST] MPU6050 IMU");
  static MPU6050 mpu(Wire);
  static bool mpuInitialized = false;
  if (!mpuInitialized) {
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("Initializing MPU6050 IMU...");
    byte status = mpu.begin();
    if (status == 0) {
      Serial.println("✅ IMU initialized successfully.");
      mpuInitialized = true;
    } else {
      Serial.print("❌ IMU initialization failed! Status: ");
      Serial.println(status);
      return;
    }
    Serial.println("Calibrating IMU...");
    mpu.calcOffsets(true, true); // gyro and accel
    Serial.println("IMU calibration complete.");
  }
  mpu.update();
  Serial.print("  - Angle X: "); Serial.print(mpu.getAngleX());
  Serial.print(" | Angle Y: "); Serial.print(mpu.getAngleY());
  Serial.print(" | Angle Z: "); Serial.println(mpu.getAngleZ());
}

// --- WiFi Test (Stub) ---
void runWiFiTest() {
  Serial.println("\n[TEST] WiFi Connection");
  // Credentials are now included at the top
  static bool wifiInitialized = false;
  static int reconnectAttempts = 0;
  static unsigned long lastReconnectAttempt = 0;
  if (!wifiInitialized) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      wifiInitialized = true;
    } else {
      Serial.println();
      Serial.println("[ERROR] WiFi connection timed out.");
      return;
    }
  }
  Serial.print("  - IP: "); Serial.println(WiFi.localIP());
  Serial.print("  - RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
}

void runESPNOWTest() {
  Serial.println("\n[TEST] ESP-NOW");
  // Ensure WiFi is in STA mode for ESP-NOW
  WiFi.mode(WIFI_STA);
  static bool espnowInitialized = false;
  static int sendSuccessCount = 0;
  static int sendFailCount = 0;
  static int receivedCount = 0;
  static char lastReceivedData[64] = {0};
  static uint8_t lastReceivedMac[6] = {0};
  static int lastReceivedLen = 0;

  auto printMac = [](const uint8_t *mac) {
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", mac[i]);
      if (i < 5) Serial.print(":");
    }
  };
  auto OnDataSent = [printMac](const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
      sendSuccessCount++;
      Serial.print("[ESP-NOW] Last Packet Send Status: Success to ");
    } else {
      sendFailCount++;
      Serial.print("[ESP-NOW] Last Packet Send Status: Fail to ");
    }
    printMac(mac_addr);
    Serial.println();
  };
  auto OnDataRecv = [printMac](const uint8_t *mac_addr, const uint8_t *data, int len) {
    receivedCount++;
    memcpy(lastReceivedMac, mac_addr, 6);
    lastReceivedLen = len > 63 ? 63 : len;
    memcpy(lastReceivedData, data, lastReceivedLen);
    lastReceivedData[lastReceivedLen] = '\0';
    Serial.print("  - [ESP-NOW] Data received from: ");
    printMac(mac_addr);
    Serial.print(" | Data: ");
    Serial.print(lastReceivedData);
    Serial.print(" | Length: ");
    Serial.println(lastReceivedLen);
  };

  if (!espnowInitialized) {
    if (esp_now_init() != ESP_OK) {
      Serial.println("[ESP-NOW] Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peerInfo = {};
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ESP-NOW] Failed to add broadcast peer");
      return;
    }
    Serial.println("[ESP-NOW] Broadcast peer added");
    espnowInitialized = true;
  }
  // Send test data as broadcast
  const char *testMsg = "Hello ESP-NOW!";
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)testMsg, strlen(testMsg));
  if (result == ESP_OK) {
    Serial.println("  - [ESP-NOW] Test message sent (broadcast)");
  } else {
    Serial.print("  - [ESP-NOW] Error sending test message: ");
    Serial.println(result);
  }
  Serial.print("  - Send Success: "); Serial.print(sendSuccessCount);
  Serial.print(" | Send Fail: "); Serial.print(sendFailCount);
  Serial.print(" | Received: "); Serial.println(receivedCount);
}

// --- OTA Test (Stub) ---
void runOTATest() {
  Serial.println("\n[TEST] OTA (Over-The-Air Update)");
  Serial.println("This is a placeholder for OTA update test.");
  Serial.println("Implement OTA logic here (e.g., ArduinoOTA, HTTPUpdate, etc.)");
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  // ToF and MPU6050 use I2C, initialized in their test functions
  Serial.println("[MASTER TEST] Starting hardware and connection tests...");
  nextStepTime = millis() + stepDelay;
}

void loop() {
  if (millis() >= nextStepTime) {
    // Stop previous test's hardware if necessary
    stopMotors();
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);

    switch (currentTest) {
      case TEST_BUZZER:
        runBuzzerTest();
        break;
      case TEST_RGB_LED:
        runRGBLEDTest();
        break;
      case TEST_MOTORS:
        runMotorsTest();
        break;
      case TEST_ENCODERS:
        runEncodersTest();
        break;
      case TEST_SOUND_SENSOR:
        runSoundSensorTest();
        break;
      case TEST_TOF:
        runToFTest();
        break;
      case TEST_MPU6050:
        runMPU6050Test();
        break;
      case TEST_WIFI:
        runWiFiTest();
        break;
      case TEST_ESPNOW:
        runESPNOWTest();
        break;
      case TEST_OTA:
        runOTATest();
        break;
      case TEST_DONE:
        Serial.println("\n[MASTER TEST] All tests complete. Restart to run again.");
        while (1) { delay(1000); } // Halt at the end
        break;
    }
    
    // Schedule the next test step
    currentTest = (TestStep)((int)currentTest + 1);
    nextStepTime = millis() + stepDelay;
  }

  // Handle continuous tests like encoders
  if (currentTest == TEST_ENCODERS) {
      runEncodersTest();
  }
}
