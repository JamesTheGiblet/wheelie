#include <Arduino.h>
#include "pins.h"
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
unsigned long testStartTime = 0;
const unsigned long testDuration = 5000; // ms per test

// --- Buzzer Test ---
void runBuzzerTest() {
  Serial.println("[TEST] KY-006 Passive Buzzer");
  tone(BUZZER_PIN, 2000, 200);
  delay(500);
  int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
  for (int i = 0; i < 8; i++) {
    tone(BUZZER_PIN, melody[i], 120);
    delay(150);
  }
  delay(1000);
}

// --- Motors Test (Stub) ---
void runMotorsTest() {
  Serial.println("[TEST] Motors");
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
  setRightMotor(0, 0);
  delay(1500);
  stopMotors();
  delay(500);
  // Left motor reverse
  Serial.println("Left motor reverse");
  setLeftMotor(0, 200);
  setRightMotor(0, 0);
  delay(1500);
  stopMotors();
  delay(500);
  // Right motor forward
  Serial.println("Right motor forward");
  setLeftMotor(0, 0);
  setRightMotor(200, 0);
  delay(1500);
  stopMotors();
  delay(500);
  // Right motor reverse
  Serial.println("Right motor reverse");
  setLeftMotor(0, 0);
  setRightMotor(0, 200);
  delay(1500);
  stopMotors();
  delay(500);
  // Both motors forward
  Serial.println("Both motors forward");
  setLeftMotor(200, 0);
  setRightMotor(200, 0);
  delay(1500);
  stopMotors();
  delay(500);
  // Both motors reverse
  Serial.println("Both motors reverse");
  setLeftMotor(0, 200);
  setRightMotor(0, 200);
  delay(1500);
  stopMotors();
  delay(500);
  Serial.println("Motor test complete.");
  stopMotors();
  delay(1000);
}

// --- Encoders Test (Stub) ---
void runEncodersTest() {
  Serial.println("[TEST] LM393 H2010 Encoders");
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
  unsigned long start = millis();
  while (millis() - start < 4000) { // Run for 4 seconds
    if (millis() - lastPrint > 500) {
      Serial.print("Left: "); Serial.print(leftCount);
      Serial.print(" (dir "); Serial.print(leftDir); Serial.print(")  ");
      Serial.print("Right: "); Serial.print(rightCount);
      Serial.print(" (dir "); Serial.print(rightDir); Serial.print(")  ");
      Serial.print("Left dt: "); Serial.print(micros() - leftLastTime);
      Serial.print(" us  Right dt: "); Serial.print(micros() - rightLastTime); Serial.println(" us");
      lastPrint = millis();
    }
    delay(10);
  }
  Serial.println("Encoder test complete.");
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
  Serial.println("Red");
  delay(700);
  // Green
  setColor(0, 255, 0);
  Serial.println("Green");
  delay(700);
  // Blue
  setColor(0, 0, 255);
  Serial.println("Blue");
  delay(700);
  // Yellow
  setColor(255, 255, 0);
  Serial.println("Yellow");
  delay(700);
  // Cyan
  setColor(0, 255, 255);
  Serial.println("Cyan");
  delay(700);
  // Magenta
  setColor(255, 0, 255);
  Serial.println("Magenta");
  delay(700);
  // White
  setColor(255, 255, 255);
  Serial.println("White");
  delay(700);
  // Off
  setColor(0, 0, 0);
  Serial.println("Off");
  delay(500);
  Serial.println("RGB LED test complete.");
}

// --- Sound Sensor Test (Stub) ---
void runSoundSensorTest() {
  Serial.println("[TEST] LM393 Sound Sensor");
  pinMode(SOUND_SENSOR_PIN, INPUT);
  Serial.println("Digital value: 0 = quiet, 1 = sound detected");
  unsigned long start = millis();
  while (millis() - start < 3000) { // Run for 3 seconds
    int sound = digitalRead(SOUND_SENSOR_PIN);
    Serial.print("Sound Detected: ");
    Serial.println(sound);
    delay(200);
  }
  Serial.println("Sound sensor test complete.");
}

// --- ToF Sensor Test (Stub) ---
void runToFTest() {
  Serial.println("[TEST] VL53L0X ToF Sensor");
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
  // Take 5 readings
  for (int i = 0; i < 5; i++) {
    int dist = tof.readRangeSingleMillimeters();
    if (tof.timeoutOccurred()) {
      Serial.println("[ToF] Timeout!");
    } else {
      Serial.print("[ToF] Distance: ");
      Serial.print(dist);
      Serial.println(" mm");
    }
    delay(400);
  }
  Serial.println("ToF sensor test complete.");
}

// --- MPU6050 Test (Stub) ---
void runMPU6050Test() {
  Serial.println("[TEST] MPU6050 IMU");
  #include <Wire.h>
  #include <MPU6050_light.h>
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
  // Print 5 readings
  for (int i = 0; i < 5; i++) {
    mpu.update();
    Serial.print("Angle X: "); Serial.print(mpu.getAngleX());
    Serial.print(" | Angle Y: "); Serial.print(mpu.getAngleY());
    Serial.print(" | Angle Z: "); Serial.println(mpu.getAngleZ());
    delay(400);
  }
  Serial.println("MPU6050 test complete.");
}

// --- WiFi Test (Stub) ---
void runWiFiTest() {
  Serial.println("[TEST] WiFi Connection");
  #include "credentials.h"
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
  // Print status and RSSI
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.print("Reconnects: ");
  Serial.println(reconnectAttempts);
  delay(2000);
}

void runESPNOWTest() {
  Serial.println("[TEST] ESP-NOW");
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
  auto OnDataSent = [](const uint8_t *mac_addr, esp_now_send_status_t status) {
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
  auto OnDataRecv = [](const uint8_t *mac_addr, const uint8_t *data, int len) {
    receivedCount++;
    memcpy(lastReceivedMac, mac_addr, 6);
    lastReceivedLen = len > 63 ? 63 : len;
    memcpy(lastReceivedData, data, lastReceivedLen);
    lastReceivedData[lastReceivedLen] = '\0';
    Serial.print("[ESP-NOW] Data received from: ");
    printMac(mac_addr);
    Serial.print(" | Data: ");
    Serial.print(lastReceivedData);
    Serial.print(" | Length: ");
    Serial.println(lastReceivedLen);
  };

  if (!espnowInitialized) {
    WiFi.mode(WIFI_STA);
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
    Serial.println("[ESP-NOW] Test message sent (broadcast)");
  } else {
    Serial.print("[ESP-NOW] Error sending test message: ");
    Serial.println(result);
  }
  delay(1000);
  Serial.print("Send Success: "); Serial.println(sendSuccessCount);
  Serial.print("Send Fail: "); Serial.println(sendFailCount);
  Serial.print("Received Count: "); Serial.println(receivedCount);
  if (receivedCount > 0) {
    Serial.print("Last Received From: "); printMac(lastReceivedMac); Serial.println();
    Serial.print("Last Received Data: "); Serial.println(lastReceivedData);
    Serial.print("Last Received Length: "); Serial.println(lastReceivedLen);
  }
  Serial.println("ESP-NOW test complete.");
}

// --- OTA Test (Stub) ---
void runOTATest() {
  Serial.println("[TEST] OTA (Over-The-Air Update)");
  Serial.println("This is a placeholder for OTA update test.");
  Serial.println("Implement OTA logic here (e.g., ArduinoOTA, HTTPUpdate, etc.)");
  delay(2000);
  Serial.println("OTA test complete.");
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
  testStartTime = millis();
}

void loop() {
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
      Serial.println("[MASTER TEST] All tests complete. Restart to run again.");
      while (1) delay(1000);
      break;
  }
  delay(500); // Small delay between tests
  currentTest = (TestStep)((int)currentTest + 1);
  if (currentTest == TEST_DONE) {
    currentTest = TEST_DONE;
  }
  testStartTime = millis();
}
