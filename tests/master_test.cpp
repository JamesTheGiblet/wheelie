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

// --- Buzzer Test (Non-blocking) ---
void runBuzzerTest(bool begin, bool &done) {
  static int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
  static int note = 0;
  static unsigned long noteStart = 0;
  static bool playing = false;
  if (begin) {
    Serial.println("[TEST] KY-006 Passive Buzzer");
    note = 0;
    noteStart = millis();
    playing = true;
    tone(BUZZER_PIN, melody[note], 120);
  }
  if (playing && millis() - noteStart >= 150) {
    note++;
    if (note < 4) {
      tone(BUZZER_PIN, melody[note], 120);
      noteStart = millis();
    } else {
      noTone(BUZZER_PIN);
      playing = false;
      done = true;
    }
  }
}

// --- Motors Test (Stub) ---
// --- Motors Control Functions ---
void setLeftMotor(int pwmA, int pwmB) {
  analogWrite(IN1_PIN, pwmA);
  analogWrite(IN2_PIN, pwmB);
}
void setRightMotor(int pwmA, int pwmB) {
  analogWrite(IN3_PIN, pwmA);
  analogWrite(IN4_PIN, pwmB);
}
void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
}

// --- Motors Test (Non-blocking) ---
void runMotorsTest(bool begin, bool &done) {
  static int phase = 0;
  static unsigned long phaseStart = 0;
  if (begin) {
    Serial.println("\n[TEST] Motors");
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    stopMotors();
    phase = 0;
    phaseStart = millis();
    Serial.println("Left motor forward");
    setLeftMotor(200, 0);
  }
  if (phase == 0 && millis() - phaseStart > 600) {
    Serial.println("Left motor reverse");
    setLeftMotor(0, 200);
    phase = 1;
    phaseStart = millis();
  } else if (phase == 1 && millis() - phaseStart > 600) {
    setLeftMotor(0, 0);
    Serial.println("Right motor forward");
    setRightMotor(200, 0);
    phase = 2;
    phaseStart = millis();
  } else if (phase == 2 && millis() - phaseStart > 600) {
    Serial.println("Right motor reverse");
    setRightMotor(0, 200);
    phase = 3;
    phaseStart = millis();
  } else if (phase == 3 && millis() - phaseStart > 600) {
    setRightMotor(0, 0);
    done = true;
  }
}

// --- Encoders Test (Stub) ---
// --- Encoder Interrupt Handlers ---
volatile long leftCount = 0;
volatile long rightCount = 0;
volatile int8_t leftDir = 0;
volatile int8_t rightDir = 0;
volatile unsigned long leftLastTime = 0;
volatile unsigned long rightLastTime = 0;

void onLeftEncoder() {
  int b = digitalRead(ENCODER_B_PIN);
  leftDir = b ? 1 : -1;
  leftCount += leftDir;
  leftLastTime = micros();
}
void onRightEncoder() {
  int a = digitalRead(ENCODER_A_PIN);
  rightDir = a ? 1 : -1;
  rightCount += rightDir;
  rightLastTime = micros();
}

// --- Encoders Test (Non-blocking) ---
void runEncodersTest(bool begin, bool &done) {
  static bool attached = false;
  if (begin) {
    Serial.println("\n[TEST] LM393 H2010 Encoders");
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    if (!attached) {
      attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onLeftEncoder, RISING);
      attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onRightEncoder, RISING);
      attached = true;
    }
    Serial.println("Move wheels by hand. Counts, direction, and timing will be displayed.");
    Serial.println("Debug: Stuck detection every 2s, direction: +1=forward, -1=reverse");
  }
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Left: "); Serial.print(leftCount);
    Serial.print(" (dir "); Serial.print(leftDir); Serial.print(")  ");
    Serial.print("Right: "); Serial.print(rightCount);
    Serial.print(" (dir "); Serial.print(rightDir); Serial.print(")  ");
    Serial.println();
    lastPrint = millis();
  }
  // End after 4 seconds
  static unsigned long start = 0;
  if (begin) start = millis();
  if (millis() - start > 4000) done = true;
}

// --- RGB LED Test (Stub) ---
// --- RGB LED Test (Non-blocking) ---
void runRGBLEDTest(bool begin, bool &done) {
  static int phase = 0;
  static unsigned long phaseStart = 0;
  if (begin) {
    Serial.println("[TEST] KY-009 RGB LED");
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    phase = 0;
    phaseStart = millis();
    analogWrite(LED_RED_PIN, 255);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
    Serial.println("  - Red");
  }
  if (phase == 0 && millis() - phaseStart > 600) {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 255);
    analogWrite(LED_BLUE_PIN, 0);
    Serial.println("  - Green");
    phase = 1;
    phaseStart = millis();
  } else if (phase == 1 && millis() - phaseStart > 600) {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 255);
    Serial.println("  - Blue");
    phase = 2;
    phaseStart = millis();
  } else if (phase == 2 && millis() - phaseStart > 600) {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
    done = true;
  }
}

// --- Sound Sensor Test (Stub) ---
// --- Sound Sensor Test (Non-blocking) ---
void runSoundSensorTest(bool begin, bool &done) {
  static unsigned long start = 0;
  if (begin) {
    Serial.println("\n[TEST] LM393 Sound Sensor");
    pinMode(SOUND_SENSOR_PIN, INPUT);
    Serial.println("Digital value: 0 = quiet, 1 = sound detected");
    start = millis();
  }
  int sound = digitalRead(SOUND_SENSOR_PIN);
  Serial.print("  - Sound Detected: ");
  Serial.println(sound);
  if (millis() - start > 1000) done = true;
}

// --- ToF Sensor Test (Stub) ---
// --- ToF Sensor Test (Non-blocking) ---
#include <Wire.h>
#include <VL53L0X.h>
void runToFTest(bool begin, bool &done) {
  static VL53L0X tof;
  static bool tofInitialized = false;
  static unsigned long lastRead = 0;
  if (begin) {
    Serial.println("\n[TEST] VL53L0X ToF Sensor");
    if (!tofInitialized) {
      Wire.begin(I2C_SDA, I2C_SCL);
      tof.setTimeout(500);
      if (tof.init()) {
        Serial.println("✅ ToF sensor initialized successfully.");
        tofInitialized = true;
      } else {
        Serial.println("❌ ToF sensor initialization failed!");
        done = true;
        return;
      }
    }
    lastRead = millis();
  }
  if (millis() - lastRead > 300) {
    int dist = tof.readRangeSingleMillimeters();
    if (tof.timeoutOccurred()) {
      Serial.println("  - [ToF] Timeout!");
    } else {
      Serial.print("  - [ToF] Distance: ");
      Serial.print(dist);
      Serial.println(" mm");
    }
    done = true;
  }
}

// --- MPU6050 Test (Stub) ---
#include <Wire.h>
#include <MPU6050_light.h>

// --- MPU6050 Test (Non-blocking) ---
#include <MPU6050_light.h>
void runMPU6050Test(bool begin, bool &done) {
  static MPU6050 mpu(Wire);
  static bool mpuInitialized = false;
  static unsigned long lastRead = 0;
  if (begin) {
    Serial.println("\n[TEST] MPU6050 IMU");
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
        done = true;
        return;
      }
      Serial.println("Calibrating IMU...");
      mpu.calcOffsets(true, true); // gyro and accel
      Serial.println("IMU calibration complete.");
    }
    lastRead = millis();
  }
  if (millis() - lastRead > 300) {
    mpu.update();
    Serial.print("  - Angle X: "); Serial.print(mpu.getAngleX());
    Serial.print(" | Angle Y: "); Serial.print(mpu.getAngleY());
    Serial.print(" | Angle Z: "); Serial.println(mpu.getAngleZ());
    done = true;
  }
}

// --- WiFi Test (Stub) ---
// --- WiFi Test (Non-blocking) ---
void runWiFiTest(bool begin, bool &done) {
  static bool wifiInitialized = false;
  static unsigned long startAttempt = 0;
  if (begin) {
    Serial.println("\n[TEST] WiFi Connection");
    wifiInitialized = false;
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    startAttempt = millis();
  }
  if (!wifiInitialized && WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());
    wifiInitialized = true;
    done = true;
  } else if (!wifiInitialized && millis() - startAttempt > 10000) {
    Serial.println();
    Serial.println("[ERROR] WiFi connection timed out.");
    done = true;
  } else if (!wifiInitialized) {
    if ((millis() - startAttempt) % 500 < 50) Serial.print(".");
  } else {
    Serial.print("  - IP: "); Serial.println(WiFi.localIP());
    Serial.print("  - RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    done = true;
  }
}

// --- ESP-NOW Test (Non-blocking) ---
void runESPNOWTest(bool begin, bool &done) {
  static bool espnowInitialized = false;
  static int sendSuccessCount = 0;
  static int sendFailCount = 0;
  static int receivedCount = 0;
  static char lastReceivedData[64] = {0};
  static uint8_t lastReceivedMac[6] = {0};
  static int lastReceivedLen = 0;
  static unsigned long lastSend = 0;
  static bool sent = false;
  static bool cbRegistered = false;
  static const char *testMsg = "Hello ESP-NOW!";
  if (begin) {
    Serial.println("\n[TEST] ESP-NOW");
    WiFi.mode(WIFI_STA);
    if (!espnowInitialized) {
      if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Error initializing ESP-NOW");
        done = true;
        return;
      }
      espnowInitialized = true;
    }
    if (!cbRegistered) {
      esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status) {
        if (status == ESP_NOW_SEND_SUCCESS) {
          sendSuccessCount++;
          Serial.print("[ESP-NOW] Last Packet Send Status: Success\n");
        } else {
          sendFailCount++;
          Serial.print("[ESP-NOW] Last Packet Send Status: Fail\n");
        }
      });
      esp_now_register_recv_cb([](const uint8_t *mac_addr, const uint8_t *data, int len) {
        receivedCount++;
        int copyLen = len > 63 ? 63 : len;
        memcpy(lastReceivedData, data, copyLen);
        lastReceivedData[copyLen] = '\0';
        Serial.print("  - [ESP-NOW] Data received | Data: ");
        Serial.print(lastReceivedData);
        Serial.print(" | Length: ");
        Serial.println(copyLen);
      });
      cbRegistered = true;
    }
    esp_now_peer_info_t peerInfo = {};
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    lastSend = millis();
    sent = false;
  }
  if (!sent && millis() - lastSend > 300) {
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)testMsg, strlen(testMsg));
    if (result == ESP_OK) {
      Serial.println("  - [ESP-NOW] Test message sent (broadcast)");
    } else {
      Serial.print("  - [ESP-NOW] Error sending test message: ");
      Serial.println(result);
    }
    sent = true;
    lastSend = millis();
  }
  if (sent && millis() - lastSend > 600) {
    Serial.print("  - Send Success: "); Serial.print(sendSuccessCount);
    Serial.print(" | Send Fail: "); Serial.print(sendFailCount);
    Serial.print(" | Received: "); Serial.println(receivedCount);
    done = true;
  }
}

// --- OTA Test (Stub) ---
// --- OTA Test (Non-blocking, still placeholder) ---
void runOTATest(bool begin, bool &done) {
  if (begin) {
    Serial.println("\n[TEST] OTA (Over-The-Air Update)");
    Serial.println("This is a placeholder for OTA update test.");
    Serial.println("Implement OTA logic here (e.g., ArduinoOTA, HTTPUpdate, etc.)");
    done = true;
  }
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
  Serial.println("[MASTER TEST] Starting hardware and connection tests...");
}

void loop() {
  static TestStep lastTest = TEST_DONE;
  static bool beginStep = true;
  static bool doneStep = false;
  static unsigned long lastStepEnd = 0;
  if (currentTest == TEST_DONE) {
    Serial.println("\n[MASTER TEST] All tests complete. Restart to run again.");
    while (1) { delay(1000); }
  }
  if (beginStep) {
    // Stop previous test's hardware if necessary
    stopMotors();
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
    doneStep = false;
  }
  switch (currentTest) {
    case TEST_BUZZER:
      runBuzzerTest(beginStep, doneStep);
      break;
    case TEST_RGB_LED:
      runRGBLEDTest(beginStep, doneStep);
      break;
    case TEST_MOTORS:
      runMotorsTest(beginStep, doneStep);
      break;
    case TEST_ENCODERS:
      runEncodersTest(beginStep, doneStep);
      break;
    case TEST_SOUND_SENSOR:
      runSoundSensorTest(beginStep, doneStep);
      break;
    case TEST_TOF:
      runToFTest(beginStep, doneStep);
      break;
    case TEST_MPU6050:
      runMPU6050Test(beginStep, doneStep);
      break;
    case TEST_WIFI:
      runWiFiTest(beginStep, doneStep);
      break;
    case TEST_ESPNOW:
      runESPNOWTest(beginStep, doneStep);
      break;
    case TEST_OTA:
      runOTATest(beginStep, doneStep);
      break;
    default:
      break;
  }
  if (doneStep) {
    lastTest = currentTest;
    currentTest = (TestStep)((int)currentTest + 1);
    beginStep = true;
    lastStepEnd = millis();
    delay(10); // allow serial flush
  } else {
    if (beginStep) beginStep = false;
    // Add a small delay between steps
    if (millis() - lastStepEnd < 200) return;
  }
}
