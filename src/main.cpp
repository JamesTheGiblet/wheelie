#define ESPNOW_MAX_RETRIES 5
#define ESPNOW_STATUS_INTERVAL 5000

void sendEspNowTestMessage();
#include <WiFi.h>
#include <esp_now.h>

// --- Robust ESP-NOW Debugging and Fallbacks ---
#define ESPNOW_MAX_RETRIES 5
#define ESPNOW_STATUS_INTERVAL 5000

volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
volatile unsigned long lastSendTime = 0;
volatile int sendRetries = 0;
volatile int sendSuccessCount = 0;
volatile int sendFailCount = 0;
volatile int receivedCount = 0;
char lastReceivedData[64] = {0};
uint8_t lastReceivedMac[6] = {0};
int lastReceivedLen = 0;

void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  lastSendStatus = status;
  lastSendTime = millis();
  if (status == ESP_NOW_SEND_SUCCESS) {
    sendSuccessCount++;
    sendRetries = 0;
    Serial.print("[ESP-NOW] Last Packet Send Status: Success to ");
    printMac(mac_addr);
    Serial.println();
  } else {
    sendFailCount++;
    Serial.print("[ESP-NOW] Last Packet Send Status: Fail to ");
    printMac(mac_addr);
    Serial.println();
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
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
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println("[ESP-NOW] WiFi set to STA mode");

  // Print MAC address
  Serial.print("[ESP-NOW] MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Error initializing ESP-NOW");
    return;
  }
  Serial.println("[ESP-NOW] ESP-NOW Initialized");

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register broadcast peer
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

  // Initial send
  sendEspNowTestMessage();
}

void sendEspNowTestMessage() {
  const char *testMsg = "Hello ESP-NOW!";
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)testMsg, strlen(testMsg));
  if (result == ESP_OK) {
    Serial.println("[ESP-NOW] Test message sent (broadcast)");
  } else {
    Serial.print("[ESP-NOW] Error sending test message: ");
    Serial.println(result);
    if (sendRetries < ESPNOW_MAX_RETRIES) {
      sendRetries++;
      Serial.print("[ESP-NOW] Retrying send, attempt ");
      Serial.println(sendRetries);
      delay(100);
      sendEspNowTestMessage();
    } else {
      Serial.println("[ESP-NOW] Max retries reached. Giving up.");
    }
  }
}

unsigned long lastStatusPrint = 0;
void loop() {
  // Periodically print status
  unsigned long now = millis();
  if (now - lastStatusPrint > ESPNOW_STATUS_INTERVAL) {
    Serial.println("\n[ESP-NOW] --- STATUS REPORT ---");
    Serial.print("Uptime (s): "); Serial.println(now / 1000);
    Serial.print("Send Success: "); Serial.println(sendSuccessCount);
    Serial.print("Send Fail: "); Serial.println(sendFailCount);
    Serial.print("Received Count: "); Serial.println(receivedCount);
    if (receivedCount > 0) {
      Serial.print("Last Received From: "); printMac(lastReceivedMac); Serial.println();
      Serial.print("Last Received Data: "); Serial.println(lastReceivedData);
      Serial.print("Last Received Length: "); Serial.println(lastReceivedLen);
    }
    Serial.println("[ESP-NOW] ---------------------\n");
    lastStatusPrint = now;
  }

  // Optionally, resend test message every 10 seconds
  static unsigned long lastSend = 0;
  if (now - lastSend > 10000) {
    sendEspNowTestMessage();
    lastSend = now;
  }
}