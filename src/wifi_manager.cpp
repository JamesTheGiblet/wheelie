#include "wifi_manager.h"
#include "credentials.h" // <-- FIX #3: Include credentials

// ═══════════════════════════════════════════════════════════════════════════
// WIFI MANAGEMENT IMPLEMENTATION - Worker Module
// ═══════════════════════════════════════════════════════════════════════════
//
// This module acts as a worker that manages WiFi connection and updates the
// global sysStatus struct. It does not expose getter functions to maintain
// the single source of truth principle.
//
// ═══════════════════════════════════════════════════════════════════════════

// Access to global system status
extern SystemStatus sysStatus;

// Internal state variables
static WiFiStatus currentWiFiStatus = WIFI_DISCONNECTED;
static unsigned long lastConnectionAttempt = 0;

void initializeWiFi() {
  Serial.println("📶 INITIALIZING WIFI SUBSYSTEM");
  // Set WiFi mode to station (client)
  WiFi.mode(WIFI_STA);
  // Set hostname
  WiFi.setHostname("wheelie-robot");
  Serial.print("📡 Target Network: ");
  Serial.println(WIFI_SSID);
  
  // Initialize sysStatus fields
  sysStatus.wifiConnected = false;
  strncpy(sysStatus.ipAddress, "Connecting...", sizeof(sysStatus.ipAddress));
  
  // Start the FIRST connection attempt
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  currentWiFiStatus = WIFI_CONNECTING;
  lastConnectionAttempt = millis();
}

/**
 * @brief Main update function for WiFi. Call this in the main loop.
 * This is a NON-BLOCKING state machine.
 */
void checkWiFiConnection() {
    // 1. Handle the "Connecting" state
    if (currentWiFiStatus == WIFI_CONNECTING) {
        wl_status_t status = WiFi.status();

        if (status == WL_CONNECTED) {
            // --- We are now connected ---
            currentWiFiStatus = WIFI_CONNECTED;
            sysStatus.wifiConnected = true;
            
            // Safely copy the IP address
            String ipStr = WiFi.localIP().toString();
            strncpy(sysStatus.ipAddress, ipStr.c_str(), sizeof(sysStatus.ipAddress) - 1);
            sysStatus.ipAddress[sizeof(sysStatus.ipAddress) - 1] = '\0'; // Ensure null termination
            
            Serial.println("\n✅ WiFi Connected!");
            Serial.print("   IP Address: ");
            Serial.println(sysStatus.ipAddress);
        }
        else if (millis() - lastConnectionAttempt > WIFI_TIMEOUT) {
            // --- Connection timed out ---
            currentWiFiStatus = WIFI_CONNECTION_FAILED;
            sysStatus.wifiConnected = false;
            strncpy(sysStatus.ipAddress, "Failed", sizeof(sysStatus.ipAddress));
            Serial.println("\n❌ WiFi Connection Failed (Timeout)");
            lastConnectionAttempt = millis(); // Reset timer for next retry
        }
        // else: Still connecting, do nothing and let the loop run
    }
    
    // 2. Handle the "Connected" state (check for drops)
    else if (currentWiFiStatus == WIFI_CONNECTED) {
        if (WiFi.status() != WL_CONNECTED) {
            // --- We just lost connection ---
            currentWiFiStatus = WIFI_CONNECTION_LOST;
            sysStatus.wifiConnected = false;
            strncpy(sysStatus.ipAddress, "Lost", sizeof(sysStatus.ipAddress));
            Serial.println("⚠️ WiFi Connection Lost!");
            lastConnectionAttempt = millis(); // Start retry timer
        }
        // else: Still connected, do nothing
    }

    // 3. Handle "Failed" or "Lost" states (retry logic)
    else if (currentWiFiStatus == WIFI_CONNECTION_FAILED || currentWiFiStatus == WIFI_CONNECTION_LOST) {
        if (millis() - lastConnectionAttempt > WIFI_RETRY_INTERVAL) {
            // --- Time to retry ---
            Serial.println("🔄 Attempting WiFi reconnection...");
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            currentWiFiStatus = WIFI_CONNECTING;
            lastConnectionAttempt = millis();
        }
    }
}

// -------------------------------------------------------------------
// ALL FUNCTIONS BELOW THIS LINE ARE NOW DELETED
//
// - handleWiFiReconnection() (logic is now inside checkWiFiConnection)
// - printWiFiInfo() (info is printed on connect)
// - indicateWiFiStatus() (VIOLATES SoC, a job for indicators.cpp)
// - showWiFiConnectionProgress() (VIOLATES SoC)
//
// -------------------------------------------------------------------