#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "indicators.h"

// ═══════════════════════════════════════════════════════════════════════════
// WIFI MANAGEMENT - WiFi connection and network management
// ═══════════════════════════════════════════════════════════════════════════
//
// This module acts as a "worker" to manage the WiFi connection.
// It is called by the main loop and updates the global sysStatus struct.
// It does not provide any "getter" functions to avoid competing sources of truth.
//
// ═══════════════════════════════════════════════════════════════════════════

// Internal WiFi status enumeration (used by indicator functions)
enum WiFiStatus {
    WIFI_DISCONNECTED,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    WIFI_CONNECTION_FAILED,
    WIFI_CONNECTION_LOST
};

// WiFi Management Functions
void initializeWiFi();

/**
 * @brief Main update function for WiFi. Call this in the main loop.
 * Checks connection status, handles reconnections, and updates the
 * global sysStatus.wifiConnected and sysStatus.ipAddress variables.
 */
void checkWiFiConnection();

/**
 * @brief [INTERNAL] Handles the logic for reconnecting to WiFi.
 * Called by checkWiFiConnection() when a drop is detected.
 */
void handleWiFiReconnection();

// WiFi Indicator Functions
void indicateWiFiStatus(WiFiStatus status);
void showWiFiConnectionProgress();

#endif // WIFI_MANAGER_H