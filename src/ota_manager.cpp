#include "ota_manager.h"
#include <ArduinoOTA.h>
#include "credentials.h" // For OTA_PASSWORD
#include "indicators.h"  // For LED feedback

// ═══════════════════════════════════════════════════════════════════════════
// OTA UPDATE MANAGER IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

void initializeOTA() {
    Serial.println("📡 Initializing OTA update service...");

    // Set the hostname for the device. This is how it will appear on the network.
    // You can then upload to "wheelie-robot.local" instead of an IP address.
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    // Set the password for OTA updates for security.
    // This password must match the 'upload_flags = --auth=...' in platformio.ini
    ArduinoOTA.setPassword(OTA_PASSWORD);

    // --- Configure Callbacks for the OTA Update Process ---

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        Serial.println("OTA: Starting update for " + type);
        setLEDColor(LEDColors::BLUE); // Blue for "in progress"
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA: Update complete!");
        setLEDColor(LEDColors::GREEN); // Green for success
        // You can add a success melody here if you like
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
        // You could blink an LED here to show activity
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Authentication Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        setLEDColor(LEDColors::RED); // Red for error
        indicateError();
    });

    // Start the OTA service
    ArduinoOTA.begin();
    Serial.println("✅ OTA service ready. Upload at IP: " + WiFi.localIP().toString());
}

void handleOTA() {
    // This function needs to be called in the main loop to listen for requests.
    ArduinoOTA.handle();
}