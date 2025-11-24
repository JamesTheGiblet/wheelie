#include "ota_manager.h"
#include <ArduinoOTA.h>
#include "credentials.h"
#include "indicators.h"
#include "WheelieHAL.h"
#include "main.h"

extern WheelieHAL hal;

// ═══════════════════════════════════════════════════════════════════════════
// OTA UPDATE MANAGER IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

void initializeOTA() {
    Serial.println("📡 Initializing OTA update service...");

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        Serial.println("🛑 OTA: Starting update for " + type);
        // CRITICAL: Stop the robot and enter a safe state
        hal.setVelocity(Vector2D(0, 0));
        hal.emergencyStop();
        setRobotState(ROBOT_IDLE);
        setLEDColor(LEDColors::BLUE); // Blue for "in progress"
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n✅ OTA: Update complete!");
        setLEDColor(LEDColors::GREEN);
        delay(100); // Allow TCP stack to send final ACK
        ESP.restart(); // CRITICAL: Reboot to apply update
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("❌ OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        setLEDColor(LEDColors::RED);
        indicateError();
        setRobotState(ROBOT_EXPLORING); // Resume operation on error
    });

    ArduinoOTA.begin();
    Serial.println("✅ OTA service ready. Upload at IP: " + WiFi.localIP().toString());
}

void handleOTA() {
    // This function needs to be called in the main loop to listen for requests.
    ArduinoOTA.handle();
}