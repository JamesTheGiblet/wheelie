#include "ota_manager.h"
#include <ArduinoOTA.h>
#include "indicators.h"
#include "main.h" // For setRobotState
#include "credentials.h" // For OTA_PASSWORD

#include "motors.h" // For allStop()
// ═══════════════════════════════════════════════════════════════════════════
// OTA UPDATE MANAGER - Implementation
// ═══════════════════════════════════════════════════════════════════════════

// Internal state variables
static bool otaEnabled = false;
static bool otaInProgress = false;

void initializeOTA() {
    Serial.println("📡 INITIALIZING OTA SUBSYSTEM");

    // Set hostname for easy identification on the network
    ArduinoOTA.setHostname("wheelie-robot");

    // Set password for security
    #ifdef OTA_PASSWORD
        ArduinoOTA.setPassword(OTA_PASSWORD);
        Serial.println("   🔒 OTA password protection enabled.");
    #else
        Serial.println("   ⚠️  WARNING: OTA password is not set. Updates are insecure.");
    #endif

    // --- Configure OTA Callbacks ---

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        Serial.println("\n\n🔄 OTA Update Started: " + type);
        setRobotState(ROBOT_IDLE); // Stop any current activity
        allStop(); // Now declared via motors.h
        otaInProgress = true;
        setLEDColor(LEDColors::CYAN); // Indicate update in progress
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n✅ OTA Update Complete!");
        setLEDColor(LEDColors::GREEN);
        victoryAnimation();
        otaInProgress = false;
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static int lastPercent = -1;
        int percent = (progress / (total / 100));
        if (percent != lastPercent) {
            Serial.printf("   Progress: %u%%\r", percent);
            lastPercent = percent;
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("❌ OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Authentication Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        setLEDColor(LEDColors::RED);
        errorAnimation();
        otaInProgress = false;
        ESP.restart(); // Reboot on error
    });

    // Start the OTA service
    enableOTA(true);
    Serial.println("✅ OTA service ready.");
}

void handleOTA() {
    if (otaEnabled) {
        ArduinoOTA.handle();
    }
}

bool isOTAInProgress() {
    return otaInProgress;
}

void printOTAStatus() {
    Serial.println("\n📡 OTA STATUS:");
    Serial.printf("   Service: %s\n", otaEnabled ? "✅ Enabled" : "❌ Disabled");
    Serial.printf("   Hostname: %s\n", ArduinoOTA.getHostname().c_str());
    #ifdef OTA_PASSWORD
      Serial.println("   Security: 🔒 Password Protected");
    #else
      Serial.println("   Security: ⚠️ Unprotected");
    #endif
}

void enableOTA(bool enable) {
    if (enable && !otaEnabled) {
        ArduinoOTA.begin();
        otaEnabled = true;
        Serial.println("   OTA service has been enabled.");
    } else if (!enable && otaEnabled) {
        // ArduinoOTA.end() is not available, so we just stop calling handle()
        otaEnabled = false;
        Serial.println("   OTA service has been disabled for power saving.");
    }
}
