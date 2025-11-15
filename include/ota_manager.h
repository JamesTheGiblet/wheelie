#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <Arduino.h>

/**
 * @brief Initializes the Over-the-Air (OTA) update service.
 * 
 * This function sets up the ArduinoOTA library, configures the hostname,
 * sets the authentication password, and defines callbacks for the update
 * process (onStart, onEnd, onProgress, onError).
 * 
 * It should be called once in the main setup() function after WiFi has
 * been successfully connected.
 */
void initializeOTA();

/**
 * @brief Handles incoming OTA requests.
 * 
 * This function must be called repeatedly in the main loop(). It listens for
 * OTA update requests on the network and manages the update process without
 * blocking the rest of the program.
 */
void handleOTA();

#endif // OTA_MANAGER_H