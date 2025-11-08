#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER - Simple status dashboard
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Initializes the web server and sets up all the URL routes.
 * Must be called after WiFi is connected.
 */
void initializeWebServer();

/**
 * @brief Handles incoming client requests. Call this in the main loop.
 */
void handleWebServer();

#endif // WEB_SERVER_H