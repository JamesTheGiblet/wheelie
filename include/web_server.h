#ifndef WEB_SERVER_H
#define WEB_SERVER_H

/**
 * @brief Initializes the Asynchronous Web Server, mounts the filesystem,
 * and sets up all routes (HTTP and WebSocket).
 */
void initializeWebServer();

/**
 * @brief Gathers all telemetry data, formats it as JSON, and pushes it
 * to all connected WebSocket clients.
 */
void pushTelemetryToClients();

#endif // WEB_SERVER_H