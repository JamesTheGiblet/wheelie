#ifndef WEB_SERVER_H
#define WEB_SERVER_H

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER - Provides a web interface for robot status and control.
// ═══════════════════════════════════════════════════════════════════════════

// Initializes the web server and sets up all the URL handlers.
void initializeWebServer();

// Handles incoming client requests. Call this in the main loop.
void handleWebServer();

#endif // WEB_SERVER_H