#ifndef CLI_MANAGER_H
#define CLI_MANAGER_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// COMMAND LINE INTERFACE (CLI) - Serial command processing
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Initializes the CLI and prints a welcome message.
 */
void initializeCLI();

/**
 * @brief Handles incoming serial data non-blockingly. Call this in the main loop.
 * It reads characters, builds a command string, and processes it on newline.
 */
void handleCLI();


#endif // CLI_MANAGER_H