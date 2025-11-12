#include "cli_manager.h"
#include "robot.h" // Access to all robot functions
#include "logger.h" // For printLogSummary

// ═══════════════════════════════════════════════════════════════════════════
// CLI IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════

// Buffer to store incoming command
static char cliBuffer[128];
static int bufferIndex = 0;

// Forward declaration
void processCommand(String command);

void initializeCLI() {
    Serial.println("\n✅ Command Line Interface (CLI) initialized.");
    Serial.println("Type 'help' and press Enter for a list of commands.");
    Serial.print("> ");
}

void handleCLI() {
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        // Process command on newline, otherwise buffer the character
        if (incomingChar == '\n' || incomingChar == '\r') {
            if (bufferIndex > 0) {
                cliBuffer[bufferIndex] = '\0'; // Null-terminate the string
                Serial.println(); // Echo the newline
                processCommand(String(cliBuffer));
                
                // Reset buffer for next command
                bufferIndex = 0;
                memset(cliBuffer, 0, sizeof(cliBuffer));
                Serial.print("> ");
            }
        } else if (isPrintable(incomingChar)) {
            if (bufferIndex < sizeof(cliBuffer) - 1) {
                cliBuffer[bufferIndex++] = incomingChar;
                Serial.print(incomingChar); // Echo character back to the terminal
            }
        } else if (incomingChar == '\b' && bufferIndex > 0) { // Handle backspace
            bufferIndex--;
            Serial.print("\b \b"); // Erase character from terminal
        }
    }
}

void processCommand(String command) {
    command.trim();
    command.toLowerCase();

    if (command.equals("help")) {
        Serial.println("Available commands:");
        Serial.println("  status      - Print full system status report");
        Serial.println("  navstatus   - Print detailed navigation status");
        Serial.println("  peers       - Print list of ESP-NOW peers");
        Serial.println("  reboot      - Reboot the robot");
        Serial.println("  stop        - Stop all motor movement");
        Serial.println("  explore     - Set robot state to EXPLORING");
        Serial.println("  idle        - Set robot state to IDLE");
    } 
    else if (command.equals("status")) {
        printSystemInfo();
        printBatteryStatus();
        printOTAStatus();
        printLogSummary();
    }
    else if (command.equals("navstatus")) {
        printNavigationStatus();
    }
    else if (command.equals("peers")) {
        printPeerList();
    }
    else if (command.equals("reboot")) {
        Serial.println("Rebooting now...");
        delay(100);
        ESP.restart();
    }
    else if (command.equals("stop")) {
        Serial.println("COMMAND: Stopping all motors.");
        allStop();
        setRobotState(ROBOT_IDLE);
    }
    else if (command.equals("explore")) {
        Serial.println("COMMAND: Setting state to ROBOT_EXPLORING.");
        setRobotState(ROBOT_EXPLORING);
    }
    else if (command.equals("idle")) {
        Serial.println("COMMAND: Setting state to ROBOT_IDLE.");
        allStop();
        setRobotState(ROBOT_IDLE);
    }
    else if (command.startsWith("move ")) {
        String arg = command.substring(5);
        if (arg.equals("fwd")) {
            Serial.println("COMMAND: Moving forward.");
            calibratedMoveForward(TEST_SPEED);
        } else if (arg.equals("rev")) {
            Serial.println("COMMAND: Moving reverse.");
            calibratedMoveBackward(TEST_SPEED);
        } else if (arg.equals("left")) {
            Serial.println("COMMAND: Turning left.");
            calibratedTurnLeft(TURN_SPEED);
        } else if (arg.equals("right")) {
            Serial.println("COMMAND: Turning right.");
            calibratedTurnRight(TURN_SPEED);
        } else {
            Serial.println("Invalid move command. Use: fwd, rev, left, right");
        }
    }
    else if (command.startsWith("state ")) {
        int stateNum = command.substring(6).toInt();
        if (stateNum >= ROBOT_BOOTING && stateNum <= ROBOT_ERROR) {
            Serial.printf("COMMAND: Setting state to %d\n", stateNum);
            setRobotState((RobotStateEnum)stateNum);
        } else {
            Serial.println("Invalid state number.");
        }
    }
    else {
        Serial.print("Unknown command: '");
        Serial.print(command);
        Serial.println("'");
        Serial.println("Type 'help' for a list of commands.");
    }
}