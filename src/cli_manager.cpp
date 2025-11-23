#include "main.h"
#include "cli_manager.h"
#include "SwarmCommunicator.h" // For swarm info
#include "logger.h" // For printLogSummary
#include "power_manager.h" // For printBatteryStatus()
#include "WheelieHAL.h" // For hal.setVelocity()


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
        Serial.println("  mission     - Print current mission status");
        Serial.println("  goto X Y    - Navigate to waypoint (X, Y in mm)");
        Serial.println("  explore     - Start exploration mission");
        Serial.println("  return      - Return to base (0, 0)");
        Serial.println("  abort       - Abort current mission");
        Serial.println("  role NAME   - Set role (leader/scout/worker)");
        Serial.println("  navstatus   - Print detailed navigation status");
        Serial.println("  battery     - Print detailed battery status");
        Serial.println("  peers       - Print list of ESP-NOW peers");
        Serial.println("  reboot      - Reboot the robot");
        Serial.println("  stop        - Stop all motor movement");
        Serial.println("  idle        - Set robot state to IDLE");
    } 
    else if (command.equals("status")) {
        printSystemInfo();
        printBatteryStatus();
        printLogSummary();
    }
    else if (command.equals("mission")) {
        missionController.printMissionStatus();
    }
    else if (command.startsWith("goto ")) {
        int spacePos = command.indexOf(' ', 5);
        if (spacePos > 0) {
            float x = command.substring(5, spacePos).toFloat();
            float y = command.substring(spacePos + 1).toFloat();
            Mission m;
            m.type = MISSION_GOTO_WAYPOINT;
            m.targetPosition = Vector2D(x, y);
            m.timeoutMs = 60000; // 1 minute timeout
            missionController.setMission(m);
            Serial.printf("Setting waypoint: (%.1f, %.1f)\n", x, y);
        } else {
            Serial.println("Usage: goto X Y");
        }
    }
    else if (command.equals("explore")) {
        Mission m;
        m.type = MISSION_EXPLORE;
        m.timeoutMs = 120000; // 2 minutes
        missionController.setMission(m);
        Serial.println("Starting exploration mission");
    }
    else if (command.equals("return")) {
        Mission m;
        m.type = MISSION_RETURN_TO_BASE;
        m.targetPosition = Vector2D(0, 0); // Home = origin
        missionController.setMission(m);
        Serial.println("Returning to base");
    }
    else if (command.equals("abort")) {
        missionController.abortMission();
    }
    else if (command.startsWith("role ")) {
        String roleStr = command.substring(5);
        RobotRole role = ROLE_NONE;
        if (roleStr.equals("leader")) role = ROLE_LEADER;
        else if (roleStr.equals("scout")) role = ROLE_SCOUT;
        else if (roleStr.equals("worker")) role = ROLE_WORKER;
        else {
            Serial.println("Unknown role. Use: leader, scout, or worker");
            return;
        }
        missionController.setRole(role);
    }
    else if (command.equals("battery")) {
        printBatteryStatus();
    }
    else if (command.equals("navstatus")) {
        printNavigationStatus();
    }
    else if (command.equals("peers")) {
        SwarmCommunicator::getInstance().printSwarmInfo();
    }
    else if (command.equals("reboot")) {
        Serial.println("Rebooting now...");
        delay(100);
        ESP.restart();
    } 
    else if (command.equals("stop")) {
        Serial.println("COMMAND: Stopping all motors.");
        hal.setVelocity(Vector2D(0,0)); // Correct way to stop
        setRobotState(ROBOT_IDLE); // Set state to idle to prevent navigator override
    }
    else if (command.equals("explore")) {
        Serial.println("COMMAND: Setting state to ROBOT_EXPLORING.");
        setRobotState(ROBOT_EXPLORING);
    }
    else if (command.equals("idle")) {
        Serial.println("COMMAND: Setting state to ROBOT_IDLE.");
        hal.setVelocity(Vector2D(0,0)); // Correct way to stop
        setRobotState(ROBOT_IDLE);
    }
    else if (command.startsWith("move ")) {
        String arg = command.substring(5);
        if (arg.equals("fwd")) {
            Serial.println("COMMAND: Moving forward.");
            // calibratedMoveForward(TEST_SPEED);
        } else if (arg.equals("rev")) {
            Serial.println("COMMAND: Moving reverse.");
            // calibratedMoveBackward(TEST_SPEED);
        } else if (arg.equals("left")) {
            Serial.println("COMMAND: Turning left.");
            // calibratedTurnLeft(TURN_SPEED);
        } else if (arg.equals("right")) {
            Serial.println("COMMAND: Turning right.");
            // calibratedTurnRight(TURN_SPEED);
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