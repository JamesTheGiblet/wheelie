#include "SwarmCommunicator.h"

// This is the implementation of the static callback.
// It needs to be defined in a .cpp file to avoid multiple definition errors.
void SwarmCommunicator::onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
    // TODO: Implement the logic to handle received data.
    // For example, deserialize the data into a RobotState struct
    // and update the list of other robots.
    Serial.printf("Received %d bytes from %02x:%02x:%02x:%02x:%02x:%02x\n", len, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}