#include "SwarmCommunicator.h"

SwarmCommunicator& SwarmCommunicator::getInstance() {
    static SwarmCommunicator instance;
    return instance;
}

void SwarmCommunicator::onDataReceived(const uint8_t* mac_addr, const uint8_t* incomingData, int len) {
    if (len != sizeof(SwarmState)) {
        // Ignore packets of the wrong size
        return;
    }

    // Get the singleton instance
    SwarmCommunicator& instance = getInstance();

    // Copy data into a SwarmState struct
    SwarmState receivedState;
    memcpy(&receivedState, incomingData, sizeof(SwarmState));

    // Process the received state
    instance._processReceivedState(receivedState);
}