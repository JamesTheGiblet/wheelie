#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include <Arduino.h>
#include <vector>
// #include "Vector2D.h" // Vector2D is now included via globals.h
#include "globals.h"

class MissionController {
public:
    MissionController() : myRole(ROLE_NONE), currentTaskId(0), biddingActive(false) {}

    void setMission(Mission& mission);
    void completeMission();
    void abortMission();
    
    void setRole(RobotRole role);
    RobotRole getRole() const { return myRole; }
    NavigationParameters getRoleParameters() const;

    void update(const Vector2D& currentPos);

    bool isMissionActive() const { return currentMission.type != MISSION_NONE && !currentMission.isComplete; }
    Mission getCurrentMission() const { return currentMission; }

    void printMissionStatus() const;
    String getMissionStatusString() const; // <-- ADD THIS
    const char* getMissionTypeName(MissionType type) const;
    const char* getRoleName(RobotRole role) const;

private:
    // Mission Management
    Mission currentMission;
    static uint32_t nextMissionId;
    bool isGoalReached(const Vector2D& goal, const Vector2D& currentPos, float threshold = 20.0f) const;

    // Role Management
    RobotRole myRole;

    // Task Allocation
    uint16_t myRobotId = 0; // Should be set from SwarmCommunicator
    uint32_t currentTaskId;
    bool biddingActive;
    unsigned long biddingDeadline;
    std::vector<TaskBid> activeBids;
    const unsigned long BIDDING_WINDOW_MS = 2000;

    void broadcastTask(MissionType type, const Vector2D& target);
    void submitBid(uint32_t taskId, float cost);
    void receiveBid(const TaskBid& bid);
    TaskBid* getWinningBid();
    void clearBids();
    float calculateBidCost(const Vector2D& target, const Vector2D& currentPos) const;
};

#endif // MISSION_CONTROLLER_H