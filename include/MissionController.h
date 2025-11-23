#pragma once
#include "Vector2D.h"
#include "PotentialFieldNavigator.h"
#include <vector>
#include <limits>
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// MISSION CONTROL SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

enum MissionType {
    MISSION_NONE,
    MISSION_EXPLORE,
    MISSION_PATROL,
    MISSION_RETURN_TO_BASE,
    MISSION_GOTO_WAYPOINT,
    MISSION_MONITOR_TARGET,
    MISSION_FORMATION
};

enum RobotRole {
    ROLE_NONE,
    ROLE_LEADER,      // Coordinates swarm, makes decisions
    ROLE_SCOUT,       // Fast exploration, lightweight sensors
    ROLE_WORKER       // Heavy lifting, precise tasks
};

struct Mission {
    uint32_t missionId;
    MissionType type;
    Vector2D targetPosition;
    float patrolRadius;
    std::vector<Vector2D> waypoints;
    unsigned long startTime;
    unsigned long timeoutMs;
    bool isComplete;
    uint16_t assignedRobotId; // Who is this mission for? 0 for broadcast.

    Mission()
        : missionId(0), type(MISSION_NONE), patrolRadius(0.0f),
          startTime(0), timeoutMs(0), isComplete(false), assignedRobotId(0) {}
};

struct TaskBid {
    uint32_t taskId;
    uint16_t robotId;
    float cost;           // Lower is better (distance, battery, etc.)
    unsigned long timestamp;
    
    TaskBid() : taskId(0), robotId(0),
                cost(std::numeric_limits<float>::max()), timestamp(0) {}
};

class MissionController {
private:
    Mission currentMission;
    RobotRole myRole;
    uint16_t myRobotId;

    // Task auction state
    std::vector<TaskBid> activeBids;
    uint32_t currentTaskId;
    bool biddingActive;
    unsigned long biddingDeadline;
    static const unsigned long BIDDING_WINDOW_MS = 2000; // 2 seconds to bid
    
    static uint32_t nextMissionId; // Static to ensure unique IDs across all instances

    // Helper functions
    float calculateBidCost(const Vector2D& target, const Vector2D& currentPos) const;
    bool isGoalReached(const Vector2D& goal, const Vector2D& currentPos, float threshold = 100.0f) const;
    
public:
    MissionController()
        : myRole(ROLE_NONE), myRobotId(0),
          currentTaskId(0), biddingActive(false), biddingDeadline(0) {}
    
    void setRobotId(uint16_t id) { myRobotId = id; }
    
    // Mission management
    void setMission(Mission& mission); // Pass by non-const reference to assign ID
    Mission getCurrentMission() const { return currentMission; }
    bool isMissionActive() const { return currentMission.type != MISSION_NONE; }
    void completeMission();
    void abortMission();
    
    // Role-based behavior
    void setRole(RobotRole role);
    RobotRole getRole() const { return myRole; }
    NavigationParameters getRoleParameters() const;
    
    // Task allocation (auction-based)
    void broadcastTask(MissionType type, const Vector2D& target);
    void receiveBid(const TaskBid& bid);
    void submitBid(uint32_t taskId, float cost);
    TaskBid* getWinningBid();
    void clearBids();
    bool isBiddingActive() const { return biddingActive; }
    
    // Update loop (call from main)
    void update(const Vector2D& currentPos);
    
    // Public helper functions for external use (e.g., telemetry)
    const char* getMissionTypeName(MissionType type) const;
    const char* getRoleName(RobotRole role) const;

    // Diagnostics
    void printMissionStatus() const;
};