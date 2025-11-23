#include "MissionController.h"
#include "SwarmCommunicator.h"

// Definition for the static member variable
uint32_t MissionController::nextMissionId = 1;

// ═══════════════════════════════════════════════════════════════════════════
// MISSION MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

void MissionController::setMission(Mission& mission) {
    currentMission = mission;
    currentMission.missionId = nextMissionId++;
    currentMission.startTime = millis();
    currentMission.isComplete = false;
    
    Serial.printf("🎯 NEW MISSION [%d]: %s\n", 
                  currentMission.missionId, getMissionTypeName(mission.type));
    
    if (mission.type == MISSION_GOTO_WAYPOINT || 
        mission.type == MISSION_RETURN_TO_BASE) {
        Serial.printf("   Target: (%.1f, %.1f)\n", 
                      mission.targetPosition.x, mission.targetPosition.y);
    }
    
    if (mission.timeoutMs > 0) {
        Serial.printf("   Timeout: %lu seconds\n", mission.timeoutMs / 1000);
    }
}

void MissionController::completeMission() {
    Serial.printf("✅ MISSION COMPLETE [%d]: %s\n", 
                  currentMission.missionId, 
                  getMissionTypeName(currentMission.type));
    
    currentMission.isComplete = true;
    currentMission.type = MISSION_NONE;
}

void MissionController::abortMission() {
    Serial.printf("⛔ MISSION ABORTED [%d]: %s\n", 
                  currentMission.missionId, 
                  getMissionTypeName(currentMission.type));
    
    currentMission.type = MISSION_NONE;
    currentMission.isComplete = false;
}

// ═══════════════════════════════════════════════════════════════════════════
// ROLE-BASED BEHAVIOR
// ═══════════════════════════════════════════════════════════════════════════

void MissionController::setRole(RobotRole role) {
    myRole = role;
    Serial.printf("🎭 Role assigned: %s\n", getRoleName(role));
}

NavigationParameters MissionController::getRoleParameters() const {
    NavigationParameters params;
    
    switch (myRole) {
        case ROLE_LEADER:
            params.maxSpeed = 30.0f;          // Moderate speed
            params.attractionConstant = 2.5f;
            params.repulsionConstant = 18.0f;
            params.damping = 0.85f;           // More stable
            break;
            
        case ROLE_SCOUT:
            params.maxSpeed = 45.0f;          // Fast!
            params.attractionConstant = 3.0f;
            params.repulsionConstant = 12.0f; // Less cautious
            params.damping = 0.75f;           // More agile
            break;
            
        case ROLE_WORKER:
            params.maxSpeed = 25.0f;          // Slow and steady
            params.attractionConstant = 2.0f;
            params.repulsionConstant = 25.0f; // Very cautious
            params.damping = 0.90f;           // Very stable
            break;
            
        default:
            // Default parameters (from existing code)
            params.maxSpeed = 35.0f;
            params.attractionConstant = 2.5f;
            params.repulsionConstant = 15.0f;
            params.damping = 0.8f;
    }
    
    return params;
}

// ═══════════════════════════════════════════════════════════════════════════
// TASK ALLOCATION (AUCTION-BASED)
// ═══════════════════════════════════════════════════════════════════════════

void MissionController::broadcastTask(MissionType type, const Vector2D& target) {
    currentTaskId++;
    biddingActive = true;
    biddingDeadline = millis() + BIDDING_WINDOW_MS;
    activeBids.clear();
    
    Serial.printf("📢 Broadcasting task %d: %s at (%.1f, %.1f)\n", 
                  currentTaskId, getMissionTypeName(type), target.x, target.y);
    
    // TODO: Broadcast via ESP-NOW (extend SwarmState with task info)
    // For now, we'll just handle local bidding
    
    // Calculate my bid cost and submit
    // This will be called on each robot when they receive the broadcast
}

void MissionController::submitBid(uint32_t taskId, float cost) {
    TaskBid bid;
    bid.taskId = taskId;
    bid.robotId = myRobotId;
    bid.cost = cost;
    bid.timestamp = millis();
    
    activeBids.push_back(bid);
    
    Serial.printf("💰 Submitted bid: Task %d, Cost %.2f\n", taskId, cost);
    
    // TODO: Broadcast bid via ESP-NOW
}

void MissionController::receiveBid(const TaskBid& bid) {
    // Called when a bid is received from another robot
    if (bid.taskId == currentTaskId && biddingActive) {
        activeBids.push_back(bid);
        Serial.printf("📥 Received bid from Robot %d: Cost %.2f\n", 
                      bid.robotId, bid.cost);
    }
}

TaskBid* MissionController::getWinningBid() {
    if (activeBids.empty()) return nullptr;
    
    // Find lowest cost bid
    TaskBid* winner = &activeBids[0];
    for (auto& bid : activeBids) {
        if (bid.cost < winner->cost) {
            winner = &bid;
        }
    }
    
    Serial.printf("🏆 Winning bid: Robot %d with cost %.2f\n", 
                  winner->robotId, winner->cost);
    
    return winner;
}

void MissionController::clearBids() {
    activeBids.clear();
    biddingActive = false;
}

float MissionController::calculateBidCost(const Vector2D& target, 
                                          const Vector2D& currentPos) const {
    // Simple distance-based cost for now
    // In future: consider battery level, current mission, role, etc.
    float distance = currentPos.distanceTo(target);
    
    // Role-based cost modifiers
    float roleFactor = 1.0f;
    switch (myRole) {
        case ROLE_SCOUT:  roleFactor = 0.8f; break; // Scouts prefer exploration
        case ROLE_WORKER: roleFactor = 1.2f; break; // Workers prefer staying put
        case ROLE_LEADER: roleFactor = 1.0f; break;
        default:          roleFactor = 1.0f; break;
    }
    
    return distance * roleFactor;
}

// ═══════════════════════════════════════════════════════════════════════════
// UPDATE LOOP
// ═══════════════════════════════════════════════════════════════════════════

void MissionController::update(const Vector2D& currentPos) {
    // Handle bidding deadline
    if (biddingActive && millis() > biddingDeadline) {
        TaskBid* winner = getWinningBid();
        if (winner && winner->robotId == myRobotId) {
            Serial.println("🎯 I won the bid! Accepting task.");
            // Set mission based on task
            // (Implementation depends on how task is structured)
        }
        clearBids();
    }
    
    if (!isMissionActive()) return;
    
    // Check mission timeout
    if (currentMission.timeoutMs > 0 && 
        millis() - currentMission.startTime > currentMission.timeoutMs) {
        Serial.println("⏰ Mission timeout!");
        abortMission();
        return;
    }
    
    // Mission-specific logic
    switch (currentMission.type) {
        case MISSION_GOTO_WAYPOINT:
        case MISSION_RETURN_TO_BASE:
            // Check if goal reached
            if (isGoalReached(currentMission.targetPosition, currentPos)) {
                completeMission();
            }
            break;
            
        case MISSION_PATROL:
            // Patrol behavior (circular movement around target)
            // TODO: Implement patrol state machine
            break;
            
        case MISSION_EXPLORE:
            // Exploration continues until timeout or user abort
            // No specific completion condition
            break;
            
        case MISSION_MONITOR_TARGET:
            // Stay near target position
            // TODO: Implement monitoring behavior
            break;
    }
}

bool MissionController::isGoalReached(const Vector2D& goal, 
                                      const Vector2D& currentPos, 
                                      float threshold) const {
    return currentPos.distanceTo(goal) < threshold;
}

// ═══════════════════════════════════════════════════════════════════════════
// DIAGNOSTICS
// ═══════════════════════════════════════════════════════════════════════════

void MissionController::printMissionStatus() const {
    Serial.println("\n🎯 MISSION STATUS:");
    Serial.println("──────────────────────────────────────");
    Serial.printf("Current Mission: %s\n", getMissionTypeName(currentMission.type));
    
    if (isMissionActive()) {
        Serial.printf("Mission ID: %d\n", currentMission.missionId);
        Serial.printf("Runtime: %lu seconds\n", 
                      (millis() - currentMission.startTime) / 1000);
        
        if (currentMission.type == MISSION_GOTO_WAYPOINT || 
            currentMission.type == MISSION_RETURN_TO_BASE) {
            Serial.printf("Target: (%.1f, %.1f)\n", 
                          currentMission.targetPosition.x, 
                          currentMission.targetPosition.y);
        }
        
        if (currentMission.timeoutMs > 0) {
            unsigned long remaining = currentMission.timeoutMs - 
                                     (millis() - currentMission.startTime);
            Serial.printf("Time remaining: %lu seconds\n", remaining / 1000);
        }
    }
    
    Serial.printf("Role: %s\n", getRoleName(myRole));
    
    if (biddingActive) {
        Serial.printf("Active bids: %d\n", activeBids.size());
    }
    
    Serial.println("──────────────────────────────────────");
}

const char* MissionController::getMissionTypeName(MissionType type) const {
    switch (type) {
        case MISSION_NONE:           return "NONE";
        case MISSION_EXPLORE:        return "EXPLORE";
        case MISSION_PATROL:         return "PATROL";
        case MISSION_RETURN_TO_BASE: return "RETURN_TO_BASE";
        case MISSION_GOTO_WAYPOINT:  return "GOTO_WAYPOINT";
        case MISSION_MONITOR_TARGET: return "MONITOR_TARGET";
        case MISSION_FORMATION:      return "FORMATION";
        default:                     return "UNKNOWN";
    }
}

const char* MissionController::getRoleName(RobotRole role) const {
    switch (role) {
        case ROLE_NONE:   return "NONE";
        case ROLE_LEADER: return "LEADER";
        case ROLE_SCOUT:  return "SCOUT";
        case ROLE_WORKER: return "WORKER";
        default:          return "UNKNOWN";
    }
}