#pragma once
#include "Vector2D.h"
#include <vector>

struct RememberedObstacle {
    Vector2D position;
    float strength;
    unsigned long timestamp;
    uint8_t id;  // For tracking specific obstacles
};

class ObstacleMemory {
private:
    std::vector<RememberedObstacle> obstacles;
    uint8_t nextId;
    static const unsigned long MEMORY_DURATION = 10000;  // 10 seconds
    static const float MEMORY_RADIUS = 100.0f;  // cm
    
public:
    ObstacleMemory() : nextId(1) {}
    
    void addObstacle(const Vector2D& position, float strength) {
        // Check if we already have a similar obstacle
        for (auto& obstacle : obstacles) {
            if (obstacle.position.distanceTo(position) < 15.0f) {
                // Update existing obstacle
                obstacle.position = position;
                obstacle.strength = strength;
                obstacle.timestamp = millis();
                return;
            }
        }
        
        // Add new obstacle
        RememberedObstacle newObstacle;
        newObstacle.position = position;
        newObstacle.strength = strength;
        newObstacle.timestamp = millis();
        newObstacle.id = nextId++;
        obstacles.push_back(newObstacle);
        
        // Limit memory size
        if (obstacles.size() > 20) {
            obstacles.erase(obstacles.begin());
        }
    }
    
    Vector2D getMemoryRepulsion(const Vector2D& robotPos) {
        Vector2D total(0, 0);
        unsigned long currentTime = millis();
        
        auto it = obstacles.begin();
        while (it != obstacles.end()) {
            if (currentTime - it->timestamp > MEMORY_DURATION) {
                it = obstacles.erase(it);
            } else {
                Vector2D toObstacle = robotPos - it->position;
                float dist = toObstacle.magnitude();
                
                if (dist < MEMORY_RADIUS && dist > 5.0f) {
                    float strength = it->strength / (dist * dist);
                    // Reduce strength over time
                    float timeFactor = 1.0f - ((currentTime - it->timestamp) / float(MEMORY_DURATION));
                    strength *= timeFactor;
                    
                    total += toObstacle.normalize() * strength;
                }
                ++it;
            }
        }
        return total;
    }
    
    void clear() {
        obstacles.clear();
    }
    
    size_t getMemorySize() const {
        return obstacles.size();
    }
};