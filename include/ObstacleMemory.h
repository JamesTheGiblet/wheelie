#pragma once
#include "Vector2D.h"
#include <vector>

// ═══════════════════════════════════════════════════════════════════════════
// OBSTACLE MEMORY SYSTEM - Spatial memory for navigation learning
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Classification of obstacles by their behavior and permanence
 */
enum ObstacleType {
    STATIC,      // Fixed obstacles (walls, furniture)
    DYNAMIC,     // Moving obstacles (other robots, people)
    WALL,        // Continuous barriers detected by multiple sensor readings
    HUMAN,       // Detected by PIR or movement patterns
    UNKNOWN      // Not yet classified
};

/**
 * @brief Shape representation for obstacle collision avoidance
 */
enum ObstacleShapeType {
    CIRCLE,      // Point obstacles or approximated circular objects
    RECTANGLE,   // Box-shaped obstacles
    POLYGON      // Complex shapes (future use)
};

/**
 * @brief Geometric representation of an obstacle
 */
struct ObstacleShape {
    ObstacleShapeType type;
    
    // For CIRCLE type
    Vector2D center;
    float radius;
    
    // For RECTANGLE type
    float width;
    float height;
    
    // For POLYGON type (future expansion)
    std::vector<Vector2D> vertices;
    
    // Default constructor: circular obstacle with 10cm radius
    ObstacleShape() 
        : type(CIRCLE), center(0, 0), radius(10.0f), width(0), height(0) {}
};

/**
 * @brief A remembered obstacle with classification and geometric data
 */
struct RememberedObstacle {
    Vector2D position;          // World-frame position (mm)
    float strength;             // Repulsion strength (0.0+)
    unsigned long timestamp;    // When it was last detected (millis())
    uint8_t id;                 // Unique identifier for tracking
    
    // Enhanced classification (NEW)
    ObstacleType type;          // What kind of obstacle
    ObstacleShape shape;        // Geometric representation
    float confidence;           // How certain we are (0.0 - 1.0)
    
    // Default constructor
    RememberedObstacle() 
        : position(0, 0), strength(1.0f), timestamp(0), id(0),
          type(UNKNOWN), confidence(1.0f) {}
};

/**
 * @brief Spatial memory system that remembers obstacle locations
 * 
 * This class maintains a short-term memory of detected obstacles, allowing
 * the robot to avoid recently-seen hazards even when they're not currently
 * visible to sensors. Obstacles fade from memory over time.
 */
class ObstacleMemory {
private:
    std::vector<RememberedObstacle> obstacles;
    uint8_t nextId;
    
    static const unsigned long MEMORY_DURATION = 10000;  // 10 seconds
    static constexpr float MEMORY_RADIUS = 300.0f;       // 30cm influence radius
    static constexpr float MERGE_THRESHOLD = 150.0f;     // 15cm - merge closer obstacles
    static const size_t MAX_OBSTACLES = 20;              // Memory limit
    
public:
    ObstacleMemory() : nextId(1) {}
    
    /**
     * @brief Adds or updates an obstacle in memory
     * @param position World-frame position of the obstacle (mm)
     * @param strength Repulsion strength (higher = more dangerous)
     * @param type Classification of the obstacle (default: UNKNOWN)
     * @param confidence How certain the classification is (0.0 - 1.0)
     */
    void addObstacle(const Vector2D& position, float strength, 
                     ObstacleType type = UNKNOWN, float confidence = 1.0f) {
        unsigned long currentTime = millis();
        
        // Check if we already have a similar obstacle nearby
        for (auto& obstacle : obstacles) {
            if (obstacle.position.distanceTo(position) < MERGE_THRESHOLD) {
                // Update existing obstacle with weighted average
                float alpha = 0.3f; // How much to trust the new reading
                obstacle.position = obstacle.position * (1.0f - alpha) + position * alpha;
                obstacle.strength = obstacle.strength * (1.0f - alpha) + strength * alpha;
                obstacle.timestamp = currentTime;
                
                // Update classification if new one has higher confidence
                if (confidence > obstacle.confidence) {
                    obstacle.type = type;
                    obstacle.confidence = confidence;
                }
                
                return; // Found and updated, done
            }
        }
        
        // Add new obstacle
        RememberedObstacle newObstacle;
        newObstacle.position = position;
        newObstacle.strength = strength;
        newObstacle.timestamp = currentTime;
        newObstacle.id = nextId++;
        newObstacle.type = type;
        newObstacle.confidence = confidence;
        
        // Default shape: circular with radius proportional to strength
        newObstacle.shape.type = CIRCLE;
        newObstacle.shape.center = position;
        newObstacle.shape.radius = 50.0f + (strength * 10.0f); // 5cm base + strength factor
        
        obstacles.push_back(newObstacle);
        
        // Limit memory size - remove oldest if full
        if (obstacles.size() > MAX_OBSTACLES) {
            obstacles.erase(obstacles.begin());
        }
    }
    
    /**
     * @brief Calculates total repulsive force from remembered obstacles
     * @param robotPos Current robot position in world frame (mm)
     * @return Vector2D repulsion force
     */
    Vector2D getMemoryRepulsion(const Vector2D& robotPos) {
        Vector2D total(0, 0);
        unsigned long currentTime = millis();
        
        // Iterate and clean up stale memories
        auto it = obstacles.begin();
        while (it != obstacles.end()) {
            // Remove expired obstacles
            if (currentTime - it->timestamp > MEMORY_DURATION) {
                it = obstacles.erase(it);
                continue;
            }
            
            // Calculate repulsion from this obstacle
            Vector2D toRobot = robotPos - it->position;
            float dist = toRobot.magnitude();
            
            // Only apply force if within influence radius and not too close
            if (dist < MEMORY_RADIUS && dist > 5.0f) {
                // Inverse square law for repulsion strength
                float baseStrength = it->strength / (dist * dist);
                
                // Time decay factor - obstacles fade from memory
                float age = currentTime - it->timestamp;
                float timeFactor = 1.0f - (age / float(MEMORY_DURATION));
                
                // Type-based strength modifier
                float typeMultiplier = 1.0f;
                switch (it->type) {
                    case WALL:     typeMultiplier = 1.5f; break; // Walls are more "solid"
                    case HUMAN:    typeMultiplier = 2.0f; break; // Avoid humans more strongly
                    case DYNAMIC:  typeMultiplier = 0.8f; break; // Dynamic obstacles might have moved
                    case STATIC:   typeMultiplier = 1.0f; break;
                    case UNKNOWN:  typeMultiplier = 0.9f; break; // Slightly less certain
                }
                
                // Combine all factors
                float finalStrength = baseStrength * timeFactor * typeMultiplier * it->confidence;
                
                // Add to total repulsion (away from obstacle)
                total += toRobot.normalize() * finalStrength;
            }
            
            ++it;
        }
        
        return total;
    }
    
    /**
     * @brief Gets obstacles of a specific type
     * @param type The type to filter for
     * @return Vector of matching obstacles
     */
    std::vector<RememberedObstacle> getObstaclesByType(ObstacleType type) const {
        std::vector<RememberedObstacle> result;
        for (const auto& obs : obstacles) {
            if (obs.type == type) {
                result.push_back(obs);
            }
        }
        return result;
    }
    
    /**
     * @brief Finds the nearest obstacle to a given position
     * @param position Query position (mm)
     * @param maxDistance Maximum search distance (mm)
     * @return Pointer to nearest obstacle, or nullptr if none found
     */
    const RememberedObstacle* getNearestObstacle(const Vector2D& position, float maxDistance = 1000.0f) const {
        const RememberedObstacle* nearest = nullptr;
        float minDist = maxDistance;
        
        for (const auto& obs : obstacles) {
            float dist = obs.position.distanceTo(position);
            if (dist < minDist) {
                minDist = dist;
                nearest = &obs;
            }
        }
        
        return nearest;
    }
    
    /**
     * @brief Clears all remembered obstacles
     */
    void clear() {
        obstacles.clear();
    }
    
    /**
     * @brief Gets the number of obstacles in memory
     */
    size_t getMemorySize() const {
        return obstacles.size();
    }
    
    /**
     * @brief Prints debug information about all remembered obstacles
     */
    void printMemoryStatus() const {
        Serial.printf("\n🧠 OBSTACLE MEMORY: %d obstacles\n", obstacles.size());
        unsigned long now = millis();
        
        for (const auto& obs : obstacles) {
            unsigned long age = now - obs.timestamp;
            const char* typeStr = "UNKNOWN";
            switch (obs.type) {
                case STATIC: typeStr = "STATIC"; break;
                case DYNAMIC: typeStr = "DYNAMIC"; break;
                case WALL: typeStr = "WALL"; break;
                case HUMAN: typeStr = "HUMAN"; break;
                case UNKNOWN: typeStr = "UNKNOWN"; break;
            }
            
            Serial.printf("  ID:%d [%s] Pos(%.1f,%.1f) Str:%.2f Age:%lums Conf:%.1f%%\n",
                obs.id, typeStr, obs.position.x, obs.position.y, 
                obs.strength, age, obs.confidence * 100.0f);
        }
    }
};