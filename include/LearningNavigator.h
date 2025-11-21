#pragma once
#include "PotentialFieldNavigator.h"
#include "ObstacleMemory.h"
#include <vector>

// ═══════════════════════════════════════════════════════════════════════════
// NAVIGATION EXPERIENCE - Learning from past navigation attempts
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Encodes the obstacle environment for experience matching
 */
struct ObstacleConfiguration {
    size_t obstacleCount;           // How many obstacles were present
    float averageObstacleStrength;  // Average repulsion strength
    float maxObstacleStrength;      // Strongest obstacle encountered
    Vector2D dominantDirection;     // Primary direction of obstacle forces
    
    ObstacleConfiguration() 
        : obstacleCount(0), averageObstacleStrength(0.0f), 
          maxObstacleStrength(0.0f), dominantDirection(0, 0) {}
};

/**
 * @brief Record of a navigation episode for learning
 */
struct NavigationExperience {
    Vector2D startPos;
    Vector2D goalPos;
    ObstacleConfiguration obstacleConfig;  // NOW PROPERLY ENCODED
    float successScore;                     // How well did this work? (0.0 - 1.0)
    NavigationParameters params;            // What settings were used?
    unsigned long timestamp;
    unsigned long durationMs;               // How long it took
    int stepsToGoal;                        // Number of update steps
    
    NavigationExperience() 
        : startPos(0, 0), goalPos(0, 0), successScore(0.0f),
          timestamp(0), durationMs(0), stepsToGoal(0) {}
};

// ═══════════════════════════════════════════════════════════════════════════
// LEARNING NAVIGATOR - Potential field navigation with memory and learning
// ═══════════════════════════════════════════════════════════════════════════

class LearningNavigator : public PotentialFieldNavigator {
protected:
    // Experience-based learning
    std::vector<NavigationExperience> experiences;
    static const size_t MAX_EXPERIENCES = 100;

    // Current episode tracking
    float currentEpisodeStartTime;
    Vector2D episodeStartPos;
    int stepsToGoal;
    bool goalReached;
    
    // Obstacle tracking for current episode
    Vector2D totalObstacleForce;      // Sum of all obstacle forces this episode
    float maxObstacleForce;            // Strongest obstacle force seen
    int obstacleEncounters;            // Number of times obstacles were detected

    // Spatial memory system
    ObstacleMemory obstacleMemory;

public:
    LearningNavigator() : PotentialFieldNavigator() {
        currentEpisodeStartTime = 0;
        stepsToGoal = 0;
        goalReached = false;
        totalObstacleForce = Vector2D(0, 0);
        maxObstacleForce = 0.0f;
        obstacleEncounters = 0;
    }
    
    /**
     * @brief Start a new navigation episode (call when setting a new goal)
     */
    void startEpisode() {
        currentEpisodeStartTime = millis();
        episodeStartPos = position;
        stepsToGoal = 0;
        goalReached = false;
        
        // Reset episode statistics
        totalObstacleForce = Vector2D(0, 0);
        maxObstacleForce = 0.0f;
        obstacleEncounters = 0;
        
        // Clear old obstacle memories to start fresh
        obstacleMemory.clear();
    }
    
    /**
     * @brief Enhanced update with learning and memory
     */
    void updateWithLearning(float deltaTime, const Vector2D& externalForce) {
        // Track obstacle statistics for this episode
        float obstacleMagnitude = externalForce.magnitude();
        if (obstacleMagnitude > 5.0f) {
            totalObstacleForce += externalForce;
            maxObstacleForce = max(maxObstacleForce, obstacleMagnitude);
            obstacleEncounters++;
            
            // Add to spatial memory (classify as STATIC by default)
            // In a more advanced system, you'd use sensor fusion to determine type
            obstacleMemory.addObstacle(position, obstacleMagnitude, STATIC, 0.8f);
        }

        // Get repulsion from remembered obstacles
        Vector2D memoryForce = obstacleMemory.getMemoryRepulsion(position);

        // Combine real-time sensor force with memory
        Vector2D totalForce = externalForce + memoryForce;

        // Run base navigation algorithm
        update(deltaTime, totalForce);

        stepsToGoal++;

        // Check if goal reached
        Vector2D toGoal = goal - position;
        if (toGoal.magnitude() < params.goalThreshold) {
            if (!goalReached) {
                goalReached = true;
                recordExperience();
            }
        }
    }
    
    /**
     * @brief Record successful navigation experience
     */
    void recordExperience() {
        NavigationExperience exp;
        exp.startPos = episodeStartPos;
        exp.goalPos = goal;
        exp.timestamp = millis();
        exp.durationMs = exp.timestamp - (unsigned long)currentEpisodeStartTime;
        exp.stepsToGoal = stepsToGoal;
        
        // Encode obstacle configuration properly
        exp.obstacleConfig.obstacleCount = obstacleMemory.getMemorySize();
        exp.obstacleConfig.maxObstacleStrength = maxObstacleForce;
        
        if (obstacleEncounters > 0) {
            exp.obstacleConfig.averageObstacleStrength = 
                totalObstacleForce.magnitude() / obstacleEncounters;
            exp.obstacleConfig.dominantDirection = totalObstacleForce.normalize();
        } else {
            exp.obstacleConfig.averageObstacleStrength = 0.0f;
            exp.obstacleConfig.dominantDirection = Vector2D(0, 0);
        }
        
        // Calculate success score (higher is better)
        float durationSeconds = exp.durationMs / 1000.0f;
        
        // Time efficiency: faster is better (max score at 1 second)
        float timeScore = (durationSeconds > 0.1f) ? 
            min(10.0f / durationSeconds, 10.0f) : 10.0f;
        
        // Path efficiency: fewer steps is better
        float pathScore = (stepsToGoal > 0) ? 
            min(100.0f / stepsToGoal, 10.0f) : 10.0f;
        
        // Combine scores (normalized to 0-10 range)
        exp.successScore = (timeScore + pathScore) / 2.0f;
        
        // Store the navigation parameters that were used
        exp.params = params;
        
        // Add to experience database
        experiences.push_back(exp);
        
        // Limit memory size (FIFO)
        if (experiences.size() > MAX_EXPERIENCES) {
            experiences.erase(experiences.begin());
        }
        
        Serial.printf("📚 Experience recorded! Score: %.2f Time: %.1fs Steps: %d Obstacles: %zu\n", 
            exp.successScore, durationSeconds, stepsToGoal, exp.obstacleConfig.obstacleCount);
    }
    
    /**
     * @brief Find similar past experience and use those parameters
     */
    void applyLearning() {
        if (experiences.empty()) return;
        
        // Find most similar past experience
        float bestSimilarity = 0.0f;
        int bestIndex = -1;
        
        for (size_t i = 0; i < experiences.size(); ++i) {
            // Calculate spatial similarity
            float startDist = position.distanceTo(experiences[i].startPos);
            float goalDist = goal.distanceTo(experiences[i].goalPos);
            float spatialSimilarity = 1.0f / (1.0f + startDist + goalDist);
            
            // Calculate obstacle similarity (compare obstacle counts)
            size_t currentObstacles = obstacleMemory.getMemorySize();
            float obstacleDiff = abs((int)currentObstacles - (int)experiences[i].obstacleConfig.obstacleCount);
            float obstacleSimilarity = 1.0f / (1.0f + obstacleDiff);
            
            // Combined similarity (weighted average)
            float similarity = (spatialSimilarity * 0.7f) + (obstacleSimilarity * 0.3f);
            
            if (similarity > bestSimilarity) {
                bestSimilarity = similarity;
                bestIndex = i;
            }
        }
        
        // Only apply learning if we found a similar, successful experience
        if (bestIndex >= 0 && experiences[bestIndex].successScore > 5.0f && bestSimilarity > 0.3f) {
            params = experiences[bestIndex].params;
            Serial.printf("🧠 Applied learning from experience %d (score: %.2f, similarity: %.2f)\n", 
                         bestIndex, experiences[bestIndex].successScore, bestSimilarity);
        }
    }
    
    /**
     * @brief Adapt parameters based on current situation (reactive learning)
     */
    void adaptParameters(const Vector2D& obstacleForce) {
        float obsMag = obstacleForce.magnitude();
        float velMag = velocity.magnitude();
        
        // Stuck detection: high obstacle force but low velocity
        if (obsMag > 10.0f && velMag < 5.0f) {
            // Increase attraction to try to "push through" or find alternate route
            params.attractionConstant = min(params.attractionConstant * 1.15f, 10.0f);
            params.repulsionConstant = max(params.repulsionConstant * 0.9f, 5.0f);
            Serial.println("🔧 Adapted: Increased attraction (stuck detected)");
        }
        
        // Free motion: low obstacles and good velocity
        else if (obsMag < 2.0f && velMag > 15.0f) {
            // Gradually return to balanced parameters
            params.attractionConstant = params.attractionConstant * 0.95f + 2.5f * 0.05f;
            params.repulsionConstant = params.repulsionConstant * 0.95f + 15.0f * 0.05f;
        }
        
        // Oscillation detection: if stuck in local minimum
        if (isInLocalMinima() && obsMag > 5.0f) {
            // Temporarily reduce repulsion to escape
            params.repulsionConstant *= 0.8f;
            Serial.println("🔧 Adapted: Reduced repulsion (local minimum escape)");
        }
    }
    
    /**
     * @brief Get statistics about learned experiences
     */
    void printLearningStats() const {
        if (experiences.empty()) {
            Serial.println("📚 No experiences recorded yet");
            return;
        }
        
        Serial.printf("\n📚 LEARNING STATISTICS:\n");
        Serial.printf("   Total experiences: %zu\n", experiences.size());
        
        // Calculate average success score
        float totalScore = 0.0f;
        float bestScore = 0.0f;
        for (const auto& exp : experiences) {
            totalScore += exp.successScore;
            if (exp.successScore > bestScore) {
                bestScore = exp.successScore;
            }
        }
        float avgScore = totalScore / experiences.size();
        
        Serial.printf("   Average success: %.2f\n", avgScore);
        Serial.printf("   Best success: %.2f\n", bestScore);
        
        // Print obstacle memory status
        obstacleMemory.printMemoryStatus();
    }
    
    /**
     * @brief Access the obstacle memory (for debugging/visualization)
     */
    const ObstacleMemory& getObstacleMemory() const {
        return obstacleMemory;
    }
};