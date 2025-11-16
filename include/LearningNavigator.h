#pragma once
#include "PotentialFieldNavigator.h" // Assumes this file exists
#include <vector>

struct NavigationExperience {
    Vector2D startPos;
    Vector2D goalPos;
    Vector2D obstacleConfig;  // Simplified obstacle representation
    float successScore;        // How well did this work? (0.0 - 1.0)
    NavigationParameters params; // What settings were used?
    unsigned long timestamp;
};

class LearningNavigator : public PotentialFieldNavigator {
protected: // Use protected so future classes can access them
    std::vector<NavigationExperience> experiences;
    static const size_t MAX_EXPERIENCES = 100;
    
    float currentEpisodeStartTime;
    Vector2D episodeStartPos;
    int stepsToGoal;
    bool goalReached;
    
public:
    LearningNavigator() : PotentialFieldNavigator() {
        currentEpisodeStartTime = 0;
        stepsToGoal = 0;
        goalReached = false;
    }
    
    /**
     * @brief Start a new navigation episode (call when setting a new goal)
     */
    void startEpisode() {
        currentEpisodeStartTime = millis();
        episodeStartPos = position;
        stepsToGoal = 0;
        goalReached = false;
    }
    
    /**
     * @brief Enhanced update with learning
     */
    void updateWithLearning(float deltaTime, const Vector2D& externalForce) {
        // Run normal navigation
        update(deltaTime, externalForce);
        
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
        exp.obstacleConfig = Vector2D(0, 0); // TODO: Encode obstacle info
        
        // Calculate success score
        float durationSeconds = (millis() - currentEpisodeStartTime) / 1000.0f;
        float timeScore = (durationSeconds > 0.1f) ? (10.0f / durationSeconds) : 100.0f; // Higher score for faster completion
        float pathScore = (stepsToGoal > 0) ? (1000.0f / stepsToGoal) : 1000.0f; // Higher score for fewer steps
        exp.successScore = (timeScore + pathScore) / 2.0f;
        
        exp.params = params;
        exp.timestamp = millis();
        
        experiences.push_back(exp);
        
        // Limit memory size
        if (experiences.size() > MAX_EXPERIENCES) {
            experiences.erase(experiences.begin());
        }
        
        Serial.printf("📚 Experience recorded! Score: %.2f\n", exp.successScore);
    }
    
    /**
     * @brief Find similar past experience and use those parameters
     */
    void applyLearning() {
        if (experiences.empty()) return;
        
        // Find most similar past experience
        float bestSimilarity = 0;
        int bestIndex = -1;
        
        for (size_t i = 0; i < experiences.size(); ++i) {
            // Calculate similarity (inverse distance to past start/goal)
            float startDist = position.distanceTo(experiences[i].startPos);
            float goalDist = goal.distanceTo(experiences[i].goalPos);
            float similarity = 1.0f / (1.0f + startDist + goalDist);
            
            if (similarity > bestSimilarity) {
                bestSimilarity = similarity;
                bestIndex = i;
            }
        }
        
        if (bestIndex >= 0 && experiences[bestIndex].successScore > 0.5f) {
            // Use parameters from successful similar experience
            params = experiences[bestIndex].params;
            Serial.printf("🧠 Applied learning from experience %d (score: %.2f)\n", 
                         bestIndex, experiences[bestIndex].successScore);
        }
    }
    
    /**
     * @brief Adapt parameters based on current situation
     */
    void adaptParameters(const Vector2D& obstacleForce) {
        // If stuck (high repulsion, low progress), increase attraction
        if (obstacleForce.magnitude() > 5.0f && velocity.magnitude() < 5.0f && params.attractionConstant < 10.0f) {
            params.attractionConstant = min(params.attractionConstant * 1.1f, 10.0f);
            params.repulsionConstant = max(params.repulsionConstant * 0.9f, 5.0f);
            Serial.println("🔧 Adapted: Increased attraction (stuck detected)");
        }
        
        // If moving freely, balance parameters
        if (obstacleForce.magnitude() < 2.0f && velocity.magnitude() > 15.0f) {
            params.attractionConstant = (params.attractionConstant + 2.5f) / 2.0f;
            params.repulsionConstant = (params.repulsionConstant + 15.0f) / 2.0f;
        }
    }
};