#pragma once
#include "Vector2D.h"
#include <vector>

class PotentialFieldNavigator {
protected:
    Vector2D position;
    Vector2D velocity;
    Vector2D goal;
    NavigationParameters params;
    
    Vector2D lastPositions[5];
    int positionIndex;
    unsigned long lastOscillationCheck;
    
public:
    PotentialFieldNavigator() : position(0, 0), velocity(0, 0), goal(100, 100), positionIndex(0), lastOscillationCheck(0) {}
    
    void setPosition(const Vector2D& newPos) { position = newPos; }
    void setGoal(const Vector2D& newGoal) { goal = newGoal; }
    void setParameters(const NavigationParameters& newParams) { params = newParams; }
    
    Vector2D getPosition() const { return position; }
    Vector2D getVelocity() const { return velocity; }
    Vector2D getGoal() const { return goal; }
    
    Vector2D calculateAttractiveForce() {
        Vector2D toGoal = goal - position;
        float distance = toGoal.magnitude();
        
        // Goal reached
        if (distance < params.goalThreshold) {
            return Vector2D(0, 0);
        }
        
        // Scale attraction based on distance (stronger when closer)
        float strength = params.attractionConstant;
        if (distance < 50.0f) {  // Increase attraction when close to goal
            strength = params.attractionConstant * (50.0f / distance);
        }
        
        strength = min(strength, params.maxAttraction);
        return toGoal.normalize() * strength;
    }
    
    void update(float deltaTime, const Vector2D& externalForce) {
        // Calculate internal forces
        Vector2D attractiveForce = calculateAttractiveForce();
        Vector2D dampingForce = velocity * -params.damping;
        
        // Combine all forces
        Vector2D totalForce = attractiveForce + externalForce + dampingForce;
        
        // Handle local minima
        if (isInLocalMinima() && millis() - lastOscillationCheck > 2000) {
            totalForce += calculateEscapeForce();
            lastOscillationCheck = millis();
        }
        
        // Apply physics (F = ma) and update position
        applyPhysics(totalForce, deltaTime);
    }
    
    Vector2D calculateSwarmForce(const std::vector<Vector2D>& otherRobotPositions) {
        Vector2D totalSwarmForce(0, 0);
        
        for (const auto& otherPos : otherRobotPositions) {
            Vector2D toOther = position - otherPos;
            float distance = toOther.magnitude();
            
            // Only consider nearby robots
            if (distance < 80.0f && distance > 5.0f) {
                float strength = 20.0f / (distance * distance);  // Repel other robots
                totalSwarmForce += toOther.normalize() * strength;
            }
        }
        
        return totalSwarmForce;
    }
    
    bool isInLocalMinima() {
        // Store current position
        lastPositions[positionIndex] = position;
        positionIndex = (positionIndex + 1) % 5;
        
        // Check if we're oscillating (moving very little)
        float totalMovement = 0;
        for (int i = 1; i < 5; i++) {
            int idx1 = (positionIndex + i) % 5;
            int idx2 = (positionIndex + i - 1) % 5;
            totalMovement += lastPositions[idx1].distanceTo(lastPositions[idx2]);
        }
        
        return totalMovement < params.oscillationThreshold && velocity.magnitude() < 5.0f;
    }
    
    Vector2D calculateEscapeForce() {
        // Add random perturbation to escape local minima
        float randomAngle = random(0, 1000) * 2.0f * M_PI / 1000.0f;
        return Vector2D::fromPolar(params.escapeStrength, randomAngle);
    }
    
private:
    void applyPhysics(const Vector2D& force, float deltaTime) {
        // Apply physics (F = ma)
        Vector2D acceleration = force / params.mass;
        velocity += acceleration * deltaTime;
        
        // Limit maximum speed
        velocity.limit(params.maxSpeed);
        
        // Minimum velocity to prevent getting stuck
        if (velocity.magnitude() < 2.0f && force.magnitude() > 0.5f) {
            velocity = force.normalize() * 2.0f;
        }
        
        // Update position
        position += velocity * deltaTime;
    }
};