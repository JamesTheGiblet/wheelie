#pragma once
#include <math.h>
#include <Arduino.h>

class Vector2D {
public:
    float x, y;
    
    Vector2D() : x(0), y(0) {}
    Vector2D(float x, float y) : x(x), y(y) {}
    
    // Basic arithmetic operations
    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }
    
    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }
    
    Vector2D operator*(float scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }
    
    Vector2D operator/(float scalar) const {
        if (fabs(scalar) < 0.0001f) return Vector2D(0, 0);
        return Vector2D(x / scalar, y / scalar);
    }
    
    // Compound assignment operators
    Vector2D& operator+=(const Vector2D& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    
    Vector2D& operator-=(const Vector2D& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    
    // Vector operations
    float magnitude() const {
        return sqrt(x*x + y*y);
    }
    
    float magnitudeSquared() const {
        return x*x + y*y;
    }
    
    Vector2D normalize() const {
        float mag = magnitude();
        if (mag < 0.0001f) return Vector2D(0, 0);
        return Vector2D(x/mag, y/mag);
    }
    
    float dot(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }
    
    float distanceTo(const Vector2D& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return sqrt(dx*dx + dy*dy);
    }
    
    float angle() const {
        return atan2(y, x);
    }
    
    static Vector2D fromPolar(float magnitude, float angle) {
        return Vector2D(magnitude * cos(angle), magnitude * sin(angle));
    }
    
    void limit(float maxMagnitude) {
        float mag = magnitude();
        if (mag > maxMagnitude && mag > 0.0001f) {
            x = (x / mag) * maxMagnitude;
            y = (y / mag) * maxMagnitude;
        }
    }
    
    String toString() const {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "(%.2f, %.2f)", x, y);
        return String(buffer);
    }
};