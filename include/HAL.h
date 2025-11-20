#ifndef HAL_H
#define HAL_H

#include "Vector2D.h"
#include "globals.h"

/**
 * @brief Represents the robot's 2D position and orientation.
 * This is the standard "pose" struct used by the navigation system.
 */
struct RobotPose {
    Vector2D position; // x, y position in millimeters
    float heading;     // heading in degrees
};

/**
 * @brief Hardware Abstraction Layer (HAL) Interface.
 * * This is the "Layer 1" in the Project Jumbo architecture. It is an abstract
 * class that defines the universal contract for *all* robot bodies.
 * * The "Brain" (Layer 2) interacts *only* with this interface and has no
 * knowledge of the specific hardware (ToF vs. Ultrasonic, etc.).
 */
class HAL {
public:
    virtual ~HAL() {}

    // --- LIFECYCLE & STATE ---
    
    /**
     * @brief Initializes all hardware: motors, sensors, I2C, calibration.
     * This function should block until the robot is fully calibrated and ready.
     * @return true if initialization and calibration were successful.
     */
    virtual bool init() = 0;

    /**
     * @brief Updates all hardware states. Polls sensors, updates odometry.
     * This should be called once per loop, before any "get" functions.
     */
    virtual void update() = 0;

    // --- SENSING (Input to the Brain) ---

    /**
     * @brief Gets the combined repulsive force from all obstacles.
     * The HAL is responsible for reading all its specific sensors (ToF,
     * ultrasonic, etc.) and translating them into a single, combined
     * repulsive force vector.
     * * @return Vector2D The aggregate repulsive force.
     */
    virtual Vector2D getObstacleRepulsion() = 0;

    /**
     * @brief Gets the robot's current estimated position and heading.
     * The HAL is responsible for running its own internal odometry
     * (using encoders, IMU, etc.) to track this.
     * * @return RobotPose The robot's current pose.
     */
    virtual RobotPose getPose() = 0;

    // --- ACTUATION (Output from the Brain) ---

    /**
     * @brief Sets the robot's desired velocity.
     * The "Brain" (PotentialFieldNavigator) sends a 2D velocity vector.
     * The HAL is responsible for translating this vector into the
     * correct motor/servo commands for its specific drive system
     * (e.g., differential drive, mecanum, etc.).
     * * @param velocity The desired velocity vector (in mm/s).
     */
    virtual void setVelocity(const Vector2D& velocity) = 0;

    // --- POWER & SAFETY ---
    virtual void setMaxSpeed(float speedRatio) = 0;
    virtual void setLEDBrightness(int brightness) = 0;
    virtual void emergencyStop() = 0;

    // --- UTILITIES ---
    virtual void setStatusLED(const LEDColor& color) = 0;
    virtual void playTone(int frequency, int duration) = 0;
    virtual float getBatteryVoltage() = 0;
};

#endif // HAL_H