#ifndef WHEELIE_HAL_H
#define WHEELIE_HAL_H

#include "HAL.h"
#include "motors.h"
#include "indicators.h"
#include "power_manager.h"
#include "calibration.h"
#include "logger.h"
#include "wifi_manager.h"

/**
 * @brief Wheelie's specific implementation of the Layer 1 HAL.
 * * This class "absorbs" all the hardware-specific logic from the
 * old project, translating it to the generic HAL interface.
 */
class WheelieHAL : public HAL {
public:
    WheelieHAL();
    virtual ~WheelieHAL() {}

    // --- HAL Interface Implementation ---
    virtual bool init() override;
    virtual void update() override;
    virtual Vector2D getObstacleRepulsion() override;
    virtual RobotPose getPose() override;
    virtual void setVelocity(const Vector2D& velocity) override;
    virtual void setMaxSpeed(float speedRatio) override;
    virtual void setLEDBrightness(int brightness) override;
    virtual void emergencyStop() override;
    virtual void setStatusLED(const LEDColor& color) override;
    virtual void playTone(int frequency, int duration) override;
    virtual float getBatteryVoltage() override;

private:
    /**
     * @brief Scans I2C bus and configures pins for available sensors.
     */
    void autoDetectSensors();

    /**
     * @brief Initializes the drivers for all detected sensors.
     */
    void initializeSensors();

    /**
     * @brief Updates the robot's (x,y) position and heading from encoders/IMU.
     */
    void updateOdometry();    

    /**
     * @brief Performs MPU6050 accelerometer and gyroscope calibration.
     */
    CalibrationResult calibrateMPU();
    
public:
    // Public so calibration can call it
    void updateAllSensors(); 
    // --- Internal State ---
    RobotPose currentPose;
    long lastLeftEncoder = 0;
    long lastRightEncoder = 0;
    float lastHeading = 0.0;
};

#endif // WHEELIE_HAL_H