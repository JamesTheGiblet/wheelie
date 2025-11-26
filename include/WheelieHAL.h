#ifndef WHEELIE_HAL_H
#define WHEELIE_HAL_H

#include "HAL.h"
#include "motors.h"
#include "indicators.h"
#include "power_manager.h"
#include "calibration.h"
#include "logger.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "wifi_manager.h"
#include "RobustSensorReader.h" // Include the new error-handling wrapper
#include <VL53L0X.h>            // Include the actual sensor library


/**
 * @brief Wheelie's specific implementation of the Layer 1 HAL.
 * * This class "absorbs" all the hardware-specific logic from the
 * old project, translating it to the generic HAL interface.
 */
class WheelieHAL : public HAL {
public:
    WheelieHAL();
    ~WheelieHAL() override {}

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
    // --- HAL Interface Implementation ---
    bool init() override;
    void update() override;
    Vector2D getVelocity() const;
    Vector2D getObstacleRepulsion() override;
    RobotPose getPose() override;
    void setVelocity(const Vector2D& velocity) override;
    void setMaxSpeed(float speedRatio) override;
    void setLEDBrightness(int brightness) override;
    void emergencyStop() override;
    void setStatusLED(const LEDColor& color) override;
    void playTone(int frequency, int duration) override;
    float getBatteryVoltage() override;

    // --- Public Methods for Internal Use ---
    void updateAllSensors();
    float scaleVelocityToPWM(float velocity);
    void setMotorsFromVector(const Vector2D& velocity, float turnCorrection);

    // --- Internal State ---
    RobotPose currentPose;
    long lastLeftEncoder;
    long lastRightEncoder;
    float lastHeading;

    // --- PID Controller for Heading Correction ---
    float pid_p = 2.5f, pid_i = 0.02f, pid_d = 0.5f;
    float integral = 0.0f, previous_error = 0.0f;
    Vector2D targetVelocity;
    unsigned long lastPidTime = 0;

    // --- Sensor Hardware and Robust Readers ---
    VL53L0X tofSensor;
    Adafruit_MPU6050 mpu;

    RobustSensorReader sensorReader;
};

#endif // WHEELIE_HAL_H