#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h> // Include the MPU6050 library

// Forward-declare the sensor classes instead of including the full headers.
// This breaks the dependency chain and solves the "No such file or directory" error.
class VL53L0X;

// ═══════════════════════════════════════════════════════════════════════════
// ROBUST SENSOR READER - Error handling and data filtering for sensors
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief A structure to hold sensor state, including error tracking.
 * Overloaded operators allow it to be used with the SensorState moving average filter.
 */
struct IMUData {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;

    IMUData() : accX(0), accY(0), accZ(0), gyroX(0), gyroY(0), gyroZ(0) {}

    // Overload the + operator to sum two IMUData objects component-wise
    IMUData operator+(const IMUData& other) const {
        IMUData result;
        result.accX = this->accX + other.accX;
        result.accY = this->accY + other.accY;
        result.accZ = this->accZ + other.accZ;
        result.gyroX = this->gyroX + other.gyroX;
        result.gyroY = this->gyroY + other.gyroY;
        result.gyroZ = this->gyroZ + other.gyroZ;
        return result;
    }

    // Overload the / operator to divide by a scalar (for averaging)
    IMUData operator/(int divisor) const {
        IMUData result;
        if (divisor == 0) return result; // Avoid division by zero
        result.accX = this->accX / divisor;
        result.accY = this->accY / divisor;
        result.accZ = this->accZ / divisor;
        result.gyroX = this->gyroX / divisor;
        result.gyroY = this->gyroY / divisor;
        result.gyroZ = this->gyroZ / divisor;
        return result;
    }
};

/**
 * @brief A structure to hold sensor state, including error tracking.
 * @tparam T The data type of the sensor reading (e.g., float, int).
 */
template<typename T>
struct SensorState {    
    // --- Public State ---
    T currentValue;         // The filtered (averaged) value, for external use.
    T lastGoodValue;        // The last known valid RAW reading.
    unsigned int errorCount;  // How many consecutive times the read has failed.
    unsigned long lastReadTime; // Timestamp of the last successful read.

    // --- For Moving Average Filter ---
    static const int HISTORY_SIZE = 5; // Window size for the moving average
    T history[HISTORY_SIZE];
    int historyIndex;
    bool historyInitialized; // Becomes true once the history buffer is full.

    SensorState() : errorCount(0), lastReadTime(0),
                    historyIndex(0), historyInitialized(false) {
        // Initialize history buffer to zero
        for (int i = 0; i < HISTORY_SIZE; ++i) {
            history[i] = T(); // Use default constructor for template type T
        }
    }

    /**
     * @brief Adds a new raw reading to the history and updates the moving average.
     * @param newValue The new raw sensor reading.
     */
    void addReading(T newValue) {
        history[historyIndex] = newValue;
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;

        // The buffer is considered "initialized" (full) after one full loop.
        if (!historyInitialized && historyIndex == 0) {
            historyInitialized = true;
        }

        // Calculate the average of the values in the buffer.
        T sum = T(); // Use default constructor for template type T
        // Use the full buffer size if initialized, otherwise use the current index.
        int count = historyInitialized ? HISTORY_SIZE : historyIndex;
        if (count == 0) count = 1; // Avoid division by zero on the very first reading.

        for (int i = 0; i < (historyInitialized ? HISTORY_SIZE : count); ++i) { sum = sum + history[i]; }
        currentValue = sum / (historyInitialized ? HISTORY_SIZE : count);
    }
};


class RobustSensorReader {
public:
    // --- Public State ---
    SensorState<int> tof;
    SensorState<float> ultrasonic;
    SensorState<IMUData> mpu;

    // --- Constants ---
    static const int MAX_CONSECUTIVE_ERRORS = 5; // Max failures before we consider the sensor offline.
    static const int SENSOR_RESET_THRESHOLD = 10; // After this many errors, try to re-init the sensor.

    /**
     * @brief Reads the VL53L0X ToF sensor with error handling and retries.
     * 
     * @param sensor The VL53L0X sensor object.
     * @return True if a valid reading was obtained, false otherwise.
     */
    bool readTofSensor(VL53L0X& sensor);

    /**
     * @brief Reads the Ultrasonic sensor with basic filtering.
     * @param trigPin The trigger pin for the HC-SR04.
     * @param echoPin The echo pin for the HC-SR04.
     * @return True always, as it provides a filtered value.
     */
    bool readUltrasonicSensor(int trigPin, int echoPin);

    /**
     * @brief Reads the MPU6050 IMU with error handling and filtering.
     * @param mpuSensor The Adafruit_MPU6050 sensor object.
     * @return True if a valid reading was obtained, false otherwise.
     */
    bool readMPUSensor(Adafruit_MPU6050& mpuSensor);
};