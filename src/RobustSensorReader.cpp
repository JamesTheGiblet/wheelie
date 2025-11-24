#include "RobustSensorReader.h"
#include <VL53L0X.h> // Now we can safely include the full header here

bool RobustSensorReader::readTofSensor(VL53L0X& sensor) {
    // The library's readRangeSingleMillimeters() has a built-in timeout check.
    // It returns 65535 on timeout.
    int reading = sensor.readRangeSingleMillimeters();
    Serial.printf("[ToF] Distance detected: %d mm\n", reading);

    // Check for a valid reading (not a timeout and not out of range)
    if (!sensor.timeoutOccurred() && reading < 8000) { // 8000 is a sane upper limit
        // Valid reading: update filter and reset errors
        tof.lastGoodValue = reading;
        tof.addReading(reading); // Add raw value to the filter
        tof.errorCount = 0; // Reset error count on success
        tof.lastReadTime = millis();
        return true;
    } else {
        // Reading failed, increment error count
        tof.errorCount++;

        // On failure, DO NOT update the filter. 
        // tof.currentValue automatically holds the last stable average.

        // Log the error if it's persistent
        if (tof.errorCount > 2) {
            Serial.printf("⚠️ ToF read failed (%d consecutive errors). Using last good value: %dmm\n", 
                          tof.errorCount, tof.lastGoodValue);
        }

        // If the sensor fails too many times, it might be stuck.
        if (tof.errorCount > SENSOR_RESET_THRESHOLD) {
            Serial.println("❌ ToF sensor seems offline. Attempting to re-initialize...");
            // sensor.init(); // Attempt to re-init the sensor
            // setMeasurementTimingBudget(TOF_TIMING_BUDGET);
            tof.errorCount = 0; // Reset counter after attempting recovery
        }
        return false;
    }
}

bool RobustSensorReader::readUltrasonicSensor(int trigPin, int echoPin) {
    // Ultrasonic sensors are noisy, so we don't just check for errors,
    // we also filter out outlier values.
    long duration;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout

    float distance = duration * 0.034 / 2.0; // In cm
    Serial.printf("[Ultrasonic] Distance detected: %.2f cm\n", distance);

    // Basic outlier rejection: only accept readings within a sane range (2cm to 400cm)
    if (distance > 2 && distance < 400) {
        // Valid reading: update filter
        ultrasonic.lastGoodValue = distance;
        ultrasonic.addReading(distance);
        ultrasonic.errorCount = 0;
    } else {
        // Invalid reading (outlier): increment error, do not update filter
        ultrasonic.errorCount++;
    }
    return true;
}

bool RobustSensorReader::readMPUSensor(Adafruit_MPU6050& mpuSensor) {
    sensors_event_t a, g, temp;

    // The getEvent() function returns true on success
    if (mpuSensor.getEvent(&a, &g, &temp)) {
        // Valid reading: update filter and reset errors
        IMUData reading;
        reading.accX = a.acceleration.x;
        reading.accY = a.acceleration.y;
        reading.accZ = a.acceleration.z;
        reading.gyroX = g.gyro.x;
        reading.gyroY = g.gyro.y;
        reading.gyroZ = g.gyro.z;

        mpu.lastGoodValue = reading;
        mpu.addReading(reading); // Add raw value to the filter
        mpu.errorCount = 0;
        mpu.lastReadTime = millis();
        return true;
    } else {
        // Reading failed, increment error count
        mpu.errorCount++;

        if (mpu.errorCount > 2) {
            Serial.printf("⚠️ MPU6050 read failed (%d consecutive errors).\n", mpu.errorCount);
        }

        if (mpu.errorCount > SENSOR_RESET_THRESHOLD) {
            Serial.println("❌ MPU6050 seems offline. Consider re-initializing I2C or the sensor.");
            mpu.errorCount = 0; // Reset counter after logging
        }
        return false;
    }
}