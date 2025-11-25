#include "RobustSensorReader.h"
#include <VL53L0X.h> // Now we can safely include the full header here
#include "calibration.h" // Include for CalibrationData definition

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

    // Access the global calibration data
    extern CalibrationData calibData;
    extern bool isCalibrated;

    // The getEvent() function returns true on success
    if (mpuSensor.getEvent(&a, &g, &temp)) {
        // Valid reading: update filter and reset errors
        IMUData reading;
        float accX_cal = a.acceleration.x - (isCalibrated ? calibData.mpuOffsets.accelX : 0);
        float accY_cal = a.acceleration.y - (isCalibrated ? calibData.mpuOffsets.accelY : 0);
        float accZ_cal = a.acceleration.z - (isCalibrated ? calibData.mpuOffsets.accelZ : 0);

        // Apply software calibration offsets if available
        // Correctly calculate Roll (X-axis tilt) and Pitch (Y-axis tilt) from the calibrated acceleration vector
        float roll = atan2(accY_cal, accZ_cal) * 180.0 / M_PI;
        float pitch = atan2(-accX_cal, sqrt(accY_cal * accY_cal + accZ_cal * accZ_cal)) * 180.0 / M_PI;

        reading.accX = roll - (isCalibrated ? calibData.mpuOffsets.baselineTiltX : 0);
        reading.accY = pitch - (isCalibrated ? calibData.mpuOffsets.baselineTiltY : 0);
        reading.accZ = a.acceleration.z - (isCalibrated ? calibData.mpuOffsets.accelZ : 0);
        reading.gyroX = g.gyro.x - (isCalibrated ? calibData.mpuOffsets.gyroX : 0);
        reading.gyroY = g.gyro.y - (isCalibrated ? calibData.mpuOffsets.gyroY : 0);
        reading.gyroZ = g.gyro.z - (isCalibrated ? calibData.mpuOffsets.gyroZ : 0);
        reading.temp = temp.temperature; // Store the temperature

        // --- Sanity Check: Detect if sensor is stuck returning zeros ---
        if (reading.accX == 0 && reading.accY == 0 && reading.gyroZ == 0 && reading.temp == 0) {
            mpu.errorCount++;
            Serial.printf("⚠️ MPU6050 returned all zeros (%d consecutive times).\n", mpu.errorCount);
            Serial.println("   This often indicates a bad I2C connection or a faulty sensor.");
            return false; // Treat this as a failure
        }

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