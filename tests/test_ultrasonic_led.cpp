#include <Arduino.h>

// Pin definitions from your request
#define FRONT_ULTRASONIC_TRIG_PIN 16
#define FRONT_ULTRASONIC_ECHO_PIN 32

// Speed of sound in cm/microsecond
#define SOUND_SPEED 0.0343

// --- Internal State for Ultrasonic Filtering ---
const int ULTRASONIC_FILTER_SIZE = 5;
float ultrasonicReadings[ULTRASONIC_FILTER_SIZE] = {0};
int ultrasonicReadingIndex = 0;
bool ultrasonicFilterPrimed = false;

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- HC-SR04 Basic pulseIn Test ---");
  
  // Set pin modes
  pinMode(FRONT_ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ULTRASONIC_ECHO_PIN, INPUT);
}

unsigned long nextReadingTime = 0;
const unsigned long readingInterval = 100; // ms

void loop() {
  if (millis() >= nextReadingTime) {
    // --- 1. Send the Trigger Pulse ---
    // This part is very short and acceptable to block for microseconds
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);

    // --- 2. Read the Echo Pulse ---
    // pulseIn() is blocking, but for a simple test script, it's acceptable
    // as we have already refactored the main HAL to be non-blocking.
    long duration_us = pulseIn(FRONT_ULTRASONIC_ECHO_PIN, HIGH, 38000);

    // --- 3. Calculate, Filter, and Store Distance ---
    if (duration_us > 0) {
        float newReading = (duration_us * SOUND_SPEED) / 2.0;

        // The HC-SR04 sensor's minimum distance is ~2cm. Ignore noise.
        if (newReading > 2.0) {
            // Add the new valid reading to our circular buffer
            ultrasonicReadings[ultrasonicReadingIndex] = newReading;
            ultrasonicReadingIndex = (ultrasonicReadingIndex + 1) % ULTRASONIC_FILTER_SIZE;

            if (!ultrasonicFilterPrimed && ultrasonicReadingIndex == 0) {
                ultrasonicFilterPrimed = true;
            }
        }
    }

    // --- 4. Print the Result ---
    float total = 0;
    int numReadings = ultrasonicFilterPrimed ? ULTRASONIC_FILTER_SIZE : ultrasonicReadingIndex;
    if (numReadings > 0) {
        for (int i = 0; i < numReadings; i++) {
            total += ultrasonicReadings[i];
        }
        float averageDistance = total / numReadings;
        Serial.print("Filtered Distance: ");
        Serial.print(averageDistance);
        Serial.println(" cm");
    }

    // Schedule the next reading
    nextReadingTime = millis() + readingInterval;
  }
}