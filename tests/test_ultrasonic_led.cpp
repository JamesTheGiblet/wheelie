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

// --- Internal State for Non-Blocking Ultrasonic ---
enum UltrasonicState { US_IDLE, US_TRIGGERED, US_ECHO_IN_PROGRESS };
UltrasonicState ultrasonicState = US_IDLE;
unsigned long usTriggerTime = 0;
unsigned long usEchoStartTime = 0;
const unsigned long US_TIMEOUT_US = 38000;

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
  unsigned long currentMicros = micros();

  // State 1: Time to start a new reading
  if (ultrasonicState == US_IDLE && millis() >= nextReadingTime) {
      digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, HIGH);
      usTriggerTime = currentMicros;
      ultrasonicState = US_TRIGGERED;
  }

  // State 2: End the trigger pulse after 10us
  if (ultrasonicState == US_TRIGGERED && currentMicros - usTriggerTime >= 10) {
      digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);
      ultrasonicState = US_ECHO_IN_PROGRESS;
      usEchoStartTime = currentMicros; // Start timeout timer
  }

  // State 3: Wait for the echo pulse to finish or time out
  if (ultrasonicState == US_ECHO_IN_PROGRESS) {
      if (digitalRead(FRONT_ULTRASONIC_ECHO_PIN) == LOW) {
          long duration_us = currentMicros - usEchoStartTime;
          if (duration_us > 100) { // Ignore noise
              float newReading = (duration_us * SOUND_SPEED) / 2.0;
              if (newReading > 2.0) {
                  ultrasonicReadings[ultrasonicReadingIndex] = newReading;
                  ultrasonicReadingIndex = (ultrasonicReadingIndex + 1) % ULTRASONIC_FILTER_SIZE;
                  if (!ultrasonicFilterPrimed && ultrasonicReadingIndex == 0) {
                      ultrasonicFilterPrimed = true;
                  }
              }
          }
          ultrasonicState = US_IDLE;
          nextReadingTime = millis() + readingInterval;
      } else if (currentMicros - usEchoStartTime > US_TIMEOUT_US) {
          // Timeout
          ultrasonicState = US_IDLE;
          nextReadingTime = millis() + readingInterval;
      }
  }

  // Print the result periodically
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > readingInterval) {
    float total = 0;
    int numReadings = ultrasonicFilterPrimed ? ULTRASONIC_FILTER_SIZE : ultrasonicReadingIndex;
    if (numReadings > 0) {
        for (int i = 0; i < numReadings; i++) {
            total += ultrasonicReadings[i];
        }
        float averageDistance = total / numReadings;
        Serial.printf("Filtered Distance: %.2f cm\n", averageDistance);
    }
    lastPrint = millis();
  }
}