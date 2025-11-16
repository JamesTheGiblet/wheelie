#include <Arduino.h>
#include "pins.h"

// LM393 Sound Sensor Test
// Connect OUT pin of sensor to SOUND_SENSOR_PIN (see pins.h)

void setup() {
  Serial.begin(115200);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  Serial.println("LM393 Sound Sensor Test - Only Sound Sensor enabled");
  Serial.println("Digital value: 0 = quiet, 1 = sound detected");
}

unsigned long lastCheck = 0;
const unsigned long checkInterval = 200;

void loop() {
  if (millis() - lastCheck >= checkInterval) {
    int sound = digitalRead(SOUND_SENSOR_PIN);
    Serial.print("Sound Detected: ");
    Serial.println(sound);
    lastCheck = millis();
  }
}
