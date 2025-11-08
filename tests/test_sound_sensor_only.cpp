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

void loop() {
  int sound = digitalRead(SOUND_SENSOR_PIN);
  Serial.print("Sound Detected: ");
  Serial.println(sound);
  delay(200);
}
