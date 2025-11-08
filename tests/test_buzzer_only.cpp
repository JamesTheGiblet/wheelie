#include <Arduino.h>
#include "pins.h"

// Minimal test for KY-006 Passive Buzzer
// Connect BUZZER_PIN to signal pin of buzzer

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println("KY-006 Passive Buzzer Test");
  Serial.println("Plays tones and beeps. Listen for sound.");
}

void loop() {
  // Play a simple beep
  Serial.println("Beep!");
  tone(BUZZER_PIN, 2000, 200); // 2kHz, 200ms
  delay(500);

  // Play a short melody
  int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
  for (int i = 0; i < 8; i++) {
    tone(BUZZER_PIN, melody[i], 120);
    delay(150);
  }
  delay(1000);
}
