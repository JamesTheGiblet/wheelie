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

int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
int currentNote = 0;
unsigned long nextNoteTime = 0;
const unsigned long noteDuration = 150;
const unsigned long pauseDuration = 50;

void loop() {
  if (millis() < nextNoteTime) {
    return;
  }

  if (currentNote == 0) {
    Serial.println("Playing melody...");
  }

  if (currentNote < 8) {
    tone(BUZZER_PIN, melody[currentNote], noteDuration - pauseDuration);
    currentNote++;
    nextNoteTime = millis() + noteDuration;
  } else {
    // Melody finished, wait before restarting
    currentNote = 0;
    nextNoteTime = millis() + 2000; // 2-second pause
    Serial.println("Melody complete. Restarting soon...");
  }
}
