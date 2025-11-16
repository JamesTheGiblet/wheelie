#include <Arduino.h>
#include "pins.h"

// Minimal test for KY-009 RGB LED module
// Connect LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN to RGB LED

void setColor(int r, int g, int b) {
  analogWrite(LED_RED_PIN, r);
  analogWrite(LED_GREEN_PIN, g);
  analogWrite(LED_BLUE_PIN, b);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  Serial.println("KY-009 RGB LED Test");
  Serial.println("Cycling through colors. Observe the LED.");
}

enum ColorStep { RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, OFF };
ColorStep currentColorStep = RED;
unsigned long nextColorTime = 0;
const unsigned long colorInterval = 1000; // 1 second per color

void loop() {
  if (millis() < nextColorTime) {
    return;
  }

  nextColorTime = millis() + colorInterval;

  switch (currentColorStep) {
    case RED:
      Serial.println("Red"); setColor(255, 0, 0);
      break;
    case GREEN:
      Serial.println("Green"); setColor(0, 255, 0);
      break;
    case BLUE:
      Serial.println("Blue"); setColor(0, 0, 255);
      break;
    case YELLOW:
      Serial.println("Yellow"); setColor(255, 255, 0);
      break;
    case CYAN:
      Serial.println("Cyan"); setColor(0, 255, 255);
      break;
    case MAGENTA:
      Serial.println("Magenta"); setColor(255, 0, 255);
      break;
    case WHITE:
      Serial.println("White"); setColor(255, 255, 255);
      break;
    case OFF:
      Serial.println("Off"); setColor(0, 0, 0);
      break;
  }

  currentColorStep = (ColorStep)(((int)currentColorStep + 1) % 8);
}
