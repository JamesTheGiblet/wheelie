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

void loop() {
  // Red
  setColor(255, 0, 0);
  Serial.println("Red");
  delay(700);
  // Green
  setColor(0, 255, 0);
  Serial.println("Green");
  delay(700);
  // Blue
  setColor(0, 0, 255);
  Serial.println("Blue");
  delay(700);
  // Yellow
  setColor(255, 255, 0);
  Serial.println("Yellow");
  delay(700);
  // Cyan
  setColor(0, 255, 255);
  Serial.println("Cyan");
  delay(700);
  // Magenta
  setColor(255, 0, 255);
  Serial.println("Magenta");
  delay(700);
  // White
  setColor(255, 255, 255);
  Serial.println("White");
  delay(700);
  // Off
  setColor(0, 0, 0);
  Serial.println("Off");
  delay(700);
}
