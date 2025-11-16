#include <Arduino.h>
#include "pins.h"

// Minimal test for LM393 H2010 encoders (manual wheel movement)
volatile long leftCount = 0;
volatile long rightCount = 0;
volatile int8_t leftDir = 0;   // +1 or -1
volatile int8_t rightDir = 0;
volatile unsigned long leftLastTime = 0;
volatile unsigned long rightLastTime = 0;

// For direction: sample B pin on A edge (simple, not quadrature)
void IRAM_ATTR onLeftEncoder() {
  int b = digitalRead(ENCODER_B_PIN);
  leftDir = b ? 1 : -1;
  leftCount += leftDir;
  leftLastTime = micros();
}

void IRAM_ATTR onRightEncoder() {
  int a = digitalRead(ENCODER_A_PIN);
  rightDir = a ? 1 : -1;
  rightCount += rightDir;
  rightLastTime = micros();
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onRightEncoder, RISING);
  Serial.println("LM393 H2010 Encoder Test");
  Serial.println("Move wheels by hand. Counts, direction, and timing will be displayed.");
  Serial.println("Debug: Stuck detection every 2s, direction: +1=forward, -1=reverse");
}

void loop() {
  static long lastLeft = 0, lastRight = 0;
  static unsigned long lastPrint = 0;
  static unsigned long lastCheck = 0;
  static unsigned long lastLeftTime = 0, lastRightTime = 0;
  static int stuckWarned = 0;

  // Print on change or every 500ms
  if (leftCount != lastLeft || rightCount != lastRight || millis() - lastPrint > 500) {
    noInterrupts();
    long l = leftCount, r = rightCount;
    int8_t ld = leftDir, rd = rightDir;
    unsigned long lTime = leftLastTime, rTime = rightLastTime;
    interrupts();
    Serial.print("Left: "); Serial.print(l);
    Serial.print(" (dir: "); Serial.print(ld); Serial.print(", dt: ");
    Serial.print(lTime - lastLeftTime); Serial.print("us) | ");
    Serial.print("Right: "); Serial.print(r);
    Serial.print(" (dir: "); Serial.print(rd); Serial.print(", dt: ");
    Serial.print(rTime - lastRightTime); Serial.println("us)");
    lastLeft = l; lastRight = r;
    lastLeftTime = lTime; lastRightTime = rTime;
    lastPrint = millis();
    stuckWarned = 0;
  }

  // Fallback: stuck detection (no change for 2s)
  if ((millis() - lastCheck > 2000) && !stuckWarned) {
    if (leftCount == lastLeft && rightCount == lastRight) {
      Serial.println("Warning: Encoder counts stuck for 2 seconds!");
      stuckWarned = 1;
    }
    lastCheck = millis();
  }
  // No delay needed, loop runs continuously
}
