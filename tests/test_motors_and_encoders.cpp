#include <Arduino.h>
#include "pins.h"

// Combined test for motors and encoders
// Runs each motor in sequence and prints encoder counts, direction, and timing live

// Motor control
void setLeftMotor(int pwmA, int pwmB) {
  analogWrite(IN1_PIN, pwmA);
  analogWrite(IN2_PIN, pwmB);
}
void setRightMotor(int pwmA, int pwmB) {
  analogWrite(IN3_PIN, pwmA);
  analogWrite(IN4_PIN, pwmB);
}
void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
}

// Encoder variables
volatile long leftCount = 0;
volatile long rightCount = 0;
volatile int8_t leftDir = 0;   // +1 or -1
volatile int8_t rightDir = 0;
volatile unsigned long leftLastTime = 0;
volatile unsigned long rightLastTime = 0;

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
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  stopMotors();
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onRightEncoder, RISING);
  Serial.println("Motor + Encoder Test");
  Serial.println("Motors will run in sequence. Encoder counts, direction, and timing will be displayed live.");
}

void printEncoders() {
  static long lastLeft = 0, lastRight = 0;
  static unsigned long lastPrint = 0;
  static unsigned long lastLeftTime = 0, lastRightTime = 0;
  noInterrupts();
  long l = leftCount, r = rightCount;
  int8_t ld = leftDir, rd = rightDir;
  unsigned long lTime = leftLastTime, rTime = rightLastTime;
  interrupts();
  if (l != lastLeft || r != lastRight || millis() - lastPrint > 300) {
    Serial.print("Left: "); Serial.print(l);
    Serial.print(" (dir: "); Serial.print(ld); Serial.print(", dt: ");
    Serial.print(lTime - lastLeftTime); Serial.print("us) | ");
    Serial.print("Right: "); Serial.print(r);
    Serial.print(" (dir: "); Serial.print(rd); Serial.print(", dt: ");
    Serial.print(rTime - lastRightTime); Serial.println("us)");
    lastLeft = l; lastRight = r;
    lastLeftTime = lTime; lastRightTime = rTime;
    lastPrint = millis();
  }
}

void leftForward() { setLeftMotor(200, 0); setRightMotor(0, 0); }
void leftReverse() { setLeftMotor(0, 200); setRightMotor(0, 0); }
void rightForward() { setLeftMotor(0, 0); setRightMotor(200, 0); }
void rightReverse() { setLeftMotor(0, 0); setRightMotor(0, 200); }
void bothForward() { setLeftMotor(200, 0); setRightMotor(200, 0); }
void bothReverse() { setLeftMotor(0, 200); setRightMotor(0, 200); }

enum MotorEncoderTestStep {
  ME_LEFT_FWD, ME_LEFT_REV, ME_RIGHT_FWD, ME_RIGHT_REV, ME_BOTH_FWD, ME_BOTH_REV, ME_PAUSE
};
MotorEncoderTestStep me_currentStep = ME_LEFT_FWD;
unsigned long me_nextActionTime = 0;
const unsigned long me_runDuration = 1500;
const unsigned long me_pauseDuration = 500;

void loop() {
  if (millis() >= me_nextActionTime) {
    stopMotors();
    me_currentStep = (MotorEncoderTestStep)(((int)me_currentStep + 1) % 7);
    me_nextActionTime = millis() + (me_currentStep == ME_PAUSE ? me_pauseDuration : me_runDuration);

    if(me_currentStep == ME_LEFT_FWD) Serial.println("\n--- Left Fwd ---");
    if(me_currentStep == ME_LEFT_REV) Serial.println("\n--- Left Rev ---");
    if(me_currentStep == ME_RIGHT_FWD) Serial.println("\n--- Right Fwd ---");
    if(me_currentStep == ME_RIGHT_REV) Serial.println("\n--- Right Rev ---");
    if(me_currentStep == ME_BOTH_FWD) Serial.println("\n--- Both Fwd ---");
    if(me_currentStep == ME_BOTH_REV) Serial.println("\n--- Both Rev ---");
  }

  if(me_currentStep == ME_LEFT_FWD) leftForward();
  if(me_currentStep == ME_LEFT_REV) leftReverse();
  if(me_currentStep == ME_RIGHT_FWD) rightForward();
  if(me_currentStep == ME_RIGHT_REV) rightReverse();
  if(me_currentStep == ME_BOTH_FWD) bothForward();
  if(me_currentStep == ME_BOTH_REV) bothReverse();

  printEncoders();
}
