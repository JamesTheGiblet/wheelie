#ifndef INDICATORS_H
#define INDICATORS_H

#include <Arduino.h>
#include "pins.h"
#include "config.h"
#include "globals.h"

// ═══════════════════════════════════════════════════════════════════════════
// INDICATORS - LED and buzzer control for visual and audio feedback
// ═══════════════════════════════════════════════════════════════════════════

// LED Setup and Control
void setupIndicators();
void clearLEDs();
void indicators_update();
void setLEDColor(bool r, bool g, bool b);
void setLEDColor(const LEDColor& color);

// Buzzer Control
void setBuzzer(int value);
void playBeep(int duration);
void playTone(int freq, int duration);
void buzz(int freq, int duration);  // Alias for playTone
void playMelody(const int* notes, const int* durations, int length);

// Status Indication
void indicateSystemStatus(RobotStateEnum state);
void indicateError();
void indicateWarning();
void indicateSuccess();

// Special Effects
void startupAnimation();
void victoryAnimation();
void errorAnimation();  // Added for calibration errors
void errorBlinkPattern();

#endif // INDICATORS_H