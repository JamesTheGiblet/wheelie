#include "indicators.h"
#include "main.h"

// ═══════════════════════════════════════════════════════════════════════════
// INDICATORS IMPLEMENTATION - LED and buzzer control
// ═══════════════════════════════════════════════════════════════════════════

// Static variables for non-blocking operations
static unsigned long buzzerStopTime = 0;
static unsigned long ledBlinkStopTime = 0;
static bool isBlinking = false;

void setupIndicators() {
  // Configure LED pins
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  
  // Setup PWM for buzzer
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  
  // Initialize indicators to off state
  clearLEDs();
  setBuzzer(0);
}

void clearLEDs() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
}

void setLEDColor(bool r, bool g, bool b) {
  digitalWrite(LED_RED_PIN, r ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, g ? HIGH : LOW);
  digitalWrite(LED_BLUE_PIN, b ? HIGH : LOW);
}

void setLEDColor(const LEDColor& color) {
  setLEDColor(color.red, color.green, color.blue);
}

void setBuzzer(int value) {
  ledcWrite(BUZZER_CHANNEL, value);
}

void playTone(int freq, int duration) {
  ledcSetup(BUZZER_CHANNEL, freq, PWM_RESOLUTION); // Set the freq
  setBuzzer(128); // Turn buzzer on
  buzzerStopTime = millis() + duration; // Set when to turn it off
}

void playBeep(int duration) {
  // playBeep just uses the default frequency
  playTone(BUZZER_FREQ, duration);
}

void buzz(int freq, int duration) {
  // Alias for playTone - used by calibration system
  playTone(freq, duration);
}

void playMelody(const int* notes, const int* durations, int length) {
  for (int i = 0; i < length; i++) {
    // This is still blocking, but we accept it for non-critical animations
    ledcSetup(BUZZER_CHANNEL, notes[i], PWM_RESOLUTION);
    setBuzzer(128);
    delay(durations[i]);
    setBuzzer(0);
    delay(50);  // Brief pause between notes
  }
}

void indicateSystemStatus(RobotStateEnum state) {
    // This function is only called by indicators_update()
    // AFTER it has already checked for error states.
    // We only need to handle the "normal" states here.
  
  switch (state) {
    case ROBOT_BOOTING:
    case ROBOT_CALIBRATING:
    case ROBOT_TESTING:
        setLEDColor(LEDColors::YELLOW); // Booting/Working
        break;

    case ROBOT_IDLE:
        setLEDColor(LEDColors::BLUE);   // Ready and waiting
        break;

    case ROBOT_EXPLORING:
        setLEDColor(LEDColors::GREEN);  // Moving normally
        break;

    case ROBOT_AVOIDING_OBSTACLE:
    case ROBOT_PLANNING_ROUTE:
    case ROBOT_RECOVERING_STUCK:
        setLEDColor(LEDColors::CYAN);   // Thinking or avoiding
        break;
        
    case ROBOT_SAFE_MODE:
        setLEDColor(LEDColors::MAGENTA); // Limited functionality
        break;

    // NOTE: All safety-stop and error states
    // (like ROBOT_ERROR, ROBOT_TILTED, etc.)
    // are handled by errorBlinkPattern() in indicators_update().
    // We don't need to set a color for them here.
    default:
        clearLEDs();
        break;
  }
}

void indicateError() {
  setLEDColor(LEDColors::RED);
  playTone(2000, 200); // This is now non-blocking
}

void indicateWarning() {
  setLEDColor(LEDColors::YELLOW);
  playTone(1500, 100); // This is now non-blocking
}

void indicateSuccess() {
  setLEDColor(LEDColors::GREEN);
  playTone(1000, 150); // This is now non-blocking
}

void startupAnimation() {
  Serial.println("⚡ Initializing systems...\n");
  
  const char* stages[] = {
    "Loading motor controllers",
    "Initializing LED matrix",
    "Configuring audio system",
    "Starting sensor array",
    "Establishing I2C bus",
    "Calibrating systems"
  };
  
  for (int i = 0; i < 6; i++) {
    Serial.print("   [");
    for (int j = 0; j < 20; j++) {
      if (j < (i + 1) * 3) Serial.print("█");
      else Serial.print("░");
    }
    Serial.print("] ");
    Serial.println(stages[i]);
    
    // Visual feedback
    if (i < 3) digitalWrite(LED_RED_PIN + i, HIGH);
    // Temporarily blocking for startup is acceptable
    ledcSetup(BUZZER_CHANNEL, 1000 + (i * 200), PWM_RESOLUTION);
    setBuzzer(128);
    delay(80);
    setBuzzer(0);

    delay(300);
  }
  
  clearLEDs();
  Serial.println();
}

void victoryAnimation() {
  const int victoryNotes[] = {523, 659, 784, 1047};
  const int victoryDurations[] = {200, 200, 200, 400};
  // Blocking melody is acceptable here
  for (int i = 0; i < 4; i++) {
    ledcSetup(BUZZER_CHANNEL, victoryNotes[i], PWM_RESOLUTION);
    setBuzzer(128);
    delay(victoryDurations[i]);
    setBuzzer(0);
    delay(50);
  }

  // LED celebration
  for (int i = 0; i < 3; i++) {
    setLEDColor(LEDColors::GREEN);
    delay(200);
    clearLEDs();
    delay(200);
  }
}

void errorBlinkPattern() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  
  if (millis() - lastBlink > 250) {
    blinkState = !blinkState;
    setLEDColor(blinkState, false, false); // Red blink
    lastBlink = millis();
  }
}

void errorAnimation() {
  // Error animation for calibration failures
  for (int i = 0; i < 5; i++) {
    setLEDColor(LEDColors::RED);
    // Blocking is acceptable in a halt-on-failure loop
    ledcSetup(BUZZER_CHANNEL, 2000, PWM_RESOLUTION);
    setBuzzer(128);
    delay(100);
    setBuzzer(0);
    clearLEDs();
    delay(100);
  }
}

void indicators_update() {
    // 1. Handle buzzer state
    if (buzzerStopTime > 0 && millis() >= buzzerStopTime) {
        setBuzzer(0); // Turn off buzzer
        buzzerStopTime = 0; // Clear the timer
        
        // Restore default frequency
        ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, PWM_RESOLUTION);
    }
    
    // 2. Handle global robot state indicators
    RobotStateEnum currentState = getCurrentState();
    
    if (currentState == ROBOT_ERROR ||
        currentState == ROBOT_SAFETY_STOP_EDGE ||
        currentState == ROBOT_SAFETY_STOP_TILT)
    {
        // If in an error state, run the non-blocking blinker
        errorBlinkPattern();
    } 
    else 
    {
        // Otherwise, show the normal status color
        indicateSystemStatus(currentState);
    }
}