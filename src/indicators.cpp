#include "indicators.h"
#include "globals.h"

// ═══════════════════════════════════════════════════════════════════════════
// INDICATORS IMPLEMENTATION - LED and buzzer control
// ═══════════════════════════════════════════════════════════════════════════

// Static variables for non-blocking operations
static unsigned long buzzerStopTime = 0;
static unsigned long ledBlinkStopTime = 0;
static bool isBlinking = false;

// Static variables for non-blocking melody playback
static const int* currentMelodyNotes = nullptr;
static const int* currentMelodyDurations = nullptr;
static int melodyLength = 0;
static int currentNoteIndex = 0;
static unsigned long nextMelodyEventTime = 0;

// ═══════════════════════════════════════════════════════════════════════════
// GENERIC ANIMATION ENGINE
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief How to Add a New Non-Blocking Animation
 *
 * This system uses a generic, non-blocking animation engine. To add a new
 * animation (e.g., for a "low battery" warning or "task complete" signal),
 * follow these steps:
 *
 * 1.  **Create a New Function:**
 *     Define a new function, like `void lowBatteryAnimation()`.
 *
 * 2.  **Define the Animation Sequence:**
 *     Inside your new function, create a `static const` array of `AnimationStep`
 *     structs. Each element in the array is one "frame" of your animation.
 *
 *     The `AnimationStep` struct has four fields:
 *     - `{ const LEDColor* ledColor, int soundFrequency, int soundDuration, int stepDuration }`
 *     - `ledColor`: Pointer to a color (e.g., `&LEDColors::RED`). Use `&LEDColors::OFF` to turn LEDs off.
 *     - `soundFrequency`: Tone frequency in Hz (e.g., `1500`). Use `0` for no sound.
 *     - `soundDuration`: How long the tone plays in milliseconds (e.g., `100`).
 *     - `stepDuration`: Total duration of this frame. The next frame starts after this time.
 *
 * 3.  **Call `playAnimation()`:**
 *     At the end of your function, call `playAnimation(yourStepsArray, numSteps)`,
 *     where `numSteps` is calculated with `sizeof(yourStepsArray) / sizeof(AnimationStep)`.
 *
 * 4.  **Trigger the Animation:**
 *     Call your new function (`lowBatteryAnimation()`) from anywhere in the code
 *     where the animation should start (e.g., from `power_manager.cpp` when the
 *     battery is low). The animation will run in the background without blocking.
 *
 * See `victoryAnimation()` or `startupAnimation()` for concrete examples.
 */

typedef struct {
    const LEDColor* ledColor; // Pointer to a color, or nullptr to keep current
    int soundFrequency;       // 0 for no sound
    int soundDuration;        // Duration of the sound
    int stepDuration;         // Total duration of the step
} AnimationStep;

// Static variables for the generic animation player
static const AnimationStep* currentAnimation = nullptr;
static int currentAnimationNumSteps = 0;
static int currentAnimationStepIndex = 0;
static unsigned long nextAnimationEventTime = 0;

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
  // This function is now NON-BLOCKING.
  // It starts the melody, which is then handled by indicators_update().
  if (length <= 0) {
    currentMelodyNotes = nullptr; // Stop any playing melody
    return;
  }
  currentMelodyNotes = notes;
  currentMelodyDurations = durations;
  melodyLength = length;
  currentNoteIndex = 0;
  nextMelodyEventTime = millis(); // Start playing the first note immediately
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

void playAnimation(const AnimationStep* animation, int numSteps) {
    // This function is NON-BLOCKING.
    // It starts the animation, which is then handled by indicators_update().
    currentAnimation = animation;
    currentAnimationNumSteps = numSteps;
    currentAnimationStepIndex = 0;
    nextAnimationEventTime = millis(); // Start animation immediately
}

void startupAnimation() {
  // Define the startup animation sequence using our generic structure
  static const AnimationStep startupSteps[] = {
    { &LEDColors::RED,    1000, 80, 380 }, // Step 0: Red LED, 1000Hz tone
    { &LEDColors::YELLOW, 1200, 80, 380 }, // Step 1: Red+Green LED, 1200Hz tone
    { &LEDColors::WHITE,  1400, 80, 380 }, // Step 2: All LEDs, 1400Hz tone
    { &LEDColors::WHITE,  1600, 80, 380 }, // Step 3
    { &LEDColors::WHITE,  1800, 80, 380 }, // Step 4
    { &LEDColors::WHITE,  2000, 80, 380 }  // Step 5
  };
  playAnimation(startupSteps, sizeof(startupSteps) / sizeof(AnimationStep));
  Serial.println("⚡ Initializing systems... (Animation will play in background)");
}

void victoryAnimation() {
  // Define the victory animation sequence
  static const AnimationStep victorySteps[] = {
    { &LEDColors::GREEN, 523, 150, 200 }, // Flash 1 ON
    { &LEDColors::OFF,     0,   0, 200 }, // Flash 1 OFF
    { &LEDColors::GREEN, 659, 150, 200 }, // Flash 2 ON
    { &LEDColors::OFF,     0,   0, 200 }, // Flash 2 OFF
    { &LEDColors::GREEN, 784, 150, 200 }, // Flash 3 ON
    { &LEDColors::OFF,  1047, 300, 400 }  // Flash 3 OFF (with final note)
  };
  playAnimation(victorySteps, sizeof(victorySteps) / sizeof(AnimationStep));
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
  // This is now a non-blocking animation using the generic animation engine.
  // It's used for critical failures like in calibration.
  static const AnimationStep errorSteps[] = {
    { &LEDColors::RED, 2000, 100, 100 }, // Beep ON, LED ON
    { &LEDColors::OFF,    0,   0, 100 }, // Beep OFF, LED OFF
    { &LEDColors::RED, 2000, 100, 100 },
    { &LEDColors::OFF,    0,   0, 100 },
    { &LEDColors::RED, 2000, 100, 100 },
    { &LEDColors::OFF,    0,   0, 100 }
  };
  playAnimation(errorSteps, sizeof(errorSteps) / sizeof(AnimationStep));
}
void indicators_update() {
    unsigned long currentTime = millis();

    // 1. Handle buzzer state
    if (buzzerStopTime > 0 && currentTime >= buzzerStopTime) {
        setBuzzer(0); // Turn off buzzer
        buzzerStopTime = 0; // Clear the timer
        
        // Restore default frequency
        ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, PWM_RESOLUTION);
    }

    // 2. Handle non-blocking melody playback
    if (currentMelodyNotes != nullptr && currentTime >= nextMelodyEventTime) {
        if (currentNoteIndex < melodyLength) {
            // Play the current note
            int note = currentMelodyNotes[currentNoteIndex];
            int duration = currentMelodyDurations[currentNoteIndex];

            ledcSetup(BUZZER_CHANNEL, note, PWM_RESOLUTION);
            setBuzzer(128); // Turn buzzer on

            // Schedule when the note should end and the pause should begin
            nextMelodyEventTime = currentTime + duration;
            buzzerStopTime = nextMelodyEventTime; // Use existing timer to turn off note

            currentNoteIndex++;
        } else {
            // Melody finished
            currentMelodyNotes = nullptr;
        }
    }

    // 3. Handle Generic Animation Playback
    if (currentAnimation != nullptr && currentTime >= nextAnimationEventTime) {
        if (currentAnimationStepIndex < currentAnimationNumSteps) {
            const AnimationStep* step = &currentAnimation[currentAnimationStepIndex];

            // Execute step actions
            if (step->ledColor != nullptr) {
                setLEDColor(*step->ledColor);
            }
            if (step->soundFrequency > 0 && step->soundDuration > 0) {
                playTone(step->soundFrequency, step->soundDuration);
            }

            // Schedule the next step
            nextAnimationEventTime = currentTime + step->stepDuration;
            currentAnimationStepIndex++;
        } else {
            // Animation finished
            currentAnimation = nullptr;
            clearLEDs();
        }
    }

    // If an animation is playing, it overrides other indicators
    if (currentAnimation != nullptr) return;
    
    // 4. Handle global robot state indicators
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