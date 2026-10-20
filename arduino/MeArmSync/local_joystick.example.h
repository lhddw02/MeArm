#pragma once

// OPTIONAL example for Uno / classic ATmega328P Nano, four 10k potentiometers
// (or two two-axis joysticks) and a held-enable button. All values are commanded
// positions; these input pots DO NOT measure the arm's actual servo positions.
// To enable, put these lines ABOVE #include "calibration.h" in MeArmSync.ino:
//   #define MEARM_SOURCE_LOCAL true
//   #define MEARM_LOCAL_JOYSTICKS
// Connections: A0 yaw, A1 shoulder, A2 forearm, A3 grip; pot ends to5V/GND.
// D2 enable button to GND, internal pullup. Keep these pins free of servos.
// Release the button after boot, then hold it to arm; release to detach.

void updateLocalController() {
  static bool initialized = false;
  static bool observedRelease = false;
  static bool previousRawPressed = false;
  static bool stablePressed = false;
  static bool wasEnabled = false;
  static unsigned long lastTransitionMs = 0;
  static unsigned long lastSampleMs = 0;
  const uint8_t enablePin = 2;
  const uint8_t analogPins[4] = {A0, A1, A2, A3};
  const unsigned long now = millis();
  if (!initialized) {
    pinMode(enablePin, INPUT_PULLUP);
    for (uint8_t i = 0; i < 4; ++i) {
      pinMode(analogPins[i], INPUT);
      if (MEARM_PINS[i] == enablePin ||
          MEARM_PINS[i] == A0 || MEARM_PINS[i] == A1 ||
          MEARM_PINS[i] == A2 || MEARM_PINS[i] == A3) {
        configurationValid = false;
      }
    }
    initialized = true;
    previousRawPressed = digitalRead(enablePin) == LOW;
    lastTransitionMs = now;
    return;
  }
  const bool pressed = digitalRead(enablePin) == LOW;
  if (pressed != previousRawPressed) {
    previousRawPressed = pressed;
    lastTransitionMs = now;
  }
  if (static_cast<unsigned long>(now - lastTransitionMs) < 30UL) return;
  stablePressed = pressed;
  if (!stablePressed) {
    observedRelease = true;
    if (wasEnabled) disarmLocalController();
    wasEnabled = false;
    return;
  }
  if (!observedRelease) return;
  if (!wasEnabled) {
    // One explicit attempt per button hold. TIMEOUT/ESTOP cannot silently rearm.
    wasEnabled = true;
    if (!armLocalController()) return;
    lastSampleMs = now - 20UL;
  }
  if (static_cast<unsigned long>(now - lastSampleMs) < 20UL) return;
  lastSampleMs = now;
  int pulses[4];
  for (uint8_t i = 0; i < 4; ++i) {
    const long raw = analogRead(analogPins[i]);
    pulses[i] = MEARM_MIN_US[i] +
                (raw * (MEARM_MAX_US[i] - MEARM_MIN_US[i]) + 511L) / 1023L;
  }
  setLocalTargets(pulses);
}
