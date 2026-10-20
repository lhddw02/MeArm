// One-servo commissioning aid, NOT the synchronization firmware.
// New code: MIT. See docs/hardware_sync.md before connecting servo power.
// Start with the horn/linkage removed, or support the mechanism in a fixture.
// Startup is detached. Detaching removes torque; the arm may fall.
// Serial Monitor:115200 baud, newline. Commands:
//   ARM 11 1500  (select a signal pin2..13 and an initial pulse900..2100us)
//   NUDGE 5      (or NUDGE -5; exactly5us per command)
//   KEEP         (keep attached for another5s)
//   STATE        (last applied command, NOT measured shaft angle)
//   DISARM       (detach; also done after5s with no ARM/NUDGE/KEEP)
// Choose the initial pulse only after checking the servo manufacturer's data.
// The900..2100us hard bounds are NOT guaranteed safe for your mechanism.
#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

Servo commissioningServo;
const int MINIMUM_US = 900;
const int MAXIMUM_US = 2100;
const unsigned long WATCHDOG_MS = 5000;
const float SLEW_US_PER_SECOND = 100.0f;
bool enabled = false;
int selectedPin = -1;
int desiredUs = 1500;
float outputUs = 1500.0f;
unsigned long lastActivityMs = 0;
unsigned long lastOutputUs = 0;
char line[64];
uint8_t lineLength = 0;
bool discard = false;

void stopCommissioning(const char *reason) {
  commissioningServo.detach();
  enabled = false;
  Serial.println(reason);
}

bool parseNumber(const char *text, int lower, int upper, int &result) {
  if (!text || !*text) return false;
  const char *p = text;
  bool negative = false;
  if (*p == '-') {
    negative = true;
    ++p;
  }
  if (!*p) return false;
  long value = 0;
  for (; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    value = value * 10L + (*p - '0');
    if (value > 10000L) return false;
  }
  if (negative) value = -value;
  if (value < lower || value > upper) return false;
  result = static_cast<int>(value);
  return true;
}

void commissioningState() {
  Serial.print("STATE ");
  Serial.print(enabled ? "ARMED" : "DISARMED");
  Serial.print(" PIN ");
  Serial.print(selectedPin);
  Serial.print(" APPLIED_US ");
  Serial.print(static_cast<int>(outputUs + 0.5f));
  Serial.print(" TARGET_US ");
  Serial.println(desiredUs);
}

void commissioningCommand(char *text) {
  char *context = NULL;
  char *tokens[4];
  uint8_t count = 0;
  char *token = strtok_r(text, " ", &context);
  while (token && count < 4) {
    tokens[count++] = token;
    token = strtok_r(NULL, " ", &context);
  }
  if (token || !count) {
    Serial.println("ERROR TOKENS");
    return;
  }
  if (strcmp(tokens[0], "DISARM") == 0 && count == 1) {
    stopCommissioning("DISARMED");
    return;
  }
  if (strcmp(tokens[0], "STATE") == 0 && count == 1) {
    commissioningState();
    return;
  }
  if (strcmp(tokens[0], "KEEP") == 0 && count == 1) {
    if (enabled) lastActivityMs = millis();
    Serial.println(enabled ? "OK KEEP" : "ERROR DISARMED");
    return;
  }
  if (strcmp(tokens[0], "ARM") == 0 && count == 3) {
    int requestedPin = 0;
    int requestedUs = 0;
    if (enabled) {
      Serial.println("ERROR ALREADY_ARMED");
      return;
    }
    if (!parseNumber(tokens[1], 2, 13, requestedPin) ||
        !parseNumber(tokens[2], MINIMUM_US, MAXIMUM_US, requestedUs)) {
      Serial.println("ERROR RANGE");
      return;
    }
    // Set the desired starting command before the timer can emit a pulse.
    commissioningServo.writeMicroseconds(requestedUs);
    if (commissioningServo.attach(requestedPin) == INVALID_SERVO) {
      stopCommissioning("ERROR ATTACH");
      return;
    }
    commissioningServo.writeMicroseconds(requestedUs);
    selectedPin = requestedPin;
    desiredUs = requestedUs;
    outputUs = requestedUs;
    lastActivityMs = millis();
    lastOutputUs = micros();
    enabled = true;
    commissioningState();
    return;
  }
  if (strcmp(tokens[0], "NUDGE") == 0 && count == 2) {
    int increment = 0;
    if (!enabled) {
      Serial.println("ERROR DISARMED");
      return;
    }
    if (!parseNumber(tokens[1], -5, 5, increment) || (increment != 5 && increment != -5)) {
      Serial.println("ERROR USE_NUDGE_5_OR_MINUS5");
      return;
    }
    const int next = desiredUs + increment;
    if (next < MINIMUM_US || next > MAXIMUM_US) {
      Serial.println("ERROR RANGE");
      return;
    }
    desiredUs = next;
    lastActivityMs = millis();
    commissioningState();
    return;
  }
  Serial.println("ERROR COMMAND");
}

void setup() {
  Serial.begin(115200);
  Serial.println("COMMISSIONING: DETACHED; USE ARM pin initial_us; SUPPORT THE ARM");
}

void loop() {
  const unsigned long nowMs = millis();
  if (enabled && static_cast<unsigned long>(nowMs - lastActivityMs) >= WATCHDOG_MS) {
    stopCommissioning("TIMEOUT DETACHED");
  }
  uint8_t consumed = 0;
  while (Serial.available() > 0 && consumed++ < 96) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (discard) Serial.println("ERROR LINE");
      else {
        if (lineLength && line[lineLength - 1] == '\r') --lineLength;
        line[lineLength] = '\0';
        if (lineLength) commissioningCommand(line);
      }
      lineLength = 0;
      discard = false;
    } else if (!discard) {
      if ((static_cast<uint8_t>(c) < 32 && c != '\r') || static_cast<uint8_t>(c) > 126 ||
          lineLength >= sizeof(line) - 1) discard = true;
      else line[lineLength++] = c;
    }
  }
  if (!enabled) return;
  const unsigned long nowUs = micros();
  const unsigned long elapsedUs = static_cast<unsigned long>(nowUs - lastOutputUs);
  if (elapsedUs < 10000UL) return;
  lastOutputUs = nowUs;
  const float maximumStep = SLEW_US_PER_SECOND *
      (elapsedUs > 50000UL ? 50000UL : elapsedUs) * 0.000001f;
  const float delta = desiredUs - outputUs;
  if (delta > maximumStep) outputUs += maximumStep;
  else if (delta < -maximumStep) outputUs -= maximumStep;
  else outputUs = desiredUs;
  commissioningServo.writeMicroseconds(static_cast<int>(outputUs + 0.5f));
}
