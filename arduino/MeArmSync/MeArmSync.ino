// MeArm Classic V1: nonblocking, command-only USB serial synchronization.
// New code: MIT, see the repository LICENSE.
// See docs/hardware_sync.md before connecting servo power.
// Joint order throughout: base, shoulder, elbow, gripper.
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "calibration.h"

namespace {
const uint8_t JOINTS = 4;
const uint16_t LINE_CAPACITY = 160;
const unsigned long SERVO_PERIOD_US = 10000UL;
Servo servos[JOINTS];
float appliedUs[JOINTS];
int targetUs[JOINTS];
bool armed = false;
bool estopLatched = false;
bool configurationValid = false;
const char *stateReason = "BOOT";
unsigned long lastCommandMs = 0;
unsigned long lastServoUs = 0;
char inputLine[LINE_CAPACITY];
uint16_t inputLength = 0;
bool discardLine = false;
bool malformedLine = false;

void reply(const char *payload) {
  uint8_t checksum = 0;
  for (const char *p = payload; *p; ++p) checksum ^= static_cast<uint8_t>(*p);
  const char hex[] = "0123456789ABCDEF";
  Serial.print(payload);
  Serial.print('*');
  Serial.print(hex[checksum >> 4]);
  Serial.print(hex[checksum & 15]);
  Serial.print('\n');
}

void errorReply(unsigned int sequence, const char *code) {
  char payload[64];
  snprintf(payload, sizeof(payload), "ERROR %u %s", sequence, code);
  reply(payload);
}

void acknowledge(unsigned int sequence, const char *command) {
  char payload[48];
  snprintf(payload, sizeof(payload), "ACK %u %s", sequence, command);
  reply(payload);
}

bool parseUnsigned(const char *token, unsigned long maximum, unsigned long &value) {
  if (!token || !*token) return false;
  value = 0;
  for (const char *p = token; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    const uint8_t digit = static_cast<uint8_t>(*p - '0');
    if (value > maximum / 10UL ||
        (value == maximum / 10UL && digit > maximum % 10UL)) return false;
    value = value * 10UL + digit;
  }
  return true;
}

int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool pulsesValid(const int *pulses) {
  for (uint8_t i = 0; i < JOINTS; ++i) {
    if (pulses[i] < MEARM_MIN_US[i] || pulses[i] > MEARM_MAX_US[i]) return false;
  }
  return true;
}

void detachAll(const char *reason) {
  for (uint8_t i = 0; i < JOINTS; ++i) servos[i].detach();
  armed = false;
  stateReason = reason;
}

bool attachAt(const int *pulses) {
  if (!configurationValid || !MEARM_CALIBRATED || estopLatched || armed ||
      !pulsesValid(pulses)) return false;
  // Servo.writeMicroseconds updates the stored pulse even while detached.
  // Preload BEFORE attach to avoid a transient default 1500 us command.
  for (uint8_t i = 0; i < JOINTS; ++i) {
    appliedUs[i] = pulses[i];
    targetUs[i] = pulses[i];
    servos[i].writeMicroseconds(pulses[i]);
  }
  for (uint8_t i = 0; i < JOINTS; ++i) {
    // The AVR Servo library stores attach(min,max) offsets in int8_t; narrow
    // calibrated ranges can overflow those offsets. Enforce our calibrated
    // bounds independently and keep Servo's supported default conversion range.
    if (servos[i].attach(MEARM_PINS[i]) == INVALID_SERVO) {
      detachAll("DISARM");
      return false;
    }
    servos[i].writeMicroseconds(pulses[i]);
  }
  armed = true;
  stateReason = "ARM";
  lastCommandMs = millis();
  lastServoUs = micros();
  return true;
}

void setTargets(const int *pulses) {
  for (uint8_t i = 0; i < JOINTS; ++i) targetUs[i] = pulses[i];
  lastCommandMs = millis();
  stateReason = "OK";
}

void sendState(unsigned int sequence) {
  char payload[96];
  snprintf(payload, sizeof(payload), "STATE %u %u %s %d %d %d %d", sequence,
           armed ? 1 : 0, stateReason,
           static_cast<int>(appliedUs[0] + 0.5f),
           static_cast<int>(appliedUs[1] + 0.5f),
           static_cast<int>(appliedUs[2] + 0.5f),
           static_cast<int>(appliedUs[3] + 0.5f));
  reply(payload);
}

void processLine(char *line) {
  const size_t length = strlen(line);
  if (length < 4 || line[length - 3] != '*') {
    errorReply(0, "FRAME");
    return;
  }
  const int high = hexDigit(line[length - 2]);
  const int low = hexDigit(line[length - 1]);
  if (high < 0 || low < 0) {
    errorReply(0, "CHECKSUM");
    return;
  }
  uint8_t actualChecksum = 0;
  for (size_t i = 0; i < length - 3; ++i) {
    if (line[i] < 32 || line[i] > 126 || line[i] == '*') {
      errorReply(0, "FRAME");
      return;
    }
    actualChecksum ^= static_cast<uint8_t>(line[i]);
  }
  if (actualChecksum != static_cast<uint8_t>((high << 4) | low)) {
    errorReply(0, "CHECKSUM");
    return;
  }
  line[length - 3] = '\0';
  char *tokens[7];
  uint8_t count = 0;
  char *context = NULL;
  char *token = strtok_r(line, " ", &context);
  while (token && count < 7) {
    tokens[count++] = token;
    token = strtok_r(NULL, " ", &context);
  }
  if (token || count < 2) {
    errorReply(0, "TOKENS");
    return;
  }
  unsigned long parsedSequence = 0;
  if (!parseUnsigned(tokens[1], 65535UL, parsedSequence)) {
    errorReply(0, "SEQUENCE");
    return;
  }
  const unsigned int sequence = static_cast<unsigned int>(parsedSequence);
  const char *command = tokens[0];
  const bool pulseCommand = strcmp(command, "ARM") == 0 || strcmp(command, "SET") == 0;
  if (count != (pulseCommand ? 6 : 2)) {
    errorReply(sequence, "TOKENS");
    return;
  }

  if (strcmp(command, "HELLO") == 0) {
    char payload[64];
    snprintf(payload, sizeof(payload), "READY %u 1 %s %s %u", sequence,
             MEARM_PROFILE_ID, MEARM_SOURCE_LOCAL ? "LOCAL" : "HOST", armed ? 1 : 0);
    reply(payload);
    return;
  }
  if (strcmp(command, "STATE") == 0) {
    sendState(sequence);
    return;
  }
  if (strcmp(command, "DISARM") == 0) {
    detachAll(estopLatched ? "ESTOP" : "DISARM");
    acknowledge(sequence, command);
    return;
  }
  if (strcmp(command, "ESTOP") == 0) {
    estopLatched = true;
    detachAll("ESTOP");
    acknowledge(sequence, command);
    return;
  }
  if (strcmp(command, "KEEP") == 0) {
    if (MEARM_SOURCE_LOCAL) {
      errorReply(sequence, "LOCAL_SOURCE");
      return;
    }
    if (armed) lastCommandMs = millis();
    acknowledge(sequence, command);
    return;
  }
  if (!pulseCommand) {
    errorReply(sequence, "COMMAND");
    return;
  }
  if (MEARM_SOURCE_LOCAL) {
    errorReply(sequence, "LOCAL_SOURCE");
    return;
  }
  if (!configurationValid) {
    errorReply(sequence, "CONFIG");
    return;
  }
  if (!MEARM_CALIBRATED) {
    errorReply(sequence, "UNCALIBRATED");
    return;
  }
  if (estopLatched) {
    errorReply(sequence, "ESTOP");
    return;
  }
  int pulses[JOINTS];
  for (uint8_t i = 0; i < JOINTS; ++i) {
    unsigned long value = 0;
    if (!parseUnsigned(tokens[2 + i], 2400UL, value)) {
      errorReply(sequence, "PULSE");
      return;
    }
    pulses[i] = static_cast<int>(value);
  }
  if (!pulsesValid(pulses)) {
    errorReply(sequence, "RANGE");
    return;
  }
  if (strcmp(command, "ARM") == 0) {
    if (armed) {
      errorReply(sequence, "ALREADY_ARMED");
      return;
    }
    if (!attachAt(pulses)) {
      errorReply(sequence, "ATTACH");
      return;
    }
  } else {
    if (!armed) {
      errorReply(sequence, "DISARMED");
      return;
    }
    setTargets(pulses);
  }
  acknowledge(sequence, command);
}

void pollSerial() {
  uint16_t consumed = 0;
  while (Serial.available() > 0 && consumed++ < 192) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (discardLine) errorReply(0, malformedLine ? "FRAME" : "LINE_TOO_LONG");
      else {
        if (inputLength && inputLine[inputLength - 1] == '\r') --inputLength;
        inputLine[inputLength] = '\0';
        if (inputLength) processLine(inputLine);
      }
      inputLength = 0;
      discardLine = false;
      malformedLine = false;
    } else if (!discardLine) {
      if ((static_cast<uint8_t>(c) < 32 && c != '\r') || static_cast<uint8_t>(c) > 126) {
        discardLine = true;
        malformedLine = true;
      } else if (inputLength >= LINE_CAPACITY - 1) discardLine = true;
      else inputLine[inputLength++] = c;
    }
  }
}

void updateServos() {
  if (!armed) return;
  const unsigned long nowMs = millis();
  if (static_cast<unsigned long>(nowMs - lastCommandMs) >= MEARM_WATCHDOG_MS) {
    detachAll("TIMEOUT");
    return;
  }
  const unsigned long nowUs = micros();
  const unsigned long elapsedUs = static_cast<unsigned long>(nowUs - lastServoUs);
  if (elapsedUs < SERVO_PERIOD_US) return;
  lastServoUs = nowUs;
  // Bound a delayed loop's catch-up step; never jump across a long interruption.
  const float elapsedSeconds = (elapsedUs > 50000UL ? 50000UL : elapsedUs) * 0.000001f;
  for (uint8_t i = 0; i < JOINTS; ++i) {
    const float maximumStep = MEARM_MAX_US_PER_SECOND[i] * elapsedSeconds;
    const float delta = targetUs[i] - appliedUs[i];
    if (delta > maximumStep) appliedUs[i] += maximumStep;
    else if (delta < -maximumStep) appliedUs[i] -= maximumStep;
    else appliedUs[i] = targetUs[i];
    servos[i].writeMicroseconds(static_cast<int>(appliedUs[i] + 0.5f));
  }
}
}  // namespace

// Optional LOCAL source API. Replace updateLocalController() below with your
// joystick/buttons logic, keeping all input handling nonblocking. Reported STATE
// is the applied servo COMMAND, never an encoder measurement.
bool armLocalController() {
  return MEARM_SOURCE_LOCAL && attachAt(MEARM_HOME_US);
}

bool setLocalTargets(const int pulses[4]) {
  if (!MEARM_SOURCE_LOCAL || !armed || !pulsesValid(pulses)) return false;
  setTargets(pulses);
  return true;
}

void disarmLocalController() {
  detachAll(estopLatched ? "ESTOP" : "DISARM");
}

#ifdef MEARM_LOCAL_JOYSTICKS
#include "local_joystick.example.h"
#else
void updateLocalController() {
  // Intentionally empty: LOCAL never arms itself at startup. See the guide.
  // On an explicit physical button press: armLocalController().
  // While that controller is active, call setLocalTargets(pulses) regularly.
}
#endif

void setup() {
  Serial.begin(115200);
  configurationValid = MEARM_WATCHDOG_MS >= 200UL && MEARM_WATCHDOG_MS <= 2000UL;
  for (uint8_t i = 0; i < JOINTS; ++i) {
    appliedUs[i] = MEARM_HOME_US[i];
    targetUs[i] = MEARM_HOME_US[i];
    if (MEARM_PINS[i] < 2 || MEARM_PINS[i] > 13 ||
        MEARM_MIN_US[i] < 900 || MEARM_MAX_US[i] > 2100 ||
        MEARM_MIN_US[i] >= MEARM_MAX_US[i] || MEARM_HOME_US[i] < MEARM_MIN_US[i] ||
        MEARM_HOME_US[i] > MEARM_MAX_US[i] ||
        !isfinite(MEARM_MAX_US_PER_SECOND[i]) || MEARM_MAX_US_PER_SECOND[i] <= 0.0f ||
        MEARM_MAX_US_PER_SECOND[i] > 1000.0f) {
      configurationValid = false;
    }
    for (uint8_t j = 0; j < i; ++j) {
      if (MEARM_PINS[i] == MEARM_PINS[j]) configurationValid = false;
    }
  }
  lastServoUs = micros();
}

void loop() {
  // Enforce expiry before processing a late heartbeat or a new ARM command.
  updateServos();
  pollSerial();
  updateLocalController();
}
