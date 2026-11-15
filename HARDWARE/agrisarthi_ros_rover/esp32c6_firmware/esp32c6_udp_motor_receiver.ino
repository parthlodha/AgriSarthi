#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <math.h>
#include "esp32_protocol.h"
#include "probe_fsm.h"

// ==============================================================================
// 1. NETWORK CONFIGURATION (Configurable placeholders)
// ==============================================================================
const char* WIFI_SSID = "REPLACE_WITH_WIFI_SSID";
const char* WIFI_PASSWORD = "REPLACE_WITH_WIFI_PASSWORD";
const uint16_t UDP_PORT = 5005;
const uint32_t COMMAND_TIMEOUT_MS = 700;
const uint32_t WIFI_RETRY_MS = 3000;

// Static IP configuration (Matches agrisarthi_gps_nav/config/hardware.yaml esp32_ip)
const bool USE_STATIC_IP = true;
const IPAddress LOCAL_IP(192, 168, 1, 50);
const IPAddress GATEWAY(192, 168, 1, 1);
const IPAddress SUBNET(255, 255, 255, 0);
const IPAddress PRIMARY_DNS(192, 168, 1, 1);

// ==============================================================================
// 2. BTS7960 DUAL MOTOR DRIVER CONFIGURATION (Configurable placeholders)
//    - Left BTS7960 drives two left-side DC motors connected in parallel.
//    - Right BTS7960 drives two right-side DC motors connected in parallel.
//    - Both motors on the same side receive identical PWM & direction.
// ==============================================================================
const uint32_t MOTOR_PWM_FREQUENCY = 15000;  // 15 kHz audible-noise suppression
const uint8_t MOTOR_PWM_BITS = 8;
const int PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;

// BTS7960 LEFT Channel Pins (Example ESP32-C6 GPIOs: calibrate for physical PCB)
const int LEFT_RPWM_PIN = 4;   // Left forward PWM
const int LEFT_LPWM_PIN = 5;   // Left reverse PWM
const int LEFT_EN_PIN   = -1;  // Optional Left R_EN/L_EN pin (-1 if tied to 3.3V/5V)

// BTS7960 RIGHT Channel Pins (Example ESP32-C6 GPIOs: calibrate for physical PCB)
const int RIGHT_RPWM_PIN = 6;   // Right forward PWM
const int RIGHT_LPWM_PIN = 7;   // Right reverse PWM
const int RIGHT_EN_PIN   = -1;  // Optional Right R_EN/L_EN pin (-1 if tied to 3.3V/5V)

// Motor Direction Polarities (Set true if physical wiring turns wheel backward)
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

// ==============================================================================
// 3. PROBE ACTUATOR: FEETECH FT5330M HIGH-TORQUE SERVO CONFIGURATION
//    - Controlled via 50 Hz PWM from ESP32-C6.
//    - Calibrate physical travel angles against rack-and-pinion before use.
// ==============================================================================
const int PROBE_SERVO_PIN = 3;  // Example GPIO for FT5330M PWM control
const int PROBE_INSERT_ANGLE = 120;
const int PROBE_RETRACT_ANGLE = 30;
const int PROBE_MIN_ANGLE = 10;
const int PROBE_MAX_ANGLE = 170;
const int PROBE_MIN_PULSE_US = 500;   // FEETECH FT5330M min pulse (0 deg)
const int PROBE_MAX_PULSE_US = 2500;  // FEETECH FT5330M max pulse (180 deg)
const uint32_t PROBE_PWM_FREQUENCY = 50;  // 50 Hz standard servo frame rate
const uint8_t PROBE_PWM_BITS = 14;
const uint32_t PROBE_TRAVEL_MS = 1800;   // Estimated mechanical transit time
const uint32_t PROBE_SETTLE_MS = 300;
const uint32_t PROBE_TIMEOUT_MS = 5000;

// Optional physical limit switches for mechanical endpoint confirmation
const bool USE_PROBE_LIMIT_SWITCHES = false;
const int PROBE_INSERT_LIMIT_PIN = 0;   // Active-low limit switch pin
const int PROBE_RETRACT_LIMIT_PIN = 1;  // Active-low limit switch pin
const uint32_t PROBE_LIMIT_GRACE_MS = 700;

// ==============================================================================
// 4. RUNTIME STATE & INTERFACES
// ==============================================================================
WiFiUDP udp;
ProbeStateMachine probe;
uint32_t lastCommandMs = 0, lastSeq = 0, lastWifiAttemptMs = 0;
bool haveSeq = false, haveSession = false, udpStarted = false, motorsEnabled = false;
bool servoReady = false;
float appliedLeft = 0.0f, appliedRight = 0.0f;
char activeSession[33] = "";
IPAddress ownerIP;
uint16_t ownerPort = 0;

void setBts7960Channel(int rpwmPin, int lpwmPin, int enPin, float value, bool invert) {
  if (invert) value = -value;
  value = constrain(value, -1.0f, 1.0f);
  if (fabsf(value) < 0.001f) {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, 0);
    if (enPin >= 0) digitalWrite(enPin, LOW);
    return;
  }
  if (enPin >= 0) digitalWrite(enPin, HIGH);
  int duty = (int)(fabsf(value) * PWM_MAX);
  if (value > 0.0f) {
    // Forward rotation
    ledcWrite(lpwmPin, 0);
    ledcWrite(rpwmPin, duty);
  } else {
    // Reverse rotation
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, duty);
  }
}

void stopMotors() {
  setBts7960Channel(LEFT_RPWM_PIN, LEFT_LPWM_PIN, LEFT_EN_PIN, 0.0f, false);
  setBts7960Channel(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, RIGHT_EN_PIN, 0.0f, false);
  motorsEnabled = false;
  appliedLeft = 0.0f;
  appliedRight = 0.0f;
}

bool insertLimit() {
  return USE_PROBE_LIMIT_SWITCHES && digitalRead(PROBE_INSERT_LIMIT_PIN) == LOW;
}

bool retractLimit() {
  return USE_PROBE_LIMIT_SWITCHES && digitalRead(PROBE_RETRACT_LIMIT_PIN) == LOW;
}

void publishProbeStatus(const char* type, uint32_t seq = 0, bool accepted = false,
                        bool duplicate = false, const char* commandId = nullptr) {
  if (!haveSession || WiFi.status() != WL_CONNECTED) return;
  JsonDocument reply;
  reply["type"] = type;
  reply["protocol"] = 1;
  reply["session_id"] = activeSession;
  reply["seq"] = seq;
  reply["accepted"] = accepted;
  reply["duplicate"] = duplicate;
  reply["command_id"] = commandId ? commandId : probe.activeCommandId();
  reply["probe_state"] = probeStateName(probe.state());
  reply["probe_error"] = probe.error();
  reply["position_assumed"] = probe.positionAssumed();
  reply["motors_enabled"] = motorsEnabled;
  reply["left"] = appliedLeft;
  reply["right"] = appliedRight;
  udp.beginPacket(ownerIP, ownerPort);
  serializeJson(reply, udp);
  udp.endPacket();
}

void startProbeMove(bool insert) {
  stopMotors();
  if (!servoReady) {
    probe.setConfigurationError("SERVO_PWM_NOT_READY");
    return;
  }
  int angle = insert ? PROBE_INSERT_ANGLE : PROBE_RETRACT_ANGLE;
  if (angle < PROBE_MIN_ANGLE || angle > PROBE_MAX_ANGLE) {
    probe.setConfigurationError("SERVO_ANGLE_OUT_OF_LIMITS");
    return;
  }
  const uint32_t maxDuty = (1U << PROBE_PWM_BITS) - 1U;
  uint32_t pulse = PROBE_MIN_PULSE_US + (uint32_t)(PROBE_MAX_PULSE_US - PROBE_MIN_PULSE_US) * angle / 180U;
  ledcWrite(PROBE_SERVO_PIN, (uint32_t)((uint64_t)pulse * PROBE_PWM_FREQUENCY * maxDuty / 1000000U));
}

void applyCommand(float left, float right, bool enable) {
  // Hard safety interlock: motors remain strictly stopped unless probe is confirmed retracted
  if (!enable || !probe.motorAllowed()) {
    stopMotors();
    return;
  }
  left = constrain(left, -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);
  setBts7960Channel(LEFT_RPWM_PIN, LEFT_LPWM_PIN, LEFT_EN_PIN, left, INVERT_LEFT_MOTOR);
  setBts7960Channel(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, RIGHT_EN_PIN, right, INVERT_RIGHT_MOTOR);
  motorsEnabled = (left != 0.0f || right != 0.0f);
  appliedLeft = left;
  appliedRight = right;
}

void sendReply(JsonDocument& doc, const char* type, bool accepted = false, uint32_t seq = 0) {
  JsonDocument reply;
  reply["type"] = type;
  reply["protocol"] = 1;
  reply["session_id"] = activeSession;
  if (String(type) == "ack") {
    reply["seq"] = seq;
    reply["accepted"] = accepted;
  }
  reply["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  reply["motors_enabled"] = motorsEnabled;
  reply["left"] = appliedLeft;
  reply["right"] = appliedRight;
  reply["probe_state"] = probeStateName(probe.state());
  reply["probe_error"] = probe.error();
  reply["position_assumed"] = probe.positionAssumed();
  udp.beginPacket(ownerIP, ownerPort);
  serializeJson(reply, udp);
  udp.endPacket();
}

bool packetFromOwner() {
  return haveSession && udp.remoteIP() == ownerIP && udp.remotePort() == ownerPort;
}

void handlePacket(JsonDocument& doc) {
  const char* type = doc["type"] | "";
  const char* session = doc["session_id"] | "";
  if (strlen(session) == 0 || strlen(session) > 32) return;
  if (String(type) == "hello") {
    // New Pi process announces fresh session; stop motors immediately
    bool changed = !haveSession || strcmp(activeSession, session) != 0 ||
                   udp.remoteIP() != ownerIP || udp.remotePort() != ownerPort;
    if (changed) {
      stopMotors();
      strlcpy(activeSession, session, sizeof(activeSession));
      haveSeq = false;
      lastSeq = 0;
    }
    ownerIP = udp.remoteIP();
    ownerPort = udp.remotePort();
    haveSession = true;
    lastCommandMs = millis();
    sendReply(doc, "hello_ack");
    return;
  }
  if ((String(type) != "motor_cmd" && String(type) != "probe_cmd") ||
      !packetFromOwner() || strcmp(activeSession, session) != 0) {
    return;
  }
  uint32_t seq = doc["seq"] | 0;
  bool accepted = !haveSeq || agrisarthiSequenceIsNewer(seq, lastSeq);
  if (!accepted) {
    if (String(type) == "motor_cmd") {
      sendReply(doc, "ack", false, seq);
    } else {
      const char* commandId = doc["command_id"] | "";
      const char* action = doc["action"] | "";
      bool duplicateAccepted = false;
      bool duplicate = probe.cachedResult(commandId, action, duplicateAccepted);
      publishProbeStatus("probe_ack", seq, duplicate && duplicateAccepted, duplicate, commandId);
    }
    return;
  }
  haveSeq = true;
  lastSeq = seq;
  lastCommandMs = millis();
  if (String(type) == "probe_cmd") {
    const char* commandId = doc["command_id"] | "";
    const char* action = doc["action"] | "";
    ProbeCommandResult result = probe.command(
        commandId, action, millis(), PROBE_TRAVEL_MS, PROBE_SETTLE_MS, PROBE_TIMEOUT_MS, retractLimit());
    if (result.beginInsert) startProbeMove(true);
    if (result.beginRetract) startProbeMove(false);
    stopMotors();
    publishProbeStatus("probe_ack", seq, result.accepted, result.duplicate, commandId);
    return;
  }
  bool enable = doc["enable"] | false;
  float left = doc["left"] | 0.0f, right = doc["right"] | 0.0f;
  if (!isfinite(left) || !isfinite(right)) {
    sendReply(doc, "ack", false, seq);
    return;
  }
  applyCommand(left, right, enable);
  sendReply(doc, "ack", true, seq);
}

void startUdp() {
  if (!udpStarted) {
    udp.begin(UDP_PORT);
    udpStarted = true;
    Serial.printf("Agrisarthi UDP motor receiver ready on port %u\n", UDP_PORT);
  }
}

void setup() {
  Serial.begin(115200);

  // Configure BTS7960 Enable Pins if specified
  if (LEFT_EN_PIN >= 0) pinMode(LEFT_EN_PIN, OUTPUT);
  if (RIGHT_EN_PIN >= 0) pinMode(RIGHT_EN_PIN, OUTPUT);

  // Attach BTS7960 PWM channels (LEDC Core 3.x API)
  bool pwmOk = ledcAttach(LEFT_RPWM_PIN, MOTOR_PWM_FREQUENCY, MOTOR_PWM_BITS) &&
               ledcAttach(LEFT_LPWM_PIN, MOTOR_PWM_FREQUENCY, MOTOR_PWM_BITS) &&
               ledcAttach(RIGHT_RPWM_PIN, MOTOR_PWM_FREQUENCY, MOTOR_PWM_BITS) &&
               ledcAttach(RIGHT_LPWM_PIN, MOTOR_PWM_FREQUENCY, MOTOR_PWM_BITS);

  if (!pwmOk) {
    Serial.println("FATAL: Motor LEDC PWM setup failed; halting.");
    while (true) {
      stopMotors();
      delay(1000);
    }
  }

  // Configure optional limit switches
  if (USE_PROBE_LIMIT_SWITCHES) {
    pinMode(PROBE_INSERT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(PROBE_RETRACT_LIMIT_PIN, INPUT_PULLUP);
  }

  // Configure FEETECH FT5330M probe servo PWM
  if (PROBE_MIN_ANGLE < 0 || PROBE_MAX_ANGLE > 180 || PROBE_MIN_ANGLE > PROBE_MAX_ANGLE ||
      PROBE_INSERT_ANGLE < PROBE_MIN_ANGLE || PROBE_INSERT_ANGLE > PROBE_MAX_ANGLE ||
      PROBE_RETRACT_ANGLE < PROBE_MIN_ANGLE || PROBE_RETRACT_ANGLE > PROBE_MAX_ANGLE ||
      PROBE_MIN_PULSE_US <= 0 || PROBE_MAX_PULSE_US <= PROBE_MIN_PULSE_US) {
    probe.setConfigurationError("SERVO_CONFIGURATION_OUT_OF_LIMITS");
  } else {
    if (!ledcAttach(PROBE_SERVO_PIN, PROBE_PWM_FREQUENCY, PROBE_PWM_BITS)) {
      probe.setConfigurationError("SERVO_PWM_SETUP_FAILED");
    } else {
      servoReady = true;
      const uint32_t maxDuty = (1U << PROBE_PWM_BITS) - 1U;
      uint32_t pulse = PROBE_MIN_PULSE_US + (uint32_t)(PROBE_MAX_PULSE_US - PROBE_MIN_PULSE_US) * PROBE_RETRACT_ANGLE / 180U;
      ledcWrite(PROBE_SERVO_PIN, (uint32_t)((uint64_t)pulse * PROBE_PWM_FREQUENCY * maxDuty / 1000000U));
    }
  }

  if (servoReady && USE_PROBE_LIMIT_SWITCHES && retractLimit()) {
    probe.setBootRetracted();
  }

  stopMotors();

  // Wi-Fi Connection Setup
  WiFi.mode(WIFI_STA);
  if (USE_STATIC_IP) {
    WiFi.config(LOCAL_IP, GATEWAY, SUBNET, PRIMARY_DNS);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttemptMs = millis();
  lastCommandMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    stopMotors();
    if (udpStarted) {
      udp.stop();
      udpStarted = false;
    }
    if (now - lastWifiAttemptMs >= WIFI_RETRY_MS) {
      WiFi.disconnect();
      if (USE_STATIC_IP) {
        WiFi.config(LOCAL_IP, GATEWAY, SUBNET, PRIMARY_DNS);
      }
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      lastWifiAttemptMs = now;
    }
    delay(10);
    return;
  }

  startUdp();

  int n = udp.parsePacket();
  if (n > 0) {
    char buf[768];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      JsonDocument doc;
      auto err = deserializeJson(doc, buf);
      if (!err) {
        handlePacket(doc);
      } else {
        Serial.println("Ignored malformed packet");
      }
    }
  }

  if (probe.tick(now, PROBE_LIMIT_GRACE_MS, USE_PROBE_LIMIT_SWITCHES, insertLimit(), retractLimit())) {
    stopMotors();
    publishProbeStatus("probe_status");
  }

  // Fail-safe interlock: cut motor PWM immediately if probe is not retracted
  if (!probe.motorAllowed()) {
    stopMotors();
  }

  // Fail-safe watchdog: stop motors if no valid command received within 700 ms
  if (motorsEnabled && now - lastCommandMs > COMMAND_TIMEOUT_MS) {
    stopMotors();
    Serial.println("Command timeout: watchdog stopped motors");
  }

  delay(2);
}
