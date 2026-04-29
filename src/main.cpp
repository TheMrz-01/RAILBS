#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

/*
  Istanbul rail vehicle competition firmware

  Strategy implemented here:
  - Count magnetic traverses with KY-024 digital outputs.
  - Use the front magnet sensor as the main position reference.
  - Slow down before the tunnel.
  - Stop at marker 19 only if the HC-SR04 confirms that the robot is inside the tunnel.
  - Hold the robot stopped for slightly over 5 seconds.
  - Continue to the finish and stop at marker 40.

  Important track positions:
  - Marker spacing: 0.5 m
  - Marker 18: 9.0 m
  - Marker 19: 9.5 m, safely inside the tunnel
  - Marker 20: 10.0 m, approximately tunnel exit
  - Marker 40: 20.0 m finish

  This firmware intentionally avoids encoder-based distance measurement.
  Localization is based on the competition-provided magnets.
*/

// [WARNING!] NEEDS CONFIG
// -----------------------------
// Pin configuration
// -----------------------------

// Change these pins to match your wiring.
// Use ADC1 pins for analog sensors if WiFi is enabled; ADC2 conflicts with WiFi on ESP32.

// BTS7960 driver for left motor.
static constexpr uint8_t LEFT_RPWM_PIN = 25;
static constexpr uint8_t LEFT_LPWM_PIN = 26;

// BTS7960 driver for right motor.
static constexpr uint8_t RIGHT_RPWM_PIN = 27;
static constexpr uint8_t RIGHT_LPWM_PIN = 14;

// KY-024 digital outputs.
// Front sensor is the primary marker counter.
// Rear sensor is currently logged and can later be used for speed estimation.
static constexpr uint8_t FRONT_MAGNET_PIN = 34;
static constexpr uint8_t REAR_MAGNET_PIN = 35;

// HC-SR04 ultrasonic sensor.
static constexpr uint8_t ULTRASONIC_TRIG_PIN = 5;
static constexpr uint8_t ULTRASONIC_ECHO_PIN = 18;

// Optional physical button to start the mission.
// Use INPUT_PULLUP, so the button should connect this pin to GND when pressed.
static constexpr uint8_t START_BUTTON_PIN = 13;

// -----------------------------
// PWM configuration
// -----------------------------

static constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
static constexpr uint8_t PWM_RESOLUTION_BITS = 8;
static constexpr uint8_t LEFT_RPWM_CHANNEL = 0;
static constexpr uint8_t LEFT_LPWM_CHANNEL = 1;
static constexpr uint8_t RIGHT_RPWM_CHANNEL = 2;
static constexpr uint8_t RIGHT_LPWM_CHANNEL = 3;
static constexpr int PWM_MAX = 255;

// -----------------------------
// Mission constants
// -----------------------------

static constexpr int TUNNEL_APPROACH_MARKER = 17;
static constexpr int TUNNEL_STOP_MARKER = 19;
static constexpr int TUNNEL_EXIT_MARKER = 20;
static constexpr int FINISH_APPROACH_MARKER = 37;
static constexpr int FINISH_MARKER = 40;

// The stop is slightly longer than 5.000 s to avoid undershooting due to timing jitter.
static constexpr uint32_t DEFAULT_TUNNEL_STOP_MS = 5100;

// Ignore repeated magnet triggers for this long.
// KY-024 modules can chatter near the edge of a magnet, especially with vibration.
static constexpr uint32_t DEFAULT_MAGNET_DEBOUNCE_US = 120000;

// If the HC-SR04 sees a close ceiling, assume we are inside the tunnel.
// Tune this after measuring the actual sensor height on the robot.
static constexpr float DEFAULT_TUNNEL_DISTANCE_CM = 24.0f;

// HC-SR04 timeout. 12000 us covers about 2 m round-trip distance, more than enough here.
static constexpr uint32_t ULTRASONIC_TIMEOUT_US = 12000;

// -----------------------------
// Web UI configuration
// -----------------------------

static constexpr char WIFI_AP_SSID[] = "KralVonMobil";
static constexpr char WIFI_AP_PASSWORD[] = "MustiSuckz";

// -----------------------------
// Mission state machine
// -----------------------------

enum class MissionState {
  Idle,
  Cruise,
  TunnelApproach,
  TunnelStop,
  AfterTunnel,
  FinishApproach,
  Finished,
  EmergencyStop
};

static const char *stateName(MissionState state) {
  switch (state) {
    case MissionState::Idle: return "Idle";
    case MissionState::Cruise: return "Cruise";
    case MissionState::TunnelApproach: return "TunnelApproach";
    case MissionState::TunnelStop: return "TunnelStop";
    case MissionState::AfterTunnel: return "AfterTunnel";
    case MissionState::FinishApproach: return "FinishApproach";
    case MissionState::Finished: return "Finished";
    case MissionState::EmergencyStop: return "EmergencyStop";
  }
  return "Unknown";
}

// -----------------------------
// Runtime configuration
// -----------------------------

struct Config {
  int cruisePwm = 185;
  int approachPwm = 95;
  int finishPwm = 75;
  int brakePwm = 180;
  int trim = 0;
  uint32_t tunnelStopMs = DEFAULT_TUNNEL_STOP_MS;
  uint32_t magnetDebounceUs = DEFAULT_MAGNET_DEBOUNCE_US;
  float tunnelDistanceCm = DEFAULT_TUNNEL_DISTANCE_CM;
};

Config config;
Preferences preferences;
WebServer server(80);

// -----------------------------
// Interrupt-shared sensor data
// -----------------------------

// Variables written by ISRs must be volatile.
// Keep ISRs short: record time, increment counters, and exit.
volatile int frontMarkerCount = 0;
volatile int rearMarkerCount = 0;
volatile uint32_t lastFrontMagnetUs = 0;
volatile uint32_t lastRearMagnetUs = 0;
volatile uint32_t previousFrontMagnetUs = 0;
volatile bool frontMarkerEvent = false;
volatile bool rearMarkerEvent = false;

// -----------------------------
// Normal runtime data
// -----------------------------

MissionState missionState = MissionState::Idle;
uint32_t stateStartedMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastUltrasonicMs = 0;
float lastDistanceCm = 999.0f;
bool tunnelConfirmed = false;
bool missionStarted = false;

// -----------------------------
// Utility helpers
// -----------------------------

static int clampPwm(int value) {
  if (value < 0) return 0;
  if (value > PWM_MAX) return PWM_MAX;
  return value;
}

static void enterState(MissionState nextState) {
  missionState = nextState;
  stateStartedMs = millis();
}

static void saveConfig() {
  preferences.begin("kral-von-mobil", false);
  preferences.putInt("cruise", config.cruisePwm);
  preferences.putInt("approach", config.approachPwm);
  preferences.putInt("finish", config.finishPwm);
  preferences.putInt("brake", config.brakePwm);
  preferences.putInt("trim", config.trim);
  preferences.putUInt("stopMs", config.tunnelStopMs);
  preferences.putUInt("debounce", config.magnetDebounceUs);
  preferences.putFloat("tunnelCm", config.tunnelDistanceCm);
  preferences.end();
}

static void loadConfig() {
  preferences.begin("kral-von-mobil", true);
  config.cruisePwm = preferences.getInt("cruise", config.cruisePwm);
  config.approachPwm = preferences.getInt("approach", config.approachPwm);
  config.finishPwm = preferences.getInt("finish", config.finishPwm);
  config.brakePwm = preferences.getInt("brake", config.brakePwm);
  config.trim = preferences.getInt("trim", config.trim);
  config.tunnelStopMs = preferences.getUInt("stopMs", config.tunnelStopMs);
  config.magnetDebounceUs = preferences.getUInt("debounce", config.magnetDebounceUs);
  config.tunnelDistanceCm = preferences.getFloat("tunnelCm", config.tunnelDistanceCm);
  preferences.end();
}

// -----------------------------
// Motor control
// -----------------------------

static void writeMotorChannels(int leftForward, int leftReverse, int rightForward, int rightReverse) {
  ledcWrite(LEFT_RPWM_CHANNEL, clampPwm(leftForward));
  ledcWrite(LEFT_LPWM_CHANNEL, clampPwm(leftReverse));
  ledcWrite(RIGHT_RPWM_CHANNEL, clampPwm(rightForward));
  ledcWrite(RIGHT_LPWM_CHANNEL, clampPwm(rightReverse));
}

static void coastMotors() {
  // Both PWM inputs low usually coasts the BTS7960 output.
  writeMotorChannels(0, 0, 0, 0);
}

static void brakeMotors() {
  // This is a conservative active reverse brake pulse.
  // Test your BTS7960 wiring: some builds may prefer coasting or both-input braking.
  writeMotorChannels(0, config.brakePwm, 0, config.brakePwm);
}

static void driveForward(int pwm) {
  // Trim compensates for one motor being stronger than the other.
  // Positive trim increases right side and decreases left side.
  const int leftPwm = clampPwm(pwm - config.trim);
  const int rightPwm = clampPwm(pwm + config.trim);
  writeMotorChannels(leftPwm, 0, rightPwm, 0);
}

static void setupMotors() {
  ledcSetup(LEFT_RPWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(LEFT_LPWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(RIGHT_RPWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(RIGHT_LPWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);

  ledcAttachPin(LEFT_RPWM_PIN, LEFT_RPWM_CHANNEL);
  ledcAttachPin(LEFT_LPWM_PIN, LEFT_LPWM_CHANNEL);
  ledcAttachPin(RIGHT_RPWM_PIN, RIGHT_RPWM_CHANNEL);
  ledcAttachPin(RIGHT_LPWM_PIN, RIGHT_LPWM_CHANNEL);

  coastMotors();
}

// -----------------------------
// Sensor interrupts
// -----------------------------

void IRAM_ATTR onFrontMagnet() {
  const uint32_t nowUs = micros();

  // Debounce inside ISR so vibration does not create extra marker counts.
  if (nowUs - lastFrontMagnetUs < config.magnetDebounceUs) {
    return;
  }

  previousFrontMagnetUs = lastFrontMagnetUs;
  lastFrontMagnetUs = nowUs;
  frontMarkerCount++;
  frontMarkerEvent = true;
}

void IRAM_ATTR onRearMagnet() {
  const uint32_t nowUs = micros();

  if (nowUs - lastRearMagnetUs < config.magnetDebounceUs) {
    return;
  }

  lastRearMagnetUs = nowUs;
  rearMarkerCount++;
  rearMarkerEvent = true;
}

static void setupSensors() {
  pinMode(FRONT_MAGNET_PIN, INPUT);
  pinMode(REAR_MAGNET_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // FALLING is common when KY-024 digital output goes LOW on detection.
  // If your module outputs HIGH on magnet detection, change these to RISING.
  // [WARNING!] NEEDS CONFIG
  attachInterrupt(digitalPinToInterrupt(FRONT_MAGNET_PIN), onFrontMagnet, FALLING);
  attachInterrupt(digitalPinToInterrupt(REAR_MAGNET_PIN), onRearMagnet, FALLING);
}

// -----------------------------
// HC-SR04 reading
// -----------------------------

static float readUltrasonicCm() {
  // Short blocking measurement. The timeout is intentionally small.
  // Later this can be replaced by a non-blocking RMT implementation if needed.
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  const uint32_t durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) {
    return 999.0f;
  }

  // Sound travels about 0.0343 cm/us. Divide by 2 because the pulse goes out and back.
  return (durationUs * 0.0343f) / 2.0f;
}

static bool isTunnelDetected() {
  return lastDistanceCm > 0.0f && lastDistanceCm <= config.tunnelDistanceCm;
}

// -----------------------------
// Mission control
// -----------------------------

static int getFrontMarkerCount() {
  noInterrupts();
  const int count = frontMarkerCount;
  interrupts();
  return count;
}

static void resetMissionCounters() {
  noInterrupts();
  frontMarkerCount = 0;
  rearMarkerCount = 0;
  lastFrontMagnetUs = 0;
  lastRearMagnetUs = 0;
  previousFrontMagnetUs = 0;
  frontMarkerEvent = false;
  rearMarkerEvent = false;
  interrupts();

  tunnelConfirmed = false;
}

static void startMission() {
  resetMissionCounters();
  missionStarted = true;
  enterState(MissionState::Cruise);
}

static void stopMission(MissionState stopState) {
  missionStarted = false;
  brakeMotors();
  enterState(stopState);
}

static void updateMission() {
  const int marker = getFrontMarkerCount();

  switch (missionState) {
    case MissionState::Idle:
      coastMotors();
      break;

    case MissionState::Cruise:
      driveForward(config.cruisePwm);

      // Slow down early so marker 19 stop is repeatable.
      if (marker >= TUNNEL_APPROACH_MARKER) {
        enterState(MissionState::TunnelApproach);
      }
      break;

    case MissionState::TunnelApproach:
      driveForward(config.approachPwm);

      // The HC-SR04 confirmation prevents stopping at marker 19 if the marker count is wrong.
      if (isTunnelDetected()) {
        tunnelConfirmed = true;
      }

      // Third strategy: marker 19 is the main stop trigger.
      // Marker 19 is around 9.5 m, safely inside the 9.12-10.00 m tunnel.
      if (marker >= TUNNEL_STOP_MARKER && tunnelConfirmed) {
        brakeMotors();
        enterState(MissionState::TunnelStop);
      }

      // Safety fallback: if marker 20 arrives without confirmation, do not stop outside the tunnel.
      // Continue the mission, but this means the tunnel sensor threshold or wiring needs tuning.
      if (marker >= TUNNEL_EXIT_MARKER && !tunnelConfirmed) {
        enterState(MissionState::AfterTunnel);
      }
      break;

    case MissionState::TunnelStop:
      // Keep braking briefly, then coast so the motors do not heat while waiting.
      if (millis() - stateStartedMs < 250) {
        brakeMotors();
      } else {
        coastMotors();
      }

      if (millis() - stateStartedMs >= config.tunnelStopMs) {
        enterState(MissionState::AfterTunnel);
      }
      break;

    case MissionState::AfterTunnel:
      driveForward(config.cruisePwm);

      if (marker >= FINISH_APPROACH_MARKER) {
        enterState(MissionState::FinishApproach);
      }
      break;

    case MissionState::FinishApproach:
      driveForward(config.finishPwm);

      // Stop at the finish marker. This should be tuned with sensor placement.
      // If the front sensor is ahead of the vehicle center, you may need a small delay before braking.
      if (marker >= FINISH_MARKER) {
        stopMission(MissionState::Finished);
      }
      break;

    case MissionState::Finished:
      coastMotors();
      break;

    case MissionState::EmergencyStop:
      coastMotors();
      break;
  }
}

// -----------------------------
// Web UI
// -----------------------------

static String htmlPage() {
  String page;
  page.reserve(5000);

  page += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>KRAL VON 4EVER</title><style>");
  page += F("body{font-family:system-ui;margin:20px;background:#10131a;color:#eef}button,input{font-size:16px;margin:4px;padding:8px}input{width:90px}.card{background:#1b2030;padding:14px;border-radius:12px;margin:12px 0}.danger{background:#b00020;color:white}.ok{background:#1f8f4d;color:white}</style>");
  page += F("</head><body><h1>Kral Von Mobil Control</h1>");

  page += F("<div class='card'><button class='ok' onclick=\"fetch('/start')\">Start Mission</button>");
  page += F("<button class='danger' onclick=\"fetch('/stop')\">Emergency Stop</button>");
  page += F("<button onclick=\"fetch('/reset')\">Reset Counters</button></div>");

  page += F("<div class='card'><h2>Status</h2><pre id='status'>Loading...</pre></div>");

  page += F("<div class='card'><h2>Tuning</h2>");
  page += F("<form action='/config' method='get'>");
  page += F("Cruise PWM <input name='cruise' value='"); page += config.cruisePwm; page += F("'><br>");
  page += F("Approach PWM <input name='approach' value='"); page += config.approachPwm; page += F("'><br>");
  page += F("Finish PWM <input name='finish' value='"); page += config.finishPwm; page += F("'><br>");
  page += F("Brake PWM <input name='brake' value='"); page += config.brakePwm; page += F("'><br>");
  page += F("Motor Trim <input name='trim' value='"); page += config.trim; page += F("'><br>");
  page += F("Tunnel Stop ms <input name='stopMs' value='"); page += config.tunnelStopMs; page += F("'><br>");
  page += F("Tunnel Distance cm <input name='tunnelCm' value='"); page += config.tunnelDistanceCm; page += F("'><br>");
  page += F("Debounce us <input name='debounce' value='"); page += config.magnetDebounceUs; page += F("'><br>");
  page += F("<button type='submit'>Save</button></form></div>");

  page += F("<script>async function tick(){let r=await fetch('/status');document.getElementById('status').textContent=await r.text()}setInterval(tick,500);tick();</script>");
  page += F("</body></html>");
  return page;
}

static void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

static void handleStatus() {
  noInterrupts();
  const int frontCount = frontMarkerCount;
  const int rearCount = rearMarkerCount;
  const uint32_t lastFrontUs = lastFrontMagnetUs;
  const uint32_t previousFrontUs = previousFrontMagnetUs;
  interrupts();

  const uint32_t segmentUs = lastFrontUs - previousFrontUs;
  const float segmentSeconds = segmentUs > 0 ? segmentUs / 1000000.0f : 0.0f;
  const float estimatedSpeedMps = segmentSeconds > 0.0f ? 0.5f / segmentSeconds : 0.0f;

  String status;
  status.reserve(1200);
  status += "state: "; status += stateName(missionState); status += '\n';
  status += "frontMarkerCount: "; status += frontCount; status += '\n';
  status += "rearMarkerCount: "; status += rearCount; status += '\n';
  status += "lastDistanceCm: "; status += String(lastDistanceCm, 1); status += '\n';
  status += "tunnelDetected: "; status += isTunnelDetected() ? "yes" : "no"; status += '\n';
  status += "tunnelConfirmed: "; status += tunnelConfirmed ? "yes" : "no"; status += '\n';
  status += "segmentSeconds: "; status += String(segmentSeconds, 3); status += '\n';
  status += "estimatedSpeedMps: "; status += String(estimatedSpeedMps, 2); status += '\n';
  status += "cruisePwm: "; status += config.cruisePwm; status += '\n';
  status += "approachPwm: "; status += config.approachPwm; status += '\n';
  status += "finishPwm: "; status += config.finishPwm; status += '\n';
  status += "tunnelStopMs: "; status += config.tunnelStopMs; status += '\n';
  status += "tunnelDistanceCm: "; status += String(config.tunnelDistanceCm, 1); status += '\n';
  status += "ip: "; status += WiFi.softAPIP().toString(); status += '\n';

  server.send(200, "text/plain", status);
}

static void handleConfig() {
  if (server.hasArg("cruise")) config.cruisePwm = clampPwm(server.arg("cruise").toInt());
  if (server.hasArg("approach")) config.approachPwm = clampPwm(server.arg("approach").toInt());
  if (server.hasArg("finish")) config.finishPwm = clampPwm(server.arg("finish").toInt());
  if (server.hasArg("brake")) config.brakePwm = clampPwm(server.arg("brake").toInt());
  if (server.hasArg("trim")) config.trim = constrain(server.arg("trim").toInt(), -80, 80);
  if (server.hasArg("stopMs")) config.tunnelStopMs = constrain(server.arg("stopMs").toInt(), 4500, 6500);
  if (server.hasArg("debounce")) config.magnetDebounceUs = constrain(server.arg("debounce").toInt(), 20000, 400000);
  if (server.hasArg("tunnelCm")) config.tunnelDistanceCm = constrain(server.arg("tunnelCm").toFloat(), 5.0f, 60.0f);

  saveConfig();
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleStart() {
  startMission();
  server.send(200, "text/plain", "started");
}

static void handleStop() {
  stopMission(MissionState::EmergencyStop);
  server.send(200, "text/plain", "stopped");
}

static void handleReset() {
  resetMissionCounters();
  enterState(MissionState::Idle);
  coastMotors();
  server.send(200, "text/plain", "reset");
}

static void setupWebUi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/config", handleConfig);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/reset", handleReset);
  server.begin();
}

// -----------------------------
// Arduino entry points
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);

  loadConfig();
  setupMotors();
  setupSensors();
  setupWebUi();

  enterState(MissionState::Idle);

  Serial.println("Kral Von Mobil firmware ready");
  Serial.print("WiFi AP: ");
  Serial.println(WIFI_AP_SSID);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  // Read ultrasonic periodically instead of every loop.
  // This keeps the web server and motor control responsive.
  if (millis() - lastUltrasonicMs >= 80) {
    lastUltrasonicMs = millis();
    lastDistanceCm = readUltrasonicCm();
  }

  // Physical start button. The web UI can also start the mission.
  if (missionState == MissionState::Idle && digitalRead(START_BUTTON_PIN) == LOW) {
    delay(30);
    if (digitalRead(START_BUTTON_PIN) == LOW) {
      startMission();
    }
  }

  // Control loop at about 50 Hz.
  if (millis() - lastControlMs >= 20) {
    lastControlMs = millis();
    updateMission();
  }
}
