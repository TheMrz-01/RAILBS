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
static constexpr uint8_t RIGHT_RPWM_PIN = 18;
static constexpr uint8_t RIGHT_LPWM_PIN = 5;

// KY-024 digital outputs.
// Front sensor is the primary marker counter.
// Rear sensor is currently logged and can later be used for speed estimation.
static constexpr uint8_t FRONT_MAGNET_PIN = 34;
static constexpr uint8_t REAR_MAGNET_PIN = 35;

// HC-SR04 ultrasonic sensor.
static constexpr uint8_t ULTRASONIC_TRIG_PIN = 27;
static constexpr uint8_t ULTRASONIC_ECHO_PIN = 14;

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
static constexpr uint16_t DEFAULT_MARKER_SPACING_CM = 50;
static constexpr uint16_t DEFAULT_TRACK_DISTANCE_CM = 2000;

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
// Logging configuration
// -----------------------------

// The ESP32 only keeps a recent backup log in RAM.
// Your laptop/Bun app should poll /status during the mission and save the full log.
static constexpr uint16_t TELEMETRY_LOG_CAPACITY = 600;
static constexpr uint16_t EVENT_LOG_CAPACITY = 120;
static constexpr uint32_t TELEMETRY_LOG_INTERVAL_MS = 100;

// -----------------------------
// Web UI configuration
// -----------------------------

// [BEWARE!]
static constexpr char WIFI_AP_SSID[] = "Zeron-Mobil";
static constexpr char WIFI_AP_PASSWORD[] = "macka124";

// -----------------------------
// Mission state machine
// -----------------------------

enum class MissionState {
  Idle,
  TestMode,
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
    case MissionState::TestMode: return "TestMode";
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

static const char *stateNameTr(MissionState state) {
  switch (state) {
    case MissionState::Idle: return "Beklemede";
    case MissionState::TestMode: return "Test Modu";
    case MissionState::Cruise: return "Seyir";
    case MissionState::TunnelApproach: return "Tünel Yaklaşma";
    case MissionState::TunnelStop: return "Tünelde Durma";
    case MissionState::AfterTunnel: return "Tünel Sonrası";
    case MissionState::FinishApproach: return "Bitiş Yaklaşma";
    case MissionState::Finished: return "Bitti";
    case MissionState::EmergencyStop: return "Acil Durdurma";
  }
  return "Bilinmiyor";
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
  uint16_t markerSpacingCm = DEFAULT_MARKER_SPACING_CM;
  uint16_t trackDistanceCm = DEFAULT_TRACK_DISTANCE_CM;
  int tunnelApproachMarker = TUNNEL_APPROACH_MARKER;
  int tunnelStopMarker = TUNNEL_STOP_MARKER;
  int tunnelExitMarker = TUNNEL_EXIT_MARKER;
  int finishApproachMarker = FINISH_APPROACH_MARKER;
  int finishMarker = FINISH_MARKER;
  uint32_t tunnelStopMs = DEFAULT_TUNNEL_STOP_MS;
  uint32_t magnetDebounceUs = DEFAULT_MAGNET_DEBOUNCE_US;
  float tunnelDistanceCm = DEFAULT_TUNNEL_DISTANCE_CM;
};

Config config;
Preferences preferences;
WebServer server(80);

// -----------------------------
// Telemetry and event logs
// -----------------------------

struct TelemetrySample {
  uint32_t timeMs;
  MissionState state;
  int16_t frontMarker;
  int16_t rearMarker;
  int16_t distanceCm10;
  int16_t leftPwm;
  int16_t rightPwm;
  int16_t speedCmps;
  bool tunnelDetected;
  bool tunnelConfirmed;
};

struct EventSample {
  uint32_t timeMs;
  char event[28];
  char value[24];
};

TelemetrySample telemetryLog[TELEMETRY_LOG_CAPACITY];
EventSample eventLog[EVENT_LOG_CAPACITY];
uint16_t telemetryLogHead = 0;
uint16_t telemetryLogCount = 0;
uint16_t eventLogHead = 0;
uint16_t eventLogCount = 0;
uint32_t missionStartMs = 0;
uint32_t lastTelemetryLogMs = 0;

// Signed motor command used for telemetry.
// Positive means forward, negative means reverse braking, zero means coast.
int16_t currentLeftPwm = 0;
int16_t currentRightPwm = 0;

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

static uint32_t relativeTimeMs() {
  if (missionStartMs == 0) {
    return millis();
  }
  return millis() - missionStartMs;
}

static void clearLogs() {
  telemetryLogHead = 0;
  telemetryLogCount = 0;
  eventLogHead = 0;
  eventLogCount = 0;
  lastTelemetryLogMs = 0;
}

static void logEvent(const char *event, const char *value = "") {
  EventSample &sample = eventLog[eventLogHead];
  sample.timeMs = relativeTimeMs();
  strlcpy(sample.event, event, sizeof(sample.event));
  strlcpy(sample.value, value, sizeof(sample.value));

  eventLogHead = (eventLogHead + 1) % EVENT_LOG_CAPACITY;
  if (eventLogCount < EVENT_LOG_CAPACITY) {
    eventLogCount++;
  }
}

static void logEventValue(const char *event, int32_t value) {
  char valueText[24];
  snprintf(valueText, sizeof(valueText), "%ld", static_cast<long>(value));
  logEvent(event, valueText);
}

static void logEventFloat(const char *event, float value) {
  char valueText[24];
  snprintf(valueText, sizeof(valueText), "%.1f", value);
  logEvent(event, valueText);
}

static void enterState(MissionState nextState) {
  if (missionState != nextState) {
    logEvent("STATE_CHANGED", stateName(nextState));
  }
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
  preferences.putUShort("markCm", config.markerSpacingCm);
  preferences.putUShort("trackCm", config.trackDistanceCm);
  preferences.putInt("tunApp", config.tunnelApproachMarker);
  preferences.putInt("tunStop", config.tunnelStopMarker);
  preferences.putInt("tunExit", config.tunnelExitMarker);
  preferences.putInt("finApp", config.finishApproachMarker);
  preferences.putInt("finMark", config.finishMarker);
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
  config.markerSpacingCm = preferences.getUShort("markCm", config.markerSpacingCm);
  config.trackDistanceCm = preferences.getUShort("trackCm", config.trackDistanceCm);
  config.tunnelApproachMarker = preferences.getInt("tunApp", config.tunnelApproachMarker);
  config.tunnelStopMarker = preferences.getInt("tunStop", config.tunnelStopMarker);
  config.tunnelExitMarker = preferences.getInt("tunExit", config.tunnelExitMarker);
  config.finishApproachMarker = preferences.getInt("finApp", config.finishApproachMarker);
  config.finishMarker = preferences.getInt("finMark", config.finishMarker);
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
  currentLeftPwm = 0;
  currentRightPwm = 0;
}

static void brakeMotors() {
  // This is a conservative active reverse brake pulse.
  // Test your BTS7960 wiring: some builds may prefer coasting or both-input braking.
  writeMotorChannels(0, config.brakePwm, 0, config.brakePwm);
  currentLeftPwm = -config.brakePwm;
  currentRightPwm = -config.brakePwm;
}

static void driveForward(int pwm) {
  // Trim compensates for one motor being stronger than the other.
  // Positive trim increases right side and decreases left side.
  const int leftPwm = clampPwm(pwm - config.trim);
  const int rightPwm = clampPwm(pwm + config.trim);
  writeMotorChannels(leftPwm, 0, rightPwm, 0);
  currentLeftPwm = leftPwm;
  currentRightPwm = rightPwm;
}

static void driveReverse(int pwm) {
  const int safePwm = clampPwm(pwm);
  writeMotorChannels(0, safePwm, 0, safePwm);
  currentLeftPwm = -safePwm;
  currentRightPwm = -safePwm;
}

static void testLeftMotor(int pwm) {
  const int safePwm = clampPwm(pwm);
  writeMotorChannels(safePwm, 0, 0, 0);
  currentLeftPwm = safePwm;
  currentRightPwm = 0;
}

static void testRightMotor(int pwm) {
  const int safePwm = clampPwm(pwm);
  writeMotorChannels(0, 0, safePwm, 0);
  currentLeftPwm = 0;
  currentRightPwm = safePwm;
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

static int getFrontMarkerCount();

static int getRearMarkerCount() {
  noInterrupts();
  const int count = rearMarkerCount;
  interrupts();
  return count;
}

static int16_t estimateSpeedCmps() {
  noInterrupts();
  const uint32_t lastFrontUs = lastFrontMagnetUs;
  const uint32_t previousFrontUs = previousFrontMagnetUs;
  interrupts();

  const uint32_t segmentUs = lastFrontUs - previousFrontUs;
  if (segmentUs == 0) {
    return 0;
  }

  // Speed is only updated when a new marker arrives.
  return static_cast<int16_t>(((static_cast<uint32_t>(config.markerSpacingCm) * 1000000UL) + (segmentUs / 2)) / segmentUs);
}

static TelemetrySample makeTelemetrySample() {
  TelemetrySample sample;
  sample.timeMs = relativeTimeMs();
  sample.state = missionState;
  sample.frontMarker = getFrontMarkerCount();
  sample.rearMarker = getRearMarkerCount();
  sample.distanceCm10 = static_cast<int16_t>(constrain(static_cast<int>(lastDistanceCm * 10.0f), 0, 32767));
  sample.leftPwm = currentLeftPwm;
  sample.rightPwm = currentRightPwm;
  sample.speedCmps = estimateSpeedCmps();
  sample.tunnelDetected = isTunnelDetected();
  sample.tunnelConfirmed = tunnelConfirmed;
  return sample;
}

static void appendTelemetrySample() {
  telemetryLog[telemetryLogHead] = makeTelemetrySample();
  telemetryLogHead = (telemetryLogHead + 1) % TELEMETRY_LOG_CAPACITY;
  if (telemetryLogCount < TELEMETRY_LOG_CAPACITY) {
    telemetryLogCount++;
  }
}

static void updateTelemetryLog() {
  if (!missionStarted) {
    return;
  }

  if (millis() - lastTelemetryLogMs < TELEMETRY_LOG_INTERVAL_MS) {
    return;
  }

  lastTelemetryLogMs = millis();
  appendTelemetrySample();
}

static void processSensorEvents() {
  bool frontEvent = false;
  bool rearEvent = false;
  int frontCount = 0;
  int rearCount = 0;

  noInterrupts();
  if (frontMarkerEvent) {
    frontEvent = true;
    frontCount = frontMarkerCount;
    frontMarkerEvent = false;
  }
  if (rearMarkerEvent) {
    rearEvent = true;
    rearCount = rearMarkerCount;
    rearMarkerEvent = false;
  }
  interrupts();

  if (frontEvent) {
    logEventValue("FRONT_MARKER", frontCount);
  }
  if (rearEvent) {
    logEventValue("REAR_MARKER", rearCount);
  }
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
  if (missionState == MissionState::TestMode) {
    coastMotors();
  }
  resetMissionCounters();
  clearLogs();
  missionStartMs = millis();
  missionStarted = true;
  logEvent("RUN_START", "0");
  enterState(MissionState::Cruise);
  appendTelemetrySample();
  lastTelemetryLogMs = millis();
}

static void stopMission(MissionState stopState) {
  brakeMotors();
  enterState(stopState);
  if (stopState == MissionState::Finished) {
    logEventValue("RUN_END", getFrontMarkerCount());
  } else if (stopState == MissionState::EmergencyStop) {
    logEventValue("EMERGENCY_STOP", getFrontMarkerCount());
  }
  appendTelemetrySample();
  missionStarted = false;
}

static void updateMission() {
  const int marker = getFrontMarkerCount();

  switch (missionState) {
    case MissionState::Idle:
      coastMotors();
      break;

    case MissionState::TestMode:
      // Test commands are handled by web endpoints. Do not overwrite motor PWM here.
      break;

    case MissionState::Cruise:
      driveForward(config.cruisePwm);

      // Slow down early so marker 19 stop is repeatable.
      if (marker >= config.tunnelApproachMarker) {
        enterState(MissionState::TunnelApproach);
      }
      break;

    case MissionState::TunnelApproach:
      driveForward(config.approachPwm);

      // The HC-SR04 confirmation prevents stopping at marker 19 if the marker count is wrong.
      if (isTunnelDetected() && !tunnelConfirmed) {
        tunnelConfirmed = true;
        logEventFloat("TUNNEL_CONFIRMED", lastDistanceCm);
      }

      // Third strategy: marker 19 is the main stop trigger.
      // Marker 19 is around 9.5 m, safely inside the 9.12-10.00 m tunnel.
      if (marker >= config.tunnelStopMarker && tunnelConfirmed) {
        brakeMotors();
        enterState(MissionState::TunnelStop);
        logEventValue("TUNNEL_STOP_BEGIN", marker);
      }

      // Safety fallback: if marker 20 arrives without confirmation, do not stop outside the tunnel.
      // Continue the mission, but this means the tunnel sensor threshold or wiring needs tuning.
      if (marker >= config.tunnelExitMarker && !tunnelConfirmed) {
        logEventValue("TUNNEL_MISSED", marker);
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
        logEventValue("TUNNEL_STOP_END", millis() - stateStartedMs);
        enterState(MissionState::AfterTunnel);
      }
      break;

    case MissionState::AfterTunnel:
      driveForward(config.cruisePwm);

      if (marker >= config.finishApproachMarker) {
        logEventValue("FINISH_APPROACH_BEGIN", marker);
        enterState(MissionState::FinishApproach);
      }
      break;

    case MissionState::FinishApproach:
      driveForward(config.finishPwm);

      // Stop at the finish marker. This should be tuned with sensor placement.
      // If the front sensor is ahead of the vehicle center, you may need a small delay before braking.
      if (marker >= config.finishMarker) {
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
  page.reserve(9800);

  page += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>Kral Von Mobil Paneli</title><style>");
  page += F("body{font-family:system-ui;margin:20px;background:#10131a;color:#eef}button,input{font-size:16px;margin:4px;padding:8px}input{width:90px}.card{background:#1b2030;padding:14px;border-radius:12px;margin:12px 0}.danger{background:#b00020;color:white}.ok{background:#1f8f4d;color:white}.warn{background:#9a5a00;color:white}.test{background:#203453;border:1px solid #344869;padding:10px;border-radius:10px;margin-top:8px}</style>");
  page += F("</head><body><h1>Kral Von Mobil Kontrol Paneli</h1>");

  page += F("<div class='card'><button class='ok' onclick=\"fetch('/start')\">Görevi Başlat</button>");
  page += F("<button class='danger' onclick=\"fetch('/stop')\">Acil Durdur</button>");
  page += F("<button onclick=\"fetch('/reset')\">Sayaçları Sıfırla</button></div>");

  page += F("<div class='card'><h2>Test Modu</h2>");
  page += F("<p>Bunu yalnızca araç kaldırılmış veya sabitlenmişken kullan. Test modu, otonom döngünün manuel motor komutlarını ezmesini engeller.</p>");
  page += F("<div class='test'>PWM <input id='testPwm' type='number' min='0' max='255' value='80'>");
  page += F("<button class='warn' onclick=\"fetch('/test/enter')\">Test Moduna Gir</button>");
  page += F("<button onclick=\"fetch('/test/exit')\">Test Modundan Çık</button><br>");
  page += F("<button onclick=\"testCmd('forward')\">İleri</button>");
  page += F("<button onclick=\"testCmd('reverse')\">Geri</button>");
  page += F("<button onclick=\"testCmd('left')\">Sol Motor</button>");
  page += F("<button onclick=\"testCmd('right')\">Sağ Motor</button>");
  page += F("<button class='danger' onclick=\"fetch('/test/brake')\">Frenle</button>");
  page += F("<button onclick=\"fetch('/test/coast')\">Boşa Al</button></div></div>");

  page += F("<div class='card'><h2>Durum</h2><pre id='status'>Yükleniyor...</pre></div>");

  page += F("<div class='card'><h2>Ayarlar</h2>");
  page += F("<form action='/config' method='get'>");
  page += F("Seyir PWM <input name='cruise' value='"); page += config.cruisePwm; page += F("'><br>");
  page += F("Yaklaşma PWM <input name='approach' value='"); page += config.approachPwm; page += F("'><br>");
  page += F("Bitiş PWM <input name='finish' value='"); page += config.finishPwm; page += F("'><br>");
  page += F("Fren PWM <input name='brake' value='"); page += config.brakePwm; page += F("'><br>");
  page += F("Motor Trim <input name='trim' value='"); page += config.trim; page += F("'><br>");
  page += F("Marker Aralığı cm <input name='markerSpacingCm' value='"); page += config.markerSpacingCm; page += F("'><br>");
  page += F("Parkur Mesafesi cm <input name='trackDistanceCm' value='"); page += config.trackDistanceCm; page += F("'><br>");
  page += F("Tünel Yaklaşma Markerı <input name='tunnelApproachMarker' value='"); page += config.tunnelApproachMarker; page += F("'><br>");
  page += F("Tünelde Durma Markerı <input name='tunnelStopMarker' value='"); page += config.tunnelStopMarker; page += F("'><br>");
  page += F("Tünel Çıkış Markerı <input name='tunnelExitMarker' value='"); page += config.tunnelExitMarker; page += F("'><br>");
  page += F("Bitiş Yaklaşma Markerı <input name='finishApproachMarker' value='"); page += config.finishApproachMarker; page += F("'><br>");
  page += F("Bitiş Markerı <input name='finishMarker' value='"); page += config.finishMarker; page += F("'><br>");
  page += F("Tünel Durma ms <input name='stopMs' value='"); page += config.tunnelStopMs; page += F("'><br>");
  page += F("Tünel Mesafe Eşiği cm <input name='tunnelCm' value='"); page += config.tunnelDistanceCm; page += F("'><br>");
  page += F("Manyetik Debounce us <input name='debounce' value='"); page += config.magnetDebounceUs; page += F("'><br>");
  page += F("<button type='submit'>Kaydet</button></form></div>");

  page += F("<script>function testCmd(cmd){let p=document.getElementById('testPwm').value;fetch('/test/'+cmd+'?pwm='+p)}async function tick(){let r=await fetch('/status');document.getElementById('status').textContent=await r.text()}setInterval(tick,500);tick();</script>");
  page += F("</body></html>");
  return page;
}

static void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

static void sendCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static void handleStatus() {
  const TelemetrySample sample = makeTelemetrySample();

  String status;
  status.reserve(1150);
  status += '{';
  status += "\"timeMs\":"; status += sample.timeMs; status += ',';
  status += "\"state\":\""; status += stateName(sample.state); status += "\",";
  status += "\"stateLabel\":\""; status += stateNameTr(sample.state); status += "\",";
  status += "\"frontMarker\":"; status += sample.frontMarker; status += ',';
  status += "\"rearMarker\":"; status += sample.rearMarker; status += ',';
  status += "\"distanceCm\":"; status += String(sample.distanceCm10 / 10.0f, 1); status += ',';
  status += "\"leftPwm\":"; status += sample.leftPwm; status += ',';
  status += "\"rightPwm\":"; status += sample.rightPwm; status += ',';
  status += "\"tunnelDetected\":"; status += sample.tunnelDetected ? "true" : "false"; status += ',';
  status += "\"tunnelConfirmed\":"; status += sample.tunnelConfirmed ? "true" : "false"; status += ',';
  status += "\"estimatedSpeedMps\":"; status += String(sample.speedCmps / 100.0f, 2); status += ',';
  status += "\"missionStarted\":"; status += missionStarted ? "true" : "false"; status += ',';
  status += "\"telemetryLogCount\":"; status += telemetryLogCount; status += ',';
  status += "\"eventLogCount\":"; status += eventLogCount; status += ',';
  status += "\"markerSpacingCm\":"; status += config.markerSpacingCm; status += ',';
  status += "\"trackDistanceCm\":"; status += config.trackDistanceCm; status += ',';
  status += "\"tunnelApproachMarker\":"; status += config.tunnelApproachMarker; status += ',';
  status += "\"tunnelStopMarker\":"; status += config.tunnelStopMarker; status += ',';
  status += "\"tunnelExitMarker\":"; status += config.tunnelExitMarker; status += ',';
  status += "\"finishApproachMarker\":"; status += config.finishApproachMarker; status += ',';
  status += "\"finishMarker\":"; status += config.finishMarker; status += ',';
  status += "\"cruisePwm\":"; status += config.cruisePwm; status += ',';
  status += "\"approachPwm\":"; status += config.approachPwm; status += ',';
  status += "\"finishPwm\":"; status += config.finishPwm; status += ',';
  status += "\"brakePwm\":"; status += config.brakePwm; status += ',';
  status += "\"trim\":"; status += config.trim; status += ',';
  status += "\"tunnelStopMs\":"; status += config.tunnelStopMs; status += ',';
  status += "\"tunnelDistanceCm\":"; status += String(config.tunnelDistanceCm, 1); status += ',';
  status += "\"magnetDebounceUs\":"; status += config.magnetDebounceUs; status += ',';
  status += "\"ip\":\""; status += WiFi.softAPIP().toString(); status += "\"";
  status += '}';

  sendCorsHeaders();
  server.send(200, "application/json", status);
}

static void sendTelemetryCsv(bool download) {
  sendCorsHeaders();
  if (download) {
    server.sendHeader("Content-Disposition", "attachment; filename=telemetry.csv");
  }
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("timeMs,state,frontMarker,rearMarker,distanceCm,leftPwm,rightPwm,tunnelDetected,tunnelConfirmed,estimatedSpeedMps\n");

  const uint16_t start = (telemetryLogHead + TELEMETRY_LOG_CAPACITY - telemetryLogCount) % TELEMETRY_LOG_CAPACITY;
  char row[180];

  for (uint16_t i = 0; i < telemetryLogCount; i++) {
    const TelemetrySample &sample = telemetryLog[(start + i) % TELEMETRY_LOG_CAPACITY];
    snprintf(row, sizeof(row), "%lu,%s,%d,%d,%.1f,%d,%d,%d,%d,%.2f\n",
             static_cast<unsigned long>(sample.timeMs),
             stateName(sample.state),
             sample.frontMarker,
             sample.rearMarker,
             sample.distanceCm10 / 10.0f,
             sample.leftPwm,
             sample.rightPwm,
             sample.tunnelDetected ? 1 : 0,
             sample.tunnelConfirmed ? 1 : 0,
             sample.speedCmps / 100.0f);
    server.sendContent(row);
  }
  server.sendContent("");
}

static void sendEventsCsv(bool download) {
  sendCorsHeaders();
  if (download) {
    server.sendHeader("Content-Disposition", "attachment; filename=events.csv");
  }
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("timeMs,event,value\n");

  const uint16_t start = (eventLogHead + EVENT_LOG_CAPACITY - eventLogCount) % EVENT_LOG_CAPACITY;
  char row[100];

  for (uint16_t i = 0; i < eventLogCount; i++) {
    const EventSample &sample = eventLog[(start + i) % EVENT_LOG_CAPACITY];
    snprintf(row, sizeof(row), "%lu,%s,%s\n",
             static_cast<unsigned long>(sample.timeMs),
             sample.event,
             sample.value);
    server.sendContent(row);
  }
  server.sendContent("");
}

static void handleLogRecent() {
  sendTelemetryCsv(false);
}

static void handleLogDownload() {
  sendTelemetryCsv(true);
}

static void handleEvents() {
  sendEventsCsv(false);
}

static void handleClearLog() {
  clearLogs();
  logEvent("LOG_CLEARED", "0");
  sendCorsHeaders();
  server.send(200, "text/plain", "temizlendi");
}

static int testPwmFromRequest() {
  if (!server.hasArg("pwm")) {
    return 80;
  }
  return clampPwm(server.arg("pwm").toInt());
}

static bool requireTestMode() {
  if (missionState == MissionState::TestMode) {
    return true;
  }

  sendCorsHeaders();
  server.send(409, "text/plain", "once test moduna gir");
  return false;
}

static void handleTestEnter() {
  if (missionStarted) {
    sendCorsHeaders();
    server.send(409, "text/plain", "test modundan once gorevi durdur");
    return;
  }

  missionStarted = false;
  coastMotors();
  enterState(MissionState::TestMode);
  logEvent("TEST_MODE_ENTER", "0");
  sendCorsHeaders();
  server.send(200, "text/plain", "test moduna girildi");
}

static void handleTestExit() {
  coastMotors();
  enterState(MissionState::Idle);
  logEvent("TEST_MODE_EXIT", "0");
  sendCorsHeaders();
  server.send(200, "text/plain", "test modundan cikildi");
}

static void handleTestForward() {
  if (!requireTestMode()) return;
  const int pwm = testPwmFromRequest();
  driveForward(pwm);
  logEventValue("TEST_FORWARD", pwm);
  sendCorsHeaders();
  server.send(200, "text/plain", "ileri test");
}

static void handleTestReverse() {
  if (!requireTestMode()) return;
  const int pwm = testPwmFromRequest();
  driveReverse(pwm);
  logEventValue("TEST_REVERSE", pwm);
  sendCorsHeaders();
  server.send(200, "text/plain", "geri test");
}

static void handleTestLeft() {
  if (!requireTestMode()) return;
  const int pwm = testPwmFromRequest();
  testLeftMotor(pwm);
  logEventValue("TEST_LEFT", pwm);
  sendCorsHeaders();
  server.send(200, "text/plain", "sol motor test");
}

static void handleTestRight() {
  if (!requireTestMode()) return;
  const int pwm = testPwmFromRequest();
  testRightMotor(pwm);
  logEventValue("TEST_RIGHT", pwm);
  sendCorsHeaders();
  server.send(200, "text/plain", "sag motor test");
}

static void handleTestBrake() {
  if (!requireTestMode()) return;
  brakeMotors();
  logEventValue("TEST_BRAKE", config.brakePwm);
  sendCorsHeaders();
  server.send(200, "text/plain", "fren testi");
}

static void handleTestCoast() {
  if (!requireTestMode()) return;
  coastMotors();
  logEvent("TEST_COAST", "0");
  sendCorsHeaders();
  server.send(200, "text/plain", "bosa alma testi");
}

static void handleOptions() {
  sendCorsHeaders();
  server.send(204);
}

static void handleConfig() {
  if (server.hasArg("cruise")) config.cruisePwm = clampPwm(server.arg("cruise").toInt());
  if (server.hasArg("approach")) config.approachPwm = clampPwm(server.arg("approach").toInt());
  if (server.hasArg("finish")) config.finishPwm = clampPwm(server.arg("finish").toInt());
  if (server.hasArg("brake")) config.brakePwm = clampPwm(server.arg("brake").toInt());
  if (server.hasArg("trim")) config.trim = constrain(server.arg("trim").toInt(), -80, 80);
  if (server.hasArg("markerSpacingCm")) config.markerSpacingCm = constrain(server.arg("markerSpacingCm").toInt(), 10, 200);
  if (server.hasArg("trackDistanceCm")) config.trackDistanceCm = constrain(server.arg("trackDistanceCm").toInt(), 100, 5000);
  if (server.hasArg("tunnelApproachMarker")) config.tunnelApproachMarker = constrain(server.arg("tunnelApproachMarker").toInt(), 0, 200);
  if (server.hasArg("tunnelStopMarker")) config.tunnelStopMarker = constrain(server.arg("tunnelStopMarker").toInt(), 0, 200);
  if (server.hasArg("tunnelExitMarker")) config.tunnelExitMarker = constrain(server.arg("tunnelExitMarker").toInt(), 0, 200);
  if (server.hasArg("finishApproachMarker")) config.finishApproachMarker = constrain(server.arg("finishApproachMarker").toInt(), 0, 200);
  if (server.hasArg("finishMarker")) config.finishMarker = constrain(server.arg("finishMarker").toInt(), 0, 200);
  if (server.hasArg("stopMs")) config.tunnelStopMs = constrain(server.arg("stopMs").toInt(), 4500, 6500);
  if (server.hasArg("debounce")) config.magnetDebounceUs = constrain(server.arg("debounce").toInt(), 20000, 400000);
  if (server.hasArg("tunnelCm")) config.tunnelDistanceCm = constrain(server.arg("tunnelCm").toFloat(), 5.0f, 60.0f);

  saveConfig();
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleStart() {
  startMission();
  server.send(200, "text/plain", "baslatildi");
}

static void handleStop() {
  stopMission(MissionState::EmergencyStop);
  server.send(200, "text/plain", "durduruldu");
}

static void handleReset() {
  missionStarted = false;
  resetMissionCounters();
  enterState(MissionState::Idle);
  coastMotors();
  server.send(200, "text/plain", "sifirlandi");
}

static void setupWebUi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

  server.on("/", handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/log/recent", HTTP_GET, handleLogRecent);
  server.on("/log/download", HTTP_GET, handleLogDownload);
  server.on("/events", HTTP_GET, handleEvents);
  server.on("/log/clear", HTTP_GET, handleClearLog);
  server.on("/log/clear", HTTP_POST, handleClearLog);
  server.on("/test/enter", HTTP_GET, handleTestEnter);
  server.on("/test/exit", HTTP_GET, handleTestExit);
  server.on("/test/forward", HTTP_GET, handleTestForward);
  server.on("/test/reverse", HTTP_GET, handleTestReverse);
  server.on("/test/left", HTTP_GET, handleTestLeft);
  server.on("/test/right", HTTP_GET, handleTestRight);
  server.on("/test/brake", HTTP_GET, handleTestBrake);
  server.on("/test/coast", HTTP_GET, handleTestCoast);
  server.on("/status", HTTP_OPTIONS, handleOptions);
  server.on("/log/recent", HTTP_OPTIONS, handleOptions);
  server.on("/log/download", HTTP_OPTIONS, handleOptions);
  server.on("/events", HTTP_OPTIONS, handleOptions);
  server.on("/log/clear", HTTP_OPTIONS, handleOptions);
  server.on("/test/enter", HTTP_OPTIONS, handleOptions);
  server.on("/test/exit", HTTP_OPTIONS, handleOptions);
  server.on("/test/forward", HTTP_OPTIONS, handleOptions);
  server.on("/test/reverse", HTTP_OPTIONS, handleOptions);
  server.on("/test/left", HTTP_OPTIONS, handleOptions);
  server.on("/test/right", HTTP_OPTIONS, handleOptions);
  server.on("/test/brake", HTTP_OPTIONS, handleOptions);
  server.on("/test/coast", HTTP_OPTIONS, handleOptions);
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
  processSensorEvents();

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

  updateTelemetryLog();
}
