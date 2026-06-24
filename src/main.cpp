#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

#include "EncoderReader.h"
#include "ImuManager.h"
#include "Kinematics.h"
#include "PcaMotorDriver.h"
#include "ServoGripper.h"
#include "WheelSpeedController.h"
#include "pin_config.h"

constexpr const char *ALLOWED_CONTROLLER_BTADDR = "f7:87:e0:28:75:18";

constexpr uint16_t PCA_MOTOR_PWM_FREQ_HZ = 1500;
constexpr uint32_t TELEMETRY_PERIOD_MS = 500;
constexpr uint32_t SERIAL_LINE_TIMEOUT_MS = 80;
constexpr int STICK_DEADZONE = 120;  // dinaikkan dari 60: stick drift bikin FR+RL jalan sendiri
constexpr int STICK_MAX = 512;
constexpr float DPAD_MOVE_PERCENT = 40.0f;           // D-pad speed
constexpr float MAX_DRIVE_PERCENT = 40.0f;           // 40% × 80 RPM = 32 RPM target, FL mampu (max 55)
constexpr float MAX_TURN_PERCENT = 25.0f;            // Max rotasi manual
constexpr float MAX_YAW_CORRECTION_PERCENT = 28.0f; // Harus > deadband FR (26%) agar koreksi bisa gerak
constexpr bool IDLE_YAW_HOLD_ENABLED_DEFAULT = false;  // disable dulu: IMU drift bikin motor jalan sendiri
constexpr float YAW_HOLD_DEADBAND_DEG = 3.0f;           // deadband lebih besar biar gak sensitif
constexpr float IDLE_YAW_MAX_TURN_PERCENT = 22.0f;      // harus > deadband motor (FL=18%, FR=26%)
constexpr bool INVERT_MOVE_X = false;
constexpr bool INVERT_MOVE_Y = true;
constexpr bool INVERT_ROTATE = false;
constexpr bool YAW_CORRECTION_INVERTED_DEFAULT = true;
// DRIVE_CLOSED_LOOP_DEFAULT: setelah channel mapping fix (2026-06-24), encoder FL/FR reliable.
// RL/RR pakai feed-forward only (GPIO 34/35/36/39 = input-only, noise prone).
// WheelSpeedController.computeCommand() otomatis handle via flag hasEncoder per-wheel.
constexpr bool DRIVE_CLOSED_LOOP_DEFAULT = true;
constexpr bool RESET_BLUETOOTH_PAIRING_ON_BOOT = false;

struct YawPid {
  float kp = 1.15f;
  float ki = 0.0f;
  float kd = 0.035f;
  float integral = 0.0f;
  uint32_t lastUs = 0;

  void reset() {
    integral = 0.0f;
    lastUs = micros();
  }
};
void zeroYawTarget();

ControllerPtr activeController = nullptr;
String serialLine;
uint32_t lastSerialByteMs = 0;
uint32_t lastTelemetryMs = 0;
YawPid yawPid;
float yawTargetDeg = 0.0f;
float speedMultiplier = 1.0f; // Default full — Triangle=100%, Circle=50%
bool yawHoldEnabled = true;
bool idleYawHoldEnabled = IDLE_YAW_HOLD_ENABLED_DEFAULT;
bool yawCorrectionInverted = YAW_CORRECTION_INVERTED_DEFAULT;
bool driveClosedLoopEnabled = DRIVE_CLOSED_LOOP_DEFAULT;
bool fieldCentricEnabled = true;
bool headingControlMode = true; // true = absolute pointer (default), false = rotation rate
bool telemetryEnabled = false;
bool lastManualTurn = false;
bool lastCrossButton = false;
bool lastSquareButton = false;
bool lastStartButton = false;

// ─── Configurable Parameters (NVS Preferences) ──────────────────────────
float cfg_pid_kp = 1.15f;
float cfg_pid_ki = 0.0f;
float cfg_pid_kd = 0.035f;

float cfg_claw_depan_min = AngleClaw_Depan_MIN;
float cfg_claw_depan_max = AngleClaw_Depan_MAX;
float cfg_claw_belakang_min = AngleClaw_Belakang_MIN;
float cfg_claw_belakang_max = AngleClaw_Belakang_MAX;

float cfg_lift_depan_min = AngleLifter_Depan_MIN;
float cfg_lift_depan_max = AngleLifter_Depan_MAX;
float cfg_lift_belakang_min = AngleLifter_Belakang_MIN;
float cfg_lift_belakang_max = AngleLifter_Belakang_MAX;

float cfg_antitip_pitch_forward = ANTITIP_PITCH_FORWARD_LIMIT;
float cfg_antitip_pitch_backward = ANTITIP_PITCH_BACKWARD_LIMIT;
float cfg_antitip_roll = ANTITIP_ROLL_LIMIT;
float cfg_antitip_roll_throttle = ANTITIP_ROLL_LIFTER_THROTTLE;

// State globals for telemetry & tracking
float lastTargetFront = 1.0f;
float lastTargetRear = 1.0f;

// ─── Servo & Gripper ─────────────────────────────────────────────────────────
ServoESP32 servoLiftFront;   // Servo1 GPIO12 — lifter depan
ServoESP32 servoClawFront;   // Servo3 GPIO14 — claw depan
ServoESP32 servoLiftRear;    // Servo2 GPIO13 — lifter belakang
ServoESP32 servoClawRear;    // Servo4 GPIO32 — claw belakang
Gripper gripperFront;        // gripper[0] depan
Gripper gripperRear;         // gripper[1] belakang

// State tombol (edge-detect: hanya bereaksi saat pertama kali ditekan)
bool lastL1 = false;
bool lastL2 = false;
bool lastR1 = false;
bool lastR2 = false;
bool wheelTestActive = false;  // true saat mode test arah roda aktif (tahan Triangle)

Preferences preferences;
WebServer server(80);
bool inConfigMode = false;

void loadConfigurations() {
  preferences.begin("robot_cfg", true); // read-only
  cfg_pid_kp = preferences.getFloat("pid_kp", 1.15f);
  cfg_pid_ki = preferences.getFloat("pid_ki", 0.0f);
  cfg_pid_kd = preferences.getFloat("pid_kd", 0.035f);

  cfg_claw_depan_min = preferences.getFloat("claw_f_min", AngleClaw_Depan_MIN);
  cfg_claw_depan_max = preferences.getFloat("claw_f_max", AngleClaw_Depan_MAX);
  cfg_lift_depan_min = preferences.getFloat("lift_f_min", AngleLifter_Depan_MIN);
  cfg_lift_depan_max = preferences.getFloat("lift_f_max", AngleLifter_Depan_MAX);

  cfg_claw_belakang_min = preferences.getFloat("claw_r_min", AngleClaw_Belakang_MIN);
  cfg_claw_belakang_max = preferences.getFloat("claw_r_max", AngleClaw_Belakang_MAX);
  cfg_lift_belakang_min = preferences.getFloat("lift_r_min", AngleLifter_Belakang_MIN);
  cfg_lift_belakang_max = preferences.getFloat("lift_r_max", AngleLifter_Belakang_MAX);

  cfg_antitip_pitch_forward = preferences.getFloat("tip_pitch_f", ANTITIP_PITCH_FORWARD_LIMIT);
  cfg_antitip_pitch_backward = preferences.getFloat("tip_pitch_b", ANTITIP_PITCH_BACKWARD_LIMIT);
  cfg_antitip_roll = preferences.getFloat("tip_roll", ANTITIP_ROLL_LIMIT);
  cfg_antitip_roll_throttle = preferences.getFloat("tip_roll_th", ANTITIP_ROLL_LIFTER_THROTTLE);
  preferences.end();

  // Apply to PID yaw
  yawPid.kp = cfg_pid_kp;
  yawPid.ki = cfg_pid_ki;
  yawPid.kd = cfg_pid_kd;
}

void saveConfigurations() {
  preferences.begin("robot_cfg", false); // read-write
  preferences.putFloat("pid_kp", cfg_pid_kp);
  preferences.putFloat("pid_ki", cfg_pid_ki);
  preferences.putFloat("pid_kd", cfg_pid_kd);

  preferences.putFloat("claw_f_min", cfg_claw_depan_min);
  preferences.putFloat("claw_f_max", cfg_claw_depan_max);
  preferences.putFloat("lift_f_min", cfg_lift_depan_min);
  preferences.putFloat("lift_f_max", cfg_lift_depan_max);

  preferences.putFloat("claw_r_min", cfg_claw_belakang_min);
  preferences.putFloat("claw_r_max", cfg_claw_belakang_max);
  preferences.putFloat("lift_r_min", cfg_lift_belakang_min);
  preferences.putFloat("lift_r_max", cfg_lift_belakang_max);

  preferences.putFloat("tip_pitch_f", cfg_antitip_pitch_forward);
  preferences.putFloat("tip_pitch_b", cfg_antitip_pitch_backward);
  preferences.putFloat("tip_roll", cfg_antitip_roll);
  preferences.putFloat("tip_roll_th", cfg_antitip_roll_throttle);
  preferences.end();
}

int getBootMode() {
  preferences.begin("boot", true);
  int mode = preferences.getInt("mode", 0); // 0 = Competition, 1 = Config
  preferences.end();
  return mode;
}

void setBootMode(int mode) {
  preferences.begin("boot", false);
  preferences.putInt("mode", mode);
  preferences.end();
}

void updateGripperConfigs() {
  gripperFront = Gripper(&servoClawFront, &servoLiftFront, GripperConfig{
    .claw_close = cfg_claw_depan_max,
    .claw_open  = cfg_claw_depan_min,
    .lift_down  = cfg_lift_depan_max,
    .lift_up    = cfg_lift_depan_min
  });

  gripperRear = Gripper(&servoClawRear, &servoLiftRear, GripperConfig{
    .claw_close = cfg_claw_belakang_max,
    .claw_open  = cfg_claw_belakang_min,
    .lift_down  = cfg_lift_belakang_max,
    .lift_up    = cfg_lift_belakang_min
  });
}

// ─── HTTP WebServer Handlers ────────────────────────────────────────────────
void handleGetConfig() {
  String json = "{";
  json += "\"pid_kp\":" + String(cfg_pid_kp, 3) + ",";
  json += "\"pid_ki\":" + String(cfg_pid_ki, 3) + ",";
  json += "\"pid_kd\":" + String(cfg_pid_kd, 3) + ",";
  json += "\"claw_depan_min\":" + String(cfg_claw_depan_min, 1) + ",";
  json += "\"claw_depan_max\":" + String(cfg_claw_depan_max, 1) + ",";
  json += "\"lift_depan_min\":" + String(cfg_lift_depan_min, 1) + ",";
  json += "\"lift_depan_max\":" + String(cfg_lift_depan_max, 1) + ",";
  json += "\"claw_belakang_min\":" + String(cfg_claw_belakang_min, 1) + ",";
  json += "\"claw_belakang_max\":" + String(cfg_claw_belakang_max, 1) + ",";
  json += "\"lift_belakang_min\":" + String(cfg_lift_belakang_min, 1) + ",";
  json += "\"lift_belakang_max\":" + String(cfg_lift_belakang_max, 1) + ",";
  json += "\"antitip_pitch_forward\":" + String(cfg_antitip_pitch_forward, 1) + ",";
  json += "\"antitip_pitch_backward\":" + String(cfg_antitip_pitch_backward, 1) + ",";
  json += "\"antitip_roll\":" + String(cfg_antitip_roll, 1) + ",";
  json += "\"antitip_roll_throttle\":" + String(cfg_antitip_roll_throttle, 2);
  json += "}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

float parseJsonFloat(const String &json, const String &key, float fallback) {
  int keyIndex = json.indexOf("\"" + key + "\"");
  if (keyIndex < 0) return fallback;
  int colonIndex = json.indexOf(":", keyIndex);
  if (colonIndex < 0) return fallback;
  int commaIndex = json.indexOf(",", colonIndex);
  if (commaIndex < 0) commaIndex = json.indexOf("}", colonIndex);
  if (commaIndex < 0) return fallback;
  String valStr = json.substring(colonIndex + 1, commaIndex);
  valStr.trim();
  return valStr.toFloat();
}

void handlePostConfig() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    Serial.println("Received config body: " + body);

    cfg_pid_kp = parseJsonFloat(body, "pid_kp", cfg_pid_kp);
    cfg_pid_ki = parseJsonFloat(body, "pid_ki", cfg_pid_ki);
    cfg_pid_kd = parseJsonFloat(body, "pid_kd", cfg_pid_kd);

    cfg_claw_depan_min = parseJsonFloat(body, "claw_depan_min", cfg_claw_depan_min);
    cfg_claw_depan_max = parseJsonFloat(body, "claw_depan_max", cfg_claw_depan_max);
    cfg_lift_depan_min = parseJsonFloat(body, "lift_depan_min", cfg_lift_depan_min);
    cfg_lift_depan_max = parseJsonFloat(body, "lift_depan_max", cfg_lift_depan_max);

    cfg_claw_belakang_min = parseJsonFloat(body, "claw_belakang_min", cfg_claw_belakang_min);
    cfg_claw_belakang_max = parseJsonFloat(body, "claw_belakang_max", cfg_claw_belakang_max);
    cfg_lift_belakang_min = parseJsonFloat(body, "lift_belakang_min", cfg_lift_belakang_min);
    cfg_lift_belakang_max = parseJsonFloat(body, "lift_belakang_max", cfg_lift_belakang_max);

    cfg_antitip_pitch_forward = parseJsonFloat(body, "antitip_pitch_forward", cfg_antitip_pitch_forward);
    cfg_antitip_pitch_backward = parseJsonFloat(body, "antitip_pitch_backward", cfg_antitip_pitch_backward);
    cfg_antitip_roll = parseJsonFloat(body, "antitip_roll", cfg_antitip_roll);
    cfg_antitip_roll_throttle = parseJsonFloat(body, "antitip_roll_throttle", cfg_antitip_roll_throttle);

    // Apply to PID yaw
    yawPid.kp = cfg_pid_kp;
    yawPid.ki = cfg_pid_ki;
    yawPid.kd = cfg_pid_kd;
    yawPid.reset();

    // Apply to Grippers
    updateGripperConfigs();

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"missing body\"}");
  }
}

void handleGetTelemetry() {
  const ImuTelemetry imu = Imu().telemetry();
  
  float frontLiftVal = lastTargetFront;
  float rearLiftVal = lastTargetRear;
  int frontClawClosed = gripperFront.isClawClosed() ? 1 : 0;
  int rearClawClosed = gripperRear.isClawClosed() ? 1 : 0;

  String json = "{";
  json += "\"yaw\":" + String(imu.yawDeg, 2) + ",";
  json += "\"pitch\":" + String(imu.pitchDeg, 2) + ",";
  json += "\"roll\":" + String(imu.rollDeg, 2) + ",";
  json += "\"front_lift\":" + String(frontLiftVal, 2) + ",";
  json += "\"rear_lift\":" + String(rearLiftVal, 2) + ",";
  json += "\"front_claw\":" + String(frontClawClosed) + ",";
  json += "\"rear_claw\":" + String(rearClawClosed);
  json += "}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handlePostZeroYaw() {
  zeroYawTarget();
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handlePostTestServo() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    String target = ""; // "front" or "rear"
    String type = "";   // "lift" or "claw"
    float value = 0.0f;
    
    int targetIdx = body.indexOf("\"target\"");
    if (targetIdx >= 0) {
      int colonIdx = body.indexOf(":", targetIdx);
      int quote1 = body.indexOf("\"", colonIdx);
      int quote2 = body.indexOf("\"", quote1 + 1);
      target = body.substring(quote1 + 1, quote2);
    }
    
    int typeIdx = body.indexOf("\"type\"");
    if (typeIdx >= 0) {
      int colonIdx = body.indexOf(":", typeIdx);
      int quote1 = body.indexOf("\"", colonIdx);
      int quote2 = body.indexOf("\"", quote1 + 1);
      type = body.substring(quote1 + 1, quote2);
    }
    
    int valIdx = body.indexOf("\"value\"");
    if (valIdx >= 0) {
      int colonIdx = body.indexOf(":", valIdx);
      int commaIdx = body.indexOf(",", colonIdx);
      if (commaIdx < 0) commaIdx = body.indexOf("}", colonIdx);
      String valStr = body.substring(colonIdx + 1, commaIdx);
      valStr.trim();
      value = valStr.toFloat();
    }
    
    Serial.printf("[Test Servo] target: %s, type: %s, value: %.2f\n", target.c_str(), type.c_str(), value);

    if (target == "front") {
      if (type == "lift") {
        gripperFront.setLifter(value);
        lastTargetFront = value;
      } else if (type == "claw") {
        gripperFront.setClaw(value > 0.5f);
      }
    } else if (target == "rear") {
      if (type == "lift") {
        gripperRear.setLifter(value);
        lastTargetRear = value;
      } else if (type == "claw") {
        gripperRear.setClaw(value > 0.5f);
      }
    }

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(400, "application/json", "{\"status\":\"error\"}");
  }
}

void handlePostReboot() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Rebooting to competition mode...\"}");
  delay(500);
  
  saveConfigurations();
  setBootMode(0);
  
  Serial.println("Rebooting to Competition Mode...");
  ESP.restart();
}

void handleOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

void startConfigMode() {
  inConfigMode = true;
  Serial.println("=== CONFIG CALIBRATION MODE ===");
  
  btStop(); 
  
  Serial.println("Connecting to Wi-Fi SSID: IG : Triwahyu45");
  WiFi.mode(WIFI_STA);
  WiFi.begin("IG : Triwahyu45", "@Aguswahyu45");

  uint32_t startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Wi-Fi connected! Local IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi connection failed. Falling back to SoftAP mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Robot_Transporter_TC26");
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
  }

  if (MDNS.begin("Triwahyu45Transporter")) {
    Serial.println("mDNS responder started at http://Triwahyu45Transporter.local");
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("Error setting up MDNS responder!");
  }

  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, handlePostConfig);
  server.on("/api/config", HTTP_OPTIONS, handleOptions);
  
  server.on("/api/telemetry", HTTP_GET, handleGetTelemetry);
  server.on("/api/telemetry", HTTP_OPTIONS, handleOptions);
  
  server.on("/api/zero_yaw", HTTP_POST, handlePostZeroYaw);
  server.on("/api/zero_yaw", HTTP_OPTIONS, handleOptions);
  
  server.on("/api/test_servo", HTTP_POST, handlePostTestServo);
  server.on("/api/test_servo", HTTP_OPTIONS, handleOptions);
  
  server.on("/api/reboot", HTTP_POST, handlePostReboot);
  server.on("/api/reboot", HTTP_OPTIONS, handleOptions);
  
  server.begin();
  Serial.println("HTTP server started.");
}

void initServos() {
  // LEDC channel 4-7 (0-3 dihindari karena bisa dipakai library lain)
  servoLiftFront.begin(PIN_SERVO_LIFT_FRONT, 4); // ch4, GPIO12
  servoClawFront.begin(PIN_SERVO_CLAW_FRONT,  5); // ch5, GPIO14
  servoLiftRear.begin(PIN_SERVO_LIFT_REAR,    6); // ch6, GPIO13
  servoClawRear.begin(PIN_SERVO_CLAW_REAR,    7); // ch7, GPIO32

  updateGripperConfigs();

  Serial.println("[Servo] Gripper depan (GPIO12,14) + belakang (GPIO13,32) OK.");
}

float lastX = 0.0f;
float lastY = 0.0f;
float lastTurn = 0.0f;
bool lastBrakeState = true;

void driveRobot(float x, float y, float turn);
void zeroYawTarget();

float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

String btAddrToString(const uint8_t *addr) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02x:%02x:%02x:%02x:%02x:%02x",
           addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  return String(buffer);
}

String controllerAddrToString(ControllerPtr ctl) {
  ControllerProperties properties = ctl->getProperties();
  return btAddrToString(properties.btaddr);
}

bool isControllerAllowed(ControllerPtr ctl) {
  if (strlen(ALLOWED_CONTROLLER_BTADDR) == 0) return true;

  String actual = controllerAddrToString(ctl);
  actual.toLowerCase();
  String allowed = String(ALLOWED_CONTROLLER_BTADDR);
  allowed.toLowerCase();
  return actual == allowed;
}

float axisToPercent(int axis) {
  axis = constrain(axis, -STICK_MAX, STICK_MAX);
  if (abs(axis) < STICK_DEADZONE) return 0.0f;
  return constrain((static_cast<float>(axis) / STICK_MAX) * 100.0f, -100.0f, 100.0f);
}

void applyDpadOverride(ControllerPtr ctl, float &moveX, float &moveY) {
  const uint8_t dpad = ctl->dpad();
  float dpadX = 0.0f;
  float dpadY = 0.0f;

  if (dpad & DPAD_LEFT) dpadX -= DPAD_MOVE_PERCENT;
  if (dpad & DPAD_RIGHT) dpadX += DPAD_MOVE_PERCENT;
  if (dpad & DPAD_UP) dpadY -= DPAD_MOVE_PERCENT;
  if (dpad & DPAD_DOWN) dpadY += DPAD_MOVE_PERCENT;

  if (dpadX != 0.0f || dpadY != 0.0f) {
    moveX = dpadX;
    moveY = dpadY;
  }
}

float yawPidUpdate(float targetDeg, float measuredDeg, float gyroZDegPerSec) {
  const uint32_t nowUs = micros();
  float dt = static_cast<float>(nowUs - yawPid.lastUs) / 1000000.0f;
  yawPid.lastUs = nowUs;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

  const float error = wrap180(targetDeg - measuredDeg);
  yawPid.integral = constrain(yawPid.integral + error * dt * yawPid.ki, -12.0f, 12.0f);

  // Derivative on measurement, seperti referensi UNY: pakai gyro yaw rate, bukan selisih error kasar.
  const float output = (yawPid.kp * error) + yawPid.integral - (yawPid.kd * gyroZDegPerSec);
  const float direction = yawCorrectionInverted ? -1.0f : 1.0f;
  return constrain(output * direction, -MAX_TURN_PERCENT, MAX_TURN_PERCENT);
}

void applyFieldCentric(float &x, float &y, float yawDeg) {
  if (!fieldCentricEnabled) return;

  const float yawRad = yawDeg * DEG_TO_RAD;
  const float c = cosf(yawRad);
  const float s = sinf(yawRad);
  const float rx = (x * c) + (y * s);
  const float ry = (-x * s) + (y * c);
  x = rx;
  y = ry;
}

void stopWithBrake() {
  SpeedController().stop();
  SpeedController().setEnabled(driveClosedLoopEnabled);
  BrakeAll();
  lastX = 0.0f;
  lastY = 0.0f;
  lastTurn = 0.0f;
  lastBrakeState = true;
}

void idleYawHoldOrBrake(const ImuTelemetry &imu) {
  if (!yawHoldEnabled || !idleYawHoldEnabled || !imu.ready) {
    yawPid.reset();
    if (!lastBrakeState) stopWithBrake();
    return;
  }

  const float errorDeg = wrap180(yawTargetDeg - imu.yawDeg);
  if (fabs(errorDeg) <= YAW_HOLD_DEADBAND_DEG) {
    yawPid.reset();
    if (!lastBrakeState) stopWithBrake();
    return;
  }

  const float turnCommand =
      constrain(yawPidUpdate(yawTargetDeg, imu.yawDeg, imu.gyroZ),
                -IDLE_YAW_MAX_TURN_PERCENT, IDLE_YAW_MAX_TURN_PERCENT);
  driveRobot(0.0f, 0.0f, turnCommand);
}

int16_t percentToRawCommand(float percent) {
  percent = constrain(percent, -100.0f, 100.0f);
  return static_cast<int16_t>((percent / 100.0f) * 32767.0f);
}

void driveRobotRawPercent(float x, float y, float turn) {
  SpeedController().setEnabled(false);
  const WheelCommand mix = MixOmni4(percentToRawCommand(x), percentToRawCommand(y), percentToRawCommand(turn));
  DriveAll(mix.fl, mix.fr, mix.rl, mix.rr);
}

void driveRobot(float x, float y, float turn) {
  lastBrakeState = false;
  lastX = x;
  lastY = y;
  lastTurn = turn;
  if (driveClosedLoopEnabled) {
    DriveRobotPercent(x, y, turn);
  } else {
    driveRobotRawPercent(x, y, turn);
  }
}

void updateSpeedButtons(ControllerPtr ctl) {
  // △ Triangle (y) = full speed (default 100% dari MAX_DRIVE_PERCENT)
  // ○ Circle (b)   = setengah kecepatan (50% dari MAX_DRIVE_PERCENT)
  // Tidak tekan apapun → tetap di nilai sebelumnya (default 1.0 saat boot)
  if (ctl->y() && !ctl->x()) {        // Triangle (tanpa Square)
    speedMultiplier = 1.0f;
  } else if (ctl->b()) {            // Circle
    speedMultiplier = 0.5f;
  }
}

void zeroYawTarget() {
  Imu().resetYaw();
  yawTargetDeg = 0.0f;
  yawPid.reset();
  Serial.println("Yaw zeroed. Target=0 deg.");
}

void toggleYawHoldFromGamepad() {
  yawHoldEnabled = !yawHoldEnabled;
  idleYawHoldEnabled = yawHoldEnabled;
  yawTargetDeg = Imu().telemetry().yawDeg;
  yawPid.reset();
  Serial.print("Gamepad yaw lock=");
  Serial.println(yawHoldEnabled ? "ON" : "OFF");
}

void processGamepad(ControllerPtr ctl) {
  static uint32_t lastLedUpdateMs = 0;
  static uint8_t lastBatteryLevel = 255;
  static ControllerPtr lastCtl = nullptr;

  if (ctl && ctl->isConnected()) {
    if (ctl != lastCtl || millis() - lastLedUpdateMs > 5000 || ctl->battery() != lastBatteryLevel) {
      lastLedUpdateMs = millis();
      lastBatteryLevel = ctl->battery();
      lastCtl = ctl;

      uint8_t r = 0, g = 0, b = 0;
      if (lastBatteryLevel == 0 || lastBatteryLevel == 255) {
        // Unknown or not available: Dim Blue
        r = 0;
        g = 0;
        b = 64;
      } else if (lastBatteryLevel <= 63) {
        // 1 - 63: Red
        r = 255;
        g = 0;
        b = 0;
      } else if (lastBatteryLevel <= 127) {
        // 64 - 127: Orange
        r = 255;
        g = 100;
        b = 0;
      } else if (lastBatteryLevel <= 191) {
        // 128 - 191: Yellow-Green/Lime
        r = 128;
        g = 255;
        b = 0;
      } else {
        // 192 - 254: Green
        r = 0;
        g = 255;
        b = 0;
      }
      ctl->setColorLED(r, g, b);
      Serial.printf("[Gamepad] Battery: %d, LED color set to R:%d G:%d B:%d\n", lastBatteryLevel, r, g, b);
    }
  }

  // ─── Trigger Config Mode (Share + Options ditahan 2 detik) ───
  static uint32_t configTriggerStartMs = 0;
  if (ctl && ctl->isConnected() && ctl->miscSelect() && ctl->miscStart() && !ctl->l1() && !ctl->r1()) {
    if (configTriggerStartMs == 0) {
      configTriggerStartMs = millis();
      Serial.println("[System] Tombol Share + Options ditekan. Tahan 2 detik untuk masuk mode kalibrasi WiFi...");
    } else if (millis() - configTriggerStartMs > 2000) {
      Serial.println("[System] Memasuki Mode Kalibrasi WiFi! Rebooting...");
      setBootMode(1);
      delay(500);
      ESP.restart();
    }
  } else {
    configTriggerStartMs = 0;
  }

  // ─── Trigger IMU Gyro Calibration (L1 + R1 + Share + Options ditahan 3 detik) ───
  static uint32_t imuCalibTriggerStartMs = 0;
  if (ctl && ctl->isConnected() && ctl->l1() && ctl->r1() && ctl->miscSelect() && ctl->miscStart()) {
    if (imuCalibTriggerStartMs == 0) {
      imuCalibTriggerStartMs = millis();
      Serial.println("[System] Tombol L1+R1+Share+Options ditekan. Tahan 3 detik untuk kalibrasi Gyro...");
    } else if (millis() - imuCalibTriggerStartMs > 3000) {
      Serial.println("[System] Memulai Kalibrasi Gyro dari Gamepad...");
      stopWithBrake();
      Imu().startGyroCalibration();
      imuCalibTriggerStartMs = 0;
    }
  } else {
    imuCalibTriggerStartMs = 0;
  }

  // ─── Gripper diupdate SELALU, bahkan saat calibrating / disconnect ───
  {
    // Baca state tombol (false kalau tidak ada controller)
    const bool l1 = ctl && ctl->isConnected() ? (bool)ctl->l1() : false;
    const bool r1 = ctl && ctl->isConnected() ? (bool)ctl->r1() : false;

    // Read analog trigger pressure (normally 0-1023 in Bluepad32)
    float triggerL2 = 0.0f;
    float triggerR2 = 0.0f;
    if (ctl && ctl->isConnected()) {
      triggerL2 = (float)ctl->brake() / 1023.0f;
      if (triggerL2 > 1.0f) triggerL2 = 1.0f;
      triggerR2 = (float)ctl->throttle() / 1023.0f;
      if (triggerR2 > 1.0f) triggerR2 = 1.0f;
    }
    const bool l2 = triggerL2 > 0.1f;
    const bool r2 = triggerR2 > 0.1f;

    // Jika tidak ada stik terhubung (termasuk saat awal menyala), paksa claw tutup dan lifter naik
    if (ctl == nullptr || !ctl->isConnected()) {
      if (!gripperFront.isClawClosed()) {
        gripperFront.setClaw(true);
        Serial.println("[Gripper] Default: Front claw CLOSED");
      }
      if (!gripperRear.isClawClosed()) {
        gripperRear.setClaw(true);
        Serial.println("[Gripper] Default: Rear claw CLOSED");
      }
      // (setLifter diserahkan ke logika dinamis di bawah)
    }

    // L1: toggle claw depan (edge: hanya saat pertama kali ditekan)
    if (l1 && !lastL1) {
      gripperFront.toggleClaw();
      Serial.printf("[Gripper] Depan claw: %s\n",
                    gripperFront.isClawClosed() ? "CLOSE" : "OPEN");
    }
    // R1: toggle claw belakang
    if (r1 && !lastR1) {
      gripperRear.toggleClaw();
      Serial.printf("[Gripper] Belakang claw: %s\n",
                    gripperRear.isClawClosed() ? "CLOSE" : "OPEN");
    }

    // Target awal berdasarkan trigger stik (UP = 1.0f, DOWN = 0.0f)
    float targetFront = ctl && ctl->isConnected() ? (1.0f - triggerL2) : 1.0f;
    float targetRear  = ctl && ctl->isConnected() ? (1.0f - triggerR2) : 1.0f;

    // Membaca kemiringan (pitch & roll) dari MPU6050
    const ImuTelemetry imu = Imu().telemetry();
    bool triggeredForward = false;
    bool triggeredBackward = false;
    bool triggeredSideways = false;

    if (ANTITIP_ENABLED && imu.ready && !Imu().isCalibrating()) {
      // 1. Jatuh ke depan (kemiringan pitch positif melebihi batas)
      if (imu.pitchDeg > cfg_antitip_pitch_forward) {
        targetFront = 0.0f; // Paksa Front Lifter DOWN
        triggeredForward = true;
      }
      // 2. Jatuh ke belakang (kemiringan pitch negatif melebihi batas)
      else if (imu.pitchDeg < cfg_antitip_pitch_backward) {
        targetRear = 0.0f;  // Paksa Rear Lifter DOWN
        triggeredBackward = true;
      }

      // 3. Jatuh ke samping kiri/kanan (roll absolut melebihi batas)
      if (fabs(imu.rollDeg) > cfg_antitip_roll) {
        triggeredSideways = true;
      }
    }

    if (triggeredSideways) {
      // Turunkan lifter sedikit agar tidak kepentok sasis ketika claw membuka lebar
      if (targetFront > cfg_antitip_roll_throttle) {
        targetFront = cfg_antitip_roll_throttle;
      }
      if (targetRear > cfg_antitip_roll_throttle) {
        targetRear = cfg_antitip_roll_throttle;
      }
    }

    // Override claw jika terjadi jatuh ke samping (buka claw depan & belakang sepenuhnya)
    static bool lastTriggeredSideways = false;
    if (triggeredSideways) {
      servoClawFront.setAngle(cfg_claw_depan_min);
      servoClawRear.setAngle(cfg_claw_belakang_min);
      if (!lastTriggeredSideways) {
        Serial.printf("[Anti-Tip] Jatuh SAMPING terdeteksi! Roll: %.1f. Buka claw depan ke %.1f deg, belakang ke %.1f deg.\n",
                      imu.rollDeg, cfg_claw_depan_min, cfg_claw_belakang_min);
        lastTriggeredSideways = true;
      }
    } else {
      if (lastTriggeredSideways) {
        // Kembalikan ke state normal sesuai tombol stik
        gripperFront.setClaw(gripperFront.isClawClosed());
        gripperRear.setClaw(gripperRear.isClawClosed());
        Serial.println("[Anti-Tip] Robot kembali stabil. Posisi claw dikembalikan.");
        lastTriggeredSideways = false;
      }
    }

    // Front Lifter
    if (targetFront != lastTargetFront) {
      gripperFront.setLifter(targetFront);
      lastTargetFront = targetFront;
      
      if (triggeredForward) {
        Serial.printf("[Anti-Tip] Jatuh ke DEPAN terdeteksi! Pitch: %.1f. Lifter DEPAN turun.\n", imu.pitchDeg);
      } else if (triggeredSideways) {
        Serial.printf("[Anti-Tip] Jatuh SAMPING terdeteksi! Roll: %.1f. Lifter DEPAN turun ke %.2f agar tidak kepentok.\n", imu.rollDeg, targetFront);
      } else {
        Serial.printf("[Gripper] Depan lifter set ke: %s\n", targetFront == 0.0f ? "DOWN" : "UP");
      }
    }

    // Rear Lifter
    if (targetRear != lastTargetRear) {
      gripperRear.setLifter(targetRear);
      lastTargetRear = targetRear;

      if (triggeredBackward) {
        Serial.printf("[Anti-Tip] Jatuh ke BELAKANG terdeteksi! Pitch: %.1f. Lifter BELAKANG turun.\n", imu.pitchDeg);
      } else if (triggeredSideways) {
        Serial.printf("[Anti-Tip] Jatuh SAMPING terdeteksi! Roll: %.1f. Lifter BELAKANG turun ke %.2f agar tidak kepentok.\n", imu.rollDeg, targetRear);
      } else {
        Serial.printf("[Gripper] Belakang lifter set ke: %s\n", targetRear == 0.0f ? "DOWN" : "UP");
      }
    }

    lastL1 = l1;  lastL2 = l2;
    lastR1 = r1;  lastR2 = r2;
  }
  // ───────────────────────────────────────────────────────────────

  if (ctl == nullptr || !ctl->isConnected()) {
    if (!lastBrakeState) stopWithBrake();
    return;
  }

  if (Imu().isCalibrating()) {
    if (!lastBrakeState) stopWithBrake();
    return;
  }

  updateSpeedButtons(ctl);

  // ─── Wheel Direction Test Mode (tahan Triangle + L1/R1/L2/R2) ─────────────
  // L1 = FL, R1 = FR, L2 = RL, R2 = RR — spin pelan 25% raw untuk cek arah roda
  // Lepas Triangle untuk keluar dari mode ini.
  {
    const bool triHeld = ctl->y();  // Triangle di Bluepad32 = y()
    const bool wl1 = ctl->l1();
    const bool wr1 = ctl->r1();
    const float wl2 = (float)ctl->brake()    / 1023.0f;
    const float wr2 = (float)ctl->throttle() / 1023.0f;
    const bool wl2p = wl2 > 0.1f;
    const bool wr2p = wr2 > 0.1f;

    if (triHeld && (wl1 || wr1 || wl2p || wr2p)) {
      // Mode test aktif — override semua gerakan, speed SANGAT PELAN
      wheelTestActive = true;
      constexpr int16_t TEST_CMD = 7000;  // ~21% dari 32767 — di atas deadband FL(18%) & FR(26% via omni mix)
      int16_t fl = wl1  ? TEST_CMD : 0;
      int16_t fr = wr1  ? TEST_CMD : 0;
      int16_t rl = wl2p ? TEST_CMD : 0;
      int16_t rr = wr2p ? TEST_CMD : 0;
      SpeedController().setEnabled(false);
      DriveAll(fl, fr, rl, rr);
      Serial.printf("[WheelTest] FL=%d FR=%d RL=%d RR=%d (pelan 3.7%%)\n", fl, fr, rl, rr);
      return;  // jangan proses gerakan normal
    } else if (wheelTestActive) {
      // Baru keluar dari mode test — stop dan re-enable PID
      wheelTestActive = false;
      stopWithBrake();
      if (driveClosedLoopEnabled) {
        SpeedController().setEnabled(true);
      }
      Serial.println("[WheelTest] Mode test selesai.");
    }
  }
  // ──────────────────────────────────────────────────────────────────────────

  const bool crossButton = ctl->a();
  const bool squareButton = ctl->x();
  const bool startButton = ctl->miscStart();
  const bool shareButton = ctl->miscSelect();

  // Share + Square = toggle mode manual (tanpa IMU)
  // Berguna kalau IMU mati / glitch saat kompetisi
  if (shareButton && squareButton && !lastSquareButton) {
    bool manualMode = yawHoldEnabled || fieldCentricEnabled;
    yawHoldEnabled = !manualMode;
    idleYawHoldEnabled = !manualMode;
    fieldCentricEnabled = !manualMode;
    yawPid.reset();
    Serial.printf("[System] Mode: %s (Share+Square)\n",
      (!manualMode) ? "NORMAL (IMU aktif)" : "MANUAL (IMU off, no field-centric)");
  }
  // Square saja = toggle mode heading lock / rotasi manual
  else if (squareButton && !lastSquareButton && !shareButton) {
    headingControlMode = !headingControlMode;
    yawTargetDeg = Imu().telemetry().yawDeg;
    yawPid.reset();
    Serial.printf("[System] Mode: %s\n",
      headingControlMode ? "HEADING LOCK (analog kanan = arah)" : "ROTATION RATE (analog kanan = rotasi)");
  }
  if ((crossButton && !lastCrossButton) || (startButton && !lastStartButton && !shareButton)) {
    zeroYawTarget();
  }
  lastCrossButton = crossButton;
  lastSquareButton = squareButton;
  lastStartButton = startButton;

  ImuTelemetry imu = Imu().telemetry();
  float moveX = axisToPercent(ctl->axisX());
  float moveY = axisToPercent(ctl->axisY());

  // Right stick absolute heading calculation (pointing: UP = 0 deg, RIGHT = 90 deg, DOWN = 180 deg, LEFT = -90 deg)
  float rx = ctl->axisRX();
  float ry = -ctl->axisRY(); // Negated so up is positive Y
  float mag = sqrtf(rx*rx + ry*ry);
  if (headingControlMode && mag > 200.0f) {
    yawTargetDeg = atan2f(rx, ry) * RAD_TO_DEG;
  }

  float rotate = headingControlMode ? 0.0f : axisToPercent(ctl->axisRX());

  applyDpadOverride(ctl, moveX, moveY);

  if (INVERT_MOVE_X) moveX = -moveX;
  if (INVERT_MOVE_Y) moveY = -moveY;
  if (INVERT_ROTATE) rotate = -rotate;

  const bool hasMove = moveX != 0.0f || moveY != 0.0f;
  const bool hasManualTurn = rotate != 0.0f;

  if (!hasMove && !hasManualTurn) {
    if (lastManualTurn) {
      yawTargetDeg = imu.yawDeg;
      yawPid.reset();
      lastManualTurn = false;
    }

    idleYawHoldOrBrake(imu);
    return;
  }

  float xCommand = moveX * (MAX_DRIVE_PERCENT / 100.0f) * speedMultiplier;
  float yCommand = moveY * (MAX_DRIVE_PERCENT / 100.0f) * speedMultiplier;
  float turnCommand = 0.0f;

  applyFieldCentric(xCommand, yCommand, imu.yawDeg);

  if (hasManualTurn || !yawHoldEnabled || !imu.ready) {
    turnCommand = (rotate / 100.0f) * MAX_TURN_PERCENT * speedMultiplier;
    yawTargetDeg = imu.yawDeg;
    yawPid.reset();
    lastManualTurn = hasManualTurn;
  } else {
    if (lastManualTurn) {
      yawTargetDeg = imu.yawDeg;
      yawPid.reset();
      lastManualTurn = false;
    }
    // Clamp yaw PID output agar tidak spin terlalu kencang saat angle lock
    turnCommand = constrain(
      yawPidUpdate(yawTargetDeg, imu.yawDeg, imu.gyroZ) * speedMultiplier,
      -MAX_YAW_CORRECTION_PERCENT, MAX_YAW_CORRECTION_PERCENT
    );
  }

  driveRobot(xCommand, yCommand, turnCommand);
}

void printHelp() {
  Serial.println();
  Serial.println("=== Main_Program PCA + Gamepad + MPU6050 ===");
  Serial.println("Stick kiri / D-pad: maju/mundur + geser. Stick kanan X: rotasi manual.");
  Serial.println("Saat stick kanan dilepas, heading terakhir jadi angle lock.");
  Serial.println("Tombol PS: Cross=zero yaw, Square=toggle yaw lock, Options=zero yaw, Triangle=37.5%, Circle=20%.");
  Serial.println("Command: stop | zero | config | pid kp ki kd | field on/off | yaw on/off");
  Serial.println("         yawidle on/off | yawdir normal/invert | drive closed/open | telemetry on/off");
  Serial.println("         calib imu/enc/all | enc");
  Serial.println("Debug : move x y turn | rpm fl fr rl rr | raw fl fr rl rr | rawall value");
  Serial.println();
}

struct ParsedCommand {
  String name;
  float args[4] = {0, 0, 0, 0};
  uint8_t count = 0;
};

ParsedCommand parseCommand(String line) {
  line.trim();
  line.toLowerCase();

  ParsedCommand parsed;
  int cursor = 0;
  const int firstSpace = line.indexOf(' ');
  if (firstSpace < 0) {
    parsed.name = line;
    return parsed;
  }

  parsed.name = line.substring(0, firstSpace);
  cursor = firstSpace + 1;

  while (cursor < line.length() && parsed.count < 4) {
    while (cursor < line.length() && line[cursor] == ' ') cursor++;
    if (cursor >= line.length()) break;

    const int nextSpace = line.indexOf(' ', cursor);
    const String token = nextSpace < 0 ? line.substring(cursor) : line.substring(cursor, nextSpace);
    parsed.args[parsed.count++] = token.toFloat();
    cursor = nextSpace < 0 ? line.length() : nextSpace + 1;
  }
  return parsed;
}

float argOr(const ParsedCommand &cmd, uint8_t index, float fallback) {
  return index < cmd.count ? cmd.args[index] : fallback;
}

void printConfig() {
  Serial.println();
  Serial.println("=== Transporter Main Program ===");
  Serial.println("Motor: ESP32 -> PCA9685 -> 4x TB6612FNG");
  Serial.println("Mapping: A=FL, B=FR, C=RL, D=RR");
  Serial.print("Controller allowlist: ");
  Serial.println(strlen(ALLOWED_CONTROLLER_BTADDR) == 0 ? "(any)" : ALLOWED_CONTROLLER_BTADDR);
  Serial.print("Yaw PID kp/ki/kd: ");
  Serial.print(yawPid.kp, 4);
  Serial.print(" / ");
  Serial.print(yawPid.ki, 4);
  Serial.print(" / ");
  Serial.println(yawPid.kd, 4);
  Serial.print("Yaw hold: ");
  Serial.print(yawHoldEnabled ? "ON" : "OFF");
  Serial.print(" | Idle yaw: ");
  Serial.print(idleYawHoldEnabled ? "ON" : "OFF");
  Serial.print(" | Yaw dir: ");
  Serial.print(yawCorrectionInverted ? "INVERT" : "NORMAL");
  Serial.print(" | Field-centric: ");
  Serial.print(fieldCentricEnabled ? "ON" : "OFF");
  Serial.print(" | Telemetry: ");
  Serial.print(telemetryEnabled ? "ON" : "OFF");
  Serial.print(" | Drive: ");
  Serial.println(driveClosedLoopEnabled ? "CLOSED-LOOP" : "RAW");
  Serial.println("Default drive is RAW/open-loop for movement + MPU testing. Use `drive closed` only after encoder is valid.");
  Encoders().printConfig(Serial);
  Serial.println();
}

void printEncoderSnapshot() {
  const char *names[WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};
  Serial.println("=== Encoder Snapshot ===");
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const WheelTelemetry t = SpeedController().telemetry(i);
    Serial.print(names[i]);
    Serial.print(" count=");
    Serial.print(Encoders().count(i));
    Serial.print(" target=");
    Serial.print(t.targetRpm, 1);
    Serial.print(" rpm=");
    Serial.print(t.measuredRpm, 1);
    Serial.print(" cmd=");
    Serial.print(t.command);
    Serial.print(" fault=");
    Serial.println(t.fault ? "yes" : "no");
  }
}

void resetEncodersAndSpeedPid() {
  Encoders().reset();
  SpeedController().resetPid();
  if (driveClosedLoopEnabled) {
    SpeedController().stop();
    SpeedController().setEnabled(true);
  }
}

void handleCalibrationCommand(const String &line) {
  static float pprCalibTurns = 20.0f;

  if (line.indexOf(" ppr start") != -1) {
    int index = line.indexOf("start ");
    if (index != -1) {
      pprCalibTurns = line.substring(index + 6).toFloat();
      if (pprCalibTurns <= 0.0f) pprCalibTurns = 20.0f;
    } else {
      pprCalibTurns = 20.0f;
    }
    SpeedController().setEnabled(false);
    CoastAll();
    Encoders().reset();
    Serial.printf("[Calib] PPR Calibration STARTED. Rotate each wheel exactly %.1f full turns by hand, then type 'calib ppr stop'\n", pprCalibTurns);
    return;
  } else if (line.endsWith(" ppr stop")) {
    int32_t fl = abs(Encoders().count(WHEEL_FL));
    int32_t fr = abs(Encoders().count(WHEEL_FR));
    int32_t rl = abs(Encoders().count(WHEEL_RL));
    int32_t rr = abs(Encoders().count(WHEEL_RR));

    float pprFL = static_cast<float>(fl) / pprCalibTurns;
    float pprFR = static_cast<float>(fr) / pprCalibTurns;
    float pprRL = static_cast<float>(rl) / pprCalibTurns;
    float pprRR = static_cast<float>(rr) / pprCalibTurns;

    Serial.println("\n=== PPR Calibration Results ===");
    Serial.printf("Raw Ticks collected: FL=%ld, FR=%ld, RL=%ld, RR=%ld\n", (long)fl, (long)fr, (long)rl, (long)rr);
    Serial.printf("Calculated PPR (for %.1f turns):\n", pprCalibTurns);
    Serial.printf("  FL: %.2f\n", pprFL);
    Serial.printf("  FR: %.2f\n", pprFR);
    Serial.printf("  RL: %.2f\n", pprRL);
    Serial.printf("  RR: %.2f\n", pprRR);

    Encoders().setPpr(WHEEL_FL, pprFL);
    Encoders().setPpr(WHEEL_FR, pprFR);
    Encoders().setPpr(WHEEL_RL, pprRL);
    Encoders().setPpr(WHEEL_RR, pprRR);
    Serial.println("PPR settings saved to NVS and applied dynamically.");

    stopWithBrake();
    return;
  }

  stopWithBrake();

  if (line.endsWith(" imu")) {
    Imu().startGyroCalibration();
  } else if (line.endsWith(" enc")) {
    resetEncodersAndSpeedPid();
    Serial.println("Encoder counts and wheel PID reset.");
  } else if (line.endsWith(" all")) {
    resetEncodersAndSpeedPid();
    Imu().startGyroCalibration();
  } else {
    Serial.println("Usage: calib imu | calib enc | calib all | calib ppr start [turns] | calib ppr stop");
  }
}

void handleRawCommand(const ParsedCommand &cmd) {
  SpeedController().setEnabled(false);

  if (cmd.name == "rawall") {
    const int16_t value = static_cast<int16_t>(constrain(argOr(cmd, 0, 0), -32767.0f, 32767.0f));
    DriveAll(value, value, value, value);
    return;
  }

  const int16_t fl = static_cast<int16_t>(constrain(argOr(cmd, 0, 0), -32767.0f, 32767.0f));
  const int16_t fr = static_cast<int16_t>(constrain(argOr(cmd, 1, fl), -32767.0f, 32767.0f));
  const int16_t rl = static_cast<int16_t>(constrain(argOr(cmd, 2, fl), -32767.0f, 32767.0f));
  const int16_t rr = static_cast<int16_t>(constrain(argOr(cmd, 3, fr), -32767.0f, 32767.0f));
  DriveAll(fl, fr, rl, rr);
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  const ParsedCommand cmd = parseCommand(line);
  if (cmd.name.length() == 0) return;

  if (cmd.name == "h" || cmd.name == "help") {
    printHelp();
  } else if (cmd.name == "p" || cmd.name == "config") {
    printConfig();
  } else if (cmd.name == "s" || cmd.name == "stop") {
    stopWithBrake();
  } else if (cmd.name == "z" || cmd.name == "zero") {
    resetEncodersAndSpeedPid();
    zeroYawTarget();
    Serial.println("Zero: encoders, yaw, and PID reset.");
  } else if (cmd.name == "calib") {
    handleCalibrationCommand(line);
  } else if (cmd.name == "enc") {
    printEncoderSnapshot();
  } else if (cmd.name == "drive") {
    if (line.endsWith(" closed")) {
      driveClosedLoopEnabled = true;
      SpeedController().stop();
      SpeedController().setEnabled(true);
    } else if (line.endsWith(" open") || line.endsWith(" raw")) {
      driveClosedLoopEnabled = false;
      SpeedController().setEnabled(false);
      BrakeAll();
    } else {
      Serial.println("Usage: drive closed | drive open");
    }
    printConfig();
  } else if (cmd.name == "pid") {
    yawPid.kp = argOr(cmd, 0, yawPid.kp);
    yawPid.ki = argOr(cmd, 1, yawPid.ki);
    yawPid.kd = argOr(cmd, 2, yawPid.kd);
    yawPid.reset();
    printConfig();
  } else if (cmd.name == "field") {
    if (line.endsWith(" on")) {
      fieldCentricEnabled = true;
    } else if (line.endsWith(" off")) {
      fieldCentricEnabled = false;
    } else {
      fieldCentricEnabled = argOr(cmd, 0, fieldCentricEnabled ? 1.0f : 0.0f) > 0.5f;
    }
    printConfig();
  } else if (cmd.name == "telemetry") {
    if (line.endsWith(" on")) {
      telemetryEnabled = true;
    } else if (line.endsWith(" off")) {
      telemetryEnabled = false;
    } else {
      telemetryEnabled = argOr(cmd, 0, telemetryEnabled ? 1.0f : 0.0f) > 0.5f;
    }
    printConfig();
  } else if (cmd.name == "yaw") {
    if (line.endsWith(" on")) {
      yawHoldEnabled = true;
      idleYawHoldEnabled = true;
    } else if (line.endsWith(" off")) {
      yawHoldEnabled = false;
      idleYawHoldEnabled = false;
    } else {
      yawHoldEnabled = argOr(cmd, 0, yawHoldEnabled ? 1.0f : 0.0f) > 0.5f;
      idleYawHoldEnabled = yawHoldEnabled;
    }
    printConfig();
  } else if (cmd.name == "yawidle") {
    if (line.endsWith(" on")) {
      idleYawHoldEnabled = true;
      yawHoldEnabled = true;
    } else if (line.endsWith(" off")) {
      idleYawHoldEnabled = false;
    } else {
      idleYawHoldEnabled = argOr(cmd, 0, idleYawHoldEnabled ? 1.0f : 0.0f) > 0.5f;
      if (idleYawHoldEnabled) yawHoldEnabled = true;
    }
    yawPid.reset();
    printConfig();
  } else if (cmd.name == "yawdir") {
    if (line.endsWith(" invert")) {
      yawCorrectionInverted = true;
    } else if (line.endsWith(" normal")) {
      yawCorrectionInverted = false;
    } else {
      yawCorrectionInverted = argOr(cmd, 0, yawCorrectionInverted ? 1.0f : 0.0f) > 0.5f;
    }
    yawPid.reset();
    printConfig();
  } else if (cmd.name == "move") {
    driveRobot(argOr(cmd, 0, 0), argOr(cmd, 1, 0), argOr(cmd, 2, 0));
  } else if (cmd.name == "rpm") {
    SetWheelTargetRpm(argOr(cmd, 0, 0), argOr(cmd, 1, 0), argOr(cmd, 2, 0), argOr(cmd, 3, 0));
  } else if (cmd.name == "raw" || cmd.name == "rawall") {
    handleRawCommand(cmd);
  } else if (cmd.name == "mode") {
    if (line.endsWith(" config") || line.endsWith(" calib")) {
      Serial.println("[System] Rebooting to Config (WiFi) Mode...");
      setBootMode(1);
      delay(500);
      ESP.restart();
    } else if (line.endsWith(" comp") || line.endsWith(" normal")) {
      Serial.println("[System] Rebooting to Competition (Gamepad) Mode...");
      setBootMode(0);
      delay(500);
      ESP.restart();
    } else {
      Serial.println("Usage: mode config | mode comp");
    }
  } else {
    Serial.print("Unknown command: ");
    Serial.println(cmd.name);
    printHelp();
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    lastSerialByteMs = millis();
    if (c == '\n' || c == '\r') {
      handleCommand(serialLine);
      serialLine = "";
    } else if (serialLine.length() < 64) {
      serialLine += c;
    }
  }

  if (serialLine.length() > 0 && millis() - lastSerialByteMs > SERIAL_LINE_TIMEOUT_MS) {
    handleCommand(serialLine);
    serialLine = "";
  }
}

void printTelemetry() {
  const ImuTelemetry imu = Imu().telemetry();
  Serial.print("ctl=");
  Serial.print(activeController && activeController->isConnected() ? "yes" : "no");
  Serial.print(" cmd x/y/t=");
  Serial.print(lastX, 1);
  Serial.print("/");
  Serial.print(lastY, 1);
  Serial.print("/");
  Serial.print(lastTurn, 1);
  Serial.print(" speed=");
  Serial.print(MAX_DRIVE_PERCENT * speedMultiplier, 0);
  Serial.print("% yaw=");
  Serial.print(imu.yawDeg, 1);
  Serial.print(" target=");
  Serial.print(yawTargetDeg, 1);
  Serial.print(" gz=");
  Serial.print(imu.gyroZ, 1);
  Serial.print(" brake=");
  Serial.print(lastBrakeState ? "yes" : "no");
  Serial.print(" drive=");
  Serial.print(driveClosedLoopEnabled ? "closed" : "open");
  Serial.print(" cal=");
  if (Imu().isCalibrating()) {
    Serial.print("imu ");
    Serial.print(Imu().calibrationSamples());
  } else {
    Serial.print("ready");
  }
  Serial.print(" | ");

  const char *names[WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const WheelTelemetry t = SpeedController().telemetry(i);
    Serial.print(names[i]);
    Serial.print(":");
    Serial.print(t.targetRpm, 0);
    Serial.print("/");
    Serial.print(t.measuredRpm, 0);
    Serial.print("/");
    Serial.print(t.command);
    Serial.print(" ");
  }
  Serial.println();
}

void onConnectedController(ControllerPtr ctl) {
  const String addr = controllerAddrToString(ctl);
  Serial.print("Controller connected: ");
  Serial.println(addr);
  Serial.print("Model: ");
  Serial.println(ctl->getModelName().c_str());

  if (!isControllerAllowed(ctl)) {
    Serial.print("Rejected controller. Allowed=");
    Serial.println(ALLOWED_CONTROLLER_BTADDR);
    ctl->disconnect();
    return;
  }

  if (activeController != nullptr && activeController->isConnected()) {
    Serial.println("Rejected controller, another controller already active.");
    ctl->disconnect();
    return;
  }

  activeController = ctl;
  yawTargetDeg = Imu().telemetry().yawDeg;
  yawPid.reset();
  Serial.println("Controller accepted.");
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected.");
  if (ctl == activeController) {
    activeController = nullptr;
  }
  stopWithBrake();
}

void setupGamepad() {
  const uint8_t *localAddr = BP32.localBdAddress();
  Serial.print("ESP32 Bluetooth address: ");
  Serial.println(btAddrToString(localAddr));

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(true);
  if (RESET_BLUETOOTH_PAIRING_ON_BOOT) {
    // BP32.forgetBluetoothKeys();
    Serial.println("Bluetooth pairing keys cleared on boot.");
  }
}

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // Muat konfigurasi dari Preferences (NVS Flash)
  loadConfigurations();

  printHelp();
  printConfig();

  InitDriveMotors(PCA_MOTOR_PWM_FREQ_HZ);
  Encoders().begin();
  Encoders().reset();
  InitWheelSpeedController();
  
  // IMU MPU6050 diinisialisasi
  Imu().begin(true);
  Imu().resetYaw();
  yawTargetDeg = 0.0f;
  yawPid.reset();
  
  // Inisialisasi servo dengan konfigurasi terdaftar
  initServos();

  int mode = getBootMode();
  if (mode == 1) {
    startConfigMode();
  } else {
    setupGamepad();
    Serial.println("Ready. Pair/connect gamepad atau gunakan serial commands.");
  }
  
  stopWithBrake();
}

void loop() {
  pollSerial();
  Imu().update();

  if (inConfigMode) {
    server.handleClient();
  } else {
    BP32.update();
    processGamepad(activeController);
    UpdateWheelSpeedController();
  }

  const uint32_t nowMs = millis();
  if (telemetryEnabled && nowMs - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry();
  }
}
