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
// Speed: open-loop, 55% PWM langsung ke ZK-5AD. Deadband FL=20%, FR=25%.
// 55% = cukup cepat tanpa terlalu agresif. Kalau terlalu cepat, turunkan ke 45%.
constexpr float MAX_DRIVE_PERCENT = 85.0f;
// Max rotasi manual (stick kanan di headingControlMode=false)
constexpr float MAX_TURN_PERCENT = 30.0f;
// Yaw correction: HARUS jauh di atas deadband FR=25%.
// Terlalu kencang -> kurangi. Terlalu lemah -> naikkan. Osilasi -> turunkan KP di struct YawPid.
constexpr float MAX_YAW_CORRECTION_PERCENT = 45.0f;  // DRIVING: batas saat gerak, tidak dominasi movement
// IDLE_YAW_HOLD: aktifkan agar right stick bisa aim bahkan saat robot diam.
// Saat idle + right stick arah kanan → robot rotate ke kanan & hold.
// Deadband 3°: stop koreksi kecil di bawah batas drift IMU.
// YAW_MIN_CORRECTION: snap output ke minimum ini agar motor PASTI bergerak (motor deadband ~25%).
// Efek: robot hold heading kuat, error >3° selalu dikoreksi dengan MINIMUM 28% power.
constexpr float YAW_HOLD_DEADBAND_DEG      = 2.0f;   // lock angle: koreksi mulai dari 2.0deg
constexpr float YAW_MIN_CORRECTION_PERCENT  = 35.0f;  // snap min: lebih dari deadband motor (was 28)
constexpr bool  IDLE_YAW_HOLD_ENABLED_DEFAULT = true;
constexpr float IDLE_YAW_MAX_TURN_PERCENT   = 100.0f; // IDLE: full speed snap (robot diam)
constexpr bool INVERT_MOVE_X = false;
constexpr bool INVERT_MOVE_Y = true;
constexpr bool INVERT_ROTATE = false;
constexpr bool YAW_CORRECTION_INVERTED_DEFAULT = true;   // BENAR: +error→-turn→CCW→yaw naik→reach target

// DRIVE_CLOSED_LOOP: DEFAULT = false (open-loop) karena motor ZK-5AD punya respons non-linear.
// Di RPM rendah (<50% PWM), actual RPM JAUH lebih tinggi dari prediksi linear feedforward.
// Akibat: PID error besar → correction -10000 raw → command turun ke deadband → robot berhenti!
// Open-loop: raw PWM langsung dari joystick, tanpa RPM targeting. Lebih andal untuk kompetisi.
// Untuk enable closed-loop: serial command 'drive closed'
constexpr bool DRIVE_CLOSED_LOOP_DEFAULT = false;
constexpr bool RESET_BLUETOOTH_PAIRING_ON_BOOT = false;

struct YawPid {
  float kp = 4.00f;  // Tuned: was 5.0, kurangi untuk redam osilasi koreksi yaw
  float ki = 0.0f;
  float kd = 0.70f;  // LOW KD = terminal velocity tinggi, zero overshoot (was 1.5)
  float integral  = 0.0f;
  float lastError = 0.0f;  // untuk hysteresis ±180° boundary
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
float cfg_pid_kp = 4.00f;   // Tuned: was 5.0
 float cfg_pid_ki = 0.0f;
float cfg_pid_kd = 0.70f;

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
float lastTargetRear  = 1.0f;

// Delta tracking untuk right stick heading (hindari jump di ±180° boundary)
float prevStickAngleDeg = 0.0f;  // sudut stick sebelumnya
bool  prevStickActive   = false; // apakah stick aktif di frame sebelumnya

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
bool calibLockActive = false;  // true = semua kontrol dikunci saat kalibrasi IMU aktif
bool safeOverride    = false;  // true = bypass safe mode untuk testing tanpa stick

Preferences preferences;
WebServer server(80);
bool inConfigMode = false;

void loadConfigurations() {
  preferences.begin("robot_cfg", true); // read-only
  cfg_pid_kp = preferences.getFloat("pid_kp", 4.00f);
  cfg_pid_ki = preferences.getFloat("pid_ki", 0.0f);
  cfg_pid_kd = preferences.getFloat("pid_kd", 0.70f);

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
  Serial.flush();
  
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
    yield();
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

  // wrap180 pada KEDUANYA dulu sebelum hitung error.
  // Kalau IMU akumulasi yaw tak terbatas (mis. 540°), wrap180(0-540)=-180 → robot muter setengah!
  float error = wrap180(wrap180(targetDeg) - wrap180(measuredDeg));

  // Hysteresis ±180° boundary: mencegah chattering saat target tepat berlawanan dengan robot.
  // Threshold 150° (bukan 170°) agar aktif LEBIH AWAL sebelum osilasi sempat mulai.
  if (fabsf(error) > 150.0f && fabsf(yawPid.lastError) > 150.0f && error * yawPid.lastError < 0.0f) {
    error = yawPid.lastError;  // pertahankan arah sebelumnya agar tidak flip
  }
  yawPid.lastError = error;
  yawPid.integral = constrain(yawPid.integral + error * dt * yawPid.ki, -12.0f, 12.0f);

  // Derivative on measurement, seperti referensi UNY: pakai gyro yaw rate, bukan selisih error kasar.
  const float output = (yawPid.kp * error) + yawPid.integral - (yawPid.kd * gyroZDegPerSec);
  const float direction = yawCorrectionInverted ? -1.0f : 1.0f;
  // JANGAN constrain di sini! Biarkan caller yang set batas sesuai kebutuhan.
  // Dulu: constrain ke MAX_TURN_PERCENT=55% → heading correction cuma 55% meski limit=100%!
  return output * direction;
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

  const float rawTurn =
      constrain(yawPidUpdate(yawTargetDeg, imu.yawDeg, imu.gyroZ),
                -IDLE_YAW_MAX_TURN_PERCENT, IDLE_YAW_MAX_TURN_PERCENT);
  // Snap: jika output < deadband motor, snap ke minimum agar motor PASTI bergerak.
  // Tidak pakai graduated snap: motor butuh min ~25% untuk bergerak, berapapun errornya.
  const float turnCommand = (fabsf(rawTurn) > 0.5f && fabsf(rawTurn) < YAW_MIN_CORRECTION_PERCENT)
      ? copysignf(YAW_MIN_CORRECTION_PERCENT, rawTurn)
      : rawTurn;
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

// ─── Non-blocking gripper wiggle state machine ───────────────────────────────────
// times=1: combo terdeteksi | times=2: mulai kalibrasi | times=3: selesai
static uint8_t  wgl_remaining = 0;
static uint8_t  wgl_step      = 0;
static uint32_t wgl_lastMs    = 0;
static bool     wgl_origFC    = false;
static bool     wgl_origRC    = false;

void startGripperWiggle(int times) {
  if (times <= 0) return;
  wgl_origFC    = gripperFront.isClawClosed();
  wgl_origRC    = gripperRear.isClawClosed();
  wgl_remaining = (uint8_t)(times * 2);
  wgl_step      = 0;
  wgl_lastMs    = millis();
}

bool isWiggleBusy() { return wgl_remaining > 0; }

void updateGripperWiggle() {
  if (!wgl_remaining) return;
  if (millis() - wgl_lastMs < 130) return;
  wgl_lastMs = millis();
  bool flip = (wgl_step % 2 == 0);
  gripperFront.setClaw(flip ? !wgl_origFC : wgl_origFC);
  gripperRear.setClaw(flip  ? !wgl_origRC : wgl_origRC);
  ++wgl_step;
  if (!--wgl_remaining) {
    gripperFront.setClaw(wgl_origFC);  // restore posisi asal
    gripperRear.setClaw(wgl_origRC);
  }
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

  // ─── BT Re-pair Restart (Share + Options ditahan 2 detik) ───
  // Share = miscSelect, Options = miscStart
  static uint32_t btRestartTriggerMs = 0;
  if (ctl && ctl->isConnected() && ctl->miscSelect() && ctl->miscStart() && !ctl->l1() && !ctl->r1()) {
    if (btRestartTriggerMs == 0) {
      btRestartTriggerMs = millis();
      Serial.println("[System] Share+Options ditekan. Tahan 2 detik untuk restart BT re-pair...");
    } else if (millis() - btRestartTriggerMs > 2000) {
      Serial.println("[System] Restarting untuk BT re-pair...");
      stopWithBrake();
      Serial.flush();
      ESP.restart();
    }
  } else {
    btRestartTriggerMs = 0;
  }

  // ─── WiFi Config Mode (L1 + R1 + Options ditahan 3 detik) ───
  static uint32_t wifiConfigTriggerMs = 0;
  if (ctl && ctl->isConnected() && ctl->l1() && ctl->r1() && ctl->miscStart() && !ctl->miscSelect()) {
    if (wifiConfigTriggerMs == 0) {
      wifiConfigTriggerMs = millis();
      Serial.println("[System] L1+R1+Options ditekan. Tahan 3 detik untuk masuk WiFi Config Mode...");
    } else if (millis() - wifiConfigTriggerMs > 3000) {
      Serial.println("[System] Memasuki WiFi Config Mode! Rebooting...");
      setBootMode(1);
      Serial.flush();
      ESP.restart();
    }
  } else {
    wifiConfigTriggerMs = 0;
  }

  // ─── IMU Gyro Calibration (L1+R1+Share+Options ditahan 3 detik) ───
  static uint32_t imuCalibTriggerMs  = 0;
  static uint32_t imuCalibLastBeepMs = 0;
  static uint32_t calibSafeGripMs    = 0;
  const bool calibCombo = ctl && ctl->isConnected()
      && ctl->l1() && ctl->r1() && ctl->miscSelect() && ctl->miscStart();

  if (calibLockActive) {
    // ── Dalam lock mode: detect exit (combo ditahan 3 detik) ─────────────
    if (calibCombo) {
      if (imuCalibTriggerMs == 0) {
        imuCalibTriggerMs = millis();
        Serial.println("[Calib] Tahan 3 detik untuk keluar mode kalibrasi...");
      } else if (millis() - imuCalibTriggerMs > 3000) {
        calibLockActive   = false;
        imuCalibTriggerMs = 0;
        calibSafeGripMs   = 0;
        Serial.println("[Calib] Mode kalibrasi diakhiri secara manual.");
        startGripperWiggle(3);
      }
    } else {
      imuCalibTriggerMs = 0;
    }
    // ── Safe grip wait: setelah 1 detik, mulai kalibrasi ─────────────────
    if (calibSafeGripMs > 0 && millis() - calibSafeGripMs > 1000) {
      calibSafeGripMs = 0;
      stopWithBrake();
      Imu().startGyroCalibration();
      Serial.println("[Calib] Memulai Kalibrasi IMU - robot harus DIAM!");
    }
    // Semua kontrol dikunci selama dalam mode kalibrasi
    return;
  } else {
    // ── Normal: detect enter combo ────────────────────────────────────────
    if (calibCombo) {
      if (imuCalibTriggerMs == 0) {
        // Frame pertama combo terdeteksi: mulai timer, TIDAK ada aksi fisik
        imuCalibTriggerMs  = millis();
        imuCalibLastBeepMs = 0;
        Serial.println("[Calib] L1+R1+Share+Options terdeteksi. Tahan 3 detik...");
      } else {
        uint32_t held = millis() - imuCalibTriggerMs;
        // Countdown tiap detik
        if (held > 1000 && imuCalibLastBeepMs < 1000) {
          imuCalibLastBeepMs = 1000;
          Serial.println("[Calib] 2 detik lagi...");
        }
        if (held > 2000 && imuCalibLastBeepMs < 2000) {
          imuCalibLastBeepMs = 2000;
          Serial.println("[Calib] 1 detik lagi...");
        }
        if (held > 3000) {
          // Masuk calib lock setelah TEPAT 3 detik
          calibLockActive    = true;
          imuCalibTriggerMs  = 0;
          imuCalibLastBeepMs = 0;
          gripperFront.setClaw(true);   // safe: tutup semua gripper
          gripperRear.setClaw(true);
          calibSafeGripMs = millis();   // tunggu 1 detik lalu mulai kalibrasi
          startGripperWiggle(2);        // 2 wiggle = entering calib mode
          Serial.println("[Calib] Masuk mode kalibrasi! Safe grip... mulai dalam 1 detik.");
        }
      }
    } else {
      // Combo dilepas sebelum selesai: reset timer (tidak ada aksi)
      imuCalibTriggerMs = 0;
    }
  }

  // Slow rotate modifier: lifter trigger + bumper = fine rotation
  // Set inside gripper block, dipakai di movement block.
  bool slowRotateActive = false;
  float slowRotateTurn  = 0.0f;

  // ─── Claw Calibration Mode via Gamepad ────────────────────────────────────
  // Toggle: tahan Triangle+Share 3 detik
  // L1 tahan = belakang open +1°  |  L2 tahan = belakang open -1°
  // R1 tahan = depan    open +1°  |  R2 tahan = depan    open -1°
  // Share (lagi) = save & keluar
  {
    static bool     clawCalibMode = false;
    static uint32_t debMs         = 0;
    static uint32_t tickMs        = 0;
    static uint32_t printMs       = 0;

    const bool hasCtl  = ctl && ctl->isConnected();
    // Triangle (y) + Share (miscSelect) — aman, tidak konflik dengan gerak robot
    const bool calibBtn = hasCtl && ctl->y() && ctl->miscSelect();

    // Toggle: tahan Triangle+Share selama 3 detik
    static uint32_t holdStartMs = 0;
    static bool     holdActive  = false;
    if (calibBtn && !clawCalibMode) {
      if (!holdActive) { holdActive = true; holdStartMs = millis(); }
      uint32_t held = millis() - holdStartMs;
      if (held >= 1000 && held < 1100) Serial.println("[ClawCalib] Tahan Triangle+Share... 2 detik lagi");
      if (held >= 2000 && held < 2100) Serial.println("[ClawCalib] Tahan Triangle+Share... 1 detik lagi");
      if (held >= 3000 && millis() - debMs > 500) {
        clawCalibMode = true; debMs = millis(); holdActive = false;
        startGripperWiggle(2);
        Serial.println("[ClawCalib] MODE ON!");
        Serial.println("  L1=R-open++ | L2=R-open-- | R1=F-open++ | R2=F-open--");
        Serial.println("  Share=save&keluar | Triangle+Share 3det=keluar");
      }
    } else if (!calibBtn) {
      holdActive = false;
    }

    if (clawCalibMode && hasCtl) {
      const bool l1b  = ctl->l1();
      const bool r1b  = ctl->r1();
      const float tL2 = (float)ctl->brake()    / 1023.0f;
      const float tR2 = (float)ctl->throttle() / 1023.0f;

      // Tick 100ms: tambah/kurang 1 derajat
      if (millis() - tickMs > 100) {
        bool changedR = false, changedF = false;
        if (l1b)        { cfg_claw_belakang_min = constrain(cfg_claw_belakang_min - 1.0f, 0.0f, 175.0f); changedR = true; }
        if (tL2 > 0.3f) { cfg_claw_belakang_min = constrain(cfg_claw_belakang_min + 1.0f, 1.0f, 180.0f); changedR = true; }
        if (r1b)        { cfg_claw_depan_min    = constrain(cfg_claw_depan_min    - 1.0f, 0.0f, 175.0f); changedF = true; }
        if (tR2 > 0.3f) { cfg_claw_depan_min    = constrain(cfg_claw_depan_min    + 1.0f, 1.0f, 180.0f); changedF = true; }
        if (changedR || changedF) {
          updateGripperConfigs();        // ← update dulu baru setClaw!
          tickMs = millis();
          if (changedR) gripperRear.setClaw(false);   // servo gerak ke angle BARU
          if (changedF) gripperFront.setClaw(false);
        }
      }

      // Print tiap 300ms
      if (millis() - printMs > 300) {
        printMs = millis();
        Serial.printf("[ClawCalib] F-open=%.0f R-open=%.0f\n",
                      cfg_claw_depan_min, cfg_claw_belakang_min);
      }

      // Share (tanpa △) = save & keluar — debounce 3det biar wiggle masuk selesai dulu
      if (ctl->miscSelect() && !ctl->y() && millis() - debMs > 3000) {
        saveConfigurations(); clawCalibMode = false; debMs = millis();
        startGripperWiggle(3);
        Serial.printf("[ClawCalib] SAVED! F:%.0f->%.0f R:%.0f->%.0f\n",
                      cfg_claw_depan_min, cfg_claw_depan_max,
                      cfg_claw_belakang_min, cfg_claw_belakang_max);
      }

      // Triangle+Share 3det juga bisa keluar
      if (calibBtn) {
        if (!holdActive) { holdActive = true; holdStartMs = millis(); }
        uint32_t held = millis() - holdStartMs;
        if (held >= 1000 && held < 1100) Serial.println("[ClawCalib] Tahan 2 detik lagi untuk keluar...");
        if (held >= 2000 && held < 2100) Serial.println("[ClawCalib] Tahan 1 detik lagi untuk keluar...");
        if (held >= 3000 && millis() - debMs > 3000) {
          saveConfigurations(); clawCalibMode = false; debMs = millis(); holdActive = false;
          startGripperWiggle(3);
          Serial.printf("[ClawCalib] OFF+SAVED F:%.0f->%.0f R:%.0f->%.0f\n",
                        cfg_claw_depan_min, cfg_claw_depan_max,
                        cfg_claw_belakang_min, cfg_claw_belakang_max);
        }
      }

      stopWithBrake(); return;
    }
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

    // ── Slow rotate modifier: lifter trigger + bumper/trigger ─────────────
    // R2 held (front lifter) : L1 = kiri (CCW), L2 = kanan (CW)
    // L2 held (rear lifter)  : R1 = kanan (CW), R2 = kiri (CCW)  <- sebaliknya/mirror
    constexpr float SLOW_ROT = 35.0f;
    if (triggerR2 > 0.3f && triggerL2 <= 0.3f) {
      // R2 ONLY modifier
      if (l1) { slowRotateActive=true; slowRotateTurn=-SLOW_ROT; }       // kiri
      else if (l2) { slowRotateActive=true; slowRotateTurn=+SLOW_ROT; }  // kanan
    } else if (triggerL2 > 0.3f && triggerR2 <= 0.3f) {
      // L2 ONLY modifier (sebaliknya)
      if (r1) { slowRotateActive=true; slowRotateTurn=+SLOW_ROT; }       // kanan
      else if (r2) { slowRotateActive=true; slowRotateTurn=-SLOW_ROT; }  // kiri
    } else if (triggerR2 > 0.3f && triggerL2 > 0.3f) {
      // Kedua trigger: R2 modifier, L2 side controls rotation
      if (l1) { slowRotateActive=true; slowRotateTurn=-SLOW_ROT; }  // kiri
      else     { slowRotateActive=true; slowRotateTurn=+SLOW_ROT; } // kanan
    }


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

    // R1: toggle claw depan — SKIP jika sedang hold calib combo atau slow rotate
    if (r1 && !lastR1 && !calibCombo && !slowRotateActive) {
      gripperFront.toggleClaw();
      Serial.printf("[Gripper] Depan claw: %s\n",
                    gripperFront.isClawClosed() ? "CLOSE" : "OPEN");
    }
    // L1: toggle claw belakang — SKIP jika sedang hold calib combo atau slow rotate
    if (l1 && !lastL1 && !calibCombo && !slowRotateActive) {
      gripperRear.toggleClaw();
      Serial.printf("[Gripper] Belakang claw: %s\n",
                    gripperRear.isClawClosed() ? "CLOSE" : "OPEN");
    }

    // Target awal berdasarkan trigger stik (UP = 1.0f, DOWN = 0.0f)
    // R2 = lifter depan, L2 = lifter belakang (ditukar dari sebelumnya)
    // Jika slow rotate aktif dan L2 dipakai untuk rotasi, rear lifter tetap naik
    float targetFront = ctl && ctl->isConnected() ? (1.0f - triggerR2) : 1.0f;
    float targetRear  = (ctl && ctl->isConnected() && !slowRotateActive) ? (1.0f - triggerL2) : 1.0f;

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
    const float stickAngleDeg = atan2f(-rx, ry) * RAD_TO_DEG;  // -rx: stick kanan = CW target = robot hadap kanan
    if (prevStickActive) {
      // Delta tracking: ikuti ARAH GERAK stick, bukan absolute angle.
      // Ini yang mencegah robot muter salah arah saat stick sweep melewati ±180°.
      const float delta = wrap180(stickAngleDeg - prevStickAngleDeg);
      yawTargetDeg = wrap180(yawTargetDeg + delta);
    } else {
      // Stick baru aktif: langsung set ke angle absolut stick sebagai starting point
      yawTargetDeg = stickAngleDeg;
    }
    prevStickAngleDeg = stickAngleDeg;
    prevStickActive   = true;
  } else {
    prevStickActive = false;
  }

  float rotate = headingControlMode ? 0.0f : axisToPercent(ctl->axisRX());

  applyDpadOverride(ctl, moveX, moveY);

  if (INVERT_MOVE_X) moveX = -moveX;
  if (INVERT_MOVE_Y) moveY = -moveY;
  if (INVERT_ROTATE) rotate = -rotate;

  const bool hasMove = moveX != 0.0f || moveY != 0.0f;
  const bool hasManualTurn = rotate != 0.0f;

  if (!hasMove && !hasManualTurn && !slowRotateActive) {
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

  // ── NOS Speed Boost ─────────────────────────────────────────────────
  // Normal: kecepatan dibatasi 60% untuk presisi & keamanan.
  // Setelah stick mentok (>88%) selama 2 detik: boost ke 100% (NOS).
  // Reset ke normal saat stick dilepas.
  {
    static uint32_t nosHoldStartMs = 0;
    static bool nosBoostActive     = false;
    constexpr float NOS_BASE_MULT  = 0.60f;   // 60% normal
    constexpr float NOS_THRESHOLD  = 88.0f;   // % dianggap "stick mentok"
    constexpr uint32_t NOS_HOLD_MS = 2000;    // tahan 2 detik untuk boost

    const float stickMag = sqrtf(moveX*moveX + moveY*moveY);
    if (stickMag > NOS_THRESHOLD) {
      if (nosHoldStartMs == 0) nosHoldStartMs = millis();
      if (!nosBoostActive && millis() - nosHoldStartMs > NOS_HOLD_MS) {
        nosBoostActive = true;
        Serial.println("[NOS] Speed boost ON! Full throttle.");
      }
    } else {
      if (nosBoostActive) Serial.println("[NOS] Speed boost OFF.");
      nosHoldStartMs = 0;
      nosBoostActive = false;
    }
    const float nosMult = nosBoostActive ? 1.0f : NOS_BASE_MULT;
    xCommand *= nosMult;
    yCommand *= nosMult;
  }

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
    // Clamp yaw PID output saat driving: batasi agar tidak swing/osilasi
    // IDLE pakai IDLE_YAW_MAX=100% + snap, DRIVING pakai 45% TANPA snap
    // Saat driving motor sudah muter dari movement → tidak perlu snap untuk overcome deadband
    const float rawYawPid = yawPidUpdate(yawTargetDeg, imu.yawDeg, imu.gyroZ);
    // NO SNAP untuk driving: output proporsional, tidak loncat ke 35% (penyebab osilasi!)
    turnCommand = constrain(rawYawPid, -MAX_YAW_CORRECTION_PERCENT, MAX_YAW_CORRECTION_PERCENT)
                  * speedMultiplier;
  }

  // ── Slow rotate override ──────────────────────────────────────────────────
  // Aktif saat modifier combo (R2+L1, R2+L2, L2+R1). Override yaw hold.
  if (slowRotateActive) {
    turnCommand     = slowRotateTurn * speedMultiplier;
    yawTargetDeg    = imu.yawDeg;  // track heading baru terus
    lastManualTurn  = true;
    yawPid.reset();
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
    calibLockActive = true;  // Kunci motor selama kalibrasi (safe mode)
    Serial.println("[Calib] Motor dikunci. Robot harus DIAM selama kalibrasi!");
  } else if (line.endsWith(" enc")) {
    resetEncodersAndSpeedPid();
    Serial.println("Encoder counts and wheel PID reset.");
  } else if (line.endsWith(" all")) {
    resetEncodersAndSpeedPid();
    Imu().startGyroCalibration();
    calibLockActive = true;  // Kunci motor selama kalibrasi
    Serial.println("[Calib] Motor dikunci. Robot harus DIAM selama kalibrasi!");
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
    // Simpan ke preferences agar tetap setelah reboot / ganti ESP32
    cfg_pid_kp = yawPid.kp;
    cfg_pid_ki = yawPid.ki;
    cfg_pid_kd = yawPid.kd;
    saveConfigurations();
    Serial.printf("[PID] Saved: KP=%.3f KI=%.3f KD=%.3f\n", yawPid.kp, yawPid.ki, yawPid.kd);
    printConfig();
  } else if (cmd.name == "save") {
    cfg_pid_kp = yawPid.kp;
    cfg_pid_ki = yawPid.ki;
    cfg_pid_kd = yawPid.kd;
    saveConfigurations();
    Serial.println("[Save] Semua konfigurasi disimpan ke NVS.");
  } else if (cmd.name == "claw") {
    // Usage: claw reset | claw open ANGLE | claw close ANGLE | claw r close ANGLE | claw ANGLE
    if (line.endsWith(" reset")) {
      cfg_claw_depan_min    = AngleClaw_Depan_MIN;
      cfg_claw_depan_max    = AngleClaw_Depan_MAX;
      cfg_claw_belakang_min = AngleClaw_Belakang_MIN;
      cfg_claw_belakang_max = AngleClaw_Belakang_MAX;
      saveConfigurations(); updateGripperConfigs();
      Serial.printf("[Claw] Reset! open=%.1f close=%.1f\n", cfg_claw_depan_min, cfg_claw_depan_max);
    } else if (line.startsWith("claw open ") || line.startsWith("claw o ")) {
      float ang = argOr(cmd, 1, cfg_claw_depan_min);
      cfg_claw_depan_min = cfg_claw_belakang_min = ang;
      saveConfigurations(); updateGripperConfigs();
      Serial.printf("[Claw] Open keduanya -> %.1f deg\n", ang);
    } else if (line.startsWith("claw r open ") || line.startsWith("claw r o ")) {
      // claw r open ANGLE — open belakang saja
      float ang = argOr(cmd, 2, cfg_claw_belakang_min);
      cfg_claw_belakang_min = ang;
      saveConfigurations(); updateGripperConfigs();
      Serial.printf("[Claw] Belakang open -> %.1f deg (range %.1f)\n", ang, cfg_claw_belakang_max - ang);
    } else if (line.startsWith("claw f open ") || line.startsWith("claw f o ")) {
      // claw f open ANGLE — open depan saja
      float ang = argOr(cmd, 2, cfg_claw_depan_min);
      cfg_claw_depan_min = ang;
      saveConfigurations(); updateGripperConfigs();
      Serial.printf("[Claw] Depan open -> %.1f deg (range %.1f)\n", ang, cfg_claw_depan_max - ang);

    } else if (line.startsWith("claw close ") || line.startsWith("claw c ")) {
      // claw close ANGLE — tutup kedua claw (grip lebih kuat)
      float ang = argOr(cmd, 1, cfg_claw_depan_max);
      cfg_claw_depan_max = cfg_claw_belakang_max = ang;
      saveConfigurations(); updateGripperConfigs();
      Serial.printf("[Claw] Close -> %.1f deg (range %.1f)\n", ang, ang - cfg_claw_depan_min);
    } else if (line.startsWith("claw r close ") || line.startsWith("claw r c ")) {
      // claw r close ANGLE — close angle claw belakang (R1) saja
      float ang = argOr(cmd, 2, cfg_claw_belakang_max);
      cfg_claw_belakang_max = ang;
      saveConfigurations(); updateGripperConfigs();
      Serial.printf("[Claw] Belakang close -> %.1f deg (range %.1f)\n", ang, ang - cfg_claw_belakang_min);
    } else if (line.startsWith("claw f close ") || line.startsWith("claw f c ")) {
      // claw f close ANGLE — close angle claw depan (L1) saja
      float ang = argOr(cmd, 2, cfg_claw_depan_max);
      cfg_claw_depan_max = ang;
      saveConfigurations(); updateGripperConfigs();
      Serial.printf("[Claw] Depan close -> %.1f deg (range %.1f)\n", ang, ang - cfg_claw_depan_min);
    } else if (cmd.args[0] > 0.0f) {
      // claw ANGLE — shorthand: set open angle keduanya
      float openAng = argOr(cmd, 0, cfg_claw_depan_min);
      cfg_claw_depan_min = cfg_claw_belakang_min = openAng;
      saveConfigurations(); updateGripperConfigs();
      Serial.printf("[Claw] Open angle -> %.1f deg (range %.1f)\n", openAng, cfg_claw_depan_max - openAng);
    } else {
      Serial.printf("[Claw] Depan   : open=%.1f close=%.1f\n", cfg_claw_depan_min, cfg_claw_depan_max);
      Serial.printf("[Claw] Belakang: open=%.1f close=%.1f\n", cfg_claw_belakang_min, cfg_claw_belakang_max);
      Serial.println("Usage: claw reset | claw ANGLE | claw open A | claw close A | claw r close A | claw f close A");
    }


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
  } else if (cmd.name == "override") {
    // override on/off - bypass safe mode untuk test motor tanpa stick
    if (line.endsWith(" on")) {
      safeOverride = true;
      Serial.println("[Override] SAFE MODE BYPASSED - motor bisa gerak tanpa stick!");
      Serial.println("[Override] Ketik 'override off' untuk kembali ke safe mode.");
    } else if (line.endsWith(" off")) {
      safeOverride = false;
      stopWithBrake();
      Serial.println("[Override] Safe mode AKTIF kembali - motor OFF.");
    } else {
      Serial.printf("[Override] Status: %s\n", safeOverride ? "ON (bypass)" : "OFF (safe mode aktif)");
    }
  } else if (cmd.name == "i2c") {
    // I2C scan - cek PCA9685 dan device lain
    Serial.println("[I2C] Scanning 0x01-0x7F...");
    Wire.begin();
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0) {
        Serial.printf("  [I2C] Device at 0x%02X", addr);
        if (addr == 0x40) Serial.print(" <- PCA9685 MOTOR DRIVER");
        if (addr == 0x41) Serial.print(" <- PCA9685 (alt addr)");
        if (addr == 0x68 || addr == 0x69) Serial.print(" <- MPU6050 IMU");
        Serial.println();
        found++;
      }
    }
    if (found == 0) Serial.println("  [I2C] TIDAK ADA DEVICE! Cek wiring SDA/SCL.");
    else Serial.printf("  [I2C] Total: %d device ditemukan.\n", found);
  } else if (cmd.name == "mode") {
    if (line.endsWith(" config") || line.endsWith(" calib")) {
      Serial.println("[System] Rebooting to Config (WiFi) Mode...");
      setBootMode(1);
      Serial.flush();
      ESP.restart();
    } else if (line.endsWith(" comp") || line.endsWith(" normal")) {
      Serial.println("[System] Rebooting to Competition (Gamepad) Mode...");
      setBootMode(0);
      Serial.flush();
      ESP.restart();
    } else {
      Serial.println("Usage: mode config | mode comp");
    }
  } else if (cmd.name == "restart" || cmd.name == "rst") {
    Serial.println("[System] Restarting ESP32 for BT re-pair...");
    Serial.flush();
    ESP.restart();
  } else if (cmd.name == "heading") {
    // heading <deg> - set target heading dari serial, aktifkan yaw hold
    if (cmd.count > 0) {
      yawTargetDeg = argOr(cmd, 0, yawTargetDeg);
      yawHoldEnabled = true;
      idleYawHoldEnabled = true;
      yawPid.reset();
      Serial.print("[Heading] Target set to ");
      Serial.print(yawTargetDeg, 1);
      Serial.print(" deg (current yaw=");
      Serial.print(Imu().telemetry().yawDeg, 1);
      Serial.println(" deg)");
    } else {
      Serial.print("[Heading] target=");
      Serial.print(yawTargetDeg, 1);
      Serial.print(" yaw=");
      Serial.println(Imu().telemetry().yawDeg, 1);
    }
  } else if (cmd.name == "tare") {
    // tare - set target ke yaw sekarang (sama dengan tekan X button)
    yawTargetDeg = Imu().telemetry().yawDeg;
    yawPid.reset();
    Serial.print("[Tare] Target reset to current yaw=");
    Serial.println(yawTargetDeg, 1);
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
  // Reset ke state seperti baru nyala: matiin yaw lock & calib lock
  // supaya robot bisa diangkat/dipindah tanpa motor melawan
  yawHoldEnabled     = false;
  idleYawHoldEnabled = false;
  calibLockActive    = false;
  yawPid.reset();
  Serial.println("[Disconnect] Yaw lock & calib lock dimatikan. Robot bebas diangkat.");
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

    const bool noStick = (activeController == nullptr || !activeController->isConnected());
    static bool safeModePrinted = false;

    if (noStick && !calibLockActive && !safeOverride) {
      // ── SAFE MODE: no stick, not calibrating, no override ───────────────
      // Motor mati, gripper ke posisi aman. Robot tidak bergerak sama sekali.
      if (!safeModePrinted) {
        Serial.println("[Safe] No controller - motor OFF, gripper SAFE.");
        Serial.println("[Safe] Ketik 'override on' untuk test motor tanpa stick.");
        safeModePrinted = true;
      }
      if (!lastBrakeState) stopWithBrake();
      gripperFront.setClaw(true);
      gripperRear.setClaw(true);
      yawPid.reset();
    } else if (noStick && safeOverride) {
      // ── OVERRIDE MODE: test tanpa stick ───────────────────────────────────
      // Jangan panggil processGamepad(nullptr) karena akan trigger stopWithBrake di line 965-966!
      // Motor dikendalikan via serial command (move/raw/rpm), biarkan state bertahan.
      safeModePrinted = false;
    } else {
      // ── STICK CONNECTED: full gamepad control ─────────────────────────────
      safeModePrinted = false;
      processGamepad(activeController);
    }

    UpdateWheelSpeedController();
  }

  // Detect kalibrasi selesai: unlock control + wiggle 3x
  static bool wasCalibrating = false;
  const bool nowCalibrating = Imu().isCalibrating();
  if (wasCalibrating && !nowCalibrating) {
    calibLockActive = false;
    Serial.println("[Calib] Kalibrasi IMU SELESAI! Gyro & Accel terkalibrasi.");
    startGripperWiggle(3);  // 3 wiggles = kalibrasi selesai
    yawTargetDeg = Imu().telemetry().yawDeg;
    yawPid.reset();
  }
  wasCalibrating = nowCalibrating;
  updateGripperWiggle();  // update wiggle state machine setiap loop

  const uint32_t nowMs = millis();
  if (telemetryEnabled && nowMs - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    printTelemetry();
  }
}
