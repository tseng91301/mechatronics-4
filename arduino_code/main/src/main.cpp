#include "Ultrasonic.h"
#include "connection.h"
#include "imu.h"
#include "motor.h"
#include <Arduino.h>
#include <Wire.h>

// 超音波感測器腳位設定 (含 5-Frame 中位數濾波與掉幀保持)
Ultrasonic front_ult(5, 6);
Ultrasonic right_ult(4, 7);

SingleMotor leftMotor(3, 9);
SingleMotor rightMotor(10, 11);

// 雙馬達整合驅動
DualMotor dualMotor(leftMotor, rightMotor);

// 序列埠通訊解析器
SimpleConnection connection(&Serial);

// MPU-6050 IMU 控制器 (支援任意目標角度導航、正弦動態擺頭與直線自動校正)
IMUController imu;

const uint8_t RELAY_PIN = 2;

// 馬達指令時間戳記與停止狀態標誌 (通訊超時安全保護)
unsigned long lastMotorCmdTime = 0;
bool motorStopped = true;

// 前方超音波極限防撞門檻 (單位: 毫米 mm，預設 150mm 即 15cm；設為 0 代表關閉防撞保護)
uint16_t safetyThreshold_mm = 150;

// --- 通訊指令回呼函式 (Callbacks) ---

// 1. 測試連線 Ping (亦可作為通訊守門心跳更新)
void onPing(const uint8_t *payload, uint8_t len) {
  lastMotorCmdTime = millis();
  connection.sendPacket(CMD_PING);
  connection.sendLog("Arduino Pong!");
}

// 2. 雙馬達手動控制 (Payload 長度 4 bytes: turn int16, speed int16)
void onDualMotor(const uint8_t *payload, uint8_t len) {
  int16_t turn = SimpleConnection::readInt16(payload, 0);
  int16_t speed = SimpleConnection::readInt16(payload, 2);

  lastMotorCmdTime = millis();

  // 輸入手動控制指令 (若啟用自動校正且直走，會鎖定航向微調)
  imu.setDriveCommand(turn, speed);

  int16_t actTurn, actSpeed;
  imu.getActualDrive(actTurn, actSpeed);

  if (actTurn == 0 && actSpeed == 0) {
    dualMotor.stop();
    motorStopped = true;
  } else {
    dualMotor.drive(actTurn, actSpeed);
    motorStopped = false;
  }
}

// 3. 單馬達獨立控制 (Payload 長度 3 bytes: motor_id uint8, speed int16)
void onSingleMotor(const uint8_t *payload, uint8_t len) {
  uint8_t motorId = SimpleConnection::readUInt8(payload, 0);
  int16_t speed = SimpleConnection::readInt16(payload, 1);

  lastMotorCmdTime = millis();
  motorStopped = (speed == 0);

  // 進行單馬達獨立控制時，暫停自動模式以免干擾
  imu.stopMotion();

  if (motorId == 0) {
    leftMotor.setSpeed(speed);
  } else if (motorId == 1) {
    rightMotor.setSpeed(speed);
  }
}

// 4. 繼電器控制 (Payload 長度 2 bytes: relay_id uint8, state uint8)
void onRelay(const uint8_t *payload, uint8_t len) {
  uint8_t relayId = SimpleConnection::readUInt8(payload, 0);
  uint8_t state = SimpleConnection::readUInt8(payload, 1);

  if (relayId == 1 || relayId == 13) {
    // 繼電器模組通常為低電位觸發 (LOW: 導通 ON, HIGH: 斷開 OFF)
    digitalWrite(RELAY_PIN, state ? LOW : HIGH);
  }
}

// 5. 自動校正開關控制 (Payload 長度 1 byte: 1: 開啟, 0: 關閉)
void onSetAutoCorrect(const uint8_t *payload, uint8_t len) {
  uint8_t enable = SimpleConnection::readUInt8(payload, 0);
  imu.setAutoCorrect(enable != 0);
  if (enable != 0) {
    connection.sendLog("IMU Auto-Correct: ON");
  } else {
    connection.sendLog("IMU Auto-Correct: OFF");
  }
}

// 6. 朝向歸零 / 重設 IMU (Payload 長度 0 bytes)
void onResetIMU(const uint8_t *payload, uint8_t len) {
  imu.resetYaw();
  connection.sendLog("IMU Yaw Reset to 0 deg");
}

// 7. 指定目標航向導航 (Payload 長度 4 bytes: target_yaw_x10 int16, speed int16)
void onSetHeading(const uint8_t *payload, uint8_t len) {
  int16_t yaw_x10 = SimpleConnection::readInt16(payload, 0);
  int16_t speed = SimpleConnection::readInt16(payload, 2);

  lastMotorCmdTime = millis();
  float targetYaw = (float)yaw_x10 / 10.0f;
  imu.setTargetHeading(targetYaw, speed);
  motorStopped = false;
}

// 8. 擺頭開關與參數設定 (Payload 長度 5 bytes: enable uint8, amp_x10 int16, period_ms uint16)
void onSetSweep(const uint8_t *payload, uint8_t len) {
  uint8_t enable = SimpleConnection::readUInt8(payload, 0);
  int16_t amp_x10 = SimpleConnection::readInt16(payload, 1);
  uint16_t period = SimpleConnection::readUInt16(payload, 3);

  lastMotorCmdTime = millis();
  imu.setSweep(enable != 0, (float)amp_x10 / 10.0f, period);

  if (enable != 0) {
    connection.sendLog("Sweep Mode: ON");
  } else {
    connection.sendLog("Sweep Mode: OFF");
  }
}

// 9. 煞停並停止任何航向/擺頭運動 (Payload 長度 0 bytes)
void onStopMotion(const uint8_t *payload, uint8_t len) {
  lastMotorCmdTime = millis();
  imu.stopMotion();
  dualMotor.stop();
  motorStopped = true;
  connection.sendLog("Motion Stopped");
}

// 10. 設定防撞距離門檻 (Payload 長度 2 bytes: threshold_mm uint16)
void onSetSafetyThreshold(const uint8_t *payload, uint8_t len) {
  safetyThreshold_mm = SimpleConnection::readUInt16(payload, 0);

  // 回傳最新門檻數值封包確認
  uint8_t buf[2];
  buf[0] = (uint8_t)((safetyThreshold_mm >> 8) & 0xFF);
  buf[1] = (uint8_t)(safetyThreshold_mm & 0xFF);
  connection.sendPacket(CMD_SAFETY_THRESH_DATA, buf, 2);

  char logBuf[40];
  if (safetyThreshold_mm > 0) {
    snprintf(logBuf, sizeof(logBuf), "Safety Thresh Set: %u mm", safetyThreshold_mm);
  } else {
    snprintf(logBuf, sizeof(logBuf), "Safety E-Stop: DISABLED");
  }
  connection.sendLog(logBuf);
}

// 11. 查詢當前防撞距離門檻 (Payload 長度 0 bytes)
void onGetSafetyThreshold(const uint8_t *payload, uint8_t len) {
  uint8_t buf[2];
  buf[0] = (uint8_t)((safetyThreshold_mm >> 8) & 0xFF);
  buf[1] = (uint8_t)(safetyThreshold_mm & 0xFF);
  connection.sendPacket(CMD_SAFETY_THRESH_DATA, buf, 2);
}

// 12. 設定旋轉對齊轉速上限 (Payload 長度 2 bytes: turn_speed int16, 40~200)
void onSetTurnSpeed(const uint8_t *payload, uint8_t len) {
  int16_t turnSpeed = SimpleConnection::readInt16(payload, 0);
  imu.setMaxTurnSpeed(turnSpeed);
  char logBuf[36];
  snprintf(logBuf, sizeof(logBuf), "Turn Speed Limit: %d", imu.getMaxTurnSpeed());
  connection.sendLog(logBuf);
}

unsigned long lastReportTime = 0;

void setup() {
  Serial.begin(9600);

  // 初始化 I2C 與 MPU-6050
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  bool imuOk = imu.begin();

  // 初始化超音波與腳位
  front_ult.begin();
  right_ult.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // 預設關閉 (高電位斷開)

  // 初始化馬達
  dualMotor.begin();

  // 註冊通訊指令及其對應的數值長度與處理函式
  connection.registerCommand(CMD_PING, 0, onPing);
  connection.registerCommand(CMD_DUAL_MOTOR, 4, onDualMotor);
  connection.registerCommand(CMD_SINGLE_MOTOR, 3, onSingleMotor);
  connection.registerCommand(CMD_RELAY, 2, onRelay);
  connection.registerCommand(CMD_SET_AUTO_CORRECT, 1, onSetAutoCorrect);
  connection.registerCommand(CMD_RESET_IMU_YAW, 0, onResetIMU);
  connection.registerCommand(CMD_SET_HEADING, 4, onSetHeading);
  connection.registerCommand(CMD_SET_SWEEP, 5, onSetSweep);
  connection.registerCommand(CMD_STOP_MOTION, 0, onStopMotion);
  connection.registerCommand(CMD_SET_SAFETY_THRESH, 2, onSetSafetyThreshold);
  connection.registerCommand(CMD_GET_SAFETY_THRESH, 0, onGetSafetyThreshold);
  connection.registerCommand(CMD_SET_TURN_SPEED, 2, onSetTurnSpeed);

  if (imuOk) {
    connection.sendLog("MPU-6050 Initialized OK");
  } else {
    connection.sendLog("Warning: MPU-6050 not detected on I2C");
  }
}

void loop() {
  // 處理電腦傳入之指令封包 (非阻塞)
  connection.service();

  // IMU 姿態高頻更新與運動閉迴路運算
  imu.update();

  // 若處於自主導航、或開啟動態擺頭、或手動直線鎖定模式，依據閉迴路更新馬達輸出
  ControlMode currentMode = imu.getMode();
  if (currentMode == MODE_HEADING || imu.isSweepEnabled() ||
      (currentMode == MODE_MANUAL && imu.isAutoCorrectEnabled() && imu.isStraightDriving())) {
    int16_t actTurn, actSpeed;
    imu.getActualDrive(actTurn, actSpeed);

    // 硬體極限防撞保護：若啟用防撞(>0) 且前方超音波 < 門檻且正向前推進，強制緊急煞停
    float frontDistMm = front_ult.filteredDistance * 10.0f;
    if (safetyThreshold_mm > 0 && front_ult.filteredDistance > 0 &&
        frontDistMm < (float)safetyThreshold_mm && actSpeed > 0) {
      imu.stopMotion();
      dualMotor.stop();
      motorStopped = true;

      char warnBuf[48];
      snprintf(warnBuf, sizeof(warnBuf), "[WARN] Front obstacle < %umm! E-Stop", safetyThreshold_mm);
      connection.sendLog(warnBuf);
    } else {
      if (actTurn == 0 && actSpeed == 0) {
        dualMotor.stop();
        if (currentMode == MODE_HEADING && !imu.isSweepEnabled()) {
          // 對齊完成且未啟用擺頭時標記停止
          motorStopped = true;
        }
      } else {
        dualMotor.drive(actTurn, actSpeed);
        motorStopped = false;
      }
    }
  }

  // 通訊超時失聯安全保護：若超過 1000ms 未收到任何新指令且馬達仍在運轉，強制煞停
  unsigned long now = millis();
  if (!motorStopped && (now - lastMotorCmdTime >= 1000)) {
    imu.stopMotion();
    dualMotor.stop();
    motorStopped = true;
  }

  // 超音波服務排程 (含濾波器更新)
  front_ult.service();
  right_ult.service();

  // 定期 (每 100ms) 回傳濾波後超音波與 IMU 姿態數值給電腦
  if (now - lastReportTime >= 100) {
    lastReportTime = now;

    // 轉換為毫米回傳 (float cm * 10 = mm)
    int32_t front_mm =
        (front_ult.filteredDistance >= 0) ? (int32_t)(front_ult.filteredDistance * 10.0f) : -1;
    int32_t right_mm =
        (right_ult.filteredDistance >= 0) ? (int32_t)(right_ult.filteredDistance * 10.0f) : -1;

    // 發送超音波封包: [0xAA] [CMD] [4 bytes int32] [0x55]
    connection.sendUltrasonic(CMD_ULTRASONIC_FRONT, front_mm);
    connection.sendUltrasonic(CMD_ULTRASONIC_RIGHT, right_mm);

    // 發送 IMU 數據: [0xAA] [CMD_IMU_DATA 0x13] [yaw_x10] [pitch_x10] [roll_x10] [mode] [0x55]
    int16_t yaw_x10 = (int16_t)round(imu.getYaw() * 10.0f);
    int16_t pitch_x10 = (int16_t)round(imu.getPitch() * 10.0f);
    int16_t roll_x10 = (int16_t)round(imu.getRoll() * 10.0f);
    // modeState 格式: bit 0: HEADING mode (1=HEADING, 0=MANUAL), bit 1: SWEEP enabled (1=ON, 0=OFF)
    uint8_t modeState = ((uint8_t)imu.getMode() & 0x01) | (imu.isSweepEnabled() ? 0x02 : 0x00);

    connection.sendIMUData(yaw_x10, pitch_x10, roll_x10, modeState);
  }
}
