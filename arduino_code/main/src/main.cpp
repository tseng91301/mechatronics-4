#include "Ultrasonic.h"
#include "connection.h"
#include "imu.h"
#include "motor.h"
#include <Arduino.h>
#include <Wire.h>

// 超音波感測器腳位設定
Ultrasonic front_ult(5, 6);
Ultrasonic right_ult(4, 7);

SingleMotor leftMotor(3, 9);
SingleMotor rightMotor(10, 11);

// 雙馬達整合驅動
DualMotor dualMotor(leftMotor, rightMotor);

// 序列埠通訊解析器
SimpleConnection connection(&Serial);

// MPU-6050 IMU 控制器與直線自動校正
IMUController imu;

const uint8_t RELAY_PIN = 2;

// 馬達指令時間戳記與停止狀態標誌 (通訊超時安全保護)
unsigned long lastMotorCmdTime = 0;
bool motorStopped = true;

// --- 通訊指令回呼函式 (Callbacks) ---

// 1. 測試連線 Ping
void onPing(const uint8_t *payload, uint8_t len) {
  connection.sendPacket(CMD_PING);
  connection.sendLog("Arduino Pong!");
}

// 2. 雙馬達控制 (Payload 長度 4 bytes: turn int16, speed int16)
void onDualMotor(const uint8_t *payload, uint8_t len) {
  int16_t turn = SimpleConnection::readInt16(payload, 0);
  int16_t speed = SimpleConnection::readInt16(payload, 2);

  lastMotorCmdTime = millis();

  // 將指令輸入 IMU 控制器 (若啟用自動校正，直走時會自動鎖定航向進行補償)
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

// 3. 單馬達控制 (Payload 長度 3 bytes: motor_id uint8, speed int16)
void onSingleMotor(const uint8_t *payload, uint8_t len) {
  uint8_t motorId = SimpleConnection::readUInt8(payload, 0);
  int16_t speed = SimpleConnection::readInt16(payload, 1);

  lastMotorCmdTime = millis();
  motorStopped = (speed == 0);

  // 進行單馬達獨立控制時，暫停直線自動校正以免干擾
  imu.cancelStraightDriving();

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
  digitalWrite(RELAY_PIN, HIGH); // 預設關閉

  // 初始化馬達
  dualMotor.begin();

  // 註冊通訊指令及其對應的數值長度與處理函式
  connection.registerCommand(CMD_PING, 0, onPing);
  connection.registerCommand(CMD_DUAL_MOTOR, 4, onDualMotor);
  connection.registerCommand(CMD_SINGLE_MOTOR, 3, onSingleMotor);
  connection.registerCommand(CMD_RELAY, 2, onRelay);
  connection.registerCommand(CMD_SET_AUTO_CORRECT, 1, onSetAutoCorrect);
  connection.registerCommand(CMD_RESET_IMU_YAW, 0, onResetIMU);

  if (imuOk) {
    connection.sendLog("MPU-6050 Initialized OK");
  } else {
    connection.sendLog("Warning: MPU-6050 not detected on I2C");
  }
}

void loop() {
  // 處理電腦傳入之指令封包 (非阻塞)
  connection.service();

  // IMU 姿態高頻更新與直線偏差計算
  imu.update();

  // 若處於自動校正且正在直線運動中，持續依據 IMU 閉迴路更新馬達微調
  if (imu.isAutoCorrectEnabled() && imu.isStraightDriving()) {
    int16_t actTurn, actSpeed;
    imu.getActualDrive(actTurn, actSpeed);
    dualMotor.drive(actTurn, actSpeed);
  }

  // 通訊超時失聯安全保護：若超過 1000ms 未收到任何馬達指令且馬達仍在運轉，強制煞停
  unsigned long now = millis();
  if (!motorStopped && (now - lastMotorCmdTime >= 1000)) {
    imu.cancelStraightDriving();
    dualMotor.stop();
    motorStopped = true;
  }

  // 超音波服務排程
  front_ult.service();
  right_ult.service();

  // 定期 (每 100ms) 回傳超音波與 IMU 姿態數值給電腦
  if (now - lastReportTime >= 100) {
    lastReportTime = now;

    // 轉換為毫米 (float cm * 10 = mm)
    int32_t front_mm =
        (front_ult.distance >= 0) ? (int32_t)(front_ult.distance * 10.0f) : -1;
    int32_t right_mm =
        (right_ult.distance >= 0) ? (int32_t)(right_ult.distance * 10.0f) : -1;

    // 發送超音波封包: [0xAA] [CMD] [4 bytes int32] [0x55]
    connection.sendUltrasonic(CMD_ULTRASONIC_FRONT, front_mm);
    connection.sendUltrasonic(CMD_ULTRASONIC_RIGHT, right_mm);

    // 發送 IMU 數據: [0xAA] [CMD_IMU_DATA 0x13] [yaw_x10] [pitch_x10] [roll_x10] [autoCorrect] [0x55]
    int16_t yaw_x10 = (int16_t)round(imu.getYaw() * 10.0f);
    int16_t pitch_x10 = (int16_t)round(imu.getPitch() * 10.0f);
    int16_t roll_x10 = (int16_t)round(imu.getRoll() * 10.0f);
    uint8_t autoCorrectState = imu.isAutoCorrectEnabled() ? 1 : 0;

    connection.sendIMUData(yaw_x10, pitch_x10, roll_x10, autoCorrectState);
  }
}
