#include "Ultrasonic.h"
#include "connection.h"
#include "motor.h"
#include <Arduino.h>

// 超音波感測器腳位設定
Ultrasonic front_ult(5, 6);
Ultrasonic right_ult(4, 7);

SingleMotor leftMotor(3, 9);
SingleMotor rightMotor(10, 11);

// 雙馬達整合驅動
DualMotor dualMotor(leftMotor, rightMotor);

// 序列埠通訊解析器
SimpleConnection connection(&Serial);

const uint8_t RELAY_PIN = 2;

// --- 通訊指令回呼函式 (Callbacks) ---

// 1. 測試連線 Ping
void onPing(const uint8_t *payload, uint8_t len) {
  // 回傳 Ping 確認封包給電腦
  connection.sendPacket(CMD_PING);
  connection.sendLog("Arduino Pong!");
}

// 2. 雙馬達控制 (Payload 長度 4 bytes: turn int16, speed int16)
void onDualMotor(const uint8_t *payload, uint8_t len) {
  int16_t turn = SimpleConnection::readInt16(payload, 0);
  int16_t speed = SimpleConnection::readInt16(payload, 2);

  dualMotor.drive(turn, speed);
}

// 3. 單馬達控制 (Payload 長度 3 bytes: motor_id uint8, speed int16)
void onSingleMotor(const uint8_t *payload, uint8_t len) {
  uint8_t motorId = SimpleConnection::readUInt8(payload, 0);
  int16_t speed = SimpleConnection::readInt16(payload, 1);

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
    digitalWrite(RELAY_PIN, state ? LOW : HIGH);
  }
}

unsigned long lastReportTime = 0;

void setup() {
  Serial.begin(9600);

  // 初始化超音波與腳位
  front_ult.begin();
  right_ult.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  // 初始化馬達
  dualMotor.begin();

  // 註冊通訊指令及其對應的數值長度與處理函式
  connection.registerCommand(CMD_PING, 0, onPing);
  connection.registerCommand(CMD_DUAL_MOTOR, 4, onDualMotor);
  connection.registerCommand(CMD_SINGLE_MOTOR, 3, onSingleMotor);
  connection.registerCommand(CMD_RELAY, 2, onRelay);
}

void loop() {
  // 處理電腦傳入之指令封包 (非阻塞)
  connection.service();

  // 超音波服務排程
  front_ult.service();
  right_ult.service();

  // 定期 (每 200ms) 回傳超音波數值給電腦 (單位: 毫米 mm, 4 bytes int32)
  unsigned long now = millis();
  if (now - lastReportTime >= 200) {
    lastReportTime = now;

    // 轉換為毫米 (float cm * 10 = mm)
    int32_t front_mm =
        (front_ult.distance >= 0) ? (int32_t)(front_ult.distance * 10.0f) : -1;
    int32_t right_mm =
        (right_ult.distance >= 0) ? (int32_t)(right_ult.distance * 10.0f) : -1;

    // 發送超音波封包: [0xAA] [CMD] [4 bytes int32] [0x55]
    connection.sendUltrasonic(CMD_ULTRASONIC_FRONT, front_mm);
    connection.sendUltrasonic(CMD_ULTRASONIC_RIGHT, right_mm);
  }
}
