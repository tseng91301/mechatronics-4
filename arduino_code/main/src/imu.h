#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief MPU-6050 慣性測量單元驅動與直線航向自動校正類別
 * 
 * - 透過 I2C (Wire) 讀取 MPU-6050 加速度計與陀螺儀
 * - 陀螺儀 Z 軸積分計算偏航角 (Yaw / Heading)
 * - 走直線時利用 PD 控制器與陀螺儀變化量自動微調左右馬達速度進行方向補償
 */
class IMUController {
public:
  static const uint8_t MPU_ADDR = 0x68;

private:
  bool _detected;
  bool _autoCorrectEnabled;

  // 零偏校正 (Gyro Z offset)
  float _gyroZOffset;

  // 姿態角度 (單位: 度 deg)
  float _yaw;
  float _pitch;
  float _roll;
  float _gyroZ; // 當前 Z 軸角速度 (deg/s)

  unsigned long _lastMicros;
  unsigned long _lastSampleTime;

  // 走直線航向鎖定與校正
  bool _isStraightDriving;
  float _targetHeading;
  int16_t _cmdTurn;
  int16_t _cmdSpeed;

  int16_t _actualTurn;
  int16_t _actualSpeed;

  // PD 控制參數 (P: 航向偏差拉回, D: 抑制旋轉角速度)
  float _kp;
  float _kd;

public:
  IMUController()
      : _detected(false),
        _autoCorrectEnabled(false),
        _gyroZOffset(0.0f),
        _yaw(0.0f),
        _pitch(0.0f),
        _roll(0.0f),
        _gyroZ(0.0f),
        _lastMicros(0),
        _lastSampleTime(0),
        _isStraightDriving(false),
        _targetHeading(0.0f),
        _cmdTurn(0),
        _cmdSpeed(0),
        _actualTurn(0),
        _actualSpeed(0),
        _kp(2.2f),
        _kd(0.12f) {}

  /**
   * @brief 初始化 MPU-6050 並執行陀螺儀零偏校準
   */
  bool begin() {
    Wire.begin();

    // 嘗試喚醒 MPU-6050 (寫入 PWR_MGMT_1 = 0)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x00); // 喚醒
    if (Wire.endTransmission() != 0) {
      _detected = false;
      return false;
    }

    // 設定陀螺儀滿量程為 ±250 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0x00); // ±250 deg/s, 131 LSB/(deg/s)
    Wire.endTransmission();

    // 設定加速度計滿量程為 ±2g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); // ACCEL_CONFIG
    Wire.write(0x00); // ±2g, 16384 LSB/g
    Wire.endTransmission();

    _detected = true;
    calibrateGyro();

    _lastMicros = micros();
    _lastSampleTime = millis();
    return true;
  }

  /**
   * @brief 校準陀螺儀靜止零偏 (取樣 100 次計算平均)
   */
  void calibrateGyro() {
    if (!_detected) return;

    float sumZ = 0.0f;
    const int samples = 100;
    int validSamples = 0;

    for (int i = 0; i < samples; i++) {
      int16_t gz;
      if (readRawGyroZ(gz)) {
        sumZ += (float)gz / 131.0f;
        validSamples++;
      }
      delay(3);
    }

    if (validSamples > 0) {
      _gyroZOffset = sumZ / (float)validSamples;
    }
    _yaw = 0.0f;
    _targetHeading = 0.0f;
  }

  /**
   * @brief 週期性更新 IMU 數值 (建議在 loop() 中高頻呼叫，約 20~50Hz)
   */
  void update() {
    if (!_detected) {
      _actualTurn = _cmdTurn;
      _actualSpeed = _cmdSpeed;
      return;
    }

    unsigned long nowMicros = micros();
    float dt = (nowMicros - _lastMicros) / 1000000.0f;
    _lastMicros = nowMicros;

    // 防止時間溢位或過大步長
    if (dt <= 0.0f || dt > 0.2f) {
      dt = 0.02f;
    }

    // 讀取加速度與陀螺儀
    int16_t ax, ay, az, gx, gy, gz;
    if (readRawAll(ax, ay, az, gx, gy, gz)) {
      // 轉換角速度 (deg/s)
      float rateZ = ((float)gz / 131.0f) - _gyroZOffset;

      // 死區雜訊過濾 (小於 0.15 deg/s 視為靜止無漂移)
      if (abs(rateZ) < 0.15f) {
        rateZ = 0.0f;
      }
      _gyroZ = rateZ;

      // 陀螺儀積分累積偏航角 Yaw
      _yaw += _gyroZ * dt;

      // 限制偏航角在 -180.0 ~ +180.0 度之間
      while (_yaw > 180.0f) _yaw -= 360.0f;
      while (_yaw < -180.0f) _yaw += 360.0f;

      // 由加速度計計算俯仰角 (Pitch) 與滾轉角 (Roll)
      float accX = (float)ax / 16384.0f;
      float accY = (float)ay / 16384.0f;
      float accZ = (float)az / 16384.0f;

      _pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2957795f;
      _roll = atan2(accY, accZ) * 57.2957795f;
    }

    // 執行直線航向閉迴路校正演算法
    processStraightLineCorrection();
  }

  /**
   * @brief 設定從通訊接收到的雙馬達指令
   */
  void setDriveCommand(int16_t turn, int16_t speed) {
    if (_autoCorrectEnabled) {
      // 檢查是否換向 (例如由前進直接切為後退，或後退切為前進)
      bool signChanged = (_cmdSpeed > 0 && speed < 0) || (_cmdSpeed < 0 && speed > 0);

      // 判斷是否為走直線操作 (turn == 0 且 speed != 0)
      if (turn == 0 && speed != 0) {
        if (!_isStraightDriving || signChanged) {
          // 進入走直線狀態 (或剛換向)，鎖定當前姿態為目標直線方向
          _targetHeading = _yaw;
          _isStraightDriving = true;
        }
      } else {
        // 使用者有操作轉向或停止 (speed == 0)，解除直線鎖定
        _isStraightDriving = false;
      }
    } else {
      _isStraightDriving = false;
    }

    _cmdTurn = turn;
    _cmdSpeed = speed;

    // 立即計算最新輸出值，確保在回呼函式中取得最新指令！
    processStraightLineCorrection();
  }

  /**
   * @brief 取消走直線鎖定 (例如單馬達微調或緊急停止時)
   */
  void cancelStraightDriving() {
    _isStraightDriving = false;
    _cmdTurn = 0;
    _cmdSpeed = 0;
    _actualTurn = 0;
    _actualSpeed = 0;
  }

  /**
   * @brief 取得校正後實際應驅動馬達的 turn 與 speed
   */
  void getActualDrive(int16_t &outTurn, int16_t &outSpeed) const {
    outTurn = _actualTurn;
    outSpeed = _actualSpeed;
  }

  bool isStraightDriving() const {
    return _isStraightDriving;
  }

  /**
   * @brief 切換或設定自動校正開關
   */
  void setAutoCorrect(bool enable) {
    _autoCorrectEnabled = enable;
    if (!enable) {
      _isStraightDriving = false;
    } else {
      if (_cmdTurn == 0 && _cmdSpeed != 0) {
        _targetHeading = _yaw;
        _isStraightDriving = true;
      }
    }
    processStraightLineCorrection();
  }

  bool isAutoCorrectEnabled() const {
    return _autoCorrectEnabled;
  }

  /**
   * @brief 重設偏航角 Yaw 為 0 (朝向歸零)
   */
  void resetYaw() {
    _yaw = 0.0f;
    _targetHeading = 0.0f;
  }

  float getYaw() const { return _yaw; }
  float getPitch() const { return _pitch; }
  float getRoll() const { return _roll; }
  float getGyroZ() const { return _gyroZ; }
  bool isDetected() const { return _detected; }

private:
  /**
   * @brief 直線自動校正演算法
   */
  void processStraightLineCorrection() {
    if (_autoCorrectEnabled && _isStraightDriving && _cmdSpeed != 0 && _detected) {
      // 計算目前偏航角與目標直線角度之偏差
      float error = _targetHeading - _yaw;
      while (error > 180.0f) error -= 360.0f;
      while (error < -180.0f) error += 360.0f;

      // 差速底盤航向 PD 回授補償:
      // error > 0 代表車頭朝向偏右，需要向左旋轉補償 (turn > 0)
      // error < 0 代表車頭朝向偏左，需要向右旋轉補償 (turn < 0)
      // 差速底盤的旋轉力矩僅由左右輪速差 (v_R - v_L = 2*turn) 決定，
      // 無論前進 (speed > 0) 還是後退 (speed < 0)，使車體逆時針轉動的 turn 方向恆為正，
      // 故後退時絕對不可將 correction 反號！
      // -_kd * _gyroZ 用以抑制旋轉角速度，防止超調震盪
      float correction = (_kp * error) - (_kd * _gyroZ);

      // 限制微調補償範圍在 -60 ~ +60 內，溫和微調不暴衝
      int16_t corrTurn = (int16_t)constrain(round(correction), -60, 60);

      _actualTurn = corrTurn;
      _actualSpeed = _cmdSpeed;
    } else {
      _actualTurn = _cmdTurn;
      _actualSpeed = _cmdSpeed;
    }
  }

  bool readRawGyroZ(int16_t &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2) != 2) return false;
    gz = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
  }

  bool readRawAll(int16_t &ax, int16_t &ay, int16_t &az,
                  int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;

    // 連續讀取 14 bytes (Accel 6 + Temp 2 + Gyro 6)
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;

    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());

    // 跳過溫度 2 bytes
    Wire.read();
    Wire.read();

    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
  }
};

#endif // IMU_H

