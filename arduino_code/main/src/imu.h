#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief 控制模式列舉
 */
enum ControlMode {
  MODE_MANUAL = 0,   // 手動/遙控混控模式 (含直線定向校正，可疊加擺頭)
  MODE_HEADING = 1   // 指定目標航向導航 (先原地轉正對齊，再定向直線行進，可疊加擺頭)
};

/**
 * @brief MPU-6050 慣性測量單元驅動與全向航向/動態疊加擺頭閉迴路控制類別
 */
class IMUController {
public:
  static const uint8_t MPU_ADDR = 0x68;

private:
  bool _detected;
  bool _autoCorrectEnabled;
  ControlMode _mode;

  // 陀螺儀 Z 軸零偏 (deg/s)
  float _gyroZOffset;

  // 姿態角度 (單位: 度 deg)
  float _yaw;
  float _pitch;
  float _roll;
  float _gyroZ; // 當前 Z 軸角速度 (deg/s)

  unsigned long _lastMicros;
  unsigned long _lastSampleTime;

  // 手動直線鎖定參數
  bool _isStraightDriving;
  int16_t _cmdTurn;
  int16_t _cmdSpeed;

  // 目標航向導航參數 (MODE_HEADING)
  float _targetHeading;
  int16_t _targetSpeed;

  // 動態擺頭疊加層參數 (Sweep Overlay: 振幅與週期，可疊加於手動或航向導航上)
  bool _sweepEnabled;
  float _sweepAmplitude;
  unsigned long _sweepPeriodMs;
  unsigned long _sweepStartTime;

  // 實際輸出至馬達之數值
  int16_t _actualTurn;
  int16_t _actualSpeed;

  // 旋轉轉速控制 (避免旋轉過快導致輪胎打滑與陀螺儀飽和漂移)
  int16_t _maxTurnSpeed; // 原地旋轉與對齊轉速上限 (PWM 40~200，預設 90 溫和轉動)
  int16_t _minTurnSpeed; // 克服靜摩擦力之最低啟動轉速 (預設 38)

  // PD 控制參數 (P: 航向偏差拉回, D: 抑制旋轉角速度)
  float _kp;
  float _kd;

public:
  IMUController()
      : _detected(false),
        _autoCorrectEnabled(false),
        _mode(MODE_MANUAL),
        _gyroZOffset(0.0f),
        _yaw(0.0f),
        _pitch(0.0f),
        _roll(0.0f),
        _gyroZ(0.0f),
        _lastMicros(0),
        _lastSampleTime(0),
        _isStraightDriving(false),
        _cmdTurn(0),
        _cmdSpeed(0),
        _targetHeading(0.0f),
        _targetSpeed(0),
        _sweepEnabled(false),
        _sweepAmplitude(20.0f),
        _sweepPeriodMs(2000),
        _sweepStartTime(0),
        _actualTurn(0),
        _actualSpeed(0),
        _maxTurnSpeed(90),
        _minTurnSpeed(38),
        _kp(2.3f),
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

    // 設定陀螺儀滿量程為 ±1000 deg/s (32.768 LSB/deg/s)，防止快速旋轉時讀數飽和截斷導致偏航角失真
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0x10); // ±1000 deg/s
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
        sumZ += (float)gz / 32.768f;
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
   * @brief 週期性更新 IMU 數值與閉迴路控制 (建議在 loop() 中高頻呼叫，約 20~50Hz)
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
      // 轉換角速度 (deg/s): 車身向右旋轉 (順時針 CW) 定義為正角速度 (+Yaw)，向左旋轉 (逆時針 CCW) 定義為負角速度 (-Yaw)
      float rateZ = -(((float)gz / 32.768f) - _gyroZOffset);

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

    // 根據當前模式與擺頭疊加層執行運算
    processMotionControl();
  }

  /**
   * @brief 1. 手動/遙控雙馬達指令輸入 (相容原有 Web 搖桿控制)
   * 若開啟擺頭，馬達速度將完全遵循手動指令，同時自動疊加左右擺動！
   */
  void setDriveCommand(int16_t turn, int16_t speed) {
    _mode = MODE_MANUAL;

    if (_autoCorrectEnabled && !_sweepEnabled) {
      bool signChanged = (_cmdSpeed > 0 && speed < 0) || (_cmdSpeed < 0 && speed > 0);
      if (turn == 0 && speed != 0) {
        if (!_isStraightDriving || signChanged) {
          _targetHeading = _yaw;
          _isStraightDriving = true;
        }
      } else {
        _isStraightDriving = false;
      }
    } else {
      _isStraightDriving = false;
    }

    _cmdTurn = turn;
    _cmdSpeed = speed;
    processMotionControl();
  }

  /**
   * @brief 2. 設定任意目標航向導航 (MODE_HEADING)
   * @param targetYaw 目標基準角度 (-180.0 ~ +180.0 度)
   * @param speed 前進速度 (-255 ~ +255，若 speed==0 則純原地轉向對準目標)
   */
  void setTargetHeading(float targetYaw, int16_t speed) {
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    _targetHeading = targetYaw;
    _targetSpeed = speed;
    _mode = MODE_HEADING;
    _isStraightDriving = false;
    processMotionControl();
  }

  /**
   * @brief 3. 設定或開關動態擺頭模式 (作為運動控制的疊加振盪層)
   * @param enable 是否啟用擺頭 (true: 開啟, false: 關閉)
   * @param amplitude 擺動振幅 (度，例如 20.0 度)
   * @param periodMs 擺動週期 (毫秒，例如 2000 代表 2 秒來回一次)
   */
  void setSweep(bool enable, float amplitude = 20.0f, unsigned long periodMs = 2000) {
    _sweepEnabled = enable;
    if (enable) {
      _sweepAmplitude = amplitude;
      _sweepPeriodMs = (periodMs < 500) ? 500 : periodMs;
      _sweepStartTime = millis();
    }
    processMotionControl();
  }

  bool isSweepEnabled() const {
    return _sweepEnabled;
  }

  float getSweepOffset() const {
    if (!_sweepEnabled || _sweepPeriodMs == 0) return 0.0f;
    unsigned long elapsed = millis() - _sweepStartTime;
    float phase = (float)(elapsed % _sweepPeriodMs) / (float)_sweepPeriodMs * 6.2831853f; // 2 * PI
    return _sweepAmplitude * sin(phase);
  }

  /**
   * @brief 煞車並停止任何航向鎖定與運動 (亦關閉擺頭)
   */
  void stopMotion() {
    _mode = MODE_MANUAL;
    _isStraightDriving = false;
    _sweepEnabled = false;
    _cmdTurn = 0;
    _cmdSpeed = 0;
    _targetSpeed = 0;
    _actualTurn = 0;
    _actualSpeed = 0;
  }

  void cancelStraightDriving() {
    stopMotion();
  }

  /**
   * @brief 取得計算後實際應驅動馬達的 turn 與 speed
   */
  void getActualDrive(int16_t &outTurn, int16_t &outSpeed) const {
    outTurn = _actualTurn;
    outSpeed = _actualSpeed;
  }

  bool isStraightDriving() const {
    return _isStraightDriving;
  }

  ControlMode getMode() const {
    return _mode;
  }

  /**
   * @brief 判斷目標航向是否已對齊 (MODE_HEADING 專用)
   * @param tolerance 容許誤差角度 (度，預設 ±4.0 度)
   */
  bool isHeadingAligned(float tolerance = 4.0f) const {
    if (_mode != MODE_HEADING) return false;
    float err = _targetHeading - _yaw;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return (abs(err) <= tolerance) && (abs(_gyroZ) < 6.0f);
  }

  float getYawError() const {
    float err = _targetHeading - _yaw;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
  }

  void setAutoCorrect(bool enable) {
    _autoCorrectEnabled = enable;
    if (!enable) {
      _isStraightDriving = false;
    }
  }

  bool isAutoCorrectEnabled() const {
    return _autoCorrectEnabled;
  }

  void resetYaw() {
    _yaw = 0.0f;
    _targetHeading = 0.0f;
  }

  float getYaw() const { return _yaw; }
  float getPitch() const { return _pitch; }
  float getRoll() const { return _roll; }
  float getGyroZ() const { return _gyroZ; }
  float getTargetHeading() const { return _targetHeading; }
  bool isDetected() const { return _detected; }

  /**
   * @brief 設定原地旋轉/對齊之轉速上限 (PWM 40~200，預設 90)
   */
  void setMaxTurnSpeed(int16_t speed) {
    _maxTurnSpeed = constrain(speed, 40, 220);
  }

  int16_t getMaxTurnSpeed() const {
    return _maxTurnSpeed;
  }

private:
  /**
   * @brief 多模式運動控制演算法核心 (整合基本運動與擺頭振盪疊加)
   */
  void processMotionControl() {
    if (!_detected) {
      _actualTurn = _cmdTurn;
      _actualSpeed = _cmdSpeed;
      return;
    }

    float sweepOffset = getSweepOffset();

    if (_mode == MODE_MANUAL) {
      if (_sweepEnabled) {
        // 手動模式下啟用了擺頭振盪：
        // 前進/後退速度完全由使用者的 _cmdSpeed 控制！
        // 若使用者操作 turn 轉向，漸進調整基準航向 (_cmdTurn < 0 為右轉，航向應增加)
        if (_cmdTurn != 0) {
          _targetHeading -= (_cmdTurn / 255.0f) * 1.5f;
          while (_targetHeading > 180.0f) _targetHeading -= 360.0f;
          while (_targetHeading < -180.0f) _targetHeading += 360.0f;
        }

        float effectiveTarget = _targetHeading + sweepOffset;
        while (effectiveTarget > 180.0f) effectiveTarget -= 360.0f;
        while (effectiveTarget < -180.0f) effectiveTarget += 360.0f;

        float error = effectiveTarget - _yaw;
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        // 轉向閉迴路校正 (motor.drive 負值 turn 為右轉 CW，正值 turn 為左轉 CCW)
        float correction = -((2.6f * error) - (_kd * _gyroZ));
        _actualTurn = (int16_t)constrain(round(correction), -_maxTurnSpeed, _maxTurnSpeed);
        _actualSpeed = _cmdSpeed; // 速度完全遵循原本的協議！
      } else {
        // 原有手動混控模式 (含直線定向微調)
        if (_autoCorrectEnabled && _isStraightDriving && _cmdSpeed != 0) {
          float error = _targetHeading - _yaw;
          while (error > 180.0f) error -= 360.0f;
          while (error < -180.0f) error += 360.0f;

          float correction = -((_kp * error) - (_kd * _gyroZ));
          int16_t corrTurn = (int16_t)constrain(round(correction), -60, 60);

          _actualTurn = corrTurn;
          _actualSpeed = _cmdSpeed;
        } else {
          _actualTurn = _cmdTurn;
          _actualSpeed = _cmdSpeed;
        }
      }
    }
    else if (_mode == MODE_HEADING) {
      // 指定目標航向導航模式 (基準航向與速度由 setTargetHeading 設定，動態疊加擺頭)
      float effectiveTarget = _targetHeading + sweepOffset;
      while (effectiveTarget > 180.0f) effectiveTarget -= 360.0f;
      while (effectiveTarget < -180.0f) effectiveTarget += 360.0f;

      float error = effectiveTarget - _yaw;
      while (error > 180.0f) error -= 360.0f;
      while (error < -180.0f) error += 360.0f;

      // 當 error > 0 (目標在右側)，需輸出負值 turn 給馬達右轉
      float correction = -((_kp * error) - (_kd * _gyroZ));

      if (_targetSpeed == 0) {
        // 速度為 0：
        if (_sweepEnabled) {
          // 原地左右擺頭
          int16_t turnSign = (correction > 0) ? 1 : -1;
          float turnMag = abs(correction);
          if (turnMag < _minTurnSpeed && abs(error) > 2.0f) turnMag = _minTurnSpeed;
          _actualTurn = (int16_t)constrain(turnSign * turnMag, -_maxTurnSpeed, _maxTurnSpeed);
        } else {
          // 原地旋轉對齊目標角度，對齊後停止
          if (abs(error) > 3.0f) {
            float turnMag = abs(correction);
            if (turnMag < _minTurnSpeed) turnMag = _minTurnSpeed;
            int16_t turnSign = (correction > 0) ? 1 : -1;
            _actualTurn = (int16_t)constrain(turnSign * turnMag, -_maxTurnSpeed, _maxTurnSpeed);
          } else {
            _actualTurn = 0;
          }
        }
        _actualSpeed = 0;
      } else {
        // 目標推進速度 != 0
        if (!_sweepEnabled && abs(error) > 20.0f) {
          // 導航起步若偏差較大，先平穩在原地迅速對齊
          float turnMag = abs(correction);
          if (turnMag < _minTurnSpeed) turnMag = _minTurnSpeed;
          int16_t turnSign = (correction > 0) ? 1 : -1;
          _actualTurn = (int16_t)constrain(turnSign * turnMag, -_maxTurnSpeed, _maxTurnSpeed);
          _actualSpeed = 0;
        } else {
          // 邊走邊校正 / 邊走邊擺頭
          _actualTurn = (int16_t)constrain(round(correction), -_maxTurnSpeed, _maxTurnSpeed);
          _actualSpeed = _targetSpeed;
        }
      }
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
