# Arduino UNO 底層韌體模組 (`arduino_code/`)

本目錄存放運行於 **Arduino UNO R3** 的底層馬達驅動、感測器讀取、IMU 閉迴路控制與序列埠通訊韌體（採用 **PlatformIO** 開發與編譯）。

---

## 一、 專案結構

```text
arduino_code/
├── README.zh.md              # 本文件：Arduino 韌體架構、腳位定義與演算法原理
└── main/
    ├── platformio.ini        # PlatformIO 設定 (atmelavr / board = uno / arduino)
    └── src/
        ├── main.cpp          # 主迴圈排程、指令回呼註冊、硬體 E-Stop 與 Watchdog
        ├── connection.h      # 0xAA ... 0x55 序列埠通訊狀態機解析器與編碼工具
        ├── motor.h           # SingleMotor 單馬達與 DualMotor 差速混控驅動
        ├── imu.h             # MPU-6050 姿態解算、PD 閉迴路航向控制與動態擺頭疊加層
        └── Ultrasonic.h      # 超音波非阻塞排程、5 幀中位數濾波與掉幀保持
```

---

## 二、 硬體腳位配置表 (`main.cpp`)

| 元件 | 模組／物件名稱 | Arduino 腳位 | 備註 |
| :--- | :--- | :--- | :--- |
| **左輪馬達** | `leftMotor` (`SingleMotor`) | `Pin 3`, `Pin 9` | 雙 PWM 腳位控制正反轉 (`-255 ~ +255`) |
| **右輪馬達** | `rightMotor` (`SingleMotor`) | `Pin 10`, `Pin 11` | 雙 PWM 腳位控制正反轉 (`-255 ~ +255`) |
| **前方超音波** | `front_ult` (`Ultrasonic`) | Trig: `Pin 5`, Echo: `Pin 6` | 預設每 `80ms` 量測一次 |
| **右方超音波** | `right_ult` (`Ultrasonic`) | Trig: `Pin 4`, Echo: `Pin 7` | 預設每 `80ms` 量測一次 |
| **吸塵繼電器** | `RELAY_PIN` | `Pin 2` | 低電位觸發 (`LOW` 導通 ON、`HIGH` 斷開 OFF，開機預設 OFF) |
| **MPU-6050 IMU** | `imu` (`IMUController`) | `A4` (SDA), `A5` (SCL) | I2C 介面（地址 `0x68`，啟用 3000μs `Wire` 超時復位） |
| **序列埠通訊** | `connection` (`SimpleConnection`) | `Pin 0` (RX), `Pin 1` (TX) / USB | Baudrate: **`9600 bps`** |

---

## 三、 各模組控制原理與實作邏輯

### 1. 主迴圈排程與安全保護 ([main.cpp](main/src/main.cpp))
`loop()` 採完全非阻塞式設計，執行順序如下：
1. **`connection.service()`**：逐位元組解析上位機傳入的封包並觸發對應 `on*` 回呼函式。
2. **`imu.update()`**：讀取 MPU-6050 並進行高頻閉迴路控制計算。
3. **馬達驅動與雙重硬體安全保護**：
   * **前方超音波極限防撞 (`E-Stop`)**：當 `safetyThreshold_mm > 0`（預設 `150 mm`）、前方濾波距離有效且 `< safetyThreshold_mm`，同時馬達正向前推進 (`actSpeed > 0`) 時，立即呼叫 `imu.stopMotion()` 與 `dualMotor.stop()`，並回傳 Log：`[WARN] Front obstacle < ...mm! E-Stop`。（註：若僅原地旋轉 `actSpeed == 0` 或後退時不會觸發鎖死，方便脫困轉向。）
   * **通訊失聯看門狗 (`Watchdog`)**：只要收到 `onPing`、`onDualMotor`、`onSingleMotor`、`onSetHeading` 或 `onSetSweep` 皆會刷新 `lastMotorCmdTime`。若馬達仍在運轉且超過 **`1000 ms`** 未收到任何新指令，強制停機煞車。
4. **`front_ult.service()` / `right_ult.service()`**：依量測週期輪流觸發超音波測距與濾波計算。
5. **週期性遙測回報（每 `100 ms` / `10 Hz`）**：回傳 `CMD_ULTRASONIC_FRONT (0x11)`、`CMD_ULTRASONIC_RIGHT (0x12)` 與 `CMD_IMU_DATA (0x13)`。

### 2. 雙輪差速混控 ([motor.h](main/src/motor.h))
* `DualMotor::drive(int turn, int speed)` 採用標準差速混控公式：
  $$\text{leftSpeed} = \text{constrain}(\text{speed} - \text{turn},\ -255,\ 255)$$
  $$\text{rightSpeed} = \text{constrain}(\text{speed} + \text{turn},\ -255,\ 255)$$
* **方向極性**：`turn > 0` 時左輪減速／反轉、右輪加速（**逆時針左轉**）；`turn < 0` 時左輪加速、右輪減速／反轉（**順時針右轉**）。

### 3. IMU 姿態解算、PD 閉迴路與擺頭疊加層 ([imu.h](main/src/imu.h))
* **感測器量程配置**：
  * 陀螺儀滿量程設為 **`±1000 deg/s`**（`GYRO_CONFIG 0x1B = 0x10`，換算係數 `32.768 LSB/(deg/s)`），徹底解決雙輪快速原地旋轉時超過預設 `±250 deg/s` 導致 ADC 飽和截斷而航向偏掉的問題。
  * 加速度計設為 **`±2g`**（`16384 LSB/g`），用於計算 `Pitch` 與 `Roll`。
* **開機校準與角度積分**：
  * 開機執行 `calibrateGyro()` 靜止取樣 100 次計算 `_gyroZOffset`。
  * `update()` 中將角速度極性統一定義為**右轉順時針為正 (`+Yaw`)、左轉逆時針為負 (`-Yaw`)**：`rateZ = -(((float)gz / 32.768f) - _gyroZOffset)`，濾除 `< 0.15 deg/s` 死區漂移後進行積分並正規化至 `[-180°, +180°)`。
* **運動控制模式與 PD 控制器**：
  * **`MODE_MANUAL`**：遵循手動 `turn`、`speed`；若開啟 `autoCorrect` 且純直行 (`turn == 0, speed != 0`)，鎖定當下航向並將修正輸出限幅於 `±60` PWM。
  * **`MODE_HEADING`**：依 `targetHeading` 與當前 `_yaw` 的最短角度誤差 `error` 計算 PD 控制律（`_kp = 2.3`, `_kd = 0.12`）：
    $$\text{correction} = -\big(K_p \cdot \text{error} - K_d \cdot \text{gyroZ}\big)$$
    * 當 `targetSpeed == 0` 時原地轉向對齊（誤差 `<= 3.0°` 停止）；當 `targetSpeed != 0` 且偏差 `> 20°`（且未開擺頭）時，會先在原地轉向對齊後再加速前進。
    * 轉向力道受 `_maxTurnSpeed`（預設 **`90` PWM**，可由 `CMD 0x0B` 調整範圍 `40~220`）與克服靜摩擦力最低門檻 `_minTurnSpeed`（`38` PWM）約束。
  * **動態擺頭疊加層 (`setSweep`)**：可疊加於 `MODE_HEADING` 或 `MODE_MANUAL` 上運行，依正弦波 `sweepOffset = amplitude * sin(2π * elapsed / periodMs)` 週期性改變有效目標航向（最小週期限制 `500 ms`），同時保有原本前進／轉向控制能力。

### 4. 超音波中位數濾波與掉幀保持 ([Ultrasonic.h](main/src/Ultrasonic.h))
* 觸發週期預設 **`80 ms`**，`pulseIn` 逾時設為 `30000 μs`（約 `500 cm`）。
* 僅採納 `1.0 cm < distance < 400.0 cm` 的有效物理反射推入 **5 幀環形緩衝區**並排序取中位數 (`filteredDistance`)，消除雜訊突波。
* **掉幀零階保持 (Zero-Order Hold)**：遇到短暫 1~2 幀無回波（例如斜角反射）時保持上一次濾波距離；僅當連續逾時超過 **`1000 ms`**（約 12 幀以上），才將 `filteredDistance` 設為 `-1.0f` 並清空緩衝區。

### 5. 通訊協議與指令註冊 ([connection.h](main/src/connection.h))
* 封包格式：`[0xAA] [CMD] [Payload (0~32 bytes, Big-Endian)] [0x55]`，最多支援註冊 `16` 組指令（目前已註冊 `12` 組：`0x00 ~ 0x0B`）。
* `CMD_IMU_DATA (0x13)` 的最後 1 byte `modeState` 位元定義：`bit 0` 代表是否處於 `MODE_HEADING`，`bit 1` 代表是否啟用 `Sweep` 擺頭。

---

## 四、 編譯與燒錄 (PlatformIO)

```bash
cd arduino_code/main

# 僅編譯檢查
pio run

# 編譯並燒錄至連接的 Arduino UNO
pio run --target upload
```

> 燒錄前請先確認 Raspberry Pi 上的 `server.py` 或 `test_connection.py` 沒有佔用 `/dev/ttyUSB0` / `/dev/ttyACM0`（可先執行根目錄 `./stop.sh` 或在網頁中點選「斷開連線」）。
