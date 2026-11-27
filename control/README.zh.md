# 車輛控制核心與 Web 後端服務模組 (`control/`)

> 語言：**繁體中文** | [English](README.md)

本目錄負責機器人上位機的**底層硬體通訊控制 (`RobotController`)**、**FastAPI Web 控制後端 (`server.py`)**、**三大網頁介面 (`/`、`/sim`、`/legacy`)** 與 **CLI 除錯工具 (`test_connection.py`)**。

---

## 一、 程式架構與分層設計

為了讓「自動任務策略」與「網頁除錯介面」能同時運作且不發生序列埠（Serial Port）搶佔衝突，本模組採用**控制與服務分離**的架構：

```text
control/
├── README.zh.md          # 本文件：控制核心原理、架構與 API 參考手冊
├── robot_controller.py   # 【底層控制核心】純 Python 控制器，無 Web 依賴
├── server.py             # 【Web 服務層】FastAPI + Uvicorn + WebSocket + 掛載 StrategyRunner
├── test_connection.py    # 【CLI 測試工具】直接呼叫 RobotController API 的互動式選單工具
├── run_web.bash          # Web 伺服器啟動腳本 (呼叫專案根目錄共用 venv/)
└── templates/
    ├── index.html        # 主頁 (/)：自主除錯儀表板、極座標向量搖桿與多策略控制台
    ├── sim.html          # 模擬資料注入台 (/sim)：餵入超音波、IMU 與模擬鏡頭畫面
    └── legacy.html       # 傳統遙控頁 (/legacy)：雙虛擬搖桿、方向鍵與鍵盤遙控
```

### 共用實體機制 (`attach_controller`)
* 系統執行時僅保留**唯一一個 `RobotController` 實例**管理實體序列埠或 Mock Mode。
* 網頁後端 (`server.py`) 透過 `bridge.add_listener(...)` 訂閱底層感測器事件並轉發至 WebSocket；`StrategyRunner` 也直接操作同一個 `bridge` 實例，因此能**邊執行自動任務、邊在網頁即時監控**。

---

## 二、 控制原理與核心邏輯

### 1. 二進位通訊協議 (`0xAA ... 0x55`)
所有封包皆採用 Big-Endian (`>`) 格式，封包框架為 `[0xAA] [CMD] [Payload...] [0x55]`：

| 方向 | CMD | 名稱 | Payload 長度與格式 | 說明 |
| :--- | :---: | :--- | :--- | :--- |
| **TX** | `0x00` | `CMD_PING` | `0 bytes` | 連線測試 Ping |
| **TX** | `0x01` | `CMD_DUAL_MOTOR` | `4 bytes` (`>hh`: `turn`, `speed`) | 手動雙馬達差速混控 (`-255 ~ +255`) |
| **TX** | `0x02` | `CMD_SINGLE_MOTOR` | `3 bytes` (`>Bh`: `motor_id`, `speed`) | 單馬達測試 (`0`: 左, `1`: 右) |
| **TX** | `0x03` | `CMD_RELAY` | `2 bytes` (`>BB`: `relay_id`, `state`) | 繼電器開關 (`1`: 吸塵馬達) |
| **TX** | `0x04` | `CMD_SET_AUTO_CORRECT` | `1 byte` (`>B`: `enable`) | 手動模式直線自動校正開關 |
| **TX** | `0x05` | `CMD_RESET_IMU_YAW` | `0 bytes` | 重設 IMU 航向角歸零 (`Yaw = 0°`) |
| **TX** | `0x06` | `CMD_SET_HEADING` | `4 bytes` (`>hh`: `yaw_x10`, `speed`) | 指定目標航向角 (`-1800~1800`) 與行進速度 |
| **TX** | `0x07` | `CMD_SET_SWEEP` | `5 bytes` (`>BhH`: `en`, `amp_x10`, `period_ms`) | 開關／設定動態擺頭振幅與週期 |
| **TX** | `0x08` | `CMD_STOP_MOTION` | `0 bytes` | 緊急煞停並解除航向／擺頭鎖定 |
| **TX** | `0x09` | `CMD_SET_SAFETY_THRESH` | `2 bytes` (`>H`: `threshold_mm`) | 設定前方超音波防撞門檻 (`0` 代表關閉) |
| **TX** | `0x0A` | `CMD_GET_SAFETY_THRESH` | `0 bytes` | 查詢當前 Arduino 防撞門檻 |
| **TX** | `0x0B` | `CMD_SET_TURN_SPEED` | `2 bytes` (`>h`: `turn_speed`) | 設定原地旋轉對齊轉速上限 (`40~220` PWM) |
| **RX** | `0x11` | `CMD_ULTRASONIC_FRONT` | `4 bytes` (`>i`: `mm`) | 前方超音波距離（`-1` 代表超出範圍） |
| **RX** | `0x12` | `CMD_ULTRASONIC_RIGHT` | `4 bytes` (`>i`: `mm`) | 右方超音波距離（`-1` 代表超出範圍） |
| **RX** | `0x13` | `CMD_IMU_DATA` | `7 bytes` (`>hhhB`: `y, p, r, mode`) | IMU 姿態角 (`x10`) 與底盤模式位元 |
| **RX** | `0x14` | `CMD_SAFETY_THRESH_DATA` | `2 bytes` (`>H`: `mm`) | 回傳目前設定的防撞門檻 |
| **RX** | `0x20` | `CMD_LOG_MSG` | 可變長度 UTF-8 字串 | Arduino 回報系統訊息或 `E-Stop` 警告 |

### 2. 接收端狀態機解析 (`_rx_worker` / `_process_byte`)
* 背景執行緒持續讀取序列埠位元組並透過四階段有限狀態機解析：
  `STATE_WAIT_START (0xAA)` → `STATE_WAIT_CMD` → `STATE_WAIT_DATA` → `STATE_WAIT_END (0x55)`。
* 依據 `cmd_lengths` 查表決定預期 Payload 長度；遇到 `CMD_LOG_MSG (0x20)` 則採動態長度讀取，直到遇到結尾碼 `0x55` 即提早派發。

### 3. 5Hz 巡航心跳機制 (`_heartbeat_worker`)
* Arduino 端具備 **1000ms 通訊逾時看門狗**：超過 1 秒未收到新馬達指令會自動停機以防失聯暴衝。
* 當呼叫 `send_set_heading(target_yaw, speed)` 且 `speed != 0` 時，`RobotController` 會將目標記錄於 `self.active_heading`，並由背景 `_heartbeat_worker` 每 **200ms (5Hz)** 以靜音模式 (`quiet=True`) 重複補送 `CMD_SET_HEADING` 維持巡航。
* 一旦呼叫 `send_stop_motion()`、`send_dual_motor(0, 0)` 或收到 Arduino 回傳的硬體防撞 `E-Stop` 警告，系統立即清除 `active_heading` 停止心跳。

### 4. 執行緒安全與觀察者事件系統
* **三組鎖保護**：
  * `write_lock`：保護 `ser.write(...)` 序列埠寫入不交錯。
  * `_state_lock`：保護 `latest_data` 感測器快照與 `active_heading` 跨執行緒讀寫。
  * `_listeners_lock`：保護回呼監聽器列表，確保任一監聽器異常不影響底層通訊執行緒。
* **事件廣播 (`notify`)**：解碼完成後主動發出 `ultrasonic`、`imu`、`safety_threshold`、`relay`、`e_stop`、`pong`、`log` 與 `sim_state` 事件。

### 5. 升級版 Mock Mode 與 `/sim` 模擬注入 (`_mock_worker`)
* 啟動 `start_mock()` 後以 **10Hz (`0.1s`)** 更新模擬狀態：
  * **預設保留手動數值**（僅當 `auto_wave=True` 時才產生亂數波形），支援 `/sim` 網頁精準指定 `front_mm`、`right_mm` 與 `yaw`。
  * **運動學輔助推演 (`auto_kinematics=True`)**：收到 `CMD_SET_HEADING` 或 `CMD_DUAL_MOTOR` 時，模擬車體的 `yaw` 會依目前 `turn_speed` 平滑收斂至目標角度。
  * **模擬硬體 `E-Stop`**：當 `safety_threshold_mm > 0` 且 `front_mm < safety_threshold_mm` 且正向前進 (`target_speed > 0`) 時，同步觸發並廣播 `e_stop` 事件。
  * **模擬鏡頭緩衝區 (`inject_camera_frame`)**：提供給 `VisionSource` 在 Mock Mode 下直接讀取 `/sim` 餵入的 BGR 影格。

---

## 三、 API Function Entries 完整參考

### 1. Python 類別 API：`RobotController` ([robot_controller.py](robot_controller.py))

```python
from control.robot_controller import RobotController

robot = RobotController(verbose=False)
```

#### (A) 連線與模擬管理
| 函式名稱 | 參數與回傳值 | 說明 |
| :--- | :--- | :--- |
| `get_ports()` | `-> List[dict]` | 列出可用序列埠（含 `device`、`description`） |
| `connect(port, baudrate=9600, wait_reset_sec=1.8)` | `-> bool` | 連線 Arduino 並啟動 `_rx_worker` 與 `_heartbeat_worker` |
| `disconnect()` | `-> None` | 發送雙馬達歸零煞停並關閉序列埠／模擬執行緒 |
| `reconnect()` | `-> bool` | 重新連線目前序列埠或重新啟動模擬模式 |
| `start_mock()` / `stop_mock()` | `-> None` | 開啟或關閉無硬體模擬模式 |
| `add_listener(callback)` / `remove_listener(callback)` | `callback: Callable[[dict], None]` | 註冊／移除即時狀態事件監聽器 |

#### (B) 執行緒安全感測器與狀態 Getters
| 函式名稱 | 回傳型別 | 說明 |
| :--- | :--- | :--- |
| `get_state()` | `dict` | 完整狀態快照（含連線狀態、超音波、IMU、模式、防撞、E-Stop 等） |
| `get_ultrasonic()` | `dict` | 回傳 `{"front_mm", "front_cm", "right_mm", "right_cm"}` |
| `get_front_mm()` / `get_front_cm()` | `int` / `float` | 取得前方超音波距離（`-1` 代表無效／超距） |
| `get_right_mm()` / `get_right_cm()` | `int` / `float` | 取得右方超音波距離（`-1` 代表無效／無牆） |
| `get_imu()` | `dict` | 回傳 `yaw, pitch, roll, mode, mode_name, sweep_enabled, auto_correct` |
| `get_yaw()` | `float` | 取得目前 IMU 航向角 (`-180.0° ~ +180.0°`) |
| `get_safety_threshold_mm()` | `int` | 取得目前已知的防撞安全距離門檻 (mm) |
| `is_e_stopped()` / `clear_e_stop()` | `bool` / `None` | 查詢或清除硬體防撞緊急煞停旗標 |

#### (C) 車輛控制指令與高階別名
| 底層發送方法 (`send_*`) | 高階別名 (策略常用) | 參數與功能說明 |
| :--- | :--- | :--- |
| `send_set_heading(target_yaw, speed=0, quiet=False)` | `set_heading(target_yaw, speed=0)` | 鎖定目標航向角並以 `speed` 前進／後退（`0` 為純原地旋轉） |
| `send_set_sweep(enable, amp=20.0, period_ms=2000)` | `set_sweep(enable, amp=20.0, period_ms=2000)` | 開關動態擺頭清掃（振幅 `deg`、週期 `ms`） |
| `send_stop_motion()` | `stop()` | 煞停車輛並解除航向鎖定與擺頭，同時停發巡航心跳 |
| `send_relay(relay_id, state)` | `set_vacuum(enable, relay_id=1)` | 控制繼電器（預設 ID `1` 為吸塵馬達） |
| `send_reset_imu()` | `reset_yaw()` | 重設目前 IMU `Yaw` 為 `0.0°` |
| `send_set_turn_speed(turn_speed)` | `set_turn_speed(turn_speed)` | 設定原地對齊旋轉轉速上限 (`40~220` PWM) |
| `send_set_safety_threshold(threshold_mm)` | `set_safety_threshold(threshold_mm)` | 設定前方超音波緊急防撞距離 (`mm`，`0` 關閉) |
| `send_get_safety_threshold()` | — | 主動向 Arduino 查詢目前防撞距離門檻 |
| `send_dual_motor(turn, speed, is_heartbeat=False)` | — | 手動差速雙馬達控制 |
| `send_single_motor(motor_id, speed)` | — | 單馬達獨立測試控制 |
| `send_auto_correct(enable)` | — | 手動模式直線校正開關 |
| `send_ping()` / `send_raw_bytes(raw, log_desc)` | — | 發送 Ping 或自訂原始封包 |

#### (D) 模擬器資料注入 API (Mock Mode 專用)
| 函式名稱 | 說明 |
| :--- | :--- |
| `inject_ultrasonic(front_mm=None, right_mm=None)` | 注入模擬前方／右方超音波距離 (mm) |
| `inject_imu(yaw=None, pitch=None, roll=None)` | 注入模擬 IMU 姿態角 (度) |
| `set_sim_options(auto_kinematics=None, auto_wave=None)` | 切換模擬轉向物理收斂或隨機波動 |
| `inject_camera_frame(frame)` / `get_injected_frame()` | 注入／讀取模擬攝影機 BGR 影格與時間戳 |
| `get_sim_state()` | 取得模擬器完整輸入與馬達目標輸出快照 |

---

### 2. Web 後端 REST API ([server.py](server.py))

#### (A) 網頁頁面與連線管理
* `GET /`：自主導航除錯與多策略控制台主頁（[index.html](templates/index.html)）
* `GET /sim`：獨立模擬器資料注入頁（[sim.html](templates/sim.html)）
* `GET /legacy`：傳統手動遙控與雙搖桿介面（[legacy.html](templates/legacy.html)）
* `GET /api/ports`：列出系統序列埠與目前連線資訊
* `POST /api/connect` (`{"port": "/dev/ttyUSB0" | "MOCK", "baudrate": 9600}`)：連線實體或啟動模擬模式
* `POST /api/disconnect`：停止策略並斷開連線
* `GET /api/status`：取得完整連線與感測器狀態快照

#### (B) 車輛控制端點
* `POST /api/heading` (`{"target_yaw": 90.0, "speed": 120}`)
* `POST /api/sweep` (`{"enable": true, "amplitude": 20.0, "period_ms": 2000}`)
* `POST /api/stop_motion`
* `POST /api/turn_speed` (`{"turn_speed": 90}`)
* `POST /api/safety_threshold` / `GET /api/safety_threshold` (`{"threshold_mm": 150}`)
* `POST /api/relay` (`{"relay_id": 1, "state": true}`)
* `POST /api/reset_imu`
* `POST /api/dual_motor` (`{"turn": 0, "speed": 150}`)
* `POST /api/ping`

#### (C) 多策略控制與除錯端點
* `GET /api/strategy/list`：取得自動探索到的所有策略清單
* `POST /api/strategy/select` (`{"strategy_id": "arena_strategy"}`)：切換目前策略
* `POST /api/strategy/reload`：重新掃描 `strategies/` 目錄載入新策略
* `GET /api/strategy/status`：取得策略執行狀態與當前步驟進度
* `POST /api/strategy/start` (`{"strategy_id", "start_step", "enable_vision", "rtsp_url", "reset_imu", "step_mode"}`)
* `POST /api/strategy/step_next`：單步除錯模式下放行下一步
* `POST /api/strategy/continue`：解除單步暫停並繼續連續執行
* `POST /api/strategy/step_mode` (`{"step_mode": true}`)：執行中動態切換單步除錯模式
* `POST /api/strategy/stop`：立即停止策略並煞停

#### (D) 模擬器資料注入端點 (`/sim`)
* `GET /api/sim/state`：同時取得模擬器狀態與策略狀態
* `POST /api/sim/sensors` (`{"front_mm", "right_mm", "yaw", "pitch", "roll", "auto_kinematics", "auto_wave"}`)
* `POST /api/sim/frame` (`{"image_base64": "..."}`)：注入模擬影格並回傳即時紙屑辨識結果

---

### 3. WebSocket 雙向介面 (`/ws`)
* **前端發送 `action`**：
  * 連線／底層：`connect`, `disconnect`, `reconnect`, `refresh_ports`, `ping`, `send_hex`
  * 馬達／導航：`dual_motor`, `single_motor`, `stop`, `set_heading`, `set_sweep`, `stop_motion`, `set_turn_speed`, `set_safety_threshold`, `get_safety_threshold`, `relay`, `set_auto_correct`, `reset_imu`
  * 策略控制：`strategy_start`, `strategy_select`, `strategy_reload`, `strategy_step_next`, `strategy_continue`, `strategy_set_step_mode`, `strategy_stop`, `get_strategy_status`
  * 模擬注入：`sim_get_state`, `sim_set_sensors`, `sim_set_frame`
* **伺服器推播 `type`**：`status`, `ports`, `ultrasonic`, `imu`, `safety_threshold`, `relay`, `e_stop`, `strategy_status`, `sim_state`, `sim_vision_result`, `log`。

---

## 四、 啟動方式

請先確認已依 [setup/README.zh.md](../setup/README.zh.md) 在專案根目錄建立共用 `venv/` 並安裝 [requirements.txt](../requirements.txt)：

1. **僅啟動 Web 控制台（支援 `/`、`/sim`、`/legacy`）**：
   ```bash
   ./control/run_web.bash
   ```
   或於專案根目錄執行：
   ```bash
   venv/bin/python -m uvicorn control.server:app --host 0.0.0.0 --port 8000 --reload
   ```
2. **啟動完整系統（含攝影機 RTSP 串流、錄影與 Web）**：請在根目錄執行 [../start.sh](../start.sh)。
3. **執行 CLI 互動式測試工具**：
   ```bash
   venv/bin/python control/test_connection.py
   ```
