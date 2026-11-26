# Mechatronics-4 機電整合自走清潔機器人系統

本專案為結合 **Raspberry Pi 4B**（上位機策略決策、OpenCV 影像辨識與 Web 控制台）與 **Arduino UNO R3**（底層馬達驅動、IMU 姿態解算、超音波測距與硬體安全保護）的自走清潔機器人平台。系統同時支援**手動遠端遙控**、**Python API 控制**、**多策略自動巡航與單步除錯**，以及**無硬體 `/sim` 模擬器測試**。

> 系統硬體接線與資料流方塊圖請參閱：[BLOCK_DIAGRAM.md](BLOCK_DIAGRAM.md)；版本變更紀錄請參閱：[CHANGELOG.zh.md](CHANGELOG.zh.md)。

---

## 一、 車輛核心功能

### 1. 底層運動與感測能力（Arduino UNO）
* **雙輪差速驅動 (`motor.h`)**：支援雙馬達混控（轉向 `turn` + 速度 `speed`）與單馬達獨立測試，內建 1000ms 通訊逾時失聯安全煞停（Watchdog）。
* **MPU-6050 IMU 閉迴路控制 (`imu.h`)**：
  * 陀螺儀滿量程設為 $\pm 1000^\circ/\text{s}$，避免快速原地旋轉時飽和失真（座標定義：右轉／順時針為 `+Yaw`，左轉／逆時針為 `-Yaw`）。
  * 支援指定絕對目標航向角 PD 閉迴路導航（`CMD 0x06`）、原地旋轉轉速上限調節（`CMD 0x0B`，預設 90 PWM）與動態正弦擺頭清掃（`CMD 0x07`）。
* **前／右雙超音波測距與硬體防撞 (`Ultrasonic.h`)**：
  * 採用 5 幀中位數濾波與短暫掉幀零階保持，穩定回傳前方與右方距離（mm）。
  * 支援動態設定／查詢前方硬體極限防撞門檻（`CMD 0x09` / `0x0A`，預設 150mm），低於門檻前進時立即強制緊急煞停（`E-Stop`）。
* **吸塵繼電器控制**：可透過軟體與策略即時啟閉吸塵馬達（`CMD 0x03`）。

### 2. 手動遙控與除錯監控（Web 平台）
* **自主除錯主頁 (`/`)**：即時影像視野、跟隨車體同步旋轉的 2D 戰術雷達圖（含前方防撞安全警戒圈）、極座標航向向量搖桿、擺頭與防撞參數設定、底層通訊日誌終端。
* **傳統手動遙控 (`/legacy`)**：保留傳統雙虛擬搖桿（無人機 Mode 2 手感）、方向按鍵與 `WASD` 鍵盤遙控介面。
* **獨立模擬注入頁 (`/sim`)**：支援在 Mock Mode 下手動餵入前方／右方超音波距離、IMU 航向角與虛擬／上傳攝影機畫面，讓自動策略不需修改即可完成測試。

### 3. 自動任務策略與影像紙屑辨識
* **多策略動態切換與單步除錯 (`strategies/`)**：支援網頁下拉切換多個策略、指定任意起始步驟、即時緊急停止，以及 **🐛 單步除錯模式**（每跑完一步自動暫停等待點擊下一步）。
* **紙屑影像辨識 (`frame_recognize/`)**：輕量化 OpenCV HSV 多色域分割與幾何特徵過濾，輸出歸一化 `(has_confetti, bbox, center)` 百分比座標，引導車輛對準並啟動擺頭吸取。

---

## 二、 專案目錄與主要檔案說明

| 目錄 / 檔案 | 主要功能與說明文件 |
| :--- | :--- |
| [arduino_code/main/](arduino_code/main/) | Arduino UNO R3 韌體專案（PlatformIO），架構、腳位與演算法詳見 [arduino_code/README.zh.md](arduino_code/README.zh.md)，原始碼位於 [src/](arduino_code/main/src/)：<br>• [main.cpp](arduino_code/main/src/main.cpp)：主迴圈排程、指令註冊與硬體 E-Stop 保護<br>• [connection.h](arduino_code/main/src/connection.h)：`0xAA ... 0x55` 雙向二進位通訊協議解析<br>• [imu.h](arduino_code/main/src/imu.h)：MPU-6050 姿態積分、航向鎖定 PD 與擺頭控制<br>• [Ultrasonic.h](arduino_code/main/src/Ultrasonic.h)：雙超音波非阻塞量測與中位數濾波<br>• [motor.h](arduino_code/main/src/motor.h)：雙馬達差速混控與 PWM 驅動 |
| [control/](control/) | 上位機 Python 控制核心與 FastAPI Web 控制台（原理與 API 詳見 [control/README.zh.md](control/README.zh.md)）：<br>• [robot_controller.py](control/robot_controller.py)：獨立車輛控制 API（`RobotController`），含串口管理、5Hz 心跳與 Mock Mode 模擬注入<br>• [server.py](control/server.py)：FastAPI + WebSocket 後端伺服器，掛載車輛控制與策略引擎<br>• [test_connection.py](control/test_connection.py)：CLI 互動式指令測試與單行即時儀表板<br>• [templates/index.html](control/templates/index.html)：自主導航除錯與多策略控制台主頁 (`/`)<br>• [templates/sim.html](control/templates/sim.html)：獨立模擬器感測器與畫面注入頁 (`/sim`)<br>• [templates/legacy.html](control/templates/legacy.html)：傳統手動遙控介面 (`/legacy`)<br>• [run_web.bash](control/run_web.bash)：Web 單獨啟動腳本（使用根目錄共用 `venv/`） |
| [strategies/](strategies/) | 自動任務策略引擎與場地策略實作（架構與新建教學詳見 [strategies/README.zh.md](strategies/README.zh.md)）：<br>• [arena_strategy.md](strategies/arena_strategy.md)：競賽場地 16 步驟清掃策略規格書<br>• [arena_strategy.py](strategies/arena_strategy.py)：16 步驟主策略實作<br>• [runner.py](strategies/runner.py)：背景執行緒策略引擎（支援跳步、急停、單步除錯）<br>• [registry.py](strategies/registry.py)：自動掃描與動態註冊多組策略<br>• [techniques/](strategies/techniques/)：可重用組合技函式庫（直行、沿右牆、視覺對準、擺頭吸取，詳見 [techniques/README.zh.md](strategies/techniques/README.zh.md)） |
| [frame_recognize/](frame_recognize/) | OpenCV 紙屑影像偵測模組（說明詳見 [frame_recognize/README.zh.md](frame_recognize/README.zh.md)）：<br>• [confetti_detector.py](frame_recognize/confetti_detector.py)：核心偵測類別 `ConfettiDetector`<br>• [detect_confetti_video.py](frame_recognize/detect_confetti_video.py)：影片與攝影機測試工具 |
| [circuit/](circuit/) | 硬體電路接線設計檔（[base.fzz](circuit/base.fzz)、[base_bb.png](circuit/base_bb.png)、[base_bb.pdf](circuit/base_bb.pdf)） |
| [setup/](setup/) | 共用 Python 環境初始化、USB 攝影機固定名稱與 MediaMTX RTSP 設定教學（[setup/README.zh.md](setup/README.zh.md)） |
| [records/](records/) | `start.sh` 執行時 `ffmpeg` 同步錄影之 `mp4` 儲存目錄 |
| [requirements.txt](requirements.txt) | 全專案共用的 Python 套件相依清單（安裝於根目錄 `venv/`） |
| [start.sh](start.sh) / [stop.sh](stop.sh) | 系統一鍵啟動與安全停止腳本 |

---

## 三、 快速開啟與關閉方式

### 1. 一鍵啟動完整系統（攝影機串流 + 錄影 + Web 控制台）
在專案根目錄執行 [start.sh](start.sh)：
```bash
./start.sh
```
該腳本會依序於背景啟動：
1. **MediaMTX**：串流轉發伺服器（提供 RTSP `rtsp://localhost:8554/live/stream` 與網頁 WebRTC port `8889`）。
2. **FFmpeg**：擷取 `/dev/logi_camera`（640×360 @ 20fps），同時推流至 MediaMTX 並錄製存檔至 [records/output.mp4](records/)。
3. **Web 控制台**：透過根目錄共用的 `venv/` 啟動 Uvicorn（預設聆聽 `http://0.0.0.0:8000`）。

啟動後可於瀏覽器開啟：
* **主控制台**：`http://<RaspberryPi_IP>:8000/`
* **模擬資料注入台**：`http://<RaspberryPi_IP>:8000/sim`
* **傳統遙控介面**：`http://<RaspberryPi_IP>:8000/legacy`

### 2. 一鍵安全停止系統
執行 [stop.sh](stop.sh)：
```bash
./stop.sh
```
> 腳本會優先對 `ffmpeg` 發送 `SIGINT` 確保 `output.mp4` 影片檔案正常封裝寫入不毀損，隨後關閉 `uvicorn` 與 `mediamtx`（若仍有殘留行程則補發強制終止）。

### 3. 僅啟動 Web 控制台（離線模擬或免攝影機開發）
若不需開啟攝影機推流（例如僅測試馬達或使用 `/sim` 模擬模式），可直接執行 [run_web.bash](control/run_web.bash)：
```bash
cd control
./run_web.bash
```
