# 專案變更日誌 (Changelog)

本檔案記錄專案中針對 **非 Arduino 端**（包括 Python 後端、HTML 樣板、JavaScript 邏輯等）進行的全面架構優化、程式碼去重（DRY 原則）以及冗餘註解清理。

---

## [優化版本] - 2026-11-13

### 🎯 優化目標
1. **去除重複撰寫的程式碼（DRY 原則）**：將分散於多處的重複邏輯提取為共用函式與單一 API Entry，提高可維護性與可讀性。
2. **清除無意義與冗餘註解**：嚴格清查全專案（特別是 `index.html` 結構標籤中多達 60+ 處直觀贅述的註解），保留具備實質架構指導價值的註解。
3. **建立常數單一真相來源（SSOT）**：統一通訊協定定義，避免多處維護造成的不一致。
4. **零破壞性（Zero Regression）**：確保後端序列埠控制、Mock 模擬、心跳保活、自動校正、雷達圖渲染與手機/平板雙搖桿觸控體驗完全維持原功能與穩定性。

---

### 🛠️ 詳細修改內容

#### 1. 後端 Python 服務 (`control/server.py`)
- **底層序列埠傳輸 API 統一 (`_write_raw`)**：
  - 原先 `send_packet` 與 `send_raw_bytes` 各自包含重複的 `self.mock_mode` 判斷、`self.is_connected` 序列埠開啟檢查與 `with self.write_lock:` 執行緒鎖保護程式碼。
  - 提取統一底層函式 `_write_raw(raw, log_desc, quiet, is_ping_sim)`，由 `send_packet` 與 `send_raw_bytes` 共同調用，徹底消除重複代碼。
- **超音波資料推播邏輯整合 (`_notify_ultrasonic`)**：
  - 處理 Arduino 回傳的 `CMD_ULTRASONIC_FRONT`、`CMD_ULTRASONIC_RIGHT`、`CMD_ULTRASONIC`（4 位元組及 5 位元組版）時，原本 4 處重複撰寫了 8 行結構相同的 WebSocket 通知字典。
  - 統一抽象出 `_notify_ultrasonic()` 輔助方法，簡化為單一調用入口。
- **WebSocket 初始連線狀態集中回傳 (`get_initial_messages`)**：
  - 在 `ArduinoBridge` 新增 `get_initial_messages() -> List[dict]` 方法。
  - 將 FastAPI `websocket_endpoint` 函式中原先手動建構並逐行 `send_json` 的 30 餘行字典代碼精簡為單一迴圈：`for init_msg in bridge.get_initial_messages(): await websocket.send_json(init_msg)`。

#### 2. 測試與通訊工具 (`control/test_connection.py`)
- **通訊常數單一真相來源 (SSOT)**：
  - 原先在獨立測試腳本中硬編碼重複定義了 `START_BYTE`、`END_BYTE`、`CMD_*`（0x00 ~ 0x20）以及狀態機常數 `STATE_*`。
  - 改為優先由 `control.server` 模組動態引入，並保留兩層 `try...except ImportError` 的本機回退機制，兼顧常數一致性與獨立免依賴運行的便攜性。

#### 3. 前端頁面與使用者介面 (`control/templates/index.html`)
- **徹底清查並移除 HTML 冗餘註解**：
  - 全數清除 HTML 標籤中超過 60 處無實質說明的標籤註解（例如：`<!-- 頂部 HUD 工具列 -->`、`<!-- 模式 1: HTML5 Canvas 搖桿 -->`、`<!-- 背景參考網格 -->`、`<!-- 🚀 左搖桿 (前進/後退) -->` 等）。
  - 保留必要的 Jinja2 模板結構標籤與乾淨語意化 DOM。
- **桌面雙馬達驅動與狀態更新統一 API (`sendDesktopDrive`, `updateDesktopDriveStats`)**：
  - 原本在「單搖桿拖曳 (`calculateAndEmitSpeed`)」、「回中煞停 (`resetJoystick`, `stopDpad`)」、「方向按鈕盤 (`sendDpadStep`)」、「鍵盤控制 (`applyKeyboardDrive`)」以及「心跳保活 (`setupHeartbeatTimer`)」中，重複撰寫了 5 次混控計算（`speed - turn`, `speed + turn`、範圍限制 `[-255, 255]`）與 4 個 DOM 節點更新（`statTurn`, `statSpeed`, `statLeft`, `statRight`）。
  - 新增 `updateDesktopDriveStats(turn, speed)` 負責數值試算與介面更新。
  - 新增 `sendDesktopDrive(turn, speed, force, isHeartbeat)` 統一處理發送間隔（50ms）、最後時間記時、變數同步與 WebSocket 指令組裝。
- **統一繼電器控制與 UI 聯動入口 (`setRelay`)**：
  - 獨立按鈕 (`relayOnBtn`, `relayOffBtn`) 原本僅送出 WebSocket 封包，未同步 UI 狀態與震動日誌。
  - 封裝統一的 `setRelay(state)`，使手把介面 toggle 與桌面按鈕行為保持高度一致。
- **無人機雙搖桿繪圖邏輯模組化 (`renderStickCanvas`)**：
  - 左垂直軌道搖桿 (`drawLeftStick`) 與右水平軌道搖桿 (`drawRightStick`) 高達 80% 的 Canvas 繪製指令完全相同（外底盤、滑軌凹槽、中心刻度線、徑向漸層握柄、防滑咬花紋理）。
  - 提取通用繪圖函式 `renderStickCanvas(canvas, knobX, knobY, isVertical, theme)`，透過方向旗標與色彩配置主題對象完成渲染，大幅精簡畫布程式碼。
- **雙搖桿位移與死區計算通用函式 (`calcStickNormalizedDelta`)**：
  - 將左右搖桿相對於畫布頂部/左側的坐標軸偏差運算、邊界限制（`[-stickMaxR, stickMaxR]`）與 8% 靈敏度死區過濾抽出為 `calcStickNormalizedDelta`。
- **觸控與指針事件綁定抽象化 (`bindStickCanvasEvents`)**：
  - 將左右兩側搖桿各自重複編寫的 `touchstart`, `touchmove`, `touchend`, `touchcancel`, `pointerdown`, `pointermove`, `pointerup`, `pointercancel` 多點觸控與捕獲處理（合計超過 100 行代碼），抽象為通用工廠函式 `bindStickCanvasEvents(canvas, options)`。
- **心跳保活定時器精簡 (`setupHeartbeatTimer`)**：
  - 消除心跳計時器內部重複組裝 JSON 的 `wsSend` 代碼，直接調用 `sendDroneDrive(false, true)` 與 `sendDesktopDrive(desktopTurn, desktopSpeed, false, true)`。
- **雷達 SVG 距離顏色映射函式 (`getDistanceColor`)**：
  - 將前後與右方超音波距離燈號顏色映射（<15cm 紅色危險、<30cm 黃色注意、其餘正常）提取為共用函式。

---

### 📊 代碼重構前後對比統計
- `control/server.py`：簡化 30 餘行重複代碼，架構更清晰。
- `control/templates/index.html`：整體代碼減少 **250+ 行**，消除所有多餘 HTML 註解，DOM 與 JavaScript 運行更加精準輕量。
- `control/test_connection.py`：通訊常數實現單一來源管理。

