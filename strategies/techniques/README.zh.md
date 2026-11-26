# 策略組合技模組 (`strategies/techniques/`) 詳細說明與開發指南

本目錄存放可供各種自動任務策略重複呼叫的**原子動作與感測器閉迴路組合技（Techniques）**，將底層硬體命令 [RobotController](file:///home/group6/mechatronics-4/control/robot_controller.py) 與影像模組 [confetti_detector.py](file:///home/group6/mechatronics-4/frame_recognize/confetti_detector.py) 封裝為可中斷、執行緒安全且易於組合的函式。

---

## 1. 檔案結構

| 檔案 | 主要匯出項目 | 功能定位 |
| :--- | :--- | :--- |
| [context.py](file:///home/group6/mechatronics-4/strategies/techniques/context.py) | `StrategyContext`, `wrap_angle_deg`, `angle_diff_deg`, `clamp` | 執行上下文、角度正規化、可中斷休眠、硬體防撞暫時關閉／恢復 |
| [navigation.py](file:///home/group6/mechatronics-4/strategies/techniques/navigation.py) | `turn_to_heading`, `drive_blind_imu`, `drive_until_front_distance`, `wall_follow_right` | IMU 轉向、盲走直行、前方測距停靠、沿右牆動態修正巡航 |
| [vision_align.py](file:///home/group6/mechatronics-4/strategies/techniques/vision_align.py) | `VisionSource`, `align_to_confetti` | RTSP / 模擬影格讀取、紙屑視覺對準控制 |
| [vacuum_sweep.py](file:///home/group6/mechatronics-4/strategies/techniques/vacuum_sweep.py) | `sweep_and_vacuum` | 動態擺頭搭配吸塵馬達邊走邊清掃 |
| [__init__.py](file:///home/group6/mechatronics-4/strategies/techniques/__init__.py) | 以上所有符號 | 統一對外匯出入口 |

---

## 2. 各模組功能與控制原理詳解

### 2.1 `StrategyContext`（[context.py](file:///home/group6/mechatronics-4/strategies/techniques/context.py)）

#### 功能一覽
* **`ctx.is_stopped()`**：檢查 `stop_event` 是否已觸發。所有迴圈每個週期必須檢查。
* **`ctx.sleep(seconds, step=0.05)`**：基於 `threading.Event.wait()` 的可中斷休眠。不同於 `time.sleep()` 會卡住執行緒，一旦網頁按下停止，`ctx.sleep()` 會在 50ms 內立即醒來並回傳 `False`。
* **`ctx.set_substate(text)`**：更新網頁策略面板的「子狀態」顯示並寫入系統日誌。
* **`ctx.temporarily_disable_safety(restore_mm=None)`**：上下文管理器（`with ctx.temporarily_disable_safety():`）。

#### 原理：為何需要暫時關閉硬體防撞？
Arduino 韌體預設開啟前方 `150 mm`（15 cm）硬體極限防撞（`E-Stop`）。在「盲走進入窄門／入口」或「貼近吸除紙屑」時，前方若出現門檻框或雜物會誤觸硬體急停導致中斷。透過此上下文管理器在進入時下發 `CMD 0x09 (0)` 暫時解除硬體急停，並在 `finally` 區塊保證無論正常結束或被緊急停止，皆自動恢復原本的安全門檻。

#### 原理：角度規範化 (`wrap_angle_deg`, `angle_diff_deg`)
所有 IMU 目標角與誤差一律透過 `(angle + 180) % 360 - 180` 映射至 `[-180°, +180°)`，確保在跨越 $\pm 180^\circ$ 邊界（例如從 `+175°` 右轉 `10°` 變成 `-175°`）時計算出最短轉向誤差 `+10°`，不會繞遠路旋轉 `350°`。

---

### 2.2 運動與超音波導航（[navigation.py](file:///home/group6/mechatronics-4/strategies/techniques/navigation.py)）

#### (1) `turn_to_heading(ctx, target_yaw, tolerance_deg=4.0, settle_count=3, timeout_sec=6.0) -> bool`
* **功能**：原地旋轉並對齊目標絕對航向角 `target_yaw`。
* **控制原理**：
  1. 呼叫 `ctx.robot.set_heading(target_yaw, 0)`，將行進速度設為 `0`，交由 Arduino 內部 PD 閉迴路進行原地差速轉向。
  2. 上位機每 `50ms` 讀取 `ctx.robot.get_yaw()`，計算 `abs(angle_diff_deg(target_yaw, cur_yaw))`。
  3. **去抖動穩定判準**：須連續 `settle_count`（預設 3 次 = 150ms）誤差皆落於 `±4.0°` 內才判定對齊完成並煞停，避免因過衝瞬間掠過目標角而提前結束。

#### (2) `drive_blind_imu(ctx, heading_yaw, speed=140, duration_sec=3.0, disable_safety=True) -> bool`
* **功能**：純依賴 IMU 航向鎖定直行指定秒數，不採納超音波數值。
* **控制原理**：進入場地入口前段通常會有門框、起跑擋板或工作人員站立干擾，透過 `temporarily_disable_safety()` 暫時關閉硬體防撞並維持固定航向向前推進 `duration_sec` 秒。

#### (3) `drive_until_front_distance(ctx, heading_yaw, speed=130, stop_front_cm=15.0, confirm_frames=2, max_sec=None, on_tick=None) -> str`
* **功能**：鎖定 `heading_yaw` 直走，直到前方超音波距離 `<= stop_front_cm`（僅參考前方超音波與 IMU）。回傳 `'REACHED'`、`'TIMEOUT'` 或 `'STOPPED'`。
* **控制原理**：
  * **連續幀濾波 (`confirm_frames=2`)**：連續 2 次讀到 `0 < front_cm <= stop_front_cm` 才確認到達邊界（排除單幀雜訊；忽略 `-1` 無效值），若期間先觸發 Arduino 硬體 `E-Stop` 亦視為已貼近障礙並安全清除旗標回傳 `'REACHED'`。
  * **行進間插斷 (`on_tick`)**：每個 `50ms` 週期可呼叫外部檢查函式（如紙屑視覺偵測）；若 `on_tick()` 回傳 `"CONFETTI_HANDLED"`，代表剛完成中途對準與吸取，函式會自動重置計數並重新下發 `set_heading(heading_yaw, speed)` 繼續完成剩餘直線路段。

#### (4) `wall_follow_right(ctx, base_heading_yaw, speed=120, target_right_cm=15.0, stop_front_cm=15.0, stop_right_gte_cm=None, lost_wall_timeout_sec=0.6, kp_wall=1.2, max_angle_adjust=15.0, confirm_frames=2, max_sec=None, on_tick=None) -> str`
* **功能**：以 `base_heading_yaw` 為基準沿右側牆壁直走，動態維持與右牆距離約 `target_right_cm`，直到前方到達距離（`'REACHED'`）或右側變開闊（`'RIGHT_OPEN'`）。
* **控制原理（外環右牆比例校正 + 內環 IMU PD 閉迴路）**：
  1. **符號與極性**：依系統定義，**右轉 (順時針) 為 `+Yaw`、左轉 (逆時針) 為 `-Yaw`**。
  2. 計算右側牆距誤差 `err_cm = right_cm - target_right_cm`：
     * 若 `right_cm > target_right_cm`（離右牆太遠）→ `err_cm > 0` → `angle_adjust > 0` → 目標航向略微**向右修正 (`+Yaw`)**。
     * 若 `right_cm < target_right_cm`（離右牆太近）→ `err_cm < 0` → `angle_adjust < 0` → 目標航向略微**向左修正 (`-Yaw`)**。
  3. **死區與離群保護**：誤差於 `±1.5 cm` 內不做修正以抑制蛇行；僅當 `0 < right_cm < 60.0 cm` 時才介入修正，並將修正角限制在 `±15.0°` 內，避免經過凹槽或右側開闊時大幅甩尾。
  4. **右側開闊判定 (`stop_right_gte_cm`)**：除了 `right_cm >= stop_right_gte_cm` 連續確認外，若超音波因距離過遠回傳 `-1` 持續超過 `lost_wall_timeout_sec`（預設 `0.6s`），亦認定已走出右牆邊界並回傳 `'RIGHT_OPEN'`。

---

### 2.3 影像紙屑辨識與對準（[vision_align.py](file:///home/group6/mechatronics-4/strategies/techniques/vision_align.py)）

#### (1) `VisionSource`
* **功能**：非同步擷取最新影像影格並執行 [ConfettiDetector](file:///home/group6/mechatronics-4/frame_recognize/confetti_detector.py)，提供零阻塞的 `detect_latest() -> (has_confetti, bbox, center)` 查詢。
* **原理**：
  * **實車模式**：背景執行緒以 `cv2.CAP_FFMPEG` 讀取 `rtsp://localhost:8554/live/stream`（並設 `CAP_PROP_BUFFERSIZE = 1`），避免與 [start.sh](file:///home/group6/mechatronics-4/start.sh) 的 `ffmpeg` 推流程序搶奪 `/dev/logi_camera` 裝置，同時不斷消費緩衝區確保不拿到過時舊影格。
  * **模擬模式 (`frame_provider`)**：傳入 `robot.get_injected_frame` 後改由記憶體直接讀取 `/sim` 網頁餵入的影格並交給真實 `ConfettiDetector` 運算。
  * 任何超過 `3.0s` 未更新的影格會自動標記過期並回傳 `(False, None, None)`，防止依據舊影像誤動作。

#### (2) `align_to_confetti(ctx, vision, center_tolerance_x=8.0, target_y_pct=68.0, approach_speed=85, align_gain=0.45, max_duration_sec=8.0, lost_timeout_sec=0.8) -> (bool, float)`
* **功能**：依 `ConfettiDetector` 回傳的百分比座標 `(0.0% ~ 100.0%)` 引導車體正對並靠近紙屑群組中心。
* **控制原理（兩階段視覺伺服）**：
  1. **航向對準**：計算水平偏差 `err_x = center.x - 50.0`。若紙屑在右側（`err_x > 0`），目標航向增加 `+err_x * align_gain`（右轉 `+Yaw`）；反之左轉（`-Yaw`），單步修正幅度限幅於 `±25°`。
  2. **速度調控**：當橫向偏差較大（`|err_x| > 18%`）時先原地旋轉（`speed = 0`）避免斜衝錯過；對準正前方區間後以低速 `approach_speed=85` 向前逼近。
  3. **吸入口盲區容錯**：當 `|err_x| <= 8%` 且 `center.y >= 68%` 時視為對準完成；若對準過程中紙屑已經非常靠近底部（`last_seen_y >= 60%`）而因進入車底鏡頭盲區消失超過 `lost_timeout_sec`，同樣判定為進入吸塵口正下方並回傳 `True` 觸發吸取。

---

### 2.4 擺頭紙屑吸取（[vacuum_sweep.py](file:///home/group6/mechatronics-4/strategies/techniques/vacuum_sweep.py)）

#### `sweep_and_vacuum(ctx, heading_yaw, speed=85, amp_deg=20.0, period_ms=1800, stop_front_cm=15.0, duration_sec=4.0, disable_safety=True) -> bool`
* **功能**：以指定的前進速度、擺頭週期與振幅邊擺動車頭邊開啟吸塵馬達清掃。
* **控制原理**：
  1. 透過 `ctx.temporarily_disable_safety()` 暫時解除近距離防撞，避免貼近吸塵時誤停。
  2. 依序開啟吸塵繼電器 `set_vacuum(True)`、啟動 Arduino 動態正弦擺頭 `set_sweep(True, amp_deg, period_ms)` 與前進航向 `set_heading(heading_yaw, speed)`。
  3. 在軟體層持續監控 `front_cm <= stop_front_cm`（確保擺頭前進時仍不會撞牆）與清掃時間 `duration_sec`。
  4. **資源與馬達收尾保證**：放在 `try ... finally` 區塊中，確保結束、逾時或按下緊急停止時一定關閉擺頭、關閉吸塵繼電器、煞停並恢復硬體防撞門檻。

---

## 3. 如何撰寫一個新的 Technique

若要新增可重用的組合技，請遵守以下 5 項設計規範：

1. **第一個參數固定傳入 `ctx: StrategyContext`**：所有對車輛、休眠、安全開關與日誌的操作皆經由 `ctx` 進行。
2. **迴圈必須可立即中斷**：
   * 迴圈條件寫 `while not ctx.is_stopped():`。
   * 等待延遲請用 `if not ctx.sleep(0.05): break`，嚴禁使用不可中斷的 `time.sleep()` 長時間阻塞。
3. **結束／中斷保證煞停與復原**：將馬達運動與機構控制包在 `try ... finally` 中，確保跳出函式前呼叫 `ctx.robot.stop()`（若有開繼電器或擺頭亦須一併關閉）。
4. **高頻動態更新請用靜音傳輸**：若在 10~20Hz 迴圈內微調航向，請呼叫 `ctx.robot.send_set_heading(yaw, speed, quiet=True)` 以免洗版日誌。
5. **統一匯出**：完成檔案後，將函式加入 [strategies/techniques/\_\_init\_\_.py](file:///home/group6/mechatronics-4/strategies/techniques/__init__.py) 的 `__all__`。

### 範例：新增「指定航向定時後退」組合技

可在 `strategies/techniques/navigation.py` 中新增函式：

```python
def reverse_for_seconds(
    ctx: StrategyContext,
    heading_yaw: float,
    speed: int = 100,
    duration_sec: float = 1.0,
) -> bool:
    """
    維持指定航向鎖定並倒車指定秒數。
    完成回傳 True；若中途被停止則回傳 False。
    """
    heading_yaw = wrap_angle_deg(heading_yaw)
    reverse_speed = -abs(int(speed))
    ctx.set_substate(f"鎖定航向 {heading_yaw:.1f}° 倒車 {duration_sec:.1f} 秒")

    try:
        ctx.robot.set_heading(heading_yaw, reverse_speed)
        start_t = time.monotonic()
        while not ctx.is_stopped():
            if (time.monotonic() - start_t) >= duration_sec:
                return True
            if not ctx.sleep(0.05):
                break
        return False
    finally:
        ctx.robot.stop()
```

接著在 `strategies/techniques/__init__.py` 匯出 `reverse_for_seconds`，即可於策略中直接引用。
