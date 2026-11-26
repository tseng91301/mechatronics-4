#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mechatronics-4 車輛底層控制與感測器 API 模組 (RobotController)
完全獨立於 Web 伺服器，供自動任務策略、CLI 測試工具與 FastAPI 後端共同呼叫。
"""

import math
import random
import struct
import threading
import time
from datetime import datetime
from typing import Callable, Dict, List, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

# ==================== 通訊協議常數 (與 Arduino connection.h 一致) ====================
START_BYTE = 0xAA
END_BYTE   = 0x55

CMD_PING              = 0x00  # 測試連線
CMD_DUAL_MOTOR        = 0x01  # 雙馬達 (turn: int16, speed: int16)
CMD_SINGLE_MOTOR      = 0x02  # 單馬達 (motor_id: uint8, speed: int16)
CMD_RELAY             = 0x03  # 繼電器 (relay_id: uint8, state: uint8)
CMD_SET_AUTO_CORRECT  = 0x04  # 自動校正開關 (enable: uint8)
CMD_RESET_IMU_YAW     = 0x05  # 朝向歸零
CMD_SET_HEADING       = 0x06  # 指定目標航向導航 (target_yaw_x10: int16, speed: int16)
CMD_SET_SWEEP         = 0x07  # 擺頭開關與參數 (enable: uint8, amp_x10: int16, period_ms: uint16)
CMD_STOP_MOTION       = 0x08  # 煞停並停止任何航向/擺頭運動 (0 bytes)
CMD_SET_SAFETY_THRESH = 0x09  # 設定防撞距離門檻 (threshold_mm: uint16, 0代表關閉)
CMD_GET_SAFETY_THRESH = 0x0A  # 查詢當前防撞距離門檻 (0 bytes)
CMD_SET_TURN_SPEED    = 0x0B  # 設定對齊旋轉轉速上限 (turn_speed: int16, 40~220)

CMD_ULTRASONIC        = 0x10  # 4 bytes int32 (mm) 或 5 bytes (id + mm)
CMD_ULTRASONIC_FRONT  = 0x11  # 4 bytes int32 (mm)
CMD_ULTRASONIC_RIGHT  = 0x12  # 4 bytes int32 (mm)
CMD_IMU_DATA          = 0x13  # IMU 姿態數據 (7 bytes: yaw_x10, pitch_x10, roll_x10, mode)
CMD_SAFETY_THRESH_DATA= 0x14  # 回傳防撞距離門檻 (2 bytes uint16 mm)
CMD_LOG_MSG           = 0x20  # 文字訊息

STATE_WAIT_START = 0
STATE_WAIT_CMD   = 1
STATE_WAIT_DATA  = 2
STATE_WAIT_END   = 3


class RobotController:
    """
    Arduino 機器人車輛核心控制器。
    負責序列埠讀寫、二進位協議解析、5Hz 巡航心跳維護，以及執行緒安全的感測器狀態存取。
    """

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.ser: Optional["serial.Serial"] = None
        self.port: str = ""
        self.baudrate: int = 9600
        self.is_connected: bool = False
        self.rx_thread: Optional[threading.Thread] = None
        self.write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._listeners_lock = threading.Lock()
        self._listeners: List[Callable[[dict], None]] = []

        # 狀態機接收緩衝
        self.rx_state = STATE_WAIT_START
        self.current_cmd = 0
        self.expected_len = 0
        self.rx_buffer = bytearray()

        # 虛擬模擬模式
        self.mock_mode: bool = False
        self.mock_thread: Optional[threading.Thread] = None

        # 主動巡航心跳線程 (避免自主導航時因超過 1000ms 觸發 Arduino 失聯安全煞停)
        self.active_heading: Optional[tuple] = None
        self.heartbeat_thread: Optional[threading.Thread] = None
        self._last_drive: Optional[tuple] = None

        # 最新感測器數值與車輛狀態
        self.latest_data: Dict[str, object] = {
            "front_mm": -1,
            "front_cm": -1.0,
            "right_mm": -1,
            "right_cm": -1.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "mode": 0,
            "mode_name": "MANUAL",
            "sweep_enabled": False,
            "safety_threshold_mm": 150,
            "safety_threshold_cm": 15.0,
            "auto_correct": False,
            "relay_state": False,
            "e_stop_triggered": False,
            "e_stop_reason": "",
            "last_updated": 0.0,
        }

        self.cmd_lengths = {
            CMD_PING: 0,
            CMD_DUAL_MOTOR: 4,
            CMD_SINGLE_MOTOR: 3,
            CMD_RELAY: 2,
            CMD_SET_AUTO_CORRECT: 1,
            CMD_RESET_IMU_YAW: 0,
            CMD_SET_HEADING: 4,
            CMD_SET_SWEEP: 5,
            CMD_STOP_MOTION: 0,
            CMD_SET_SAFETY_THRESH: 2,
            CMD_GET_SAFETY_THRESH: 0,
            CMD_SET_TURN_SPEED: 2,
            CMD_SAFETY_THRESH_DATA: 2,
            CMD_ULTRASONIC_FRONT: 4,
            CMD_ULTRASONIC_RIGHT: 4,
            CMD_IMU_DATA: 7,
        }

    # ------------------ 事件訂閱與日誌 ------------------
    def add_listener(self, callback: Callable[[dict], None]):
        """註冊事件監聽器 (接收 status, ultrasonic, imu, relay, e_stop, log 等字典事件)"""
        with self._listeners_lock:
            if callback not in self._listeners:
                self._listeners.append(callback)

    def remove_listener(self, callback: Callable[[dict], None]):
        """移除事件監聽器"""
        with self._listeners_lock:
            if callback in self._listeners:
                self._listeners.remove(callback)

    def notify(self, message: dict):
        """通知所有已註冊的監聽器"""
        with self._listeners_lock:
            listeners = list(self._listeners)
        for cb in listeners:
            try:
                cb(message)
            except Exception:
                pass

    def log(self, direction: str, text: str, hex_data: str = ""):
        now_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if self.verbose:
            hex_part = f" | {hex_data}" if hex_data else ""
            print(f"[{now_str}] [{direction}] {text}{hex_part}")
        self.notify({
            "type": "log",
            "direction": direction,
            "text": text,
            "hex": hex_data,
            "time": now_str,
        })

    # ------------------ 連線與模擬管理 ------------------
    def get_ports(self) -> List[dict]:
        """列出目前系統可用的序列埠"""
        if not serial:
            return []
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append({
                "device": p.device,
                "description": p.description or p.device,
            })
        return ports

    def connect(self, port: str, baudrate: int = 9600, wait_reset_sec: float = 1.8) -> bool:
        """連線至指定的 Arduino 序列埠並啟動接收與巡航心跳執行緒"""
        if self.mock_mode:
            self.stop_mock()

        self.disconnect()

        if not serial:
            self.log("SYS", "錯誤：未安裝 pyserial 模組")
            return False

        try:
            self.port = port
            self.baudrate = baudrate
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            # 等待 Arduino 復位
            if wait_reset_sec > 0:
                time.sleep(wait_reset_sec)
            self.is_connected = True

            self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self.rx_thread.start()

            # 啟動心跳維護線程 (200ms 週期，5Hz)
            if not self.heartbeat_thread or not self.heartbeat_thread.is_alive():
                self.heartbeat_thread = threading.Thread(target=self._heartbeat_worker, daemon=True)
                self.heartbeat_thread.start()

            self.log("SYS", f"成功連線至序列埠: {port} (Baud: {baudrate})")
            self.notify({
                "type": "status",
                "connected": True,
                "mock_mode": False,
                "port": self.port,
                "baudrate": self.baudrate,
            })
            return True
        except Exception as e:
            self.is_connected = False
            self.log("SYS", f"連線失敗 ({port}): {e}")
            self.notify({
                "type": "status",
                "connected": False,
                "mock_mode": False,
                "port": "",
                "baudrate": self.baudrate,
                "error": str(e),
            })
            return False

    def disconnect(self):
        """安全煞停並斷開序列埠連線"""
        self.is_connected = False
        with self._state_lock:
            self.active_heading = None
        if self.ser:
            try:
                # 斷線前煞車保護
                self.send_dual_motor(0, 0)
                time.sleep(0.05)
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        if self.mock_mode:
            self.stop_mock()

        self.log("SYS", "序列埠已斷開")
        self.notify({
            "type": "status",
            "connected": False,
            "mock_mode": self.mock_mode,
            "port": "",
            "baudrate": self.baudrate,
        })

    def reconnect(self) -> bool:
        """重新連線目前的序列埠或 Mock 模式 (使 Arduino 完整重啟復位)"""
        if self.mock_mode or self.port == "MOCK":
            self.stop_mock()
            self.start_mock()
            return True
        port = self.port
        baud = self.baudrate
        if not port:
            self.log("SYS", "無法重新連線：尚未連線過任何序列埠")
            return False
        self.log("SYS", f"正在重新連線序列埠: {port} (Baud: {baud})...")
        return self.connect(port, baud)

    def start_mock(self):
        """啟動虛擬模擬模式 (無實體硬體時可用於測試)"""
        self.disconnect()
        self.mock_mode = True
        self.is_connected = True
        self.mock_thread = threading.Thread(target=self._mock_worker, daemon=True)
        self.mock_thread.start()

        self.log("SYS", "✨ 已啟動虛擬模擬模式 (Mock Mode)")
        self.notify({
            "type": "status",
            "connected": True,
            "mock_mode": True,
            "port": "MOCK_PORT",
            "baudrate": 9600,
        })

    def stop_mock(self):
        """停止虛擬模擬模式"""
        self.mock_mode = False
        self.is_connected = False

    def _notify_ultrasonic(self):
        with self._state_lock:
            msg = {
                "type": "ultrasonic",
                "front_mm": self.latest_data["front_mm"],
                "front_cm": self.latest_data["front_cm"],
                "right_mm": self.latest_data["right_mm"],
                "right_cm": self.latest_data["right_cm"],
            }
        self.notify(msg)

    def get_initial_messages(self) -> List[dict]:
        """取得用於新客戶端同步初始狀態的事件列表"""
        state = self.get_state()
        return [
            {
                "type": "status",
                "connected": self.is_connected,
                "mock_mode": self.mock_mode,
                "port": self.port,
                "baudrate": self.baudrate,
            },
            {
                "type": "ports",
                "ports": self.get_ports(),
            },
            {
                "type": "ultrasonic",
                "front_mm": state["front_mm"],
                "front_cm": state["front_cm"],
                "right_mm": state["right_mm"],
                "right_cm": state["right_cm"],
            },
            {
                "type": "imu",
                "yaw": state["yaw"],
                "pitch": state["pitch"],
                "roll": state["roll"],
                "mode": state["mode"],
                "mode_name": state["mode_name"],
                "sweep_enabled": state["sweep_enabled"],
                "auto_correct": state["auto_correct"],
            },
            {
                "type": "relay",
                "relay_id": 1,
                "state": state["relay_state"],
            },
        ]

    # ------------------ 感測器與狀態 Getter API (Thread-Safe) ------------------
    def get_state(self) -> dict:
        """取得完整車輛與感測器狀態快照"""
        with self._state_lock:
            snap = dict(self.latest_data)
        snap["connected"] = self.is_connected
        snap["mock_mode"] = self.mock_mode
        snap["port"] = self.port
        return snap

    def get_ultrasonic(self) -> dict:
        """取得前方與右方超音波測距 (mm 與 cm)"""
        with self._state_lock:
            return {
                "front_mm": self.latest_data["front_mm"],
                "front_cm": self.latest_data["front_cm"],
                "right_mm": self.latest_data["right_mm"],
                "right_cm": self.latest_data["right_cm"],
            }

    def get_front_mm(self) -> int:
        with self._state_lock:
            return int(self.latest_data["front_mm"])

    def get_front_cm(self) -> float:
        with self._state_lock:
            return float(self.latest_data["front_cm"])

    def get_right_mm(self) -> int:
        with self._state_lock:
            return int(self.latest_data["right_mm"])

    def get_right_cm(self) -> float:
        with self._state_lock:
            return float(self.latest_data["right_cm"])

    def get_imu(self) -> dict:
        """取得 IMU 姿態與控制模式資訊"""
        with self._state_lock:
            return {
                "yaw": self.latest_data["yaw"],
                "pitch": self.latest_data["pitch"],
                "roll": self.latest_data["roll"],
                "mode": self.latest_data["mode"],
                "mode_name": self.latest_data["mode_name"],
                "sweep_enabled": self.latest_data["sweep_enabled"],
                "auto_correct": self.latest_data["auto_correct"],
            }

    def get_yaw(self) -> float:
        """取得目前 IMU 航向角 (deg, 右轉為正)"""
        with self._state_lock:
            return float(self.latest_data["yaw"])

    def get_safety_threshold_mm(self) -> int:
        with self._state_lock:
            return int(self.latest_data["safety_threshold_mm"])

    def is_e_stopped(self) -> bool:
        """回傳是否曾觸發硬體防撞緊急煞停"""
        with self._state_lock:
            return bool(self.latest_data["e_stop_triggered"])

    def clear_e_stop(self):
        """清除防撞緊急煞停旗標"""
        with self._state_lock:
            self.latest_data["e_stop_triggered"] = False
            self.latest_data["e_stop_reason"] = ""

    # ------------------ 底層封包傳輸 ------------------
    def _write_raw(self, raw: bytes, log_desc: str = "", quiet: bool = False, is_ping_sim: bool = False):
        hex_str = ' '.join(f"{b:02X}" for b in raw)
        if self.mock_mode:
            if not quiet:
                self.log("TX", f"[模擬] {log_desc}", hex_str)
            if is_ping_sim:
                time.sleep(0.05)
                self.log("RX", "[模擬回應] Pong! Arduino 連線正常", "AA 00 55")
            return

        if not self.ser or not self.ser.is_open:
            if not quiet:
                self.log("SYS", "警告：未連線，指令未發出", hex_str)
            return

        with self.write_lock:
            try:
                self.ser.write(raw)
                if not quiet:
                    self.log("TX", log_desc, hex_str)
            except Exception as e:
                self.log("SYS", f"發送失敗: {e}", hex_str)

    def send_packet(self, cmd: int, payload: bytes = b'', log_desc: str = "", quiet: bool = False):
        packet = bytearray([START_BYTE, cmd])
        packet.extend(payload)
        packet.append(END_BYTE)
        desc = log_desc or f"指令 0x{cmd:02X}"
        self._write_raw(packet, desc, quiet=quiet, is_ping_sim=(cmd == CMD_PING))

    def send_raw_bytes(self, raw: bytes, log_desc: str = "直接發送原始 HEX"):
        self._write_raw(raw, log_desc)

    # ------------------ 車輛控制 API ------------------
    def send_dual_motor(self, turn: int, speed: int, is_heartbeat: bool = False):
        """手動雙馬達控制 (turn: -255~255, speed: -255~255)"""
        turn = max(-255, min(255, int(turn)))
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>hh', turn, speed)

        turn_str = "直行" if turn == 0 else f"{'左轉' if turn > 0 else '右轉'} ({abs(turn)})"
        spd_str = "停止" if speed == 0 else f"{'前進' if speed > 0 else '後退'} ({abs(speed)})"
        desc = f"雙馬達操控: 轉向={turn} ({turn_str}), 速度={speed} ({spd_str})"

        with self._state_lock:
            if turn == 0 and speed == 0:
                self.active_heading = None
            elif speed != 0 or turn != 0:
                self.latest_data["e_stop_triggered"] = False

        is_repeat = self._last_drive == (turn, speed)
        quiet = is_heartbeat or is_repeat
        self._last_drive = (turn, speed)

        self.send_packet(CMD_DUAL_MOTOR, payload, desc, quiet=quiet)

    def send_single_motor(self, motor_id: int, speed: int):
        """單馬達控制 (motor_id: 0=左, 1=右; speed: -255~255)"""
        motor_id = int(motor_id)
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>Bh', motor_id, speed)
        motor_name = "左馬達 (ID:0)" if motor_id == 0 else "右馬達 (ID:1)" if motor_id == 1 else f"馬達 ID:{motor_id}"
        desc = f"單馬達控制: {motor_name}, 速度={speed}"
        self.send_packet(CMD_SINGLE_MOTOR, payload, desc)

    def send_relay(self, relay_id: int, state: bool):
        """控制繼電器 (預設 ID=1 為吸塵馬達)"""
        relay_id = int(relay_id)
        state_bool = bool(state)
        with self._state_lock:
            self.latest_data["relay_state"] = state_bool
        payload = struct.pack('>BB', relay_id, 1 if state_bool else 0)
        desc = f"控制繼電器 ID={relay_id}, 狀態={'ON' if state_bool else 'OFF'}"
        self.send_packet(CMD_RELAY, payload, desc)
        self.notify({
            "type": "relay",
            "relay_id": relay_id,
            "state": state_bool,
        })

    def send_auto_correct(self, enable: bool):
        """開關手動模式直線自動校正"""
        state_bool = bool(enable)
        with self._state_lock:
            self.latest_data["auto_correct"] = state_bool
        payload = struct.pack('>B', 1 if state_bool else 0)
        desc = f"IMU 自動直線校正: {'開啟 (ON)' if state_bool else '關閉 (OFF)'}"
        self.send_packet(CMD_SET_AUTO_CORRECT, payload, desc)

    def send_reset_imu(self):
        """重設 IMU Yaw 朝向歸零"""
        with self._state_lock:
            self.latest_data["yaw"] = 0.0
        self.send_packet(CMD_RESET_IMU_YAW, b'', "重設 IMU 朝向歸零 (Yaw = 0°)")
        imu_state = self.get_imu()
        self.notify({
            "type": "imu",
            **imu_state,
        })

    def send_ping(self):
        """發送連線測試 Ping"""
        self.send_packet(CMD_PING, b'', "連線測試 Ping (0x00)")

    def send_set_heading(self, target_yaw: float, speed: int = 0, quiet: bool = False):
        """指定目標航向導航 (0x06): target_yaw (-180~180 deg), speed (-255~255)"""
        yaw_x10 = max(-1800, min(1800, int(round(target_yaw * 10.0))))
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>hh', yaw_x10, speed)
        desc = f"設定目標航向: {target_yaw:.1f}° (速度={speed})"

        with self._state_lock:
            self.latest_data["e_stop_triggered"] = False
            if speed != 0:
                self.active_heading = (target_yaw, speed)
            else:
                self.active_heading = None

        self.send_packet(CMD_SET_HEADING, payload, desc, quiet=quiet)

    def send_set_sweep(self, enable: bool, amp: float = 20.0, period_ms: int = 2000):
        """設定或開關動態擺頭模式 (0x07): enable (bool), amp (deg), period_ms (ms)"""
        en_val = 1 if enable else 0
        amp_x10 = max(0, min(900, int(round(amp * 10.0))))
        period_ms = max(500, min(65535, int(period_ms)))
        payload = struct.pack('>BhH', en_val, amp_x10, period_ms)
        desc = f"動態擺頭模式: {'開啟 ON' if enable else '關閉 OFF'} (幅度=±{amp:.1f}°, 週期={period_ms}ms)"
        self.send_packet(CMD_SET_SWEEP, payload, desc)

    def send_stop_motion(self):
        """煞停並停止任何航向/擺頭運動 (0x08)"""
        with self._state_lock:
            self.active_heading = None
        desc = "停止運動並解除航向/擺頭鎖定 (0x08)"
        self.send_packet(CMD_STOP_MOTION, b'', desc)

    def send_set_safety_threshold(self, threshold_mm: int):
        """設定超音波極限防撞門檻 (0x09): threshold_mm (uint16, 0 代表關閉)"""
        threshold_mm = max(0, min(65535, int(threshold_mm)))
        payload = struct.pack('>H', threshold_mm)
        desc = f"設定防撞安全門檻: {threshold_mm} mm ({threshold_mm/10.0:.1f} cm)"
        self.send_packet(CMD_SET_SAFETY_THRESH, payload, desc)

    def send_get_safety_threshold(self):
        """查詢超音波防撞門檻 (0x0A)"""
        desc = "查詢防撞安全門檻 (0x0A)"
        self.send_packet(CMD_GET_SAFETY_THRESH, b'', desc)

    def send_set_turn_speed(self, turn_speed: int):
        """設定原地旋轉對齊速度上限 (0x0B): turn_speed (int16, 40~220)"""
        turn_speed = max(40, min(220, int(turn_speed)))
        payload = struct.pack('>h', turn_speed)
        desc = f"設定旋轉轉速上限: {turn_speed} PWM"
        self.send_packet(CMD_SET_TURN_SPEED, payload, desc)

    # ------------------ 語意化高階別名 (方便策略腳本呼叫) ------------------
    def set_heading(self, target_yaw: float, speed: int = 0):
        """設定目標航向與行進速度 (send_set_heading 別名)"""
        self.send_set_heading(target_yaw, speed)

    def set_sweep(self, enable: bool, amp: float = 20.0, period_ms: int = 2000):
        """開關動態擺頭模式 (send_set_sweep 別名)"""
        self.send_set_sweep(enable, amp, period_ms)

    def stop(self):
        """立即停止所有運動並解除航向/擺頭鎖定"""
        self.send_stop_motion()

    def set_vacuum(self, enable: bool, relay_id: int = 1):
        """開關吸塵馬達 (繼電器控制別名)"""
        self.send_relay(relay_id, enable)

    def reset_yaw(self):
        """將目前 IMU 航向角歸零"""
        self.send_reset_imu()

    def set_turn_speed(self, turn_speed: int):
        """設定原地旋轉對齊速度上限 (40~220 PWM)"""
        self.send_set_turn_speed(turn_speed)

    def set_safety_threshold(self, threshold_mm: int):
        """設定前方超音波緊急防撞距離門檻 (mm, 0=關閉)"""
        self.send_set_safety_threshold(threshold_mm)

    # ------------------ 背景心跳與接收解析狀態機 ------------------
    def _heartbeat_worker(self):
        """背景定時發送巡航心跳至 Arduino，維持自主導航運轉避免 1000ms 逾時煞停"""
        while self.is_connected or self.mock_mode:
            try:
                with self._state_lock:
                    heading = self.active_heading
                if heading is not None:
                    target_yaw, speed = heading
                    if speed != 0:
                        yaw_x10 = max(-1800, min(1800, int(round(target_yaw * 10.0))))
                        spd = max(-255, min(255, int(speed)))
                        payload = struct.pack('>hh', yaw_x10, spd)
                        self.send_packet(CMD_SET_HEADING, payload, "巡航心跳", quiet=True)
                time.sleep(0.2)  # 200ms 心跳週期 (5Hz)，遠小於 Arduino 1000ms 門檻
            except Exception:
                time.sleep(0.2)

    def _rx_worker(self):
        while self.is_connected:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    for b in data:
                        self._process_byte(b)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.is_connected:
                    self.log("SYS", f"序列埠讀取異常: {e}")
                break

    def _process_byte(self, b: int):
        if self.rx_state == STATE_WAIT_START:
            if b == START_BYTE:
                self.rx_state = STATE_WAIT_CMD

        elif self.rx_state == STATE_WAIT_CMD:
            self.current_cmd = b
            self.rx_buffer.clear()
            if b in self.cmd_lengths:
                self.expected_len = self.cmd_lengths[b]
                self.rx_state = STATE_WAIT_END if self.expected_len == 0 else STATE_WAIT_DATA
            elif b == CMD_ULTRASONIC:
                self.expected_len = 4
                self.rx_state = STATE_WAIT_DATA
            elif b == CMD_LOG_MSG:
                self.expected_len = 64
                self.rx_state = STATE_WAIT_DATA
            else:
                if b != START_BYTE:
                    self.rx_state = STATE_WAIT_START

        elif self.rx_state == STATE_WAIT_DATA:
            if self.current_cmd == CMD_LOG_MSG and b == END_BYTE:
                self._dispatch_packet(self.current_cmd, bytes(self.rx_buffer))
                self.rx_state = STATE_WAIT_START
                return

            self.rx_buffer.append(b)
            if len(self.rx_buffer) >= self.expected_len:
                self.rx_state = STATE_WAIT_END

        elif self.rx_state == STATE_WAIT_END:
            if b == END_BYTE:
                self._dispatch_packet(self.current_cmd, bytes(self.rx_buffer))
            self.rx_state = STATE_WAIT_START

    def _dispatch_packet(self, cmd: int, payload: bytes):
        full_packet = bytearray([START_BYTE, cmd])
        full_packet.extend(payload)
        full_packet.append(END_BYTE)
        hex_str = ' '.join(f"{b:02X}" for b in full_packet)

        if cmd == CMD_PING:
            self.log("RX", "收到 Arduino Pong 回應 (連線正常)！", hex_str)
            self.notify({"type": "pong"})

        elif cmd == CMD_ULTRASONIC_FRONT:
            if len(payload) >= 4:
                dist_mm = struct.unpack('>i', payload[:4])[0]
                with self._state_lock:
                    self.latest_data["front_mm"] = dist_mm
                    self.latest_data["front_cm"] = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                    self.latest_data["last_updated"] = time.time()
                self._notify_ultrasonic()

        elif cmd == CMD_ULTRASONIC_RIGHT:
            if len(payload) >= 4:
                dist_mm = struct.unpack('>i', payload[:4])[0]
                with self._state_lock:
                    self.latest_data["right_mm"] = dist_mm
                    self.latest_data["right_cm"] = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                    self.latest_data["last_updated"] = time.time()
                self._notify_ultrasonic()

        elif cmd == CMD_ULTRASONIC:
            if len(payload) == 4:
                dist_mm = struct.unpack('>i', payload)[0]
                with self._state_lock:
                    self.latest_data["front_mm"] = dist_mm
                    self.latest_data["front_cm"] = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                    self.latest_data["last_updated"] = time.time()
                self._notify_ultrasonic()
            elif len(payload) == 5:
                s_id, dist_mm = struct.unpack('>Bi', payload)
                dist_cm = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                prefix = "front" if s_id == 0 else "right"
                with self._state_lock:
                    self.latest_data[f"{prefix}_mm"] = dist_mm
                    self.latest_data[f"{prefix}_cm"] = dist_cm
                    self.latest_data["last_updated"] = time.time()
                self._notify_ultrasonic()

        elif cmd == CMD_IMU_DATA:
            if len(payload) >= 7:
                y_raw, p_raw, r_raw, ac_raw = struct.unpack('>hhhB', payload[:7])
                yaw = round(y_raw / 10.0, 1)
                pitch = round(p_raw / 10.0, 1)
                roll = round(r_raw / 10.0, 1)
                auto_correct = bool(ac_raw)
                is_heading = bool(ac_raw & 0x01)
                is_sweep = bool(ac_raw & 0x02)
                base_mode = "HEADING" if is_heading else "MANUAL"
                mode_name = f"{base_mode}+SWEEP" if is_sweep else base_mode

                with self._state_lock:
                    self.latest_data["yaw"] = yaw
                    self.latest_data["pitch"] = pitch
                    self.latest_data["roll"] = roll
                    self.latest_data["mode"] = ac_raw
                    self.latest_data["mode_name"] = mode_name
                    self.latest_data["sweep_enabled"] = is_sweep
                    self.latest_data["auto_correct"] = auto_correct
                    self.latest_data["last_updated"] = time.time()

                self.notify({
                    "type": "imu",
                    "yaw": yaw,
                    "pitch": pitch,
                    "roll": roll,
                    "mode": ac_raw,
                    "mode_name": mode_name,
                    "sweep_enabled": is_sweep,
                    "auto_correct": auto_correct,
                })

        elif cmd == CMD_SAFETY_THRESH_DATA:
            if len(payload) >= 2:
                thresh_mm = struct.unpack('>H', payload[:2])[0]
                thresh_cm = round(thresh_mm / 10.0, 1)
                with self._state_lock:
                    self.latest_data["safety_threshold_mm"] = thresh_mm
                    self.latest_data["safety_threshold_cm"] = thresh_cm
                self.log("RX", f"目前超音波防撞門檻: {thresh_mm} mm ({thresh_cm:.1f} cm)")
                self.notify({
                    "type": "safety_threshold",
                    "threshold_mm": thresh_mm,
                    "threshold_cm": thresh_cm,
                })

        elif cmd == CMD_LOG_MSG:
            msg = payload.decode('utf-8', errors='ignore')
            self.log("RX", f"Arduino 訊息: {msg}", hex_str)
            # 若 Arduino 觸發極限防撞緊急煞停，清除心跳保持並廣播通知
            if "E-Stop" in msg or "obstacle" in msg:
                with self._state_lock:
                    self.active_heading = None
                    self.latest_data["e_stop_triggered"] = True
                    self.latest_data["e_stop_reason"] = msg
                self.notify({
                    "type": "e_stop",
                    "reason": msg,
                })

        else:
            self.log("RX", f"收到未註冊指令 0x{cmd:02X}", hex_str)

    # ------------------ 模擬執行緒 ------------------
    def _mock_worker(self):
        t = 0.0
        while self.mock_mode and self.is_connected:
            t += 0.2
            f_dist = int(350 + 120 * math.sin(t * 0.7) + random.uniform(-10, 10))
            r_dist = int(220 + 80 * math.cos(t * 0.5) + random.uniform(-8, 8))
            f_dist = max(50, min(2000, f_dist))
            r_dist = max(50, min(1500, r_dist))

            with self._state_lock:
                self.latest_data["front_mm"] = f_dist
                self.latest_data["front_cm"] = round(f_dist / 10.0, 1)
                self.latest_data["right_mm"] = r_dist
                self.latest_data["right_cm"] = round(r_dist / 10.0, 1)

                yaw = round((float(self.latest_data["yaw"]) + random.uniform(-0.15, 0.15)) % 360, 1)
                if yaw > 180:
                    yaw -= 360
                self.latest_data["yaw"] = yaw
                self.latest_data["last_updated"] = time.time()

                front_cm = self.latest_data["front_cm"]
                right_cm = self.latest_data["right_cm"]
                auto_correct = self.latest_data["auto_correct"]

            self.notify({
                "type": "ultrasonic",
                "front_mm": f_dist,
                "front_cm": front_cm,
                "right_mm": r_dist,
                "right_cm": right_cm,
            })
            self.notify({
                "type": "imu",
                "yaw": yaw,
                "pitch": round(1.2 * math.sin(t * 0.4), 1),
                "roll": round(0.8 * math.cos(t * 0.3), 1),
                "auto_correct": auto_correct,
            })
            time.sleep(0.2)
