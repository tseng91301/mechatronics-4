#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mechatronics-4 機器人車輛 Web 控制伺服器
採用 FastAPI + Uvicorn + WebSocket 架構
支援即時超音波數據監控 (前方與右方)、雙馬達搖桿操控、單馬達進階操控與自訂 HEX 指令發送
"""

import os
import sys
import time
import math
import random
import struct
import asyncio
import threading
from datetime import datetime
from typing import List, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel

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
CMD_SET_TURN_SPEED    = 0x0B  # 設定對齊旋轉轉速上限 (turn_speed: int16, 40~200)

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

# ==================== WebSocket 連線管理器 ====================
class ConnectionManager:
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self._lock:
            self.active_connections.add(websocket)

    async def disconnect(self, websocket: WebSocket):
        async with self._lock:
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        async with self._lock:
            dead_connections = []
            for connection in self.active_connections:
                try:
                    await connection.send_json(message)
                except Exception:
                    dead_connections.append(connection)
            for dead in dead_connections:
                self.active_connections.discard(dead)

manager = ConnectionManager()

# ==================== 序列埠與控制器核心 ====================
class ArduinoBridge:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.port: str = ""
        self.baudrate: int = 9600
        self.is_connected: bool = False
        self.rx_thread: Optional[threading.Thread] = None
        self.write_lock = threading.Lock()
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None

        # 模擬模式 (Mock Mode): 無硬體時亦可體驗測試
        self.mock_mode: bool = False
        self.mock_thread: Optional[threading.Thread] = None

        # 主動巡航心跳線程 (避免自主導航時因超過 1000ms 觸發 Arduino 失聯安全煞停)
        self.active_heading: Optional[tuple] = None
        self.heartbeat_thread: Optional[threading.Thread] = None

        # 最新感測器數值
        self.latest_data = {
            "front_mm": -1,
            "right_mm": -1,
            "front_cm": -1.0,
            "right_cm": -1.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "mode": 0,
            "mode_name": "MANUAL",
            "safety_threshold_mm": 150,
            "safety_threshold_cm": 15.0,
            "auto_correct": False,
            "relay_state": False,
            "last_updated": 0
        }

        # 狀態機變數
        self.rx_state = STATE_WAIT_START
        self.current_cmd = 0
        self.expected_len = 0
        self.rx_buffer = bytearray()
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

    def set_event_loop(self, loop: asyncio.AbstractEventLoop):
        self.event_loop = loop

    def notify(self, message: dict):
        """將訊息非同步廣播至所有 WebSocket 連線"""
        if self.event_loop and self.event_loop.is_running():
            asyncio.run_coroutine_threadsafe(manager.broadcast(message), self.event_loop)

    def log(self, direction: str, text: str, hex_data: str = ""):
        now_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        payload = {
            "type": "log",
            "direction": direction,
            "text": text,
            "hex": hex_data,
            "time": now_str
        }
        self.notify(payload)

    def get_ports(self) -> List[dict]:
        if not serial:
            return []
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append({
                "device": p.device,
                "description": p.description or p.device
            })
        return ports

    def connect(self, port: str, baudrate: int = 9600) -> bool:
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
            time.sleep(1.8)
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
                "baudrate": self.baudrate
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
                "error": str(e)
            })
            return False

    def disconnect(self):
        self.is_connected = False
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
            "baudrate": self.baudrate
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
        """啟動虛擬模擬模式 (無實體硬體時可用於測試 UI)"""
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
            "baudrate": 9600
        })

    def stop_mock(self):
        self.mock_mode = False
        self.is_connected = False

    def _notify_ultrasonic(self):
        self.notify({
            "type": "ultrasonic",
            "front_mm": self.latest_data["front_mm"],
            "front_cm": self.latest_data["front_cm"],
            "right_mm": self.latest_data["right_mm"],
            "right_cm": self.latest_data["right_cm"]
        })

    def get_initial_messages(self) -> List[dict]:
        return [
            {
                "type": "status",
                "connected": self.is_connected,
                "mock_mode": self.mock_mode,
                "port": self.port,
                "baudrate": self.baudrate
            },
            {
                "type": "ports",
                "ports": self.get_ports()
            },
            {
                "type": "ultrasonic",
                "front_mm": self.latest_data["front_mm"],
                "front_cm": self.latest_data["front_cm"],
                "right_mm": self.latest_data["right_mm"],
                "right_cm": self.latest_data["right_cm"]
            },
            {
                "type": "imu",
                "yaw": self.latest_data["yaw"],
                "pitch": self.latest_data["pitch"],
                "roll": self.latest_data["roll"],
                "auto_correct": self.latest_data["auto_correct"]
            },
            {
                "type": "relay",
                "relay_id": 1,
                "state": self.latest_data["relay_state"]
            }
        ]

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

    def send_dual_motor(self, turn: int, speed: int, is_heartbeat: bool = False):
        turn = max(-255, min(255, int(turn)))
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>hh', turn, speed)
        turn_str = "直行" if turn == 0 else f"{'左轉' if turn > 0 else '右轉'} ({abs(turn)})"
        spd_str = "停止" if speed == 0 else f"{'前進' if speed > 0 else '後退'} ({abs(speed)})"
        desc = f"雙馬達操控: 轉向={turn} ({turn_str}), 速度={speed} ({spd_str})"

        # 若停止馬達，一併清除活躍航向
        if turn == 0 and speed == 0:
            self.active_heading = None

        # 若為重複心跳且數值未變，靜默發送至序列埠維持 Arduino 運轉，避免 Web 日誌刷屏
        is_repeat = hasattr(self, '_last_drive') and self._last_drive == (turn, speed)
        quiet = is_heartbeat or is_repeat
        self._last_drive = (turn, speed)

        self.send_packet(CMD_DUAL_MOTOR, payload, desc, quiet=quiet)

    def send_single_motor(self, motor_id: int, speed: int):
        motor_id = int(motor_id)
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>Bh', motor_id, speed)
        motor_name = "左馬達 (ID:0)" if motor_id == 0 else "右馬達 (ID:1)" if motor_id == 1 else f"馬達 ID:{motor_id}"
        desc = f"單馬達控制: {motor_name}, 速度={speed}"
        self.send_packet(CMD_SINGLE_MOTOR, payload, desc)

    def send_relay(self, relay_id: int, state: bool):
        relay_id = int(relay_id)
        state_bool = bool(state)
        self.latest_data["relay_state"] = state_bool
        payload = struct.pack('>BB', relay_id, 1 if state_bool else 0)
        desc = f"控制繼電器 ID={relay_id}, 狀態={'ON' if state_bool else 'OFF'}"
        self.send_packet(CMD_RELAY, payload, desc)
        self.notify({
            "type": "relay",
            "relay_id": relay_id,
            "state": state_bool
        })

    def send_auto_correct(self, enable: bool):
        state_bool = bool(enable)
        self.latest_data["auto_correct"] = state_bool
        payload = struct.pack('>B', 1 if state_bool else 0)
        desc = f"設定直線自動校正: {'開啟 ON' if state_bool else '關閉 OFF'}"
        self.send_packet(CMD_SET_AUTO_CORRECT, payload, desc)
        self.notify({
            "type": "auto_correct",
            "enabled": state_bool
        })

    def send_reset_imu(self):
        desc = "重設 IMU 朝向歸零 (0x05)"
        self.latest_data["yaw"] = 0.0
        self.send_packet(CMD_RESET_IMU_YAW, b'', desc)
        self.notify({
            "type": "imu",
            "yaw": 0.0,
            "pitch": self.latest_data["pitch"],
            "roll": self.latest_data["roll"],
            "auto_correct": self.latest_data["auto_correct"]
        })

    def send_ping(self):
        self.send_packet(CMD_PING, b'', "連線測試 Ping (0x00)")

    def send_set_heading(self, target_yaw: float, speed: int = 0, quiet: bool = False):
        """指定目標航向導航 (0x06): target_yaw (deg), speed (-255~255)"""
        yaw_x10 = max(-1800, min(1800, int(round(target_yaw * 10.0))))
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>hh', yaw_x10, speed)
        desc = f"設定目標航向: {target_yaw:.1f}° (速度={speed})"

        # 若速度非 0，啟動背景活躍航向心跳保持巡航
        if speed != 0:
            self.active_heading = (target_yaw, speed)
        else:
            self.active_heading = None

        self.send_packet(CMD_SET_HEADING, payload, desc, quiet=quiet)

    def send_set_sweep(self, enable: bool, amp: float = 20.0, period_ms: int = 2000):
        """設定或開關擺頭模式 (0x07): enable (bool), amp (deg), period_ms (ms)"""
        en_val = 1 if enable else 0
        amp_x10 = max(0, min(900, int(round(amp * 10.0))))
        period_ms = max(500, min(65535, int(period_ms)))
        payload = struct.pack('>BhH', en_val, amp_x10, period_ms)
        desc = f"動態擺頭模式: {'開啟 ON' if enable else '關閉 OFF'} (幅度=±{amp:.1f}°, 週期={period_ms}ms)"
        self.send_packet(CMD_SET_SWEEP, payload, desc)

    def send_stop_motion(self):
        """煞停並停止任何航向/擺頭運動 (0x08)"""
        self.active_heading = None
        desc = "停止運動並解除航向/擺頭鎖定 (0x08)"
        self.send_packet(CMD_STOP_MOTION, b'', desc)

    def _heartbeat_worker(self):
        """背景定時發送巡航心跳至 Arduino，維持自主導航運轉避免 1000ms 逾時煞停"""
        while self.is_connected or self.mock_mode:
            try:
                if self.active_heading is not None:
                    target_yaw, speed = self.active_heading
                    if speed != 0:
                        yaw_x10 = max(-1800, min(1800, int(round(target_yaw * 10.0))))
                        spd = max(-255, min(255, int(speed)))
                        payload = struct.pack('>hh', yaw_x10, spd)
                        # 以靜音模式持續發送保持 Arduino 運作
                        self.send_packet(CMD_SET_HEADING, payload, "巡航心跳", quiet=True)
                time.sleep(0.2)  # 200ms 心跳週期 (5Hz)，遠小於 Arduino 1000ms 門檻
            except Exception:
                time.sleep(0.2)

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
        """設定原地旋轉對齊速度上限 (0x0B): turn_speed (int16, 40~200)"""
        turn_speed = max(40, min(220, int(turn_speed)))
        payload = struct.pack('>h', turn_speed)
        desc = f"設定旋轉轉速上限: {turn_speed} PWM"
        self.send_packet(CMD_SET_TURN_SPEED, payload, desc)

    # ------------------ 接收解析狀態機 ------------------
    def _rx_worker(self):
        self.rx_state = STATE_WAIT_START
        self.rx_buffer.clear()

        while self.is_connected and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    raw = self.ser.read(self.ser.in_waiting)
                    for b in raw:
                        self._process_byte(b)
                else:
                    time.sleep(0.005)
            except Exception as e:
                if self.is_connected:
                    self.log("SYS", f"序列埠讀取異常: {e}")
                break
        self.disconnect()

    def _process_byte(self, b: int):
        if self.rx_state == STATE_WAIT_START:
            if b == START_BYTE:
                self.rx_state = STATE_WAIT_CMD

        elif self.rx_state == STATE_WAIT_CMD:
            self.current_cmd = b
            self.rx_buffer.clear()

            if b in self.cmd_lengths:
                self.expected_len = self.cmd_lengths[b]
                if self.expected_len == 0:
                    self.rx_state = STATE_WAIT_END
                else:
                    self.rx_state = STATE_WAIT_DATA
            elif b == CMD_ULTRASONIC:
                self.expected_len = 4
                self.rx_state = STATE_WAIT_DATA
            elif b == CMD_LOG_MSG:
                self.expected_len = 32
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

        elif cmd == CMD_ULTRASONIC_FRONT:
            if len(payload) >= 4:
                dist_mm = struct.unpack('>i', payload[:4])[0]
                self.latest_data["front_mm"] = dist_mm
                self.latest_data["front_cm"] = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                self.latest_data["last_updated"] = time.time()
                self._notify_ultrasonic()

        elif cmd == CMD_ULTRASONIC_RIGHT:
            if len(payload) >= 4:
                dist_mm = struct.unpack('>i', payload[:4])[0]
                self.latest_data["right_mm"] = dist_mm
                self.latest_data["right_cm"] = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                self.latest_data["last_updated"] = time.time()
                self._notify_ultrasonic()

        elif cmd == CMD_ULTRASONIC:
            if len(payload) == 4:
                dist_mm = struct.unpack('>i', payload)[0]
                self.latest_data["front_mm"] = dist_mm
                self.latest_data["front_cm"] = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                self.latest_data["last_updated"] = time.time()
                self._notify_ultrasonic()
            elif len(payload) == 5:
                s_id, dist_mm = struct.unpack('>Bi', payload)
                dist_cm = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                prefix = "front" if s_id == 0 else "right"
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
                    "auto_correct": auto_correct
                })

        elif cmd == CMD_SAFETY_THRESH_DATA:
            if len(payload) >= 2:
                thresh_mm = struct.unpack('>H', payload[:2])[0]
                self.latest_data["safety_threshold_mm"] = thresh_mm
                self.latest_data["safety_threshold_cm"] = round(thresh_mm / 10.0, 1)
                self.log("RX", f"目前超音波防撞門檻: {thresh_mm} mm ({thresh_mm/10.0:.1f} cm)")
                self.notify({
                    "type": "safety_threshold",
                    "threshold_mm": thresh_mm,
                    "threshold_cm": round(thresh_mm / 10.0, 1)
                })

        elif cmd == CMD_LOG_MSG:
            msg = payload.decode('utf-8', errors='ignore')
            self.log("RX", f"Arduino 訊息: {msg}", hex_str)
            # 若 Arduino 觸發極限防撞緊急煞停，清除心跳保持並廣播通知
            if "E-Stop" in msg or "obstacle" in msg:
                self.active_heading = None
                self.notify({
                    "type": "e_stop",
                    "reason": msg
                })

        else:
            self.log("RX", f"收到未註冊指令 0x{cmd:02X}", hex_str)

    # ------------------ 模擬執行緒 ------------------
    def _mock_worker(self):
        t = 0.0
        while self.mock_mode and self.is_connected:
            t += 0.2
            # 產生模擬自然波動的距離數值 (mm)
            f_dist = int(350 + 120 * math.sin(t * 0.7) + random.uniform(-10, 10))
            r_dist = int(220 + 80 * math.cos(t * 0.5) + random.uniform(-8, 8))
            f_dist = max(50, min(2000, f_dist))
            r_dist = max(50, min(1500, r_dist))

            self.latest_data["front_mm"] = f_dist
            self.latest_data["front_cm"] = round(f_dist / 10.0, 1)
            self.latest_data["right_mm"] = r_dist
            self.latest_data["right_cm"] = round(r_dist / 10.0, 1)

            # 模擬 IMU 姿態自然微晃
            self.latest_data["yaw"] = round((self.latest_data["yaw"] + random.uniform(-0.15, 0.15)) % 360, 1)
            if self.latest_data["yaw"] > 180:
                self.latest_data["yaw"] -= 360

            self.notify({
                "type": "ultrasonic",
                "front_mm": f_dist,
                "front_cm": self.latest_data["front_cm"],
                "right_mm": r_dist,
                "right_cm": self.latest_data["right_cm"]
            })
            self.notify({
                "type": "imu",
                "yaw": self.latest_data["yaw"],
                "pitch": round(1.2 * math.sin(t * 0.4), 1),
                "roll": round(0.8 * math.cos(t * 0.3), 1),
                "auto_correct": self.latest_data["auto_correct"]
            })
            time.sleep(0.2)


bridge = ArduinoBridge()

from contextlib import asynccontextmanager

# ==================== FastAPI 應用與路由 ====================
@asynccontextmanager
async def lifespan(app: FastAPI):
    loop = asyncio.get_running_loop()
    bridge.set_event_loop(loop)
    print("-> Web 伺服器已就緒，請在瀏覽器開啟: http://localhost:8000")
    yield
    bridge.disconnect()

app = FastAPI(
    title="Mechatronics-4 Web Control",
    description="Arduino 車輛控制與超音波即時監控系統",
    lifespan=lifespan
)

# HTML Template 目錄
templates = Jinja2Templates(directory=os.path.join(os.path.dirname(__file__), "templates"))

@app.get("/", response_class=HTMLResponse)
async def get_index(request: Request):
    """回傳主頁面 HTML (由 Jinja2 模板渲染)"""
    """回傳自主導航與除錯主頁面 HTML (由 Jinja2 模板渲染)"""
    return templates.TemplateResponse(
        request=request,
        name="index.html",
        context={"title": "Mechatronics-4 機器人遠程監控與控制台"}
    )

@app.get("/legacy", response_class=HTMLResponse)
async def get_legacy(request: Request):
    """回傳傳統手動遙控與無人機手把模式頁面 HTML"""
    return templates.TemplateResponse(
        request=request,
        name="legacy.html",
        context={"title": "Mechatronics-4 機器人傳統遙控台 (Legacy)"}
    )

# API 模型
class ConnectRequest(BaseModel):
    port: str
    baudrate: int = 9600

class HexRequest(BaseModel):
    hex_string: str
    auto_wrap: bool = True

class HeadingRequest(BaseModel):
    target_yaw: float
    speed: int = 0

class SweepRequest(BaseModel):
    enable: bool = True
    amplitude: float = 20.0
    period_ms: int = 2000

class SafetyThresholdRequest(BaseModel):
    threshold_mm: int

class TurnSpeedRequest(BaseModel):
    turn_speed: int = 90

@app.post("/api/turn_speed")
async def api_set_turn_speed(req: TurnSpeedRequest):
    bridge.send_set_turn_speed(req.turn_speed)
    return {"success": True, "turn_speed": req.turn_speed}

@app.post("/api/heading")
async def api_heading(req: HeadingRequest):
    bridge.send_set_heading(req.target_yaw, req.speed)
    return {"success": True, "target_yaw": req.target_yaw, "speed": req.speed}

@app.post("/api/sweep")
async def api_sweep(req: SweepRequest):
    bridge.send_set_sweep(req.enable, req.amplitude, req.period_ms)
    return {"success": True, "enable": req.enable, "amplitude": req.amplitude, "period_ms": req.period_ms}

@app.post("/api/safety_threshold")
async def api_set_safety_threshold(req: SafetyThresholdRequest):
    bridge.send_set_safety_threshold(req.threshold_mm)
    return {"success": True, "threshold_mm": req.threshold_mm}

@app.get("/api/safety_threshold")
async def api_get_safety_threshold():
    bridge.send_get_safety_threshold()
    return {
        "success": True,
        "threshold_mm": bridge.latest_data.get("safety_threshold_mm", 150),
        "threshold_cm": bridge.latest_data.get("safety_threshold_cm", 15.0)
    }

@app.post("/api/stop_motion")
async def api_stop_motion():
    bridge.send_stop_motion()
    return {"success": True}

@app.get("/api/ports")
async def api_get_ports():
    ports = bridge.get_ports()
    return {"ports": ports, "connected": bridge.is_connected, "current_port": bridge.port}

@app.post("/api/connect")
async def api_connect(req: ConnectRequest):
    if req.port == "MOCK":
        bridge.start_mock()
        return {"success": True, "mock": True}
    success = bridge.connect(req.port, req.baudrate)
    return {"success": success, "port": req.port, "baudrate": req.baudrate}

@app.post("/api/disconnect")
async def api_disconnect():
    bridge.disconnect()
    return {"success": True}

@app.get("/api/status")
async def api_status():
    return {
        "connected": bridge.is_connected,
        "mock_mode": bridge.mock_mode,
        "port": bridge.port,
        "baudrate": bridge.baudrate,
        "latest": bridge.latest_data
    }

# ==================== WebSocket 端點 ====================
def parse_hex_input(hex_str: str, auto_wrap: bool = True) -> bytes:
    clean = hex_str.replace("0x", "").replace("0X", "").replace("\\x", "").replace(",", " ").strip()
    parts = clean.split()
    if len(parts) == 1 and len(parts[0]) % 2 == 0:
        raw = bytes.fromhex(parts[0])
    elif len(parts) > 1:
        raw = bytes([int(p, 16) for p in parts if p])
    else:
        raw = bytes.fromhex(clean)

    if auto_wrap:
        b_list = list(raw)
        if len(b_list) == 0 or b_list[0] != START_BYTE:
            b_list.insert(0, START_BYTE)
        if len(b_list) <= 1 or b_list[-1] != END_BYTE:
            b_list.append(END_BYTE)
        raw = bytes(b_list)

    return raw

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)

    # 連線初始狀態推播
    for init_msg in bridge.get_initial_messages():
        await websocket.send_json(init_msg)

    try:
        while True:
            data = await websocket.receive_json()
            action = data.get("action")

            if action == "dual_motor":
                turn = int(data.get("turn", 0))
                speed = int(data.get("speed", 0))
                is_heartbeat = bool(data.get("heartbeat", False))
                bridge.send_dual_motor(turn, speed, is_heartbeat=is_heartbeat)

            elif action == "single_motor":
                motor_id = int(data.get("motor_id", 0))
                speed = int(data.get("speed", 0))
                bridge.send_single_motor(motor_id, speed)

            elif action == "stop":
                bridge.send_dual_motor(0, 0)

            elif action == "set_heading":
                target_yaw = float(data.get("target_yaw", 0.0))
                speed = int(data.get("speed", 0))
                is_hb = bool(data.get("heartbeat", False))
                bridge.send_set_heading(target_yaw, speed, quiet=is_hb)

            elif action == "set_sweep":
                enable = bool(data.get("enable", True))
                amp = float(data.get("amplitude", 20.0))
                period_ms = int(data.get("period_ms", 2000))
                bridge.send_set_sweep(enable, amp, period_ms)

            elif action == "stop_motion":
                bridge.send_stop_motion()

            elif action == "set_turn_speed":
                turn_speed = int(data.get("turn_speed", 90))
                bridge.send_set_turn_speed(turn_speed)

            elif action == "set_safety_threshold":
                threshold_mm = int(data.get("threshold_mm", 150))
                bridge.send_set_safety_threshold(threshold_mm)

            elif action == "get_safety_threshold":
                bridge.send_get_safety_threshold()

            elif action == "relay":
                relay_id = int(data.get("relay_id", 1))
                state = bool(data.get("state", False))
                bridge.send_relay(relay_id, state)

            elif action == "set_auto_correct":
                enable = bool(data.get("enable", False))
                bridge.send_auto_correct(enable)

            elif action == "reset_imu":
                bridge.send_reset_imu()

            elif action == "ping":
                bridge.send_ping()

            elif action == "send_hex":
                hex_raw_str = data.get("hex", "")
                auto_wrap = data.get("auto_wrap", True)
                try:
                    raw_bytes = parse_hex_input(hex_raw_str, auto_wrap)
                    bridge.send_raw_bytes(raw_bytes, "自訂 HEX 指令發送")
                except Exception as ex:
                    bridge.log("SYS", f"HEX 解析失敗: {ex}")

            elif action == "connect":
                port = data.get("port", "")
                baud = int(data.get("baudrate", 9600))
                if port == "MOCK":
                    bridge.start_mock()
                else:
                    bridge.connect(port, baud)

            elif action == "disconnect":
                bridge.disconnect()

            elif action == "reconnect":
                port = data.get("port", "") or bridge.port
                baud = int(data.get("baudrate", 0)) or bridge.baudrate
                if port == "MOCK" or bridge.mock_mode:
                    bridge.reconnect()
                elif port:
                    bridge.connect(port, baud)
                else:
                    bridge.reconnect()

            elif action == "refresh_ports":
                await websocket.send_json({
                    "type": "ports",
                    "ports": bridge.get_ports()
                })

    except WebSocketDisconnect:
        await manager.disconnect(websocket)
    except Exception as e:
        await manager.disconnect(websocket)


if __name__ == "__main__":
    import uvicorn
    # 支援直接執行: python server.py
    # 亦支援: uvicorn server:app --host 0.0.0.0 --port 8000 --reload
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=True)
