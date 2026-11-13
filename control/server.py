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

CMD_ULTRASONIC        = 0x10  # 4 bytes int32 (mm) 或 5 bytes (id + mm)
CMD_ULTRASONIC_FRONT  = 0x11  # 4 bytes int32 (mm)
CMD_ULTRASONIC_RIGHT  = 0x12  # 4 bytes int32 (mm)
CMD_IMU_DATA          = 0x13  # IMU 姿態數據 (7 bytes: yaw_x10, pitch_x10, roll_x10, auto_correct)
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

        # 最新感測器數值
        self.latest_data = {
            "front_mm": -1,
            "right_mm": -1,
            "front_cm": -1.0,
            "right_cm": -1.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
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

    def send_packet(self, cmd: int, payload: bytes = b'', log_desc: str = "", quiet: bool = False):
        packet = bytearray([START_BYTE, cmd])
        packet.extend(payload)
        packet.append(END_BYTE)
        hex_str = ' '.join(f"{b:02X}" for b in packet)

        if self.mock_mode:
            if not quiet:
                self.log("TX", f"[模擬] {log_desc or f'發送指令 0x{cmd:02X}'}", hex_str)
            if cmd == CMD_PING:
                time.sleep(0.05)
                self.log("RX", "[模擬回應] Pong! Arduino 連線正常", "AA 00 55")
            return

        if not self.ser or not self.ser.is_open:
            if not quiet:
                self.log("SYS", "警告：未連線，指令未發出", hex_str)
            return

        with self.write_lock:
            try:
                self.ser.write(packet)
                if not quiet:
                    self.log("TX", log_desc or f"指令 0x{cmd:02X}", hex_str)
            except Exception as e:
                self.log("SYS", f"發送失敗: {e}", hex_str)

    def send_raw_bytes(self, raw: bytes, log_desc: str = "直接發送原始 HEX"):
        hex_str = ' '.join(f"{b:02X}" for b in raw)
        if self.mock_mode:
            self.log("TX", f"[模擬] {log_desc}", hex_str)
            return

        if not self.ser or not self.ser.is_open:
            self.log("SYS", "警告：未連線，無法發送原始 HEX", hex_str)
            return

        with self.write_lock:
            try:
                self.ser.write(raw)
                self.log("TX", log_desc, hex_str)
            except Exception as e:
                self.log("SYS", f"發送原始 HEX 失敗: {e}", hex_str)

    def send_dual_motor(self, turn: int, speed: int, is_heartbeat: bool = False):
        turn = max(-255, min(255, int(turn)))
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>hh', turn, speed)
        turn_str = "直行" if turn == 0 else f"{'左轉' if turn > 0 else '右轉'} ({abs(turn)})"
        spd_str = "停止" if speed == 0 else f"{'前進' if speed > 0 else '後退'} ({abs(speed)})"
        desc = f"雙馬達操控: 轉向={turn} ({turn_str}), 速度={speed} ({spd_str})"

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
                dist_cm = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                self.latest_data["front_mm"] = dist_mm
                self.latest_data["front_cm"] = dist_cm
                self.latest_data["last_updated"] = time.time()

                self.notify({
                    "type": "ultrasonic",
                    "front_mm": dist_mm,
                    "front_cm": dist_cm,
                    "right_mm": self.latest_data["right_mm"],
                    "right_cm": self.latest_data["right_cm"]
                })

        elif cmd == CMD_ULTRASONIC_RIGHT:
            if len(payload) >= 4:
                dist_mm = struct.unpack('>i', payload[:4])[0]
                dist_cm = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                self.latest_data["right_mm"] = dist_mm
                self.latest_data["right_cm"] = dist_cm
                self.latest_data["last_updated"] = time.time()

                self.notify({
                    "type": "ultrasonic",
                    "front_mm": self.latest_data["front_mm"],
                    "front_cm": self.latest_data["front_cm"],
                    "right_mm": dist_mm,
                    "right_cm": dist_cm
                })

        elif cmd == CMD_ULTRASONIC:
            if len(payload) == 4:
                dist_mm = struct.unpack('>i', payload)[0]
                dist_cm = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                self.latest_data["front_mm"] = dist_mm
                self.latest_data["front_cm"] = dist_cm
                self.notify({
                    "type": "ultrasonic",
                    "front_mm": dist_mm,
                    "front_cm": dist_cm,
                    "right_mm": self.latest_data["right_mm"],
                    "right_cm": self.latest_data["right_cm"]
                })
            elif len(payload) == 5:
                s_id, dist_mm = struct.unpack('>Bi', payload)
                dist_cm = round(dist_mm / 10.0, 1) if dist_mm >= 0 else -1.0
                if s_id == 0:
                    self.latest_data["front_mm"] = dist_mm
                    self.latest_data["front_cm"] = dist_cm
                elif s_id == 1:
                    self.latest_data["right_mm"] = dist_mm
                    self.latest_data["right_cm"] = dist_cm
                self.notify({
                    "type": "ultrasonic",
                    "front_mm": self.latest_data["front_mm"],
                    "front_cm": self.latest_data["front_cm"],
                    "right_mm": self.latest_data["right_mm"],
                    "right_cm": self.latest_data["right_cm"]
                })

        elif cmd == CMD_IMU_DATA:
            if len(payload) >= 7:
                y_raw, p_raw, r_raw, ac_raw = struct.unpack('>hhhB', payload[:7])
                yaw = round(y_raw / 10.0, 1)
                pitch = round(p_raw / 10.0, 1)
                roll = round(r_raw / 10.0, 1)
                auto_correct = bool(ac_raw)

                self.latest_data["yaw"] = yaw
                self.latest_data["pitch"] = pitch
                self.latest_data["roll"] = roll
                self.latest_data["auto_correct"] = auto_correct
                self.latest_data["last_updated"] = time.time()

                self.notify({
                    "type": "imu",
                    "yaw": yaw,
                    "pitch": pitch,
                    "roll": roll,
                    "auto_correct": auto_correct
                })

        elif cmd == CMD_LOG_MSG:
            msg = payload.decode('utf-8', errors='ignore')
            self.log("RX", f"Arduino 訊息: {msg}", hex_str)

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
    return templates.TemplateResponse(
        request=request,
        name="index.html",
        context={"title": "Mechatronics-4 機器人遠程監控與控制台"}
    )

# API 模型
class ConnectRequest(BaseModel):
    port: str
    baudrate: int = 9600

class HexRequest(BaseModel):
    hex_string: str
    auto_wrap: bool = True

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
    await websocket.send_json({
        "type": "status",
        "connected": bridge.is_connected,
        "mock_mode": bridge.mock_mode,
        "port": bridge.port,
        "baudrate": bridge.baudrate
    })
    await websocket.send_json({
        "type": "ports",
        "ports": bridge.get_ports()
    })
    await websocket.send_json({
        "type": "ultrasonic",
        "front_mm": bridge.latest_data["front_mm"],
        "front_cm": bridge.latest_data["front_cm"],
        "right_mm": bridge.latest_data["right_mm"],
        "right_cm": bridge.latest_data["right_cm"]
    })
    await websocket.send_json({
        "type": "imu",
        "yaw": bridge.latest_data["yaw"],
        "pitch": bridge.latest_data["pitch"],
        "roll": bridge.latest_data["roll"],
        "auto_correct": bridge.latest_data["auto_correct"]
    })
    await websocket.send_json({
        "type": "relay",
        "relay_id": 1,
        "state": bridge.latest_data["relay_state"]
    })

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
