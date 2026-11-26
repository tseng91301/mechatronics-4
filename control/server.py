#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mechatronics-4 機器人車輛 Web 控制伺服器
採用 FastAPI + Uvicorn + WebSocket 架構，底層車輛控制完全委派給獨立的 RobotController 模組。
"""

import asyncio
import os
from contextlib import asynccontextmanager
from typing import Optional, Set

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel

try:
    from control.robot_controller import (
        CMD_DUAL_MOTOR,
        CMD_GET_SAFETY_THRESH,
        CMD_IMU_DATA,
        CMD_LOG_MSG,
        CMD_PING,
        CMD_RELAY,
        CMD_RESET_IMU_YAW,
        CMD_SAFETY_THRESH_DATA,
        CMD_SET_AUTO_CORRECT,
        CMD_SET_HEADING,
        CMD_SET_SAFETY_THRESH,
        CMD_SET_SWEEP,
        CMD_SET_TURN_SPEED,
        CMD_SINGLE_MOTOR,
        CMD_STOP_MOTION,
        CMD_ULTRASONIC,
        CMD_ULTRASONIC_FRONT,
        CMD_ULTRASONIC_RIGHT,
        END_BYTE,
        START_BYTE,
        STATE_WAIT_CMD,
        STATE_WAIT_DATA,
        STATE_WAIT_END,
        STATE_WAIT_START,
        RobotController,
    )
except ImportError:
    from robot_controller import (
        CMD_DUAL_MOTOR,
        CMD_GET_SAFETY_THRESH,
        CMD_IMU_DATA,
        CMD_LOG_MSG,
        CMD_PING,
        CMD_RELAY,
        CMD_RESET_IMU_YAW,
        CMD_SAFETY_THRESH_DATA,
        CMD_SET_AUTO_CORRECT,
        CMD_SET_HEADING,
        CMD_SET_SAFETY_THRESH,
        CMD_SET_SWEEP,
        CMD_SET_TURN_SPEED,
        CMD_SINGLE_MOTOR,
        CMD_STOP_MOTION,
        CMD_ULTRASONIC,
        CMD_ULTRASONIC_FRONT,
        CMD_ULTRASONIC_RIGHT,
        END_BYTE,
        START_BYTE,
        STATE_WAIT_CMD,
        STATE_WAIT_DATA,
        STATE_WAIT_END,
        STATE_WAIT_START,
        RobotController,
    )


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

# ==================== 控制器實例掛載 ====================
bridge: RobotController = RobotController()
_owns_controller: bool = True
_event_loop: Optional[asyncio.AbstractEventLoop] = None


def attach_controller(ext_controller: RobotController):
    """掛載外部建立的 RobotController 實例，讓自動任務與 Web 除錯面板共用同一序列埠連線"""
    global bridge, _owns_controller
    bridge = ext_controller
    _owns_controller = False


def _on_controller_event(message: dict):
    """將 RobotController 產生的事件轉發給所有 WebSocket 前端"""
    if _event_loop and _event_loop.is_running():
        asyncio.run_coroutine_threadsafe(manager.broadcast(message), _event_loop)


# ==================== FastAPI 應用與路由 ====================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global _event_loop
    _event_loop = asyncio.get_running_loop()
    bridge.add_listener(_on_controller_event)
    print("-> Web 伺服器已就緒，請在瀏覽器開啟: http://localhost:8000")
    yield
    bridge.remove_listener(_on_controller_event)
    if _owns_controller:
        bridge.disconnect()


app = FastAPI(
    title="Mechatronics-4 Web Control",
    description="Arduino 車輛控制與超音波即時監控系統",
    lifespan=lifespan,
)

templates = Jinja2Templates(directory=os.path.join(os.path.dirname(__file__), "templates"))


@app.get("/", response_class=HTMLResponse)
async def get_index(request: Request):
    """回傳自主導航與除錯主頁面 HTML (由 Jinja2 模板渲染)"""
    return templates.TemplateResponse(
        request=request,
        name="index.html",
        context={"title": "Mechatronics-4 機器人遠程監控與控制台"},
    )


@app.get("/legacy", response_class=HTMLResponse)
async def get_legacy(request: Request):
    """回傳傳統手動遙控與無人機手把模式頁面 HTML"""
    return templates.TemplateResponse(
        request=request,
        name="legacy.html",
        context={"title": "Mechatronics-4 機器人傳統遙控台 (Legacy)"},
    )


# ==================== HTTP REST API ====================
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


class RelayRequest(BaseModel):
    relay_id: int = 1
    state: bool = False


class DualMotorRequest(BaseModel):
    turn: int = 0
    speed: int = 0


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
    state = bridge.get_state()
    return {
        "success": True,
        "threshold_mm": state.get("safety_threshold_mm", 150),
        "threshold_cm": state.get("safety_threshold_cm", 15.0),
    }


@app.post("/api/stop_motion")
async def api_stop_motion():
    bridge.send_stop_motion()
    return {"success": True}


@app.post("/api/relay")
async def api_relay(req: RelayRequest):
    bridge.send_relay(req.relay_id, req.state)
    return {"success": True, "relay_id": req.relay_id, "state": req.state}


@app.post("/api/reset_imu")
async def api_reset_imu():
    bridge.send_reset_imu()
    return {"success": True}


@app.post("/api/dual_motor")
async def api_dual_motor(req: DualMotorRequest):
    bridge.send_dual_motor(req.turn, req.speed)
    return {"success": True, "turn": req.turn, "speed": req.speed}


@app.post("/api/ping")
async def api_ping():
    bridge.send_ping()
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
        "latest": bridge.get_state(),
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
                    "ports": bridge.get_ports(),
                })

    except WebSocketDisconnect:
        await manager.disconnect(websocket)
    except Exception:
        await manager.disconnect(websocket)


def start_web_server(ext_controller: Optional[RobotController] = None, host: str = "0.0.0.0", port: int = 8000):
    """以程式方式啟動 Web 伺服器 (可傳入外部 RobotController 實例供自動任務同步監控)"""
    import uvicorn

    if ext_controller is not None:
        attach_controller(ext_controller)
    uvicorn.run(app, host=host, port=port, log_level="warning")


if __name__ == "__main__":
    import uvicorn
    # 支援直接執行: python server.py
    # 亦支援: uvicorn server:app --host 0.0.0.0 --port 8000 --reload
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=True)
