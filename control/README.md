# Vehicle Control Core & Web Backend (`control/`)

> Language: **English** | [繁體中文](README.zh.md)

This directory hosts the Raspberry Pi **low-level hardware controller (`RobotController`)**, **FastAPI Web backend (`server.py`)**, **three Web interfaces (`/`, `/sim`, `/legacy`)**, and **interactive CLI debugger (`test_connection.py`)**.

---

## 1. Architecture & Decoupled Design

To let autonomous strategy scripts and the Web debugging dashboard run concurrently without conflicting over the single OS serial port handle, control and Web serving are strictly separated:

```text
control/
├── README.md             # English documentation & API reference (this file)
├── README.zh.md          # Traditional Chinese documentation
├── robot_controller.py   # [Core Controller] Pure Python vehicle/sensor API (zero Web dependencies)
├── server.py             # [Web Backend] FastAPI + WebSocket + mounted StrategyRunner
├── test_connection.py    # [CLI Tool] Interactive menu & live dashboard using RobotController
├── run_web.bash          # Standalone Web server launcher (uses shared root venv/)
└── templates/
    ├── index.html        # Main page (/): Autonomous debug dashboard, polar joystick & strategy console
    ├── sim.html          # Simulator page (/sim): Ultrasonic, IMU & virtual camera injector
    └── legacy.html       # Legacy page (/legacy): Dual virtual joysticks, D-Pad & keyboard control
```

### Shared Instance Pattern (`attach_controller`)
* Only **one `RobotController` instance** owns the serial port or Mock worker at runtime.
* `server.py` subscribes to `bridge.add_listener(...)` to relay hardware telemetry to WebSocket clients, while `StrategyRunner` commands the exact same `bridge` instance—allowing **live Web monitoring while autonomous strategies execute**.

---

## 2. Control Principles & Core Logic

### 2.1 Binary Serial Protocol (`0xAA ... 0x55`)
All multi-byte integers are Big-Endian (`>`), wrapped in `[0xAA] [CMD] [Payload...] [0x55]`:

| Dir | CMD | Constant | Payload Format | Description |
| :--- | :---: | :--- | :--- | :--- |
| **TX** | `0x00` | `CMD_PING` | `0 bytes` | Connection ping |
| **TX** | `0x01` | `CMD_DUAL_MOTOR` | `4 bytes` (`>hh`: `turn`, `speed`) | Manual differential drive (`-255 ~ +255`) |
| **TX** | `0x02` | `CMD_SINGLE_MOTOR` | `3 bytes` (`>Bh`: `motor_id`, `speed`) | Single motor test (`0`: Left, `1`: Right) |
| **TX** | `0x03` | `CMD_RELAY` | `2 bytes` (`>BB`: `relay_id`, `state`) | Relay switch (`1`: Vacuum motor) |
| **TX** | `0x04` | `CMD_SET_AUTO_CORRECT` | `1 byte` (`>B`: `enable`) | Manual straight-line gyro correction toggle |
| **TX** | `0x05` | `CMD_RESET_IMU_YAW` | `0 bytes` | Reset IMU yaw heading to `0°` |
| **TX** | `0x06` | `CMD_SET_HEADING` | `4 bytes` (`>hh`: `yaw_x10`, `speed`) | Target heading (`-1800~1800`) & forward speed |
| **TX** | `0x07` | `CMD_SET_SWEEP` | `5 bytes` (`>BhH`: `en`, `amp_x10`, `period_ms`) | Enable/configure sinusoidal sweep oscillation |
| **TX** | `0x08` | `CMD_STOP_MOTION` | `0 bytes` | Brake motors & clear heading/sweep lock |
| **TX** | `0x09` | `CMD_SET_SAFETY_THRESH` | `2 bytes` (`>H`: `threshold_mm`) | Set front hardware E-Stop distance (`0` = off) |
| **TX** | `0x0A` | `CMD_GET_SAFETY_THRESH` | `0 bytes` | Query current Arduino safety threshold |
| **TX** | `0x0B` | `CMD_SET_TURN_SPEED` | `2 bytes` (`>h`: `turn_speed`) | Set max turning speed limit (`40~220` PWM) |
| **RX** | `0x11` | `CMD_ULTRASONIC_FRONT` | `4 bytes` (`>i`: `mm`) | Front ultrasonic distance (`-1` = out of range) |
| **RX** | `0x12` | `CMD_ULTRASONIC_RIGHT` | `4 bytes` (`>i`: `mm`) | Right ultrasonic distance (`-1` = out of range) |
| **RX** | `0x13` | `CMD_IMU_DATA` | `7 bytes` (`>hhhB`: `y, p, r, mode`) | IMU angles (`x10`) & mode bitmask |
| **RX** | `0x14` | `CMD_SAFETY_THRESH_DATA` | `2 bytes` (`>H`: `mm`) | Reported safety threshold confirmation |
| **RX** | `0x20` | `CMD_LOG_MSG` | Variable UTF-8 string | Arduino status text or `E-Stop` warning |

### 2.2 Receive State Machine (`_rx_worker` / `_process_byte`)
* A background thread parses incoming bytes via a 4-state FSM:
  `STATE_WAIT_START (0xAA)` → `STATE_WAIT_CMD` → `STATE_WAIT_DATA` → `STATE_WAIT_END (0x55)`.
* Fixed payload lengths are looked up from `cmd_lengths`; variable-length `CMD_LOG_MSG (0x20)` reads until `END_BYTE (0x55)`.

### 2.3 5Hz Cruise Heartbeat (`_heartbeat_worker`)
* Arduino enforces a **1000 ms communication watchdog** to stop runaway motors on disconnect.
* Whenever `send_set_heading(target_yaw, speed)` is called with `speed != 0`, `RobotController` records `self.active_heading` and quietly re-transmits `CMD_SET_HEADING` every **200 ms (5Hz)** (`quiet=True`) to sustain cruising.
* Calling `send_stop_motion()`, `send_dual_motor(0, 0)`, or receiving a hardware `E-Stop` immediately clears `active_heading`.

### 2.4 Thread Safety & Observer Events
* **Three dedicated locks**:
  * `write_lock`: Serializes `ser.write(...)` across threads.
  * `_state_lock`: Protects `latest_data` snapshots and `active_heading`.
  * `_listeners_lock`: Guards event listener callbacks.
* **Event Notifications (`notify`)**: Emits `ultrasonic`, `imu`, `safety_threshold`, `relay`, `e_stop`, `pong`, `log`, and `sim_state`.

### 2.5 Mock Mode & `/sim` Injection (`_mock_worker`)
* Runs at **10Hz (`0.1s`)**:
  * **Exact Manual Values by Default**: Random sin/cos waves are disabled (`auto_wave=False`) so `/sim` overrides persist deterministically.
  * **Auto Kinematics (`auto_kinematics=True`)**: Smoothly turns mock `yaw` toward `mock_target_yaw` at a rate scaled by `turn_speed` when heading commands arrive.
  * **Simulated Hardware `E-Stop`**: Triggers `e_stop` if `safety_threshold_mm > 0`, `front_mm < safety_threshold_mm`, and `target_speed > 0`.
  * **Camera Frame Buffer (`inject_camera_frame`)**: Supplies BGR frames directly to `VisionSource` during Mock Mode.

---

## 3. API Function Entries Reference

### 3.1 Python Class API: `RobotController` ([robot_controller.py](robot_controller.py))

```python
from control.robot_controller import RobotController

robot = RobotController(verbose=False)
```

#### (A) Connection & Simulation Lifecycle
| Function | Signature / Return | Description |
| :--- | :--- | :--- |
| `get_ports()` | `-> List[dict]` | List available serial ports (`device`, `description`) |
| `connect(port, baudrate=9600, wait_reset_sec=1.8)` | `-> bool` | Open serial connection & start RX/heartbeat threads |
| `disconnect()` | `-> None` | Stop motors safely and close serial/mock threads |
| `reconnect()` | `-> bool` | Reconnect active serial port or restart Mock Mode |
| `start_mock()` / `stop_mock()` | `-> None` | Start or stop hardware-free Mock Mode |
| `add_listener(callback)` / `remove_listener(callback)` | `callback: Callable[[dict], None]` | Subscribe/unsubscribe to live telemetry & log events |

#### (B) Thread-Safe Sensor & State Getters
| Function | Return Type | Description |
| :--- | :--- | :--- |
| `get_state()` | `dict` | Full state snapshot (connection, ultrasonic, IMU, mode, safety, E-Stop) |
| `get_ultrasonic()` | `dict` | Returns `{"front_mm", "front_cm", "right_mm", "right_cm"}` |
| `get_front_mm()` / `get_front_cm()` | `int` / `float` | Front ultrasonic distance (`-1` if invalid/out of range) |
| `get_right_mm()` / `get_right_cm()` | `int` / `float` | Right ultrasonic distance (`-1` if invalid/out of range) |
| `get_imu()` | `dict` | Returns `yaw, pitch, roll, mode, mode_name, sweep_enabled, auto_correct` |
| `get_yaw()` | `float` | Current IMU yaw angle (`-180.0° ~ +180.0°`) |
| `get_safety_threshold_mm()` | `int` | Current cached hardware E-Stop distance threshold (`mm`) |
| `is_e_stopped()` / `clear_e_stop()` | `bool` / `None` | Query or reset the hardware E-Stop flag |

#### (C) Motion Control Commands & Strategy Aliases
| Low-Level Method (`send_*`) | High-Level Alias | Description |
| :--- | :--- | :--- |
| `send_set_heading(target_yaw, speed=0, quiet=False)` | `set_heading(target_yaw, speed=0)` | Navigate toward `target_yaw` at `speed` (`0` = turn in place) |
| `send_set_sweep(enable, amp=20.0, period_ms=2000)` | `set_sweep(enable, amp=20.0, period_ms=2000)` | Toggle/configure sinusoidal sweep oscillation |
| `send_stop_motion()` | `stop()` | Stop motors, unlock heading/sweep, halt heartbeat |
| `send_relay(relay_id, state)` | `set_vacuum(enable, relay_id=1)` | Control relay (`1` = vacuum motor) |
| `send_reset_imu()` | `reset_yaw()` | Zero IMU yaw heading (`0.0°`) |
| `send_set_turn_speed(turn_speed)` | `set_turn_speed(turn_speed)` | Set in-place turn PWM cap (`40~220`) |
| `send_set_safety_threshold(threshold_mm)` | `set_safety_threshold(threshold_mm)` | Set front E-Stop threshold (`mm`, `0` = off) |
| `send_get_safety_threshold()` | — | Request current safety threshold from Arduino |
| `send_dual_motor(turn, speed, is_heartbeat=False)` | — | Manual differential drive command |
| `send_single_motor(motor_id, speed)` | — | Single motor test command |
| `send_auto_correct(enable)` | — | Toggle manual straight-line correction |
| `send_ping()` / `send_raw_bytes(raw, log_desc)` | — | Send ping or raw hexadecimal packet |

#### (D) Simulator Injection API (Mock Mode)
| Function | Description |
| :--- | :--- |
| `inject_ultrasonic(front_mm=None, right_mm=None)` | Override simulated front/right ultrasonic distances (`mm`) |
| `inject_imu(yaw=None, pitch=None, roll=None)` | Override simulated IMU angles (`deg`) |
| `set_sim_options(auto_kinematics=None, auto_wave=None)` | Toggle auto yaw convergence or random sensor waves |
| `inject_camera_frame(frame)` / `get_injected_frame()` | Write/read simulated BGR camera frame & timestamp |
| `get_sim_state()` | Full simulator input & motor command output snapshot |

---

### 3.2 Web Backend REST API ([server.py](server.py))

#### (A) Pages & Connection
* `GET /`: Autonomous debug & multi-strategy dashboard ([index.html](templates/index.html))
* `GET /sim`: Standalone simulator injector ([sim.html](templates/sim.html))
* `GET /legacy`: Legacy remote control UI ([legacy.html](templates/legacy.html))
* `GET /api/ports`, `POST /api/connect`, `POST /api/disconnect`, `GET /api/status`

#### (B) Vehicle Control Endpoints
* `POST /api/heading` (`{"target_yaw", "speed"}`)
* `POST /api/sweep` (`{"enable", "amplitude", "period_ms"}`)
* `POST /api/stop_motion`
* `POST /api/turn_speed` (`{"turn_speed"}`)
* `POST /api/safety_threshold` / `GET /api/safety_threshold` (`{"threshold_mm"}`)
* `POST /api/relay` (`{"relay_id", "state"}`)
* `POST /api/reset_imu`, `POST /api/dual_motor`, `POST /api/ping`

#### (C) Multi-Strategy & Step-Debug Endpoints
* `GET /api/strategy/list`: List all auto-discovered strategies
* `POST /api/strategy/select` (`{"strategy_id"}`): Switch active strategy
* `POST /api/strategy/reload`: Rescan `strategies/` directory
* `GET /api/strategy/status`: Current status, steps list & substate
* `POST /api/strategy/start` (`{"strategy_id", "start_step", "enable_vision", "rtsp_url", "reset_imu", "step_mode"}`)
* `POST /api/strategy/step_next` / `POST /api/strategy/continue` / `POST /api/strategy/step_mode` / `POST /api/strategy/stop`

#### (D) Simulator Endpoints (`/sim`)
* `GET /api/sim/state`
* `POST /api/sim/sensors` (`{"front_mm", "right_mm", "yaw", "pitch", "roll", "auto_kinematics", "auto_wave"}`)
* `POST /api/sim/frame` (`{"image_base64"}`): Inject frame and return live `ConfettiDetector` output

---

### 3.3 WebSocket Interface (`/ws`)
* **Client `action` messages**:
  * System/Low-level: `connect`, `disconnect`, `reconnect`, `refresh_ports`, `ping`, `send_hex`
  * Motion/Hardware: `dual_motor`, `single_motor`, `stop`, `set_heading`, `set_sweep`, `stop_motion`, `set_turn_speed`, `set_safety_threshold`, `get_safety_threshold`, `relay`, `set_auto_correct`, `reset_imu`
  * Strategy: `strategy_start`, `strategy_select`, `strategy_reload`, `strategy_step_next`, `strategy_continue`, `strategy_set_step_mode`, `strategy_stop`, `get_strategy_status`
  * Simulator: `sim_get_state`, `sim_set_sensors`, `sim_set_frame`
* **Server `type` broadcasts**: `status`, `ports`, `ultrasonic`, `imu`, `safety_threshold`, `relay`, `e_stop`, `strategy_status`, `sim_state`, `sim_vision_result`, `log`.

---

## 4. Running the Module

Ensure the shared root `venv/` has been initialized per [setup/README.md](../setup/README.md):

1. **Web Server Only (`/`, `/sim`, `/legacy`)**:
   ```bash
   ./control/run_web.bash
   ```
   Or from the repository root:
   ```bash
   venv/bin/python -m uvicorn control.server:app --host 0.0.0.0 --port 8000 --reload
   ```
2. **Full System (Camera RTSP + Recording + Web)**: Run [../start.sh](../start.sh) from the root directory.
3. **Interactive CLI Tester**:
   ```bash
   venv/bin/python control/test_connection.py
   ```
