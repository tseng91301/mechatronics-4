# Mechatronics-4 Autonomous Cleaning Robot System

> Language: **English** | [繁體中文](README.zh.md)

This project is an autonomous ground-cleaning robot platform combining a **Raspberry Pi 4B** (high-level strategy execution, OpenCV vision recognition, and Web debug console) with an **Arduino UNO R3** (low-level motor driving, IMU attitude integration, ultrasonic ranging, and hardware safety protection). The system supports **manual remote control**, **Python API scripting**, **multi-strategy autonomous navigation with single-step debugging**, and **hardware-free `/sim` simulator testing**.

> See [BLOCK_DIAGRAM.md](BLOCK_DIAGRAM.md) for the hardware wiring & data-flow diagram, and [CHANGELOG.zh.md](CHANGELOG.zh.md) for the changelog.

---

## 1. Core Vehicle Capabilities

### 1.1 Low-Level Motion & Sensing (Arduino UNO)
* **Dual-Motor Differential Drive (`motor.h`)**: Supports differential mixing (`turn` + `speed`) and individual motor testing, with a built-in `1000 ms` communication watchdog E-stop.
* **MPU-6050 IMU Closed-Loop Control (`imu.h`)**:
  * Gyroscope full-scale range configured to $\pm 1000^\circ/\text{s}$ to prevent ADC saturation clipping during rapid in-place spins (sign convention: Right / CW = `+Yaw`, Left / CCW = `-Yaw`).
  * Supports absolute target heading PD closed-loop navigation (`CMD 0x06`), configurable in-place turning speed limits (`CMD 0x0B`, default `90` PWM), and dynamic sinusoidal sweep oscillation (`CMD 0x07`).
* **Front / Right Ultrasonic Ranging & Hardware E-Stop (`Ultrasonic.h`)**:
  * Uses a 5-frame median filter with zero-order hold across transient dropouts, reporting filtered front and right distances in millimeters (`mm`).
  * Supports runtime configuration/query of the hardware front collision threshold (`CMD 0x09` / `0x0A`, default `150 mm`), triggering an immediate hardware emergency stop (`E-Stop`) when advancing closer than the threshold.
* **Vacuum Relay Control**: Toggles the vacuum cleaner motor via relay commands (`CMD 0x03`).

### 1.2 Manual Remote Control & Debug Monitoring (Web Platform)
* **Autonomous Debug Dashboard (`/`)**: Live camera feed, car-synced rotating 2D tactical radar (including the front safety boundary ring), polar heading-velocity joystick, sweep & safety threshold controls, and a raw hardware packet log terminal.
* **Legacy Manual Control (`/legacy`)**: Retains the classic dual virtual joysticks (Drone Mode 2 feel), D-Pad, and `WASD` keyboard remote-control UI.
* **Standalone Simulation Injector (`/sim`)**: Injects custom front/right ultrasonic distances, IMU yaw angles, and virtual/uploaded camera frames in Mock Mode so autonomous strategies run unmodified without physical hardware.

### 1.3 Autonomous Strategies & Confetti Vision Recognition
* **Multi-Strategy Selector & Step-by-Step Debugging (`strategies/`)**: Switch dynamically between registered strategies in the Web UI, start from any step, trigger immediate stop, or enable **Debug Step-by-Step Mode** (pauses automatically after each step).
* **Confetti Detection (`frame_recognize/`)**: Lightweight OpenCV HSV multi-color segmentation + geometric contour filtering, outputting normalized `(has_confetti, bbox, center)` percentages to visually align and vacuum confetti.

---

## 2. Project Directory & File Overview

| Directory / File | Description & Documentation |
| :--- | :--- |
| [arduino_code/main/](arduino_code/main/) | Arduino UNO R3 firmware (PlatformIO). Architecture, pinout & control loops in [arduino_code/README.md](arduino_code/README.md); source in [src/](arduino_code/main/src/):<br>• [main.cpp](arduino_code/main/src/main.cpp): Non-blocking main loop, command callbacks & hardware E-Stop<br>• [connection.h](arduino_code/main/src/connection.h): `0xAA ... 0x55` binary serial protocol parser<br>• [imu.h](arduino_code/main/src/imu.h): MPU-6050 integration, heading PD lock & sweep overlay<br>• [Ultrasonic.h](arduino_code/main/src/Ultrasonic.h): Non-blocking ultrasonic scheduler & 5-frame median filter<br>• [motor.h](arduino_code/main/src/motor.h): Differential drive mixing & PWM motor driver |
| [control/](control/) | Host Python control core & FastAPI Web platform (details & API reference in [control/README.md](control/README.md)):<br>• [robot_controller.py](control/robot_controller.py): Standalone vehicle API (`RobotController`) with serial state machine, 5Hz cruise heartbeat & Mock injector<br>• [server.py](control/server.py): FastAPI + WebSocket backend mounting `RobotController` and `StrategyRunner`<br>• [test_connection.py](control/test_connection.py): Interactive CLI tester & single-line live monitor<br>• [templates/index.html](control/templates/index.html): Autonomous debug & multi-strategy dashboard (`/`)<br>• [templates/sim.html](control/templates/sim.html): Standalone simulator sensor & frame injector (`/sim`)<br>• [templates/legacy.html](control/templates/legacy.html): Legacy dual-joystick manual UI (`/legacy`)<br>• [run_web.bash](control/run_web.bash): Standalone Web server launcher (uses shared root `venv/`) |
| [strategies/](strategies/) | Autonomous mission runner & arena strategy (architecture & authoring guide in [strategies/README.md](strategies/README.md)):<br>• [arena_strategy.md](strategies/arena_strategy.md): 16-step arena cleaning mission specification<br>• [arena_strategy.py](strategies/arena_strategy.py): 16-step strategy implementation<br>• [runner.py](strategies/runner.py): Background strategy engine (jump-to-step, E-stop, single-step debugging)<br>• [registry.py](strategies/registry.py): Auto-discovery & dynamic registration of multiple strategies<br>• [techniques/](strategies/techniques/): Reusable motion/vision/vacuum techniques (details in [techniques/README.md](strategies/techniques/README.md)) |
| [frame_recognize/](frame_recognize/) | OpenCV confetti vision module (details in [frame_recognize/README.md](frame_recognize/README.md)):<br>• [confetti_detector.py](frame_recognize/confetti_detector.py): Core `ConfettiDetector` class<br>• [detect_confetti_video.py](frame_recognize/detect_confetti_video.py): Video & webcam test runner |
| [circuit/](circuit/) | Hardware circuit wiring schematics ([base.fzz](circuit/base.fzz), [base_bb.png](circuit/base_bb.png), [base_bb.pdf](circuit/base_bb.pdf)) |
| [setup/](setup/) | Shared Python environment initialization, USB webcam `udev` binding & MediaMTX RTSP setup ([setup/README.md](setup/README.md)) |
| [records/](records/) | Destination directory for `output.mp4` recordings captured by `ffmpeg` in `start.sh` |
| [requirements.txt](requirements.txt) | Shared Python dependencies installed into the root `venv/` |
| [start.sh](start.sh) / [stop.sh](stop.sh) | One-click system startup and graceful shutdown scripts |

---

## 3. Quick Start & Shutdown

### 3.1 Start the Full System (Camera Stream + Recording + Web Console)
Run [start.sh](start.sh) from the repository root:
```bash
./start.sh
```
This launches three background services:
1. **MediaMTX**: RTSP (`rtsp://localhost:8554/live/stream`) and WebRTC (`8889`) media server.
2. **FFmpeg**: Captures `/dev/logi_camera` (640x360 @ 20fps), publishes to MediaMTX, and records to [records/output.mp4](records/).
3. **Web Console**: Starts Uvicorn via the shared root `venv/` (listening on `http://0.0.0.0:8000`).

Once running, open in your browser:
* **Main Dashboard**: `http://<RaspberryPi_IP>:8000/`
* **Simulator Injector**: `http://<RaspberryPi_IP>:8000/sim`
* **Legacy Remote UI**: `http://<RaspberryPi_IP>:8000/legacy`

### 3.2 Graceful Shutdown
Run [stop.sh](stop.sh):
```bash
./stop.sh
```
> Sends `SIGINT` to `ffmpeg` first so `output.mp4` finalizes cleanly without corruption, then stops `uvicorn` and `mediamtx` (falling back to `SIGKILL` if any process remains).

### 3.3 Start Only the Web Console (Offline / `/sim` Development)
If camera streaming is not required (e.g., motor bench tests or `/sim` simulation):
```bash
./control/run_web.bash
```
