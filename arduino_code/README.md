# Arduino UNO Low-Level Firmware (`arduino_code/`)

> Language: **English** | [繁體中文](README.zh.md)

This directory contains the **Arduino UNO R3** firmware responsible for motor PWM driving, sensor acquisition, IMU closed-loop control, hardware collision protection, and binary serial communication (built with **PlatformIO**).

---

## 1. Project Structure

```text
arduino_code/
├── README.md                 # English documentation (this file)
├── README.zh.md              # Traditional Chinese documentation
└── main/
    ├── platformio.ini        # PlatformIO config (atmelavr / board = uno / arduino)
    └── src/
        ├── main.cpp          # Main loop scheduler, command callbacks, E-Stop & Watchdog
        ├── connection.h      # 0xAA ... 0x55 serial state-machine parser & encoders
        ├── motor.h           # SingleMotor and DualMotor differential-drive mixing
        ├── imu.h             # MPU-6050 integration, PD heading control & sweep overlay
        └── Ultrasonic.h      # Non-blocking ultrasonic scheduler, 5-frame median filter & dropout hold
```

---

## 2. Hardware Pinout (`main.cpp`)

| Component | Object / Symbol | Arduino Pins | Notes |
| :--- | :--- | :--- | :--- |
| **Left Motor** | `leftMotor` (`SingleMotor`) | `Pin 3`, `Pin 9` | Dual PWM forward/reverse control (`-255 ~ +255`) |
| **Right Motor** | `rightMotor` (`SingleMotor`) | `Pin 10`, `Pin 11` | Dual PWM forward/reverse control (`-255 ~ +255`) |
| **Front Ultrasonic** | `front_ult` (`Ultrasonic`) | Trig: `Pin 5`, Echo: `Pin 6` | Default trigger interval `80 ms` |
| **Right Ultrasonic** | `right_ult` (`Ultrasonic`) | Trig: `Pin 4`, Echo: `Pin 7` | Default trigger interval `80 ms` |
| **Vacuum Relay** | `RELAY_PIN` | `Pin 2` | Active-LOW trigger (`LOW` = ON, `HIGH` = OFF; defaults to OFF on boot) |
| **MPU-6050 IMU** | `imu` (`IMUController`) | `A4` (SDA), `A5` (SCL) | I2C address `0x68` with `3000 μs` `Wire` timeout reset |
| **Serial Port** | `connection` (`SimpleConnection`) | `Pin 0` (RX), `Pin 1` (TX) / USB | Baudrate: **`9600 bps`** |

---

## 3. Control Principles & Implementation Logic

### 3.1 Main Loop & Dual Safety Protection ([main.cpp](main/src/main.cpp))
`loop()` runs completely non-blocking in the following order:
1. **`connection.service()`**: Parses incoming serial bytes and invokes registered `on*` callbacks.
2. **`imu.update()`**: Samples the MPU-6050 and runs closed-loop motion computation.
3. **Motor Output & Hardware Safety Guards**:
   * **Front Obstacle Emergency Stop (`E-Stop`)**: If `safetyThreshold_mm > 0` (default `150 mm`), the filtered front distance is valid and `< safetyThreshold_mm`, and the robot is driving forward (`actSpeed > 0`), it immediately executes `imu.stopMotion()`, `dualMotor.stop()`, and sends `[WARN] Front obstacle < ...mm! E-Stop`. *(Note: In-place turns with `actSpeed == 0` or reversing do not trigger E-Stop so the robot can turn away safely.)*
   * **Communication Watchdog**: Valid motion/keepalive commands (`onPing`, `onDualMotor`, `onSingleMotor`, `onSetHeading`, `onSetSweep`) refresh `lastMotorCmdTime`. If motors are active and **`1000 ms`** elapse without a new command, all motors stop immediately.
4. **`front_ult.service()` / `right_ult.service()`**: Triggers ultrasonic sensors periodically and updates median filters.
5. **Periodic Telemetry (`100 ms` / `10 Hz`)**: Transmits `CMD_ULTRASONIC_FRONT (0x11)`, `CMD_ULTRASONIC_RIGHT (0x12)`, and `CMD_IMU_DATA (0x13)`.

### 3.2 Differential Drive Mixing ([motor.h](main/src/motor.h))
* `DualMotor::drive(int turn, int speed)` computes:
  $$\text{leftSpeed} = \text{constrain}(\text{speed} - \text{turn},\ -255,\ 255)$$
  $$\text{rightSpeed} = \text{constrain}(\text{speed} + \text{turn},\ -255,\ 255)$$
* **Polarity**: `turn > 0` slows/reverses the left wheel and speeds up the right wheel (**CCW / Left Turn**); `turn < 0` speeds up the left wheel and slows/reverses the right wheel (**CW / Right Turn**).

### 3.3 IMU Integration, PD Closed-Loop & Sweep Overlay ([imu.h](main/src/imu.h))
* **Sensor Configuration**:
  * Gyroscope full-scale range is set to **`±1000 deg/s`** (`GYRO_CONFIG 0x1B = 0x10`, `32.768 LSB/(deg/s)`), eliminating ADC saturation/clipping and heading drift during rapid differential spins exceeding `±250 deg/s`.
  * Accelerometer is set to **`±2g`** (`16384 LSB/g`) for `Pitch` and `Roll`.
* **Boot Calibration & Integration**:
  * `calibrateGyro()` averages 100 stationary samples for `_gyroZOffset`.
  * `update()` standardizes angular velocity to **CW / Right Turn = `+Yaw`**, **CCW / Left Turn = `-Yaw`**: `rateZ = -(((float)gz / 32.768f) - _gyroZOffset)`, applies a `< 0.15 deg/s` deadband, integrates over `dt`, and wraps `_yaw` to `[-180°, +180°)`.
* **Control Modes & PD Law**:
  * **`MODE_MANUAL`**: Outputs raw `turn`/`speed`; when `autoCorrect` is enabled during straight driving (`turn == 0, speed != 0`), locks heading and clamps correction to `±60` PWM.
  * **`MODE_HEADING`**: Computes shortest-arc angular error between `targetHeading` and `_yaw` using PD control (`_kp = 2.3`, `_kd = 0.12`):
    $$\text{correction} = -\big(K_p \cdot \text{error} - K_d \cdot \text{gyroZ}\big)$$
    * When `targetSpeed == 0`, turns in place until `|error| <= 3.0°`. When `targetSpeed != 0` and `|error| > 20°` (with sweep off), aligns in place first before driving forward.
    * Turning PWM is clamped to `±_maxTurnSpeed` (default **`90` PWM**, configurable `40~220` via `CMD 0x0B`), with `_minTurnSpeed = 38` PWM to overcome static friction.
  * **Dynamic Sweep Overlay (`setSweep`)**: Adds a sinusoidal heading offset `sweepOffset = amplitude * sin(2π * elapsed / periodMs)` (`periodMs >= 500 ms`) on top of either `MODE_HEADING` or `MODE_MANUAL`.

### 3.4 Ultrasonic Filtering & Dropout Hold ([Ultrasonic.h](main/src/Ultrasonic.h))
* Trigger period defaults to **`80 ms`** with a `30000 μs` `pulseIn` timeout (~`500 cm`) to prevent blocking `loop()`.
* Valid readings (`1.0 < distance < 400.0 cm`) enter a **5-frame ring buffer** and output the sorted median (`filteredDistance`).
* **Zero-Order Hold on Dropouts**: Transient 1–2 frame timeouts retain the previous `filteredDistance`; only after **`1000 ms`** of continuous invalid echoes is `filteredDistance` reset to `-1.0f`.

### 3.5 Binary Protocol & Command Registry ([connection.h](main/src/connection.h))
* Packet frame: `[0xAA] [CMD] [Payload (0~32 bytes, Big-Endian)] [0x55]`, supporting up to `16` registered commands (`12` registered: `0x00 ~ 0x0B`).
* `CMD_IMU_DATA (0x13)` `modeState` byte bitmask: `bit 0` = `MODE_HEADING`, `bit 1` = `Sweep` enabled.

---

## 4. Build & Upload (PlatformIO)

```bash
cd arduino_code/main

# Compile only
pio run

# Compile and flash to connected Arduino UNO
pio run --target upload
```

> Before uploading, ensure neither `server.py` nor `test_connection.py` is holding `/dev/ttyUSB0` / `/dev/ttyACM0` (run `./stop.sh` at the repo root or disconnect from the Web UI).
