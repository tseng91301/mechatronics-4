# Reusable Strategy Techniques (`strategies/techniques/`)

> Language: **English** | [繁體中文](README.zh.md)

This module encapsulates reusable, interruptible, thread-safe **atomic motion, navigation, visual servoing, and vacuum-sweep primitives** built atop [RobotController](../../control/robot_controller.py) and [ConfettiDetector](../../frame_recognize/confetti_detector.py).

---

## 1. File Overview

| File | Exported Symbols | Role |
| :--- | :--- | :--- |
| [context.py](context.py) | `StrategyContext`, `wrap_angle_deg`, `angle_diff_deg`, `clamp` | Shared execution context, angle wrapping, interruptible sleep, hardware E-Stop guard |
| [navigation.py](navigation.py) | `turn_to_heading`, `drive_blind_imu`, `drive_until_front_distance`, `wall_follow_right` | IMU turns, blind straight entry, front ultrasonic stop, right-wall following |
| [vision_align.py](vision_align.py) | `VisionSource`, `align_to_confetti` | Asynchronous RTSP/mock frame reader & confetti visual alignment |
| [vacuum_sweep.py](vacuum_sweep.py) | `sweep_and_vacuum` | Dynamic sinusoidal sweep + vacuum motor cleaning |
| [__init__.py](__init__.py) | All symbols above | Package re-export entry point |

---

## 2. Technique Details & Control Principles

### 2.1 `StrategyContext` ([context.py](context.py))
* **`ctx.is_stopped()`**: Checks `stop_event` so loops abort cleanly on emergency stop.
* **`ctx.sleep(seconds, step=0.05)`**: Uses `threading.Event.wait()` instead of blocking `time.sleep()`, waking within `50 ms` upon stop.
* **`ctx.set_substate(text)`**: Pushes live sub-action descriptions to the Web UI and log stream.
* **`ctx.temporarily_disable_safety(restore_mm=None)`**: Context manager that sets Arduino E-Stop threshold to `0` upon entry and **guarantees restoration** to the original threshold in `finally`—essential when entering narrow gates or vacuuming close to confetti/walls.
* **Angle Normalization (`wrap_angle_deg`, `angle_diff_deg`)**: Wraps all headings to `[-180°, +180°)` to ensure shortest-path rotation across the $\pm 180^\circ$ wrap boundary.

---

### 2.2 Navigation & Ultrasonic Techniques ([navigation.py](navigation.py))

#### (1) `turn_to_heading(ctx, target_yaw, tolerance_deg=4.0, settle_count=3, timeout_sec=6.0) -> bool`
* Commands `robot.set_heading(target_yaw, 0)` for Arduino in-place PD turning.
* Requires `settle_count` (`3` consecutive `50 ms` samples = `150 ms`) within `±4.0°` before stopping, avoiding premature completion on overshoot.

#### (2) `drive_blind_imu(ctx, heading_yaw, speed=140, duration_sec=3.0, disable_safety=True) -> bool`
* Drives straight using IMU heading lock only for `duration_sec` seconds, ignoring ultrasonic obstacles and temporarily disabling Arduino E-Stop so entrance frames or starting barriers do not trigger false stops.

#### (3) `drive_until_front_distance(ctx, heading_yaw, speed=130, stop_front_cm=15.0, confirm_frames=2, max_sec=None, on_tick=None) -> str`
* Drives straight until `0 < front_cm <= stop_front_cm` for `confirm_frames` consecutive samples (or Arduino hardware E-Stop triggers). Returns `'REACHED'`, `'TIMEOUT'`, or `'STOPPED'`.
* **Mid-cruise Interruption (`on_tick`)**: Invoked every `50 ms`; if `on_tick()` returns `"CONFETTI_HANDLED"`, resets hit counters and resumes `set_heading(heading_yaw, speed)` toward the wall.

#### (4) `wall_follow_right(ctx, base_heading_yaw, speed=120, target_right_cm=15.0, stop_front_cm=15.0, stop_right_gte_cm=None, lost_wall_timeout_sec=0.6, kp_wall=1.2, max_angle_adjust=15.0, confirm_frames=2, max_sec=None, on_tick=None) -> str`
* **Outer-Loop Wall P-Controller + Inner-Loop IMU PD**:
  * Following the project sign convention (**CW/Right = `+Yaw`**, **CCW/Left = `-Yaw`**), computes `err_cm = right_cm - target_right_cm`:
    * Too far from right wall (`err_cm > 0`) → steers slightly right (`+Yaw`).
    * Too close to right wall (`err_cm < 0`) → steers slightly left (`-Yaw`).
  * Applies a `±1.5 cm` deadband to prevent weaving, clamps heading offset to `±15.0°`, and ignores readings `<= 0` or `>= 60.0 cm` so openings do not cause wild swerves.
* **Open-Right Termination (`stop_right_gte_cm`)**: Returns `'RIGHT_OPEN'` either when `right_cm >= stop_right_gte_cm` for `confirm_frames`, OR when right ultrasonic continuously reads `-1` (no echo) for `>= lost_wall_timeout_sec` (`0.6 s`).

---

### 2.3 Confetti Vision & Alignment ([vision_align.py](vision_align.py))

#### (1) `VisionSource`
* **Real Mode**: Captures frames from `rtsp://localhost:8554/live/stream` (`cv2.CAP_FFMPEG`, `CAP_PROP_BUFFERSIZE = 1`) in a background thread so it never conflicts with `ffmpeg` holding `/dev/logi_camera`.
* **Mock Mode (`frame_provider`)**: Reads injected BGR frames from `robot.get_injected_frame` and feeds them through the exact same `ConfettiDetector`.
* Frames older than `3.0 s` expire automatically (`detect_latest()` returns `(False, None, None)`).

#### (2) `align_to_confetti(ctx, vision, center_tolerance_x=8.0, target_y_pct=68.0, approach_speed=85, align_gain=0.45, max_duration_sec=8.0, lost_timeout_sec=0.8) -> (bool, float)`
* **Two-Stage Percentage Visual Servoing**:
  1. Computes horizontal error `err_x = center.x - 50.0` and shifts target yaw by `clamp(err_x * align_gain, -25°, +25°)`.
  2. Rotates in place (`speed = 0`) when `|err_x| > 18%`, then approaches slowly (`approach_speed = 85`) until `|err_x| <= 8%` and `center.y >= 68%`.
  3. **Bottom Blind-Zone Handling**: If confetti disappears after already reaching close (`last_seen_y >= 60%`), treats it as entering the under-chassis suction zone and returns `True`.

---

### 2.4 Vacuum Sweep ([vacuum_sweep.py](vacuum_sweep.py))

#### `sweep_and_vacuum(ctx, heading_yaw, speed=85, amp_deg=20.0, period_ms=1800, stop_front_cm=15.0, duration_sec=4.0, disable_safety=True) -> bool`
* Temporarily disables hardware E-Stop (`temporarily_disable_safety()`), activates vacuum relay (`set_vacuum(True)`), enables sinusoidal sweeping (`set_sweep(True, amp_deg, period_ms)`), and creeps forward until `duration_sec` elapses or `front_cm <= stop_front_cm`.
* Wrapped in `try ... finally` to guarantee disabling sweep, vacuum, and motors on completion or abort.

---

## 3. Authoring a New Technique

Follow these 5 design rules when adding a reusable technique:
1. **First parameter must be `ctx: StrategyContext`**.
2. **Keep loops interruptible**: Check `while not ctx.is_stopped():` and use `ctx.sleep(...)` instead of blocking `time.sleep(...)`.
3. **Guarantee cleanup in `try ... finally`**: Always stop motors and turn off relays/sweep on exit.
4. **Use `quiet=True` inside high-rate loops**: Call `ctx.robot.send_set_heading(yaw, speed, quiet=True)` for 10–20Hz adjustments so logs stay readable.
5. **Export in `__init__.py`**: Add the new function to [strategies/techniques/\_\_init\_\_.py](__init__.py).

### Example: Timed Reverse Technique

```python
def reverse_for_seconds(
    ctx: StrategyContext,
    heading_yaw: float,
    speed: int = 100,
    duration_sec: float = 1.0,
) -> bool:
    """Drive backward along heading_yaw for duration_sec seconds."""
    heading_yaw = wrap_angle_deg(heading_yaw)
    reverse_speed = -abs(int(speed))
    ctx.set_substate(f"Reversing along {heading_yaw:.1f}° for {duration_sec:.1f}s")

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
