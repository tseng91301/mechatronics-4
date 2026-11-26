#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
導航與超音波移動組合技 (Navigation Techniques)
包含：
1. 原地精準轉向 (turn_to_heading)
2. 純 IMU 盲走進入 (drive_blind_imu)
3. 直走直到前方超音波小於定義距離 (drive_until_front_distance)
4. 沿右牆修正直走直到前方小於一定距離或右邊開闊 (wall_follow_right)
"""

import time
from typing import Callable, Optional

from .context import StrategyContext, angle_diff_deg, clamp, wrap_angle_deg


def turn_to_heading(
    ctx: StrategyContext,
    target_yaw: float,
    tolerance_deg: float = 4.0,
    settle_count: int = 3,
    timeout_sec: float = 6.0,
) -> bool:
    """
    原地旋轉對齊目標航向角 (target_yaw: -180 ~ +180)。
    對齊穩定後停止馬達並回傳 True；若中途被停止則回傳 False。
    """
    target_yaw = wrap_angle_deg(target_yaw)
    ctx.set_substate(f"原地旋轉對齊目標航向 {target_yaw:.1f}°")
    ctx.robot.set_heading(target_yaw, 0)

    start_t = time.monotonic()
    in_tol_count = 0

    while not ctx.is_stopped():
        cur_yaw = ctx.robot.get_yaw()
        err = abs(angle_diff_deg(target_yaw, cur_yaw))

        if err <= tolerance_deg:
            in_tol_count += 1
            if in_tol_count >= settle_count:
                ctx.robot.stop()
                ctx.set_substate(f"對齊完成 (目前 Yaw={cur_yaw:.1f}°)")
                return True
        else:
            in_tol_count = 0

        if time.monotonic() - start_t >= timeout_sec:
            ctx.robot.stop()
            ctx.set_substate(f"旋轉對齊逾時 (目前 Yaw={cur_yaw:.1f}°, 目標 {target_yaw:.1f}°)")
            return True

        if not ctx.sleep(0.05):
            break

    ctx.robot.stop()
    return False


def drive_blind_imu(
    ctx: StrategyContext,
    heading_yaw: float,
    speed: int = 140,
    duration_sec: float = 3.0,
    disable_safety: bool = True,
) -> bool:
    """
    純 IMU 航向鎖定直行 (不參考超音波數值)。
    預設會暫時關閉 Arduino 硬體防撞門檻，結束或中斷後自動恢復。
    """
    heading_yaw = wrap_angle_deg(heading_yaw)
    ctx.set_substate(f"純 IMU 直行 {duration_sec:.1f} 秒 (航向 {heading_yaw:.1f}°, 速度 {speed})")

    def _run() -> bool:
        ctx.robot.set_heading(heading_yaw, speed)
        start_t = time.monotonic()
        while not ctx.is_stopped():
            elapsed = time.monotonic() - start_t
            if elapsed >= duration_sec:
                ctx.robot.stop()
                return True
            if not ctx.sleep(0.05):
                break
        ctx.robot.stop()
        return False

    if disable_safety:
        with ctx.temporarily_disable_safety():
            return _run()
    return _run()


def drive_until_front_distance(
    ctx: StrategyContext,
    heading_yaw: float,
    speed: int = 130,
    stop_front_cm: float = 15.0,
    confirm_frames: int = 2,
    max_sec: Optional[float] = None,
    on_tick: Optional[Callable[[], Optional[str]]] = None,
) -> str:
    """
    組合技 1：直走直到前方超音波距離 <= stop_front_cm (僅考慮前方超音波與 IMU)。
    可選配 on_tick 回呼 (例如行進中偵測紙屑)；若 on_tick 回傳 'CONFETTI_HANDLED' 則恢復直行。
    回傳值: 'REACHED' | 'TIMEOUT' | 'STOPPED'
    """
    heading_yaw = wrap_angle_deg(heading_yaw)
    ctx.set_substate(f"直線前進直到前方 <= {stop_front_cm:.1f} cm (航向 {heading_yaw:.1f}°)")
    ctx.robot.set_heading(heading_yaw, speed)

    start_t = time.monotonic()
    hit_count = 0

    while not ctx.is_stopped():
        # 行進中插斷檢查 (例如影像辨識紙屑)
        if on_tick is not None:
            tick_res = on_tick()
            if ctx.is_stopped():
                break
            if tick_res == "CONFETTI_HANDLED":
                hit_count = 0
                ctx.set_substate(f"繼續直線前進直到前方 <= {stop_front_cm:.1f} cm (航向 {heading_yaw:.1f}°)")
                ctx.robot.set_heading(heading_yaw, speed)

        front_cm = ctx.robot.get_front_cm()
        if 0 < front_cm <= stop_front_cm:
            hit_count += 1
            if hit_count >= confirm_frames:
                ctx.robot.stop()
                ctx.set_substate(f"已達前方目標距離 ({front_cm:.1f} cm <= {stop_front_cm:.1f} cm)")
                return "REACHED"
        else:
            hit_count = 0

        if ctx.robot.is_e_stopped():
            ctx.robot.stop()
            ctx.robot.clear_e_stop()
            ctx.set_substate(f"觸發前方硬體防撞停靠 (front={front_cm:.1f} cm)")
            return "REACHED"

        if max_sec is not None and (time.monotonic() - start_t) >= max_sec:
            ctx.robot.stop()
            ctx.set_substate(f"直行已達時間上限 {max_sec:.1f} 秒")
            return "TIMEOUT"

        if not ctx.sleep(0.05):
            break

    ctx.robot.stop()
    return "STOPPED"


def wall_follow_right(
    ctx: StrategyContext,
    base_heading_yaw: float,
    speed: int = 120,
    target_right_cm: float = 15.0,
    stop_front_cm: Optional[float] = 15.0,
    stop_right_gte_cm: Optional[float] = None,
    lost_wall_timeout_sec: float = 0.6,
    kp_wall: float = 1.2,
    max_angle_adjust: float = 15.0,
    confirm_frames: int = 2,
    max_sec: Optional[float] = None,
    on_tick: Optional[Callable[[], Optional[str]]] = None,
) -> str:
    """
    組合技 2：沿著右邊直走，使用右方超音波動態微調航向以維持與右牆約 target_right_cm 距離。
    停止條件支援：
      - 前方超音波 <= stop_front_cm (回傳 'REACHED')
      - 右方超音波 >= stop_right_gte_cm 或連續超出量測範圍(-1)超過 lost_wall_timeout_sec (回傳 'RIGHT_OPEN')
    回傳值: 'REACHED' | 'RIGHT_OPEN' | 'TIMEOUT' | 'STOPPED'
    """
    base_heading_yaw = wrap_angle_deg(base_heading_yaw)
    if stop_right_gte_cm is not None:
        ctx.set_substate(
            f"沿右牆前進 (維持 {target_right_cm:.0f}cm) 直到右方 >= {stop_right_gte_cm:.0f}cm 或無牆"
        )
    else:
        ctx.set_substate(
            f"沿右牆前進 (維持 {target_right_cm:.0f}cm) 直到前方 <= {stop_front_cm:.1f}cm"
        )

    ctx.robot.set_heading(base_heading_yaw, speed)
    last_sent_yaw = base_heading_yaw
    last_send_time = time.monotonic()

    start_t = time.monotonic()
    front_hit_count = 0
    right_open_count = 0
    lost_wall_start: Optional[float] = None

    while not ctx.is_stopped():
        # 行進中插斷檢查 (例如偵測紙屑)
        if on_tick is not None:
            tick_res = on_tick()
            if ctx.is_stopped():
                break
            if tick_res == "CONFETTI_HANDLED":
                front_hit_count = 0
                right_open_count = 0
                lost_wall_start = None
                ctx.set_substate(f"繼續沿右牆前進 (基準航向 {base_heading_yaw:.1f}°)")
                ctx.robot.set_heading(base_heading_yaw, speed)
                last_sent_yaw = base_heading_yaw
                last_send_time = time.monotonic()

        front_cm = ctx.robot.get_front_cm()
        right_cm = ctx.robot.get_right_cm()

        # 1. 檢查前方停止條件
        if stop_front_cm is not None:
            if 0 < front_cm <= stop_front_cm:
                front_hit_count += 1
                if front_hit_count >= confirm_frames:
                    ctx.robot.stop()
                    ctx.set_substate(f"沿牆到達前方目標距離 ({front_cm:.1f} cm)")
                    return "REACHED"
            else:
                front_hit_count = 0

            if ctx.robot.is_e_stopped():
                ctx.robot.stop()
                ctx.robot.clear_e_stop()
                ctx.set_substate(f"沿牆觸發前方防撞停靠 (front={front_cm:.1f} cm)")
                return "REACHED"

        # 2. 檢查右方開闊條件 (如步驟 13: 右方 >= 100cm 或連續超出量測範圍 -1)
        if stop_right_gte_cm is not None:
            if right_cm >= stop_right_gte_cm:
                right_open_count += 1
                lost_wall_start = None
                if right_open_count >= confirm_frames:
                    ctx.robot.stop()
                    ctx.set_substate(f"偵測到右方開闊 (right={right_cm:.1f} cm >= {stop_right_gte_cm:.0f} cm)")
                    return "RIGHT_OPEN"
            elif right_cm < 0:
                right_open_count = 0
                if lost_wall_start is None:
                    lost_wall_start = time.monotonic()
                elif (time.monotonic() - lost_wall_start) >= lost_wall_timeout_sec:
                    ctx.robot.stop()
                    ctx.set_substate(f"右方牆面消失超出範圍 (持續 {lost_wall_timeout_sec:.1f}s)")
                    return "RIGHT_OPEN"
            else:
                right_open_count = 0
                lost_wall_start = None

        # 3. 依據右方超音波修正航向 (右轉為 +Yaw，左轉為 -Yaw)
        #    若離右牆太遠 (right_cm > target)，需要向右修正 (+Yaw)
        #    若離右牆太近 (right_cm < target)，需要向左修正 (-Yaw)
        desired_yaw = base_heading_yaw
        if 0 < right_cm < 60.0:
            err_cm = right_cm - target_right_cm
            # 死區 ±1.5cm 內保持直行，減少擺動
            if abs(err_cm) > 1.5:
                angle_adjust = clamp(kp_wall * err_cm, -max_angle_adjust, max_angle_adjust)
                desired_yaw = wrap_angle_deg(base_heading_yaw + angle_adjust)

        now = time.monotonic()
        if abs(angle_diff_deg(desired_yaw, last_sent_yaw)) >= 0.8 or (now - last_send_time) >= 0.25:
            ctx.robot.send_set_heading(desired_yaw, speed, quiet=True)
            last_sent_yaw = desired_yaw
            last_send_time = now

        if max_sec is not None and (now - start_t) >= max_sec:
            ctx.robot.stop()
            ctx.set_substate(f"沿牆前進已達時間上限 {max_sec:.1f} 秒")
            return "TIMEOUT"

        if not ctx.sleep(0.05):
            break

    ctx.robot.stop()
    return "STOPPED"
