#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
紙屑擺頭吸取組合技 (Vacuum & Sweep Techniques)
定義擺頭清掃的前進速度、頻率 (週期 ms) 及晃動幅度，邊清掃邊向前走直到安全距離或吸取完成。
"""

import time

from .context import StrategyContext, wrap_angle_deg


def sweep_and_vacuum(
    ctx: StrategyContext,
    heading_yaw: float,
    speed: int = 85,
    amp_deg: float = 20.0,
    period_ms: int = 1800,
    stop_front_cm: float = 15.0,
    duration_sec: float = 4.0,
    disable_safety: bool = True,
) -> bool:
    """
    組合技 4：進行紙屑吸取。
    開啟吸塵馬達與動態擺頭，依指定的晃動幅度、週期與前進速度邊清掃邊前進，
    直到達到指定時間或前方超音波距離 <= stop_front_cm。
    結束或中斷時保證關閉擺頭、關閉吸塵馬達、煞停並恢復防撞門檻。
    """
    heading_yaw = wrap_angle_deg(heading_yaw)
    ctx.set_substate(
        f"啟動擺頭吸塵 (振幅=±{amp_deg:.0f}°, 週期={period_ms}ms, 速度={speed}, 上限 {duration_sec:.1f}s)"
    )

    def _run() -> bool:
        try:
            ctx.robot.set_vacuum(True)
            ctx.robot.set_sweep(True, amp=amp_deg, period_ms=period_ms)
            ctx.robot.set_heading(heading_yaw, speed)

            start_t = time.monotonic()
            hit_count = 0
            while not ctx.is_stopped():
                front_cm = ctx.robot.get_front_cm()
                if 0 < front_cm <= stop_front_cm:
                    hit_count += 1
                    if hit_count >= 2:
                        ctx.set_substate(f"擺頭吸塵碰到前方安全距離 ({front_cm:.1f} cm)，結束吸取")
                        return True
                else:
                    hit_count = 0

                if (time.monotonic() - start_t) >= duration_sec:
                    ctx.set_substate("擺頭吸塵完成！")
                    return True

                if not ctx.sleep(0.05):
                    break
            return False
        finally:
            ctx.robot.set_sweep(False)
            ctx.robot.set_vacuum(False)
            ctx.robot.stop()

    if disable_safety:
        with ctx.temporarily_disable_safety():
            return _run()
    return _run()
