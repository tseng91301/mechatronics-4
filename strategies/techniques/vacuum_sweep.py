#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
紙屑擺頭吸取組合技 (Vacuum & Sweep Techniques)
定義擺頭清掃的前進速度、頻率 (週期 ms) 及晃動幅度，邊清掃邊向前走直到安全距離或吸取完成。
"""

import time
from typing import Optional, Tuple

from .context import StrategyContext, wrap_angle_deg
from .vision_align import VisionSource


_COLOR_LABEL_ZH = {
    "red": "紅色",
    "green": "綠色",
    "blue": "藍色",
}


def _resolve_confetti_color(
    ctx: StrategyContext,
    vision: Optional[VisionSource],
    detected_color: Optional[str],
    confirm_timeout_sec: float = 0.4,
) -> Optional[str]:
    """
    在啟動擺頭吸塵前確認紙屑顏色：
    1. 優先讀取最新即時畫面偵測顏色（若鏡頭可用但當幀尚未命中，短暫輪詢等待 confirm_timeout_sec）
    2. 若因貼近吸口進入鏡頭盲區而未出現在當幀，回退採用剛對準時的近期偵測顏色
    3. 最後回退使用策略步驟傳入的 detected_color
    """
    if vision is not None and vision.is_supported():
        if hasattr(vision, "detect_latest_color"):
            has_c, _, _, live_color = vision.detect_latest_color()
            if has_c and live_color:
                return live_color.lower()

            if vision.is_available() and confirm_timeout_sec > 0:
                deadline = time.monotonic() + confirm_timeout_sec
                while not ctx.is_stopped() and time.monotonic() < deadline:
                    if not ctx.sleep(0.05):
                        return None
                    has_c, _, _, live_color = vision.detect_latest_color()
                    if has_c and live_color:
                        return live_color.lower()

        if hasattr(vision, "get_latest_color"):
            recent_color = vision.get_latest_color(recent_fallback_sec=2.5)
            if recent_color:
                return recent_color.lower()

    if detected_color:
        return str(detected_color).strip().lower()
    return None


def sweep_and_vacuum(
    ctx: StrategyContext,
    heading_yaw: float,
    speed: int = 85,
    amp_deg: float = 20.0,
    period_ms: int = 1800,
    stop_front_cm: float = 15.0,
    duration_sec: float = 4.0,
    disable_safety: bool = True,
    vision: Optional[VisionSource] = None,
    detected_color: Optional[str] = None,
    skip_colors: Tuple[str, ...] = ("red",),
    skip_wait_sec: float = 3.0,
    color_confirm_timeout_sec: float = 0.4,
) -> bool:
    """
    組合技 4：進行紙屑吸取。
    在啟動吸塵與擺頭前先確認紙屑顏色；若偵測為跳過顏色（預設紅色），
    則不啟動吸塵與擺頭，發出提示並等待 skip_wait_sec（預設 3.0 秒）後結束進入下個區段；
    其他顏色則照常開啟吸塵馬達與動態擺頭邊清掃邊前進。
    結束或中斷時保證關閉擺頭、關閉吸塵馬達、煞停並恢復防撞門檻。
    """
    heading_yaw = wrap_angle_deg(heading_yaw)

    confetti_color = _resolve_confetti_color(
        ctx,
        vision=vision,
        detected_color=detected_color,
        confirm_timeout_sec=color_confirm_timeout_sec,
    )
    if ctx.is_stopped():
        ctx.robot.stop()
        return False

    skip_set = {c.strip().lower() for c in skip_colors if c}
    if confetti_color and confetti_color in skip_set:
        color_label = _COLOR_LABEL_ZH.get(confetti_color, confetti_color)
        ctx.robot.set_sweep(False)
        ctx.robot.set_vacuum(False)
        ctx.robot.stop()
        ctx.set_substate(
            f"⚠️ 偵測到{color_label}紙屑，跳過吸塵清掃！等待 {skip_wait_sec:.0f} 秒後進入下一個區段..."
        )
        if not ctx.sleep(skip_wait_sec):
            return False
        ctx.set_substate(f"{color_label}紙屑跳過等待結束，進入下一個區段")
        return True

    color_prefix = (
        f"[紙屑顏色: {_COLOR_LABEL_ZH.get(confetti_color, confetti_color)}] "
        if confetti_color
        else ""
    )
    ctx.set_substate(
        f"{color_prefix}啟動擺頭吸塵 (振幅=±{amp_deg:.0f}°, 週期={period_ms}ms, 速度={speed}, 上限 {duration_sec:.1f}s)"
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
