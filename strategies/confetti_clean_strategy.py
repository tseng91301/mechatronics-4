#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
紙屑視覺對準與擺頭吸取策略 (Confetti Align & Vacuum Strategy)
僅執行「視覺偵測對準紙屑」與「啟動擺頭吸取」兩個步驟，不含場地巡航與沿牆流程。
"""

import time
from typing import Any, Dict, List

from .arena_strategy import StrategyStep
from .techniques import StrategyContext, align_to_confetti, sweep_and_vacuum


def build_confetti_clean_strategy(
    wait_timeout_sec: float = 20.0,
    center_tolerance_x: float = 8.0,
    target_y_pct: float = 50.0,
    approach_speed: int = 40,
    align_gain: float = 0.45,
    max_align_sec: float = 12.0,
    vacuum_speed: int = 10,
    amp_deg: float = 30.0,
    period_ms: int = 1000,
    stop_front_cm: float = 15.0,
    vacuum_duration_sec: float = 4.0,
) -> List[StrategyStep]:
    """建構「視覺對準紙屑 → 擺頭吸取」2 步驟策略"""

    def step_1_align_confetti(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
        vision = st.get("vision")
        if vision is None or not vision.is_supported():
            ctx.set_substate("⚠️ 未啟用影像辨識或 OpenCV 不可用，請確認已勾選「啟用影像紙屑對準」")
            return False

        ctx.set_substate("等待影像畫面中出現紙屑...")
        start_wait = time.monotonic()
        seen_streak = 0

        while not ctx.is_stopped():
            if vision.is_available():
                has_confetti, _, center = vision.detect_latest()
                if has_confetti and center is not None:
                    seen_streak += 1
                    if seen_streak >= 2:
                        break
                else:
                    seen_streak = 0
            else:
                seen_streak = 0

            if (time.monotonic() - start_wait) >= wait_timeout_sec:
                ctx.robot.stop()
                st["confetti_ready"] = False
                ctx.set_substate(f"等待 {wait_timeout_sec:.0f} 秒仍未偵測到紙屑")
                return True

            if not ctx.sleep(0.05):
                return False

        if ctx.is_stopped():
            ctx.robot.stop()
            return False

        aligned, final_yaw = align_to_confetti(
            ctx,
            vision=vision,
            center_tolerance_x=center_tolerance_x,
            target_y_pct=target_y_pct,
            approach_speed=approach_speed,
            align_gain=align_gain,
            max_duration_sec=max_align_sec,
        )
        if ctx.is_stopped():
            return False

        st["ref_yaw"] = final_yaw
        st["confetti_ready"] = bool(aligned)
        st["confetti_color"] = (
            vision.get_latest_color(recent_fallback_sec=2.5)
            if hasattr(vision, "get_latest_color")
            else None
        )
        color_info = f"、顏色={st['confetti_color']}" if st["confetti_color"] else ""
        ctx.set_substate(
            f"對準階段結束 ({'成功對準' if aligned else '未完全對準'}，Yaw={final_yaw:.1f}°{color_info})"
        )
        return True

    def step_2_vacuum_sweep(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
        # 若從第 1 步執行且明確未找到/未對準紙屑，跳過吸塵避免空吸
        if st.get("confetti_ready") is False:
            ctx.set_substate("未成功偵測/對準紙屑，跳過擺頭吸取")
            return True

        return sweep_and_vacuum(
            ctx,
            heading_yaw=st["ref_yaw"],
            speed=vacuum_speed,
            amp_deg=amp_deg,
            period_ms=period_ms,
            stop_front_cm=stop_front_cm,
            duration_sec=vacuum_duration_sec,
            disable_safety=False,
            skip_colors=[],
            vision=st.get("vision"),
            detected_color=st.get("confetti_color"),
        )

    return [
        StrategyStep(
            index=1,
            title="視覺對準紙屑",
            description="等待畫面出現紙屑並透過影像回饋自動轉向與靠近對準",
            execute=step_1_align_confetti,
        ),
        StrategyStep(
            index=2,
            title="擺頭吸取紙屑",
            description="維持對準航向啟動吸塵馬達與動態擺頭向前清掃",
            execute=step_2_vacuum_sweep,
        ),
    ]


# 供 strategies/registry.py 自動探索註冊
STRATEGY_ID = "confetti_clean"
STRATEGY_NAME = "紙屑對準與吸取策略 (confetti_clean)"
STRATEGY_DESC = "2 步驟單純測試流程：視覺偵測對準紙屑 → 擺頭吸塵清掃"
build_strategy = build_confetti_clean_strategy
