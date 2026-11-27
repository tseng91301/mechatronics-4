#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
單純紙屑視覺對準策略 (Confetti Alignment Strategy)
僅執行「偵測紙屑並透過影像回授對準靠近」單一步驟，適合用來單獨測試與調校鏡頭對準效果。
"""

import time
from typing import Any, Dict, List

from .arena_strategy import StrategyStep
from .techniques import StrategyContext, align_to_confetti


def build_confetti_align_strategy(
    wait_timeout_sec: float = 20.0,
    center_tolerance_x: float = 8.0,
    target_y_pct: float = 50.0,
    approach_speed: int = 30,
    align_gain: float = 0.45,
    max_align_sec: float = 12.0,
) -> List[StrategyStep]:
    """建構僅包含紙屑視覺對準的測試策略"""

    def step_align_confetti(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
        vision = st.get("vision")
        if vision is None or not vision.is_supported():
            ctx.set_substate("⚠️ 未啟用影像辨識或 OpenCV 不可用，請確認已勾選「啟用影像紙屑對準」")
            return False

        # 1. 等待串流就緒且畫面中出現紙屑
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
                ctx.set_substate(f"等待 {wait_timeout_sec:.0f} 秒仍未偵測到紙屑，結束策略")
                return True

            if not ctx.sleep(0.05):
                return False

        if ctx.is_stopped():
            ctx.robot.stop()
            return False

        # 2. 執行視覺對準組合技
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
        ctx.set_substate(
            f"紙屑對準結束 ({'成功對準' if aligned else '未完成對準'}，最終 Yaw={final_yaw:.1f}°)"
        )
        return True

    return [
        StrategyStep(
            index=1,
            title="視覺對準紙屑",
            description="等待畫面出現紙屑並透過影像回饋自動轉向與靠近對準",
            execute=step_align_confetti,
        )
    ]


# 供 strategies/registry.py 自動探索註冊
STRATEGY_ID = "confetti_align"
STRATEGY_NAME = "紙屑視覺對準測試 (confetti_align)"
STRATEGY_DESC = "僅執行影像等待與紙屑對準（不含場地巡航與吸塵動作）"
build_strategy = build_confetti_align_strategy
