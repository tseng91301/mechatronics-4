#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主策略程式 (Arena Strategy)
參照 strategies/arena_strategy.md 實作 16 步驟場地巡航、沿牆、紙屑視覺對準與擺頭吸取策略。
將 Step 1 拆分為「純 IMU 盲走進入」與「巡航至場地中間」兩步，支援從任意步驟開始執行。
"""

import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from .techniques import (
    StrategyContext,
    VisionSource,
    align_to_confetti,
    drive_blind_imu,
    drive_until_front_distance,
    sweep_and_vacuum,
    turn_to_heading,
    wall_follow_right,
    wrap_angle_deg,
)


@dataclass
class StrategyStep:
    """單一策略步驟描述"""

    index: int
    title: str
    description: str
    execute: Callable[[StrategyContext, Dict[str, Any]], bool]


def _make_confetti_tick(
    ctx: StrategyContext,
    state: Dict[str, Any],
    vision: Optional[VisionSource],
    stop_front_cm: float = 15.0,
    cooldown_sec: float = 2.5,
) -> Optional[Callable[[], Optional[str]]]:
    """
    建立行進間紙屑偵測插斷回呼：
    連續 2 幀偵測到紙屑 -> 執行視覺對準 -> 啟動擺頭吸取 -> 回報 CONFETTI_HANDLED 恢復原航向巡航。
    """
    if vision is None or not vision.is_supported():
        return None

    tracker = {"seen_streak": 0, "last_handled_t": 0.0}

    def _on_tick() -> Optional[str]:
        if ctx.is_stopped() or not vision.is_available():
            return None
        now = time.monotonic()
        if (now - tracker["last_handled_t"]) < cooldown_sec:
            return None

        has_confetti, _, center = vision.detect_latest()
        if has_confetti and center is not None:
            tracker["seen_streak"] += 1
            if tracker["seen_streak"] >= 2:
                tracker["seen_streak"] = 0
                ctx.robot.stop()
                ctx.set_substate(f"偵測到紙屑 (X={center.x:.1f}%, Y={center.y:.1f}%)，切換視覺對準...")
                aligned, final_yaw = align_to_confetti(ctx, vision)
                if ctx.is_stopped():
                    return None
                if aligned:
                    sweep_and_vacuum(
                        ctx,
                        heading_yaw=final_yaw,
                        speed=85,
                        amp_deg=20.0,
                        period_ms=1800,
                        stop_front_cm=stop_front_cm,
                        duration_sec=4.0,
                        disable_safety=True,
                    )
                tracker["last_handled_t"] = time.monotonic()
                return "CONFETTI_HANDLED"
        else:
            tracker["seen_streak"] = 0
        return None

    return _on_tick


def build_arena_strategy(
    cruise_speed: int = 130,
    wall_speed: int = 120,
    entry_speed: int = 140,
    entry_duration_sec: float = 3.0,
    center_stop_cm: float = 150.0,
    wall_dist_cm: float = 15.0,
    stop_dist_cm: float = 15.0,
    right_open_cm: float = 100.0,
) -> List[StrategyStep]:
    """
    建構 16 步驟場地清掃主策略清單 (對應 arena_strategy.md)。
    """

    def step_blind_entry(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
        return drive_blind_imu(
            ctx,
            heading_yaw=st["ref_yaw"],
            speed=entry_speed,
            duration_sec=entry_duration_sec,
            disable_safety=True,
        )

    def step_center_approach(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
        tick = _make_confetti_tick(ctx, st, st.get("vision"), stop_front_cm=stop_dist_cm)
        res = drive_until_front_distance(
            ctx,
            heading_yaw=st["ref_yaw"],
            speed=cruise_speed,
            stop_front_cm=center_stop_cm,
            on_tick=tick,
        )
        return res != "STOPPED"

    def _turn_delta(delta_deg: float):
        def _fn(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
            st["ref_yaw"] = wrap_angle_deg(st["ref_yaw"] + delta_deg)
            return turn_to_heading(ctx, st["ref_yaw"])
        return _fn

    def _forward_until_front_stop(with_confetti: bool):
        def _fn(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
            tick = _make_confetti_tick(ctx, st, st.get("vision"), stop_front_cm=stop_dist_cm) if with_confetti else None
            res = drive_until_front_distance(
                ctx,
                heading_yaw=st["ref_yaw"],
                speed=cruise_speed,
                stop_front_cm=stop_dist_cm,
                on_tick=tick,
            )
            return res != "STOPPED"
        return _fn

    def _follow_right_wall_until_front_stop(with_confetti: bool):
        def _fn(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
            tick = _make_confetti_tick(ctx, st, st.get("vision"), stop_front_cm=stop_dist_cm) if with_confetti else None
            res = wall_follow_right(
                ctx,
                base_heading_yaw=st["ref_yaw"],
                speed=wall_speed,
                target_right_cm=wall_dist_cm,
                stop_front_cm=stop_dist_cm,
                on_tick=tick,
            )
            return res != "STOPPED"
        return _fn

    def step_follow_right_wall_until_open(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
        res = wall_follow_right(
            ctx,
            base_heading_yaw=st["ref_yaw"],
            speed=wall_speed,
            target_right_cm=wall_dist_cm,
            stop_front_cm=None,
            stop_right_gte_cm=right_open_cm,
            lost_wall_timeout_sec=0.6,
        )
        return res != "STOPPED"

    def step_exit_forward(ctx: StrategyContext, st: Dict[str, Any]) -> bool:
        res = drive_until_front_distance(
            ctx,
            heading_yaw=st["ref_yaw"],
            speed=cruise_speed,
            stop_front_cm=stop_dist_cm,
            max_sec=5.0,
        )
        return res != "STOPPED"

    return [
        StrategyStep(1, "盲走進入場地", "純 IMU 鎖定前方直行進入，暫時關閉防撞", step_blind_entry),
        StrategyStep(2, "前進至場地中間", "依前方感測器巡航至場地中央，途中見紙屑即對準吸取", step_center_approach),
        StrategyStep(3, "向右轉 90 度 (使車頭朝向右方邊界)", "原地右轉對齊目標航向 (+90°)", _turn_delta(+90.0)),
        StrategyStep(4, "向前直走至右方邊界", "直線前行至前方停止距離，途中見紙屑即對準吸取", _forward_until_front_stop(with_confetti=True)),
        StrategyStep(5, "向右轉 90 度 (使車頭朝向後方邊界)", "原地右轉對齊目標航向 (+90°)", _turn_delta(+90.0)),
        StrategyStep(6, "向前直走至後方邊界", "直線前行至前方停止距離，途中見紙屑即對準吸取", _forward_until_front_stop(with_confetti=True)),
        StrategyStep(7, "向右轉 180 度 (使車頭朝向前方邊界)", "原地掉頭 180° (+180°)，右側朝向牆壁", _turn_delta(+180.0)),
        StrategyStep(8, "沿右牆直走至前方邊界", "右方貼牆修正前行至前方停止距離，途中見紙屑即對準吸取", _follow_right_wall_until_front_stop(with_confetti=True)),
        StrategyStep(9, "向左轉 90 度 (使車頭朝向左方邊界)", "原地左轉對齊目標航向 (-90°)", _turn_delta(-90.0)),
        StrategyStep(10, "沿右牆直走至左邊邊界", "右方貼牆修正前行至前方停止距離，途中見紙屑即對準吸取", _follow_right_wall_until_front_stop(with_confetti=True)),
        StrategyStep(11, "向左轉 90 度 (使車頭朝向後方邊界)", "原地左轉對齊目標航向 (-90°)", _turn_delta(-90.0)),
        StrategyStep(12, "沿右牆直走至後方邊界", "右方貼牆修正前行至前方停止距離，途中見紙屑即對準吸取", _follow_right_wall_until_front_stop(with_confetti=True)),
        StrategyStep(13, "向左轉 90 度 (使車頭朝向右方邊界)", "原地左轉對齊目標航向 (-90°)", _turn_delta(-90.0)),
        StrategyStep(14, "沿右牆直走至後側開闊", "沿右牆行進直到右側感測器開闊或無牆壁反射", step_follow_right_wall_until_open),
        StrategyStep(15, "向右轉 90 度 (出口轉向)", "原地右轉對齊出口方向 (+90°)", _turn_delta(+90.0)),
        StrategyStep(16, "向前走出出口", "直線前進離開目前區域", step_exit_forward),
    ]


ARENA_STEPS = build_arena_strategy()

# 供 strategies/registry.py 自動探索註冊之標準策略介面
STRATEGY_ID = "arena_strategy"
STRATEGY_NAME = "競賽場地清掃策略 (arena_strategy)"
STRATEGY_DESC = "16 步驟進場、場地巡航、沿右牆修正、紙屑視覺對準與擺頭吸取流程"
build_strategy = build_arena_strategy
