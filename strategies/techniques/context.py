#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
策略組合技執行上下文 (StrategyContext)
提供角度正規化工具、可中斷休眠、子狀態回報與硬體防撞暫時關閉/恢復保護。
"""

import threading
import time
from contextlib import contextmanager
from typing import Callable, Optional

try:
    from control.robot_controller import RobotController
except ImportError:
    from robot_controller import RobotController


def wrap_angle_deg(angle: float) -> float:
    """將任意角度規範化至 [-180.0, 180.0) 度"""
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return wrapped


def angle_diff_deg(target: float, current: float) -> float:
    """計算從 current 到 target 的最短有號角度差 [-180.0, 180.0)"""
    return wrap_angle_deg(target - current)


def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, val))


class StrategyContext:
    """組合技共用的執行狀態與安全控制上下文"""

    def __init__(
        self,
        robot: RobotController,
        stop_event: Optional[threading.Event] = None,
        on_substate: Optional[Callable[[str], None]] = None,
        default_safety_mm: int = 150,
    ):
        self.robot = robot
        self.stop_event = stop_event or threading.Event()
        self._on_substate = on_substate
        self.default_safety_mm = default_safety_mm

    def is_stopped(self) -> bool:
        """檢查是否收到停止策略請求"""
        return self.stop_event.is_set()

    def sleep(self, seconds: float, step: float = 0.05) -> bool:
        """
        可中斷的休眠：每 step 秒檢查一次 stop_event。
        若中途收到停止訊號立即回傳 False；完整睡完回傳 True。
        """
        end_time = time.monotonic() + max(0.0, seconds)
        while not self.stop_event.is_set():
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                return True
            self.stop_event.wait(min(step, remaining))
        return False

    def set_substate(self, substate: str):
        """更新當前步驟的子動作狀態 (推播至網頁監控與日誌)"""
        if self._on_substate:
            try:
                self._on_substate(substate)
            except Exception:
                pass
        self.robot.log("SYS", f"[策略子狀態] {substate}")

    @contextmanager
    def temporarily_disable_safety(self, restore_mm: Optional[int] = None):
        """
        暫時關閉 Arduino 硬體防撞 (設為 0)，並在離開區塊或發生異常/中斷時自動恢復原門檻。
        """
        current_mm = self.robot.get_safety_threshold_mm()
        target_restore = (
            restore_mm
            if restore_mm is not None
            else (current_mm if current_mm > 0 else self.default_safety_mm)
        )
        self.robot.set_safety_threshold(0)
        self.robot.clear_e_stop()
        try:
            yield
        finally:
            self.robot.set_safety_threshold(target_restore)
            self.robot.log("SYS", f"[安全恢復] 已將 Arduino 硬體防撞門檻恢復為 {target_restore} mm")
