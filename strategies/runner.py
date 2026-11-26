#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自動任務策略執行引擎 (StrategyRunner)
支援在背景執行緒運行策略、從任意步驟開始執行、即時緊急停止、以及透過 RobotController.notify 推播執行進度給網頁。
"""

import threading
import time
from typing import Any, Dict, List, Optional, Tuple

try:
    from control.robot_controller import RobotController
except ImportError:
    from robot_controller import RobotController

from .arena_strategy import StrategyStep, build_arena_strategy
from .techniques import StrategyContext, VisionSource


class StrategyRunner:
    """管理自動任務策略的生命週期、步驟追蹤與即時中斷"""

    def __init__(self, robot: RobotController, steps: Optional[List[StrategyStep]] = None):
        self.robot = robot
        self._steps: List[StrategyStep] = steps if steps is not None else build_arena_strategy()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None

        self._state: Dict[str, Any] = {
            "status": "IDLE",  # IDLE | RUNNING | STOPPED | COMPLETED | ERROR
            "strategy_name": "場地自動清掃策略 (STRATEGY.md)",
            "current_step": 0,
            "total_steps": len(self._steps),
            "step_title": "待命中",
            "sub_state": "",
            "start_step": 1,
            "enable_vision": True,
            "vision_available": False,
            "error": "",
        }

    def get_steps_metadata(self) -> List[Dict[str, Any]]:
        """回傳所有步驟的編號、標題與說明，供網頁渲染步驟選單與進度列表"""
        return [
            {
                "index": s.index,
                "title": s.title,
                "description": s.description,
            }
            for s in self._steps
        ]

    def get_status(self) -> Dict[str, Any]:
        """取得目前策略執行狀態快照 (含步驟列表)"""
        with self._lock:
            snap = dict(self._state)
        snap["type"] = "strategy_status"
        snap["steps"] = self.get_steps_metadata()
        return snap

    def _broadcast_status(self):
        self.robot.notify(self.get_status())

    def _update_state(self, **kwargs):
        with self._lock:
            self._state.update(kwargs)
        self._broadcast_status()

    def start(
        self,
        start_step: int = 1,
        enable_vision: bool = True,
        rtsp_url: str = "rtsp://localhost:8554/live/stream",
        reset_imu: bool = False,
    ) -> Tuple[bool, str]:
        """
        啟動自動任務策略 (非阻塞，於背景執行緒運行)。
        :param start_step: 起始步驟編號 (1 ~ total_steps)
        :param enable_vision: 是否啟用紙屑視覺偵測與對準
        :param rtsp_url: 影像串流位址 (預設 rtsp://localhost:8554/live/stream)
        :param reset_imu: 啟動前是否將 IMU Yaw 歸零
        """
        if not self.robot.is_connected:
            msg = "無法啟動策略：請先連線 Arduino 序列埠 (或開啟模擬模式)"
            self.robot.log("SYS", msg)
            return (False, msg)

        if start_step < 1 or start_step > len(self._steps):
            msg = f"無效的起始步驟：{start_step} (有效範圍 1~{len(self._steps)})"
            self.robot.log("SYS", msg)
            return (False, msg)

        with self._lock:
            if self._state["status"] == "RUNNING" and self._worker_thread and self._worker_thread.is_alive():
                return (False, "策略目前正在執行中，請先停止後再啟動")

            self._stop_event.clear()
            self._state.update({
                "status": "RUNNING",
                "current_step": start_step,
                "total_steps": len(self._steps),
                "step_title": self._steps[start_step - 1].title,
                "sub_state": "初始化策略中...",
                "start_step": start_step,
                "enable_vision": bool(enable_vision),
                "vision_available": False,
                "error": "",
            })

        self._broadcast_status()
        self._worker_thread = threading.Thread(
            target=self._run_worker,
            args=(start_step, bool(enable_vision), rtsp_url, bool(reset_imu)),
            daemon=True,
        )
        self._worker_thread.start()
        return (True, f"已從第 {start_step} 步啟動自動策略")

    def stop(self) -> Tuple[bool, str]:
        """立即中斷目前正在執行的策略並安全煞停車輛"""
        self._stop_event.set()
        self.robot.set_sweep(False)
        self.robot.set_vacuum(False)
        self.robot.stop()

        with self._lock:
            was_running = self._state["status"] == "RUNNING"
            if was_running:
                self._state["status"] = "STOPPED"
                self._state["sub_state"] = "使用者手動停止策略"

        self.robot.log("SYS", "⏹ 已發送策略停止訊號，車輛已安全煞停")
        self._broadcast_status()
        return (True, "已停止策略並煞停車輛")

    def _run_worker(self, start_step: int, enable_vision: bool, rtsp_url: str, reset_imu: bool):
        vision: Optional[VisionSource] = None
        saved_safety_mm = self.robot.get_safety_threshold_mm()
        restore_safety_mm = saved_safety_mm if saved_safety_mm > 0 else 150

        def _on_substate(sub_text: str):
            vis_ok = vision.is_available() if vision is not None else False
            self._update_state(sub_state=sub_text, vision_available=vis_ok)

        ctx = StrategyContext(
            robot=self.robot,
            stop_event=self._stop_event,
            on_substate=_on_substate,
            default_safety_mm=restore_safety_mm,
        )

        try:
            if enable_vision:
                vision = VisionSource(source=rtsp_url)
                if vision.is_supported():
                    vision.start()
                    self.robot.log("SYS", f"[策略視覺] 已背景連線影像串流: {rtsp_url}")
                    # 稍候最多 0.5 秒讓首幀有機會進來
                    ctx.sleep(0.5)
                else:
                    self.robot.log("SYS", "[策略視覺] OpenCV 不可用，將以純感測器模式運行")

            if reset_imu:
                ctx.set_substate("重設 IMU 航向歸零中...")
                self.robot.reset_yaw()
                if not ctx.sleep(0.15):
                    return

            self.robot.clear_e_stop()
            runtime_state: Dict[str, Any] = {
                "ref_yaw": self.robot.get_yaw(),
                "vision": vision,
            }
            self.robot.log(
                "SYS",
                f"▶ 策略開始執行：從第 {start_step}/{len(self._steps)} 步開始 (基準 Yaw={runtime_state['ref_yaw']:.1f}°)",
            )

            for step in self._steps[start_step - 1 :]:
                if self._stop_event.is_set():
                    break

                vis_ok = vision.is_available() if vision is not None else False
                self._update_state(
                    current_step=step.index,
                    step_title=step.title,
                    sub_state=step.description,
                    vision_available=vis_ok,
                )
                self.robot.log("SYS", f"=== [步驟 {step.index}/{len(self._steps)}] {step.title} ===")

                ok = step.execute(ctx, runtime_state)
                if not ok or self._stop_event.is_set():
                    self._update_state(status="STOPPED", sub_state=f"已於第 {step.index} 步停止")
                    return

            if not self._stop_event.is_set():
                self._update_state(
                    status="COMPLETED",
                    sub_state="全部策略步驟已執行完成！",
                    vision_available=(vision.is_available() if vision is not None else False),
                )
                self.robot.log("SYS", "🎉 自動任務策略已全部執行完成！")

        except Exception as exc:
            err_msg = str(exc)
            self.robot.log("SYS", f"❌ 策略執行異常: {err_msg}")
            self._update_state(status="ERROR", sub_state=f"發生異常: {err_msg}", error=err_msg)
        finally:
            self.robot.set_sweep(False)
            self.robot.set_vacuum(False)
            self.robot.stop()
            # 確保策略結束或中止後一定恢復防撞門檻
            if saved_safety_mm > 0:
                self.robot.set_safety_threshold(saved_safety_mm)
            if vision is not None:
                vision.stop()
            with self._lock:
                if self._state["status"] == "RUNNING":
                    self._state["status"] = "STOPPED"
                self._state["vision_available"] = False
            self._broadcast_status()
