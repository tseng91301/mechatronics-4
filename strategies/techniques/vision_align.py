#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
紙屑視覺辨識與鏡頭對準組合技 (Vision & Confetti Alignment Techniques)
整合 frame_recognize/confetti_detector.py 並讀取 MediaMTX RTSP (/live/stream) 影像串流。
"""

import os
import sys
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple, Union

from .context import StrategyContext, clamp, wrap_angle_deg

# 確保可匯入根目錄底下的 frame_recognize 模組
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    import cv2
    from frame_recognize.confetti_detector import ConfettiBBox, ConfettiCenter, ConfettiDetector
    _VISION_DEPS_OK = True
except ImportError:
    cv2 = None
    ConfettiBBox = None
    ConfettiCenter = None
    ConfettiDetector = None
    _VISION_DEPS_OK = False


class VisionSource:
    """
    背景影格擷取與紙屑辨識器。
    從 RTSP 串流 (預設 rtsp://localhost:8554/live/stream) 或攝影機持續讀取最新畫面，
    提供無延遲阻塞的最新 (has_confetti, bbox, center) 查詢介面。
    """

    def __init__(
        self,
        source: Union[str, int] = "rtsp://localhost:8554/live/stream",
        roi_top_ratio: float = 0.15,
        detect_fps: float = 12.0,
        frame_provider: Optional[Callable[[], Tuple[Optional[Any], float]]] = None,
    ):
        self.source = source
        self.roi_top_ratio = roi_top_ratio
        self.detect_interval = 1.0 / max(1.0, detect_fps)
        self.frame_provider = frame_provider

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_time = 0.0
        self._latest_result: Tuple[bool, Optional["ConfettiBBox"], Optional["ConfettiCenter"]] = (
            False,
            None,
            None,
        )
        self._latest_color: Optional[str] = None
        self._last_seen_color: Optional[str] = None
        self._last_seen_color_time: float = 0.0
        self._last_detect_time = 0.0
        self._available = False

        self._detector = ConfettiDetector(roi_top_ratio=roi_top_ratio) if _VISION_DEPS_OK else None

    def is_supported(self) -> bool:
        return _VISION_DEPS_OK and self._detector is not None

    def is_available(self) -> bool:
        """是否已成功連線並持續收到有效影格"""
        with self._lock:
            return self._available and (time.monotonic() - self._latest_frame_time < 3.0)

    def start(self) -> bool:
        if not self.is_supported():
            return False
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        with self._lock:
            self._available = False
            self._latest_color = None
            self._last_seen_color = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def _run_detection_on_frame(self, frame, frame_time: float):
        with self._lock:
            self._latest_frame = frame
            self._latest_frame_time = frame_time
            self._available = True

        now = time.monotonic()
        if self._detector is not None and (now - self._last_detect_time) >= self.detect_interval:
            try:
                res = self._detector.process(frame, visualize=False)
                with self._lock:
                    self._latest_result = (bool(res.has_confetti), res.bbox, res.center)
                    self._latest_color = res.color if res.has_confetti else None
                    if res.has_confetti and res.color:
                        self._last_seen_color = res.color
                        self._last_seen_color_time = now
                    self._last_detect_time = now
            except Exception:
                pass

    def _capture_loop(self):
        if self.frame_provider is not None:
            while self._running:
                try:
                    frame, frame_ts = self.frame_provider()
                    if frame is not None and (time.monotonic() - frame_ts) < 3.0:
                        self._run_detection_on_frame(frame, frame_ts)
                    else:
                        with self._lock:
                            self._available = False
                except Exception:
                    with self._lock:
                        self._available = False
                time.sleep(0.05)
            return

        cap = None
        while self._running:
            if cap is None or not cap.isOpened():
                try:
                    if isinstance(self.source, str) and self.source.startswith("rtsp://"):
                        cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
                    else:
                        cap = cv2.VideoCapture(self.source)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    cap = None

                if cap is None or not cap.isOpened():
                    with self._lock:
                        self._available = False
                    time.sleep(1.0)
                    continue

            ok, frame = cap.read()
            if not ok or frame is None:
                with self._lock:
                    self._available = False
                cap.release()
                cap = None
                time.sleep(0.5)
                continue

            self._run_detection_on_frame(frame, time.monotonic())

        if cap is not None:
            cap.release()

    def detect_latest(self) -> Tuple[bool, Optional["ConfettiBBox"], Optional["ConfettiCenter"]]:
        """
        取得最新影像偵測結果: (has_confetti, bbox, center)。
        若影像不可用或過期 (>3.0s)，回傳 (False, None, None)。
        """
        if not self.is_supported():
            return (False, None, None)
        with self._lock:
            if not self._available or (time.monotonic() - self._latest_frame_time > 3.0):
                return (False, None, None)
            return self._latest_result

    def detect_latest_color(
        self,
    ) -> Tuple[bool, Optional["ConfettiBBox"], Optional["ConfettiCenter"], Optional[str]]:
        """
        取得最新影像偵測結果與主要紙屑顏色: (has_confetti, bbox, center, color)。
        color 可能為 'red' | 'green' | 'blue' | None。
        """
        if not self.is_supported():
            return (False, None, None, None)
        with self._lock:
            if not self._available or (time.monotonic() - self._latest_frame_time > 3.0):
                return (False, None, None, None)
            has_c, bbox, center = self._latest_result
            return (has_c, bbox, center, self._latest_color if has_c else None)

    def get_latest_color(self, recent_fallback_sec: float = 0.0) -> Optional[str]:
        """
        取得目前偵測到的主要紙屑顏色 ('red' / 'green' / 'blue' / None)。
        若最新影格剛好未偵測到（例如剛進入車底吸口盲區），且設定 recent_fallback_sec > 0，
        則回傳在該秒數內最後一次看到的紙屑顏色。
        """
        if not self.is_supported():
            return None
        now = time.monotonic()
        with self._lock:
            if not self._available or (now - self._latest_frame_time > 3.0):
                return None
            if self._latest_result[0] and self._latest_color:
                return self._latest_color
            if (
                recent_fallback_sec > 0
                and self._last_seen_color is not None
                and (now - self._last_seen_color_time) <= recent_fallback_sec
            ):
                return self._last_seen_color
            return None


def align_to_confetti(
    ctx: StrategyContext,
    vision: VisionSource,
    center_tolerance_x: float = 8.0,
    target_y_pct: float = 40.0,
    approach_speed: int = 30,
    align_gain: float = 0.45,
    max_duration_sec: float = 8.0,
    lost_timeout_sec: float = 0.8,
) -> Tuple[bool, float]:
    """
    組合技 3：透過影像處理回傳的 has_confetti, bbox, center 進行紙屑對準並靠近。
    - center.x: 0~100% (50% 為正前方；>50% 在右側需右轉 +Yaw；<50% 在左側需左轉 -Yaw)
    - center.y: 0~100% (越大代表越靠近車底吸塵口)

    回傳: (aligned_success: bool, final_yaw: float)
    """
    if not vision.is_supported():
        ctx.set_substate("視覺模組未啟用或缺少 OpenCV，跳過紙屑對準")
        return (False, ctx.robot.get_yaw())

    start_t = time.monotonic()
    last_seen_t = time.monotonic()
    last_seen_y = 0.0
    last_target_yaw = ctx.robot.get_yaw()

    ctx.set_substate("開始視覺對準紙屑...")

    while not ctx.is_stopped():
        now = time.monotonic()
        if (now - start_t) >= max_duration_sec:
            ctx.robot.stop()
            ctx.set_substate("紙屑對準超時，結束對準")
            return (last_seen_y >= 50.0, ctx.robot.get_yaw())

        has_confetti, bbox, center = vision.detect_latest()

        if has_confetti and center is not None:
            last_seen_t = now
            last_seen_y = center.y

            err_x = center.x - 50.0  # >0 在右側 (+Yaw), <0 在左側 (-Yaw)
            cur_yaw = ctx.robot.get_yaw()
            desired_yaw = wrap_angle_deg(cur_yaw + clamp(err_x * align_gain, -25.0, 25.0))
            last_target_yaw = desired_yaw

            front_cm = ctx.robot.get_front_cm()
            if 0 < front_cm <= 15.0:
                ctx.robot.stop()
                ctx.set_substate(f"對準靠近且前方已達牆邊 ({front_cm:.1f} cm)")
                return (True, cur_yaw)

            # 判斷是否已對準且接近車頭底部準備吸取
            if abs(err_x) <= center_tolerance_x and center.y >= target_y_pct:
                ctx.robot.stop()
                ctx.set_substate(f"紙屑已對準完成！(center=({center.x:.1f}%, {center.y:.1f}%))")
                return (True, cur_yaw)

            # 若水平偏差較大，先原地轉向；否則邊對準邊前進
            spd = 0 if abs(err_x) > 18.0 else approach_speed
            ctx.robot.send_set_heading(desired_yaw, spd, quiet=True)
            ctx.set_substate(
                f"對準紙屑中 X={center.x:.1f}% (偏差 {err_x:+.1f}%), Y={center.y:.1f}%"
            )
        else:
            # 紙屑暫時消失：若先前已經貼近車底 (Y >= 60%)，表示剛進入鏡頭盲區正下方，視為對準完成
            if (now - last_seen_t) >= lost_timeout_sec:
                ctx.robot.stop()
                if last_seen_y >= 60.0:
                    ctx.set_substate("紙屑進入車底吸塵口盲區，準備啟動吸取")
                    return (True, ctx.robot.get_yaw())
                ctx.set_substate("目標紙屑已遺失，結束對準")
                return (False, ctx.robot.get_yaw())
            ctx.robot.send_set_heading(last_target_yaw, 0, quiet=True)

        if not ctx.sleep(0.08):
            break

    ctx.robot.stop()
    return (False, ctx.robot.get_yaw())
