"""
confetti_detector.py
OpenCV 輕量紙屑影像偵測核心模組 (Core Confetti Detector Module)

輸入一個 frame，經過處理流程之後，輸出：
1. 是否有紙屑 (has_confetti: bool)
2. 紙屑範圍 (bbox: ConfettiBBox | None, 範圍以 0~100% 表示)
3. 紙屑中心點 (center: ConfettiCenter | None, 座標以 0~100% 表示)

座標系統說明：
- X 軸：左至右 0.0% ~ 100.0%
- Y 軸：上至下 0.0% ~ 100.0%
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Dict, Any, List, Union
import cv2
import numpy as np


@dataclass
class ConfettiBBox:
    """
    紙屑群組範圍 (以 0~100% 百分比表示)
    座標原點 (0, 0) 為影像左上角：
      x_min: 範圍左邊界百分比 (0.0% ~ 100.0%)
      y_min: 範圍上邊界百分比 (0.0% ~ 100.0%)
      x_max: 範圍右邊界百分比 (0.0% ~ 100.0%)
      y_max: 範圍下邊界百分比 (0.0% ~ 100.0%)
    """
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def width(self) -> float:
        """寬度百分比 (0.0% ~ 100.0%)"""
        return round(max(0.0, self.x_max - self.x_min), 2)

    @property
    def height(self) -> float:
        """高度百分比 (0.0% ~ 100.0%)"""
        return round(max(0.0, self.y_max - self.y_min), 2)

    def to_tuple(self) -> Tuple[float, float, float, float]:
        """以 (x_min, y_min, x_max, y_max) tuple 回傳"""
        return (self.x_min, self.y_min, self.x_max, self.y_max)

    def to_dict(self) -> Dict[str, float]:
        """以字典形式回傳"""
        return {
            "x_min": self.x_min,
            "y_min": self.y_min,
            "x_max": self.x_max,
            "y_max": self.y_max,
            "width": self.width,
            "height": self.height,
        }

    # 支援 tuple 解包：x_min, y_min, x_max, y_max = bbox
    def __iter__(self):
        return iter((self.x_min, self.y_min, self.x_max, self.y_max))

    def __getitem__(self, index):
        return (self.x_min, self.y_min, self.x_max, self.y_max)[index]

    def __repr__(self) -> str:
        return f"ConfettiBBox(x_min={self.x_min}%, y_min={self.y_min}%, x_max={self.x_max}%, y_max={self.y_max}%, w={self.width}%, h={self.height}%)"


@dataclass
class ConfettiCenter:
    """
    紙屑整體中心點 (以 0~100% 百分比表示)
    座標原點 (0, 0) 為影像左上角：
      x: 水平方向中心百分比 (左至右 0.0% ~ 100.0%)
      y: 垂直方向中心百分比 (上至下 0.0% ~ 100.0%)
    """
    x: float
    y: float

    def to_tuple(self) -> Tuple[float, float]:
        """以 (x, y) tuple 回傳"""
        return (self.x, self.y)

    def to_dict(self) -> Dict[str, float]:
        """以字典形式回傳"""
        return {"x": self.x, "y": self.y}

    # 支援 tuple 解包：cx, cy = center
    def __iter__(self):
        return iter((self.x, self.y))

    def __getitem__(self, index):
        return (self.x, self.y)[index]

    def __repr__(self) -> str:
        return f"ConfettiCenter(x={self.x}%, y={self.y}%)"


@dataclass
class DetectionResult:
    """
    偵測完整結果結構
    """
    has_confetti: bool
    bbox: Optional[ConfettiBBox]
    center: Optional[ConfettiCenter]
    count: int = 0
    annotated_frame: Optional[np.ndarray] = None
    items: Optional[List[Dict[str, Any]]] = None

    # 支援解包前 3 個核心項目：has_confetti, bbox, center = result
    def __iter__(self):
        return iter((self.has_confetti, self.bbox, self.center))

    def __getitem__(self, index):
        return (self.has_confetti, self.bbox, self.center)[index]


class ConfettiDetector:
    """
    輕量化色彩與幾何紙屑辨識器 (專為嵌入式/Raspberry Pi 4B 設計)
    """

    def __init__(
        self,
        proc_width: int = 320,
        proc_height: int = 240,
        roi_top_ratio: float = 0.15,
        min_area: int = 20,
        max_area: int = 3500,
        min_aspect_ratio: float = 0.25,
        max_aspect_ratio: float = 4.0,
        min_solidity: float = 0.65,
    ):
        """
        :param proc_width: 運算解析度寬度 (預設 320，降低運算負擔)
        :param proc_height: 運算解析度高度 (預設 240)
        :param roi_top_ratio: 忽略畫面頂部比例 (預設 0.15 代表忽略最上方 15% 畫面，過濾遠方牆壁/干擾)
        :param min_area: 單片紙屑最小輪廓面積 (在 proc 解析度下)
        :param max_area: 單片紙屑最大輪廓面積
        :param min_aspect_ratio: 最小長寬比 (過濾過於細長反光)
        :param max_aspect_ratio: 最大長寬比
        :param min_solidity: 凸包實心度閥值 (過濾破碎噪點)
        """
        self.pw = proc_width
        self.ph = proc_height
        self.roi_top_ratio = max(0.0, min(1.0, roi_top_ratio))
        self.roi_y = int(self.ph * self.roi_top_ratio)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        self.min_area = min_area
        self.max_area = max_area
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
        self.min_solidity = min_solidity

        # --- HSV 顏色閥值 (H: 0~179, S: 0~255, V: 0~255) ---
        # 1. 綠色紙屑
        self.green_lower = np.array([35, 65, 70], dtype=np.uint8)
        self.green_upper = np.array([85, 255, 255], dtype=np.uint8)

        # 2. 藍色紙屑
        self.blue_lower = np.array([95, 65, 80], dtype=np.uint8)
        self.blue_upper = np.array([130, 255, 255], dtype=np.uint8)

        # 3. 紅色紙屑 (色相跨越 0 與 180，使用兩段遮罩)
        self.red_lower1 = np.array([0, 85, 80], dtype=np.uint8)
        self.red_upper1 = np.array([10, 255, 255], dtype=np.uint8)
        self.red_lower2 = np.array([170, 85, 80], dtype=np.uint8)
        self.red_upper2 = np.array([179, 255, 255], dtype=np.uint8)

    def detect(
        self,
        frame: np.ndarray,
        return_annotated: bool = False
    ) -> Union[
        Tuple[bool, Optional[ConfettiBBox], Optional[ConfettiCenter]],
        Tuple[bool, Optional[ConfettiBBox], Optional[ConfettiCenter], np.ndarray]
    ]:
        """
        核心偵測介面：輸入單一影像 frame，輸出 3 個核心資訊 (可選回傳標記影像)

        :param frame: 原始輸入影像 (BGR 格式，numpy ndarray)
        :param return_annotated: 若為 True，回傳第 4 項標記後的影像 annotated_frame
        :return: (has_confetti, bbox, center) 或 (has_confetti, bbox, center, annotated_frame)
          - has_confetti (bool): 是否有偵測到紙屑
          - bbox (ConfettiBBox | None): 紙屑整體外接矩形範圍 (百分比 0.0% ~ 100.0%)
          - center (ConfettiCenter | None): 紙屑整體幾何中心點 (百分比 0.0% ~ 100.0%)
          - annotated_frame (ndarray, 可選): 標註後的可視化影像
        """
        result = self.process(frame, visualize=return_annotated)
        if return_annotated:
            return result.has_confetti, result.bbox, result.center, result.annotated_frame
        return result.has_confetti, result.bbox, result.center

    def process(self, frame: np.ndarray, visualize: bool = False) -> DetectionResult:
        """
        完整處理流程，回傳 DetectionResult 物件
        """
        if frame is None or frame.size == 0:
            return DetectionResult(has_confetti=False, bbox=None, center=None, count=0)

        orig_h, orig_w = frame.shape[:2]
        scale_x = orig_w / float(self.pw)
        scale_y = orig_h / float(self.ph)

        # 1. 降解析度以確保高效運算 + 高斯平滑減少雜訊
        small = cv2.resize(frame, (self.pw, self.ph), interpolation=cv2.INTER_LINEAR)
        blurred = cv2.GaussianBlur(small, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 2. 多色域二值化閥值分割
        mask_g = cv2.inRange(hsv, self.green_lower, self.green_upper)
        mask_b = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        mask_r1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask_r2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)

        # 合併三種顏色遮罩
        mask = cv2.bitwise_or(mask_g, mask_b)
        mask = cv2.bitwise_or(mask, cv2.bitwise_or(mask_r1, mask_r2))

        # 3. 視野感興趣區域 (ROI) 遮蔽：忽略上方區域
        if self.roi_y > 0:
            mask[:self.roi_y, :] = 0

        # 4. 形態學濾波 (開運算去孤立噪點，閉運算縫合破洞)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        # 5. 尋找外輪廓並依幾何特徵過濾
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_items: List[Dict[str, Any]] = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect_ratio = float(bw) / bh if bh > 0 else 0
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area > 0 and (area / hull_area) < self.min_solidity:
                continue

            moments = cv2.moments(cnt)
            if moments["m00"] == 0:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            # 還原為原始解析度座標
            orig_bbox = (int(x * scale_x), int(y * scale_y), int(bw * scale_x), int(bh * scale_y))
            orig_center = (int(cx * scale_x), int(cy * scale_y))

            valid_items.append({
                "bbox": orig_bbox,
                "center": orig_center,
                "area": area,
            })

        # 若無符合條件之紙屑
        if not valid_items:
            annotated_frame = self.draw_annotations(
                frame, has_confetti=False, bbox=None, center=None, items=[]
            ) if visualize else None
            return DetectionResult(
                has_confetti=False,
                bbox=None,
                center=None,
                count=0,
                annotated_frame=annotated_frame,
                items=[],
            )

        # 6. 計算整體範圍與整體中心點 (換算為 0.0% ~ 100.0%)
        all_x1 = [item["bbox"][0] for item in valid_items]
        all_y1 = [item["bbox"][1] for item in valid_items]
        all_x2 = [item["bbox"][0] + item["bbox"][2] for item in valid_items]
        all_y2 = [item["bbox"][1] + item["bbox"][3] for item in valid_items]

        gx1 = min(all_x1)
        gy1 = min(all_y1)
        gx2 = max(all_x2)
        gy2 = max(all_y2)

        # 換算為 0~100%
        xmin_pct = round(max(0.0, min(100.0, (gx1 / float(orig_w)) * 100.0)), 2)
        ymin_pct = round(max(0.0, min(100.0, (gy1 / float(orig_h)) * 100.0)), 2)
        xmax_pct = round(max(0.0, min(100.0, (gx2 / float(orig_w)) * 100.0)), 2)
        ymax_pct = round(max(0.0, min(100.0, (gy2 / float(orig_h)) * 100.0)), 2)

        bbox_percent = ConfettiBBox(
            x_min=xmin_pct,
            y_min=ymin_pct,
            x_max=xmax_pct,
            y_max=ymax_pct,
        )

        sum_cx = sum(item["center"][0] for item in valid_items)
        sum_cy = sum(item["center"][1] for item in valid_items)
        avg_cx = sum_cx / float(len(valid_items))
        avg_cy = sum_cy / float(len(valid_items))

        cx_pct = round(max(0.0, min(100.0, (avg_cx / float(orig_w)) * 100.0)), 2)
        cy_pct = round(max(0.0, min(100.0, (avg_cy / float(orig_h)) * 100.0)), 2)

        center_percent = ConfettiCenter(x=cx_pct, y=cy_pct)

        annotated_frame = self.draw_annotations(
            frame,
            has_confetti=True,
            bbox=bbox_percent,
            center=center_percent,
            items=valid_items,
            group_pixel_rect=(gx1, gy1, gx2, gy2),
            group_pixel_center=(int(avg_cx), int(avg_cy)),
        ) if visualize else None

        return DetectionResult(
            has_confetti=True,
            bbox=bbox_percent,
            center=center_percent,
            count=len(valid_items),
            annotated_frame=annotated_frame,
            items=valid_items,
        )

    def draw_annotations(
        self,
        frame: np.ndarray,
        has_confetti: bool,
        bbox: Optional[ConfettiBBox],
        center: Optional[ConfettiCenter],
        items: Optional[List[Dict[str, Any]]] = None,
        group_pixel_rect: Optional[Tuple[int, int, int, int]] = None,
        group_pixel_center: Optional[Tuple[int, int]] = None,
        fps: Optional[float] = None,
    ) -> np.ndarray:
        """
        在畫面上繪製可視化標記與百分比資訊 HUD
        """
        vis_frame = frame.copy()
        orig_h, orig_w = vis_frame.shape[:2]

        # 1. 畫出 ROI 遮蔽分界線 (洋紅色)
        roi_pixel_y = int(orig_h * self.roi_top_ratio)
        cv2.line(vis_frame, (0, roi_pixel_y), (orig_w, roi_pixel_y), (255, 0, 255), 1)
        cv2.putText(
            vis_frame,
            f"ROI Top Cutoff: {int(self.roi_top_ratio * 100)}%",
            (10, max(20, roi_pixel_y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 255),
            1,
            cv2.LINE_AA,
        )

        # 2. 畫出刻度提示 (左右與上下 25%, 50%, 75%)
        tick_color = (80, 80, 80)
        for pct in (25, 50, 75):
            tx = int(orig_w * pct / 100.0)
            ty = int(orig_h * pct / 100.0)
            cv2.line(vis_frame, (tx, 0), (tx, 6), tick_color, 1)
            cv2.line(vis_frame, (0, ty), (6, ty), tick_color, 1)

        # 3. 畫出個別紙屑 (綠色框 + 紅色中心點)
        if items:
            for it in items:
                bx, by, bw, bh = it["bbox"]
                icx, icy = it["center"]
                cv2.rectangle(vis_frame, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                cv2.circle(vis_frame, (icx, icy), 3, (0, 0, 255), -1)

        # 4. 畫出整體群組範圍框 (黃色) 與整體中心點 (青藍色)
        if has_confetti and bbox is not None and center is not None:
            if group_pixel_rect is not None:
                gx1, gy1, gx2, gy2 = group_pixel_rect
            else:
                gx1 = int(orig_w * bbox.x_min / 100.0)
                gy1 = int(orig_h * bbox.y_min / 100.0)
                gx2 = int(orig_w * bbox.x_max / 100.0)
                gy2 = int(orig_h * bbox.y_max / 100.0)

            if group_pixel_center is not None:
                gcx, gcy = group_pixel_center
            else:
                gcx = int(orig_w * center.x / 100.0)
                gcy = int(orig_h * center.y / 100.0)

            # 黃色大外框
            cv2.rectangle(vis_frame, (gx1, gy1), (gx2, gy2), (0, 255, 255), 2)

            # 青藍色十字中心標記
            cross_size = 12
            cv2.circle(vis_frame, (gcx, gcy), 8, (255, 200, 0), -1)
            cv2.line(vis_frame, (gcx - cross_size, gcy), (gcx + cross_size, gcy), (0, 0, 255), 2)
            cv2.line(vis_frame, (gcx, gcy - cross_size), (gcx, gcy + cross_size), (0, 0, 255), 2)

            # 中心點百分比標籤
            center_label = f"({center.x:.1f}%, {center.y:.1f}%)"
            cv2.putText(
                vis_frame,
                center_label,
                (gcx + 12, gcy - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

        # 5. 左上角 HUD 狀態顯示面板 (附微透暗色背景)
        hud_h = 110
        hud_w = 480
        overlay = vis_frame.copy()
        cv2.rectangle(overlay, (10, 10), (10 + hud_w, 10 + hud_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.65, vis_frame, 0.35, 0, vis_frame)
        cv2.rectangle(vis_frame, (10, 10), (10 + hud_w, 10 + hud_h), (70, 70, 70), 1)

        # 狀態文字行
        cnt_num = len(items) if items is not None else (1 if has_confetti else 0)
        status_color = (0, 255, 0) if has_confetti else (0, 0, 255)
        status_text = f"Confetti: {'YES' if has_confetti else 'NO'} | Count: {cnt_num}"
        if fps is not None:
            status_text += f" | {fps:.1f} FPS"

        if has_confetti and bbox is not None and center is not None:
            center_text = f"Center: ({center.x:.1f}%, {center.y:.1f}%)  [0~100%]"
            bbox_text = f"BBox  : X:[{bbox.x_min:.1f}%~{bbox.x_max:.1f}%] Y:[{bbox.y_min:.1f}%~{bbox.y_max:.1f}%]"
        else:
            center_text = "Center: None"
            bbox_text = "BBox  : None"

        cv2.putText(vis_frame, status_text, (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2, cv2.LINE_AA)
        cv2.putText(vis_frame, center_text, (20, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(vis_frame, bbox_text, (20, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

        return vis_frame


# 向後相容別名
ConfettiTracker = ConfettiDetector
