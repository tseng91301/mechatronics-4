"""
detect_confetti_video.py
OpenCV 輕量影像偵測測試程式 (Test Runner for Confetti Detection)

功能特點：
- 接入核心模組 ConfettiDetector。
- 支援輸入來源：
  1. 攝影機鏡頭畫面 (Webcam / USB Camera / CSI Camera，如 --source 0)
  2. 影片檔案 (如 --source test_video.mp4)
- 呼叫核心模組取得 (1. 是否有紙屑, 2. 紙屑範圍 0~100%, 3. 紙屑中心點 0~100%)。
- 即時將數值與標註繪製於輸出畫面上。
- 支援儲存標註後影片、即時 FPS 計算、無介面模式 (Headless / SSH)。
"""

import os
import sys
import time
import argparse
from typing import Optional, Union
import cv2
import numpy as np

# 匯入核心模組 (相容同層目錄執行或專案根目錄執行)
try:
    from confetti_detector import ConfettiDetector, ConfettiBBox, ConfettiCenter, DetectionResult, ConfettiTracker
except ImportError:
    from frame_recognize.confetti_detector import (
        ConfettiDetector, ConfettiBBox, ConfettiCenter, DetectionResult, ConfettiTracker
    )


def run_detection(
    source: Union[int, str] = "test_video.mp4",
    output_path: Optional[str] = None,
    show_preview: bool = True,
    proc_width: int = 320,
    proc_height: int = 240,
    roi_top_ratio: float = 0.15,
    max_frames: int = -1,
):
    """
    執行紙屑偵測測試程序

    :param source: 影像來源，可以是攝影機 ID (如 0, 1) 或影片檔案路徑 (如 'test_video.mp4')
    :param output_path: 輸出標註影片路徑 (若為 None 則不寫入檔案)
    :param show_preview: 是否開啟 OpenCV 視窗即時預覽
    :param proc_width: 核心內部運算寬度 (預設 320)
    :param proc_height: 核心內部運算高度 (預設 240)
    :param roi_top_ratio: 頂部 ROI 忽略比例 (預設 0.15 即 15%)
    :param max_frames: 最大處理影格數 (-1 代表不限制)
    """
    # 判斷 source 為攝影機 index 還是檔案
    is_camera = False
    if isinstance(source, int):
        is_camera = True
        cap_source = source
    elif isinstance(source, str) and source.isdigit():
        is_camera = True
        cap_source = int(source)
    else:
        is_camera = False
        cap_source = source
        # 若是相對路徑，嘗試補全同目錄路徑
        if not os.path.exists(cap_source):
            alt_path = os.path.join(os.path.dirname(__file__), cap_source)
            if os.path.exists(alt_path):
                cap_source = alt_path

    cap = cv2.VideoCapture(cap_source)
    if not cap.isOpened():
        print(f"[錯誤] 無法開啟影像來源: {source} (解析為: {cap_source})")
        return

    # 取得輸入串流資訊
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps is None or fps <= 0 or fps > 120:
        fps = 30.0

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if not is_camera else -1

    source_desc = f"攝影機 [{cap_source}]" if is_camera else f"影片檔案 [{cap_source}]"
    print("=" * 60)
    print(f"啟動紙屑偵測測試程式")
    print(f"- 來源: {source_desc}")
    print(f"- 解析度: {width} x {height}")
    print(f"- 影格率 (FPS): {fps:.1f}")
    if total_frames > 0:
        print(f"- 總影格數: {total_frames} 幀")
    print(f"- 核心內部運算解析度: {proc_width} x {proc_height}")
    print(f"- 上方 ROI 忽略比例: {roi_top_ratio * 100:.1f}%")
    if output_path:
        print(f"- 輸出存檔: {output_path}")
    print(f"- 視窗預覽: {'開啟' if show_preview else '關閉 (Headless 模式)'}")
    print("=" * 60)
    print("操作說明: [Q / ESC] 結束 | [SPACE] 暫停/繼續 | [S] 截圖儲存")
    print("-" * 60)

    # 初始化核心辨識器
    detector = ConfettiDetector(
        proc_width=proc_width,
        proc_height=proc_height,
        roi_top_ratio=roi_top_ratio,
    )

    # 準備影片寫入器 (若有指定 output_path)
    out_writer = None
    if output_path:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out_writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    frame_idx = 0
    t_prev = time.time()
    current_fps = fps
    paused = False

    try:
        while True:
            if not paused:
                ret, frame = cap.read()
                if not ret:
                    if is_camera:
                        print("[警告] 攝影機未讀取到畫面，等待重試...")
                        time.sleep(0.1)
                        continue
                    else:
                        print("\n[資訊] 影片播放結束。")
                        break

                t_now = time.time()
                dt = t_now - t_prev
                t_prev = t_now
                if dt > 0:
                    inst_fps = 1.0 / dt
                    current_fps = 0.9 * current_fps + 0.1 * inst_fps

                # ========================================================
                # 核心呼叫：輸入 frame，取得 (是否有紙屑, 紙屑範圍, 紙屑中心點)
                # ========================================================
                result = detector.process(frame, visualize=True)
                has_confetti = result.has_confetti
                bbox = result.bbox      # ConfettiBBox (0~100% 百分比)
                center = result.center  # ConfettiCenter (0~100% 百分比)
                vis_frame = result.annotated_frame

                # 更新畫面上的 FPS 標示
                if vis_frame is not None:
                    # 在畫面右上角繪製即時 FPS
                    fps_text = f"FPS: {current_fps:.1f}"
                    cv2.putText(
                        vis_frame,
                        fps_text,
                        (width - 150, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                # 終端機格式化輸出 (每 15 幀或偵測狀態有紙屑時輸出)
                if frame_idx % 15 == 0 or has_confetti:
                    center_str = f"({center.x:5.1f}%, {center.y:5.1f}%)" if center else "None"
                    if bbox:
                        bbox_str = f"X:[{bbox.x_min:5.1f}%~{bbox.x_max:5.1f}%], Y:[{bbox.y_min:5.1f}%~{bbox.y_max:5.1f}%]"
                    else:
                        bbox_str = "None"

                    frame_label = f"Frame {frame_idx:05d}" if not is_camera else f"Time {time.strftime('%H:%M:%S')}"
                    print(
                        f"[{frame_label}] 紙屑: {str(has_confetti):<5} | "
                        f"中心: {center_str:<16} | 範圍: {bbox_str}"
                    )

                # 寫入輸出檔案
                if out_writer is not None and vis_frame is not None:
                    out_writer.write(vis_frame)

                frame_idx += 1
                if max_frames > 0 and frame_idx >= max_frames:
                    print(f"\n[資訊] 已達到指定最大影格數 ({max_frames} 幀)，停止處理。")
                    break

            # 即時視窗顯示
            if show_preview and vis_frame is not None:
                # 若解析度過大，縮小顯示以避免超出螢幕
                display_frame = vis_frame
                max_display_w = 1280
                if width > max_display_w:
                    disp_scale = max_display_w / float(width)
                    disp_h = int(height * disp_scale)
                    display_frame = cv2.resize(vis_frame, (max_display_w, disp_h))

                cv2.imshow("Confetti Detection Monitor", display_frame)

                key = cv2.waitKey(1 if not paused else 50) & 0xFF
                if key in (ord('q'), 27):  # 'q' 或 ESC 結束
                    print("\n[資訊] 使用者手動中斷。")
                    break
                elif key == ord(' '):  # 空白鍵暫停/繼續
                    paused = not paused
                    state = "暫停" if paused else "繼續"
                    print(f"\n[操作] 影像串流已{state}")
                elif key == ord('s'):  # 's' 截圖
                    screenshot_name = f"screenshot_{frame_idx:05d}.jpg"
                    cv2.imwrite(screenshot_name, vis_frame)
                    print(f"\n[操作] 已儲存截圖至: {screenshot_name}")

    except KeyboardInterrupt:
        print("\n[資訊] 接收到中斷訊號 (KeyboardInterrupt)。")

    finally:
        cap.release()
        if out_writer is not None:
            out_writer.release()
        if show_preview:
            cv2.destroyAllWindows()
        print("-" * 60)
        print(f"程式結束，共處理 {frame_idx} 影格。")
        if output_path and os.path.exists(output_path):
            print(f"輸出影片已儲存至: {os.path.abspath(output_path)}")


def main():
    parser = argparse.ArgumentParser(
        description="OpenCV 輕量紙屑偵測測試程式 (可使用攝影機或影片)"
    )
    parser.add_argument(
        "-s", "--source",
        type=str,
        default="test_video.mp4",
        help="輸入來源：攝影機編號 (如 0, 1) 或影片路徑 (預設: test_video.mp4)",
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="輸出儲存標註影片路徑 (如 test_video_detect.mp4，預設不儲存)",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="關閉視窗預覽 (適用於無螢幕之樹莓派 SSH 連線)",
    )
    parser.add_argument(
        "--proc-width",
        type=int,
        default=320,
        help="內部運算寬度 (預設: 320)",
    )
    parser.add_argument(
        "--proc-height",
        type=int,
        default=240,
        help="內部運算高度 (預設: 240)",
    )
    parser.add_argument(
        "--roi-top",
        type=float,
        default=0.15,
        help="上方忽略 ROI 比例 0.0~1.0 (預設: 0.15 代表忽略頂部 15%% 畫面)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=-1,
        help="限制最大處理影格數 (-1 為無限制)",
    )

    args = parser.parse_args()

    # 檢查預設檔案
    source = args.source
    if not source.isdigit():
        if not os.path.exists(source):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            cand = os.path.join(script_dir, source)
            if os.path.exists(cand):
                source = cand
            else:
                # 如果檔案不存在且為預設值，提示可用攝影機
                if args.source == "test_video.mp4":
                    print(f"[提示] 未找到 {args.source}，嘗試切換為攝影機 0...")
                    source = "0"

    run_detection(
        source=source,
        output_path=args.output,
        show_preview=not args.no_preview,
        proc_width=args.proc_width,
        proc_height=args.proc_height,
        roi_top_ratio=args.roi_top,
        max_frames=args.max_frames,
    )


if __name__ == "__main__":
    main()
