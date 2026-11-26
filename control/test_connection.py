#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arduino 連線、控制與感測器接收測試腳本
直接呼叫 RobotController API 進行通訊測試、手動/自主控制與感測器監看。
"""

import sys
import time

try:
    from control.robot_controller import RobotController
except ImportError:
    from robot_controller import RobotController


def auto_test_demo(controller: RobotController):
    print("\n========== 開始自動測試流程 ==========")

    print("\n1. 測試連線 Ping (CMD 0x00)...")
    controller.send_ping()
    time.sleep(1)

    print("\n2. 前進測試 (Speed: 180, Turn: 0)...")
    controller.send_dual_motor(turn=0, speed=180)
    time.sleep(2)

    print("\n3. 原地逆時針旋轉測試 (Speed: 0, Turn: 200)...")
    controller.send_dual_motor(turn=200, speed=0)
    time.sleep(2)

    print("\n4. 原地順時針旋轉測試 (Speed: 0, Turn: -200)...")
    controller.send_dual_motor(turn=-200, speed=0)
    time.sleep(2)

    print("\n5. 後退測試 (Speed: -150, Turn: 0)...")
    controller.send_dual_motor(turn=0, speed=-150)
    time.sleep(1.5)

    print("\n6. 煞車停止 (Speed: 0, Turn: 0)...")
    controller.send_dual_motor(turn=0, speed=0)
    time.sleep(1)

    print("\n7. 繼電器開啟與關閉測試...")
    controller.send_relay(relay_id=1, state=True)
    time.sleep(1)
    controller.send_relay(relay_id=1, state=False)
    time.sleep(1)

    print("\n========== 自動測試完成 ==========\n")


def live_monitor(controller: RobotController):
    """進入即時單行監看儀表板，動態更新數值不洗屏，按 Ctrl+C 返回選單"""
    print("\n[進入即時監看模式] 數值將在單行即時更新，按 Ctrl+C 返回控制選單...\n")
    try:
        while True:
            state = controller.get_state()
            f_mm = state.get("front_mm", -1)
            r_mm = state.get("right_mm", -1)
            yaw = state.get("yaw", 0.0)
            mode_name = state.get("mode_name", "MANUAL")
            f_str = f"{f_mm/10.0:5.1f}cm" if f_mm is not None and f_mm >= 0 else "  -- cm"
            r_str = f"{r_mm/10.0:5.1f}cm" if r_mm is not None and r_mm >= 0 else "  -- cm"
            sys.stdout.write(f"\r>> 前方: {f_str} | 右方: {r_str} | Yaw: {yaw:6.1f}° | 模式: {mode_name:<16}")
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("\n\n[已返回控制選單]\n")


def main():
    controller = RobotController(verbose=False)
    telemetry_state = {"print_stream": False}

    def on_robot_event(msg: dict):
        msg_type = msg.get("type")
        if msg_type == "pong":
            print("<- [RX 封包] [PING ACK] 收到 Arduino 回應 (連線正常)！")
        elif msg_type == "safety_threshold":
            th_mm = msg.get("threshold_mm", 0)
            print(f"<- [防撞門檻回報] {th_mm} mm ({th_mm/10.0:.1f} cm)")
        elif msg_type == "e_stop":
            print(f"\n⚠️ [硬體防撞緊急煞停] {msg.get('reason', '')}")
        elif msg_type == "log" and msg.get("direction") == "RX":
            print(f"<- {msg.get('text', '')}")
        elif telemetry_state["print_stream"]:
            if msg_type == "ultrasonic":
                f_mm = msg.get("front_mm", -1)
                r_mm = msg.get("right_mm", -1)
                f_str = f"{f_mm} mm ({f_mm/10.0:.1f} cm)" if f_mm >= 0 else "無效/超出範圍"
                r_str = f"{r_mm} mm ({r_mm/10.0:.1f} cm)" if r_mm >= 0 else "無效/超出範圍"
                print(f"<- [超音波] 前方: {f_str} | 右方: {r_str}")
            elif msg_type == "imu":
                print(
                    f"<- [IMU] Yaw: {msg.get('yaw', 0.0):.1f}° | "
                    f"Pitch: {msg.get('pitch', 0.0):.1f}° | "
                    f"Roll: {msg.get('roll', 0.0):.1f}° | "
                    f"模式: {msg.get('mode_name', 'MANUAL')}"
                )

    controller.add_listener(on_robot_event)

    ports = controller.get_ports()
    if not ports:
        print("[警告] 找不到任何可用的 COM 序列埠！請檢查 Arduino 是否已連接。")
        target_port = input("請手動輸入 COM 埠名稱 (例如 COM20 或 /dev/ttyUSB0): ").strip()
    else:
        print("可用序列埠列表:")
        for idx, p in enumerate(ports):
            print(f"  [{idx + 1}] {p['device']} - {p['description']}")
        choice = input(f"請選擇序列埠 (1-{len(ports)}, 預設 1): ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            target_port = ports[int(choice) - 1]["device"]
        else:
            target_port = ports[0]["device"]

    baud = input("請輸入鮑率 Baudrate (預設 9600): ").strip()
    baudrate = int(baud) if baud.isdigit() else 9600

    print(f"[資訊] 正在連接 {target_port} (Baudrate: {baudrate})...")
    if not controller.connect(target_port, baudrate):
        print(f"[錯誤] 無法開啟序列埠 {target_port}")
        return
    print("[資訊] 連線成功！RobotController 通訊協定解析器與巡航心跳已啟動。")

    try:
        while True:
            status_print = "開啟" if telemetry_state["print_stream"] else "關閉 (靜音模式)"
            print("\n========== 控制選單 ==========")
            print("  1. 自動功能測試 (Demo 流程)")
            print("  2. 手動控制雙馬達 (指定 方向 Turn 與 速度 Speed)")
            print("  3. 雙馬達停止")
            print("  4. 控制單馬達 (Motor ID, Speed)")
            print("  5. 控制繼電器 (Relay ID, State)")
            print("  6. 發送 Ping 測試")
            print(f"  7. 切換終端機連續洗頻開關 (目前: {status_print})")
            print("  8. 檢視當前感測器與狀態快照 (單次顯示不洗頻)")
            print("  9. 設定目標航向導航 (指定任意角度 Target Yaw 與 速度 Speed)")
            print(" 10. 切換動態擺頭開關 (設定振幅與週期，行進速度直接沿用馬達/航向控制)")
            print(" 11. 煞車停止自主運動 (Stop Motion)")
            print(" 12. 重設 IMU 朝向歸零 (Yaw = 0°)")
            print(" 13. 設定超音波防撞距離門檻 (Set Safety Threshold mm)")
            print(" 14. 查詢當前防撞距離門檻 (Get Safety Threshold)")
            print(" 15. 進入即時單行監看儀表板 (動態刷新不洗屏，按 Ctrl+C 返回)")
            print(" 16. 設定原地旋轉轉速上限 (Turn Speed Limit, 預設 90 PWM)")
            print("  0. 離開程式")
            print("==============================")
            opt = input("請輸入選項 (0-16): ").strip()

            if opt == '1':
                auto_test_demo(controller)
            elif opt == '2':
                try:
                    t = int(input("請輸入轉向 Turn (-255右轉 ~ +255左轉，0為直行): "))
                    s = int(input("請輸入速度 Speed (-255後退 ~ +255前進): "))
                    controller.send_dual_motor(t, s)
                    print(f"-> [TX] 雙馬達: Turn={t}, Speed={s}")
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '3':
                controller.send_dual_motor(0, 0)
                print("-> [TX] 雙馬達已停止")
            elif opt == '4':
                try:
                    m_id = int(input("請輸入 Motor ID (0 為左，1 為右): "))
                    spd = int(input("請輸入速度 Speed (-255~+255): "))
                    controller.send_single_motor(m_id, spd)
                    print(f"-> [TX] 單馬達 ID={m_id}, Speed={spd}")
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '5':
                try:
                    r_id = int(input("請輸入 Relay ID (例如 1): "))
                    st = input("請輸入狀態 (1: 開啟 / 0: 關閉): ").strip()
                    controller.send_relay(r_id, st == '1')
                    print(f"-> [TX] Relay ID={r_id}, State={'ON' if st == '1' else 'OFF'}")
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '6':
                controller.send_ping()
                print("-> [TX] 發送 Ping (0x00)")
            elif opt == '7':
                telemetry_state["print_stream"] = not telemetry_state["print_stream"]
                print(f"[設定] 終端機即時洗頻顯示已改為: {'開啟' if telemetry_state['print_stream'] else '關閉 (已靜音)'}")
            elif opt == '8':
                state = controller.get_state()
                f_mm = state.get("front_mm", -1)
                r_mm = state.get("right_mm", -1)
                yaw = state.get("yaw", 0.0)
                mode_name = state.get("mode_name", "MANUAL")
                th_mm = state.get("safety_threshold_mm", 150)
                f_str = f"{f_mm} mm ({f_mm/10.0:.1f} cm)" if f_mm is not None and f_mm >= 0 else "無資料/超出範圍"
                r_str = f"{r_mm} mm ({r_mm/10.0:.1f} cm)" if r_mm is not None and r_mm >= 0 else "無資料/超出範圍"
                print("\n[即時感測器與狀態快照]")
                print(f"  - 前方超音波: {f_str}")
                print(f"  - 右方超音波: {r_str}")
                print(f"  - 當前朝向 Yaw: {yaw:6.1f}°")
                print(f"  - 運作模式: {mode_name}")
                print(f"  - 防撞門檻: {th_mm} mm ({th_mm/10.0:.1f} cm)\n")
            elif opt == '9':
                try:
                    target_yaw = float(input("請輸入目標角度 Target Yaw (-180.0 ~ +180.0度): "))
                    spd = int(input("請輸入行進速度 Speed (-255~+255，輸入 0 代表純原地旋轉對準): "))
                    controller.send_set_heading(target_yaw, spd)
                    print(f"-> [TX] 設定目標航向={target_yaw:.1f}°, 速度={spd}")
                except ValueError:
                    print("[錯誤] 請輸入有效數值！")
            elif opt == '10':
                try:
                    enable_str = input("是否啟用動態擺頭模式 (1: 開啟 / 0: 關閉，預設 1): ").strip()
                    en = (enable_str != '0')
                    if en:
                        amp = float(input("請輸入擺頭振幅 Amplitude (預設 20.0度): ") or "20.0")
                        period = int(input("請輸入擺頭週期 Period (毫秒，預設 2000): ") or "2000")
                        controller.send_set_sweep(True, amp, period)
                        print("[提示] 擺頭已開啟！現在您可以直接輸入選項 2 (手動前進) 或選項 9 (航向導航)，車子會邊推進邊左右擺動！")
                    else:
                        controller.send_set_sweep(False)
                        print("[提示] 擺頭已關閉，恢復一般直線/手動模式。")
                except ValueError:
                    print("[錯誤] 請輸入有效數值！")
            elif opt == '11':
                controller.send_stop_motion()
                print("-> [TX] 已停止自主運動")
            elif opt == '12':
                controller.send_reset_imu()
                print("-> [TX] 已重設 IMU Yaw = 0°")
            elif opt == '13':
                try:
                    th_mm = int(input("請輸入前方防撞安全距離門檻 (毫米 mm，例如 150 代表 15cm，輸入 0 代表關閉防撞): "))
                    controller.send_set_safety_threshold(th_mm)
                    print(f"-> [TX] 設定防撞門檻={th_mm} mm")
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '14':
                controller.send_get_safety_threshold()
                print("-> [TX] 已發送防撞門檻查詢請求")
            elif opt == '15':
                live_monitor(controller)
            elif opt == '16':
                try:
                    raw_spd = input("請輸入旋轉轉速上限 (40~220 PWM，建議 60~120，預設 90): ").strip()
                    spd = int(raw_spd) if raw_spd.isdigit() else 90
                    controller.send_set_turn_speed(spd)
                    print(f"-> [TX] 設定旋轉轉速上限={spd} PWM")
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '0':
                break
            else:
                print("[錯誤] 無效選項！")
    except KeyboardInterrupt:
        print("\n使用者中斷程式。")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
