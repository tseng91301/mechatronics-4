#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arduino 連線、控制與感測器接收測試腳本
支援雙向封包解析 (接收 Arduino 發送的 4 bytes 超音波毫米數據及自訂訊息)
"""

import sys
import time
import struct
import threading
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[錯誤] 未安裝 pyserial 模組。請在命令提示字元執行: pip install pyserial")
    sys.exit(1)

try:
    from control.server import (
        START_BYTE, END_BYTE,
        CMD_PING, CMD_DUAL_MOTOR, CMD_SINGLE_MOTOR, CMD_RELAY,
        CMD_SET_AUTO_CORRECT, CMD_RESET_IMU_YAW,
        CMD_SET_HEADING, CMD_SET_SWEEP, CMD_STOP_MOTION,
        CMD_SET_SAFETY_THRESH, CMD_GET_SAFETY_THRESH, CMD_SET_TURN_SPEED,
        CMD_ULTRASONIC, CMD_ULTRASONIC_FRONT, CMD_ULTRASONIC_RIGHT,
        CMD_IMU_DATA, CMD_SAFETY_THRESH_DATA, CMD_LOG_MSG,
        STATE_WAIT_START, STATE_WAIT_CMD, STATE_WAIT_DATA, STATE_WAIT_END
    )
except ImportError:
    try:
        from server import (
            START_BYTE, END_BYTE,
            CMD_PING, CMD_DUAL_MOTOR, CMD_SINGLE_MOTOR, CMD_RELAY,
            CMD_SET_AUTO_CORRECT, CMD_RESET_IMU_YAW,
            CMD_SET_HEADING, CMD_SET_SWEEP, CMD_STOP_MOTION,
            CMD_SET_SAFETY_THRESH, CMD_GET_SAFETY_THRESH, CMD_SET_TURN_SPEED,
            CMD_ULTRASONIC, CMD_ULTRASONIC_FRONT, CMD_ULTRASONIC_RIGHT,
            CMD_IMU_DATA, CMD_SAFETY_THRESH_DATA, CMD_LOG_MSG,
            STATE_WAIT_START, STATE_WAIT_CMD, STATE_WAIT_DATA, STATE_WAIT_END
        )
    except ImportError:
        START_BYTE = 0xAA
        END_BYTE   = 0x55
        CMD_PING              = 0x00
        CMD_DUAL_MOTOR        = 0x01
        CMD_SINGLE_MOTOR      = 0x02
        CMD_RELAY             = 0x03
        CMD_SET_AUTO_CORRECT  = 0x04
        CMD_RESET_IMU_YAW     = 0x05
        CMD_SET_HEADING       = 0x06
        CMD_SET_SWEEP         = 0x07
        CMD_STOP_MOTION       = 0x08
        CMD_SET_SAFETY_THRESH = 0x09
        CMD_GET_SAFETY_THRESH = 0x0A
        CMD_SET_TURN_SPEED    = 0x0B
        CMD_ULTRASONIC        = 0x10
        CMD_ULTRASONIC_FRONT  = 0x11
        CMD_ULTRASONIC_RIGHT  = 0x12
        CMD_IMU_DATA          = 0x13
        CMD_SAFETY_THRESH_DATA= 0x14
        CMD_LOG_MSG           = 0x20
        STATE_WAIT_START = 0
        STATE_WAIT_CMD   = 1
        STATE_WAIT_DATA  = 2
        STATE_WAIT_END   = 3


class ArduinoController:
    def __init__(self, port: str, baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = False
        self.rx_thread = None

        # 是否在終端機即時列印接收到的感測器封包 (預設關閉靜音，避免連續洗頻干擾選單輸入)
        self.print_ultrasonic = False
        self.print_telemetry = False

        # 巡航心跳保持
        self.active_heading = None
        self.heartbeat_thread = None

        # 最新接收到的狀態數據
        self.latest_data = {
            'front_mm': None,
            'right_mm': None,
            'yaw': 0.0,
            'pitch': 0.0,
            'roll': 0.0,
            'mode': 0,
            'mode_str': '手動 (MANUAL)',
            'safety_threshold_mm': 150,
            'last_seen': 0
        }

        # 狀態機接收緩衝
        self.rx_state = STATE_WAIT_START
        self.current_cmd = 0
        self.expected_len = 0
        self.rx_buffer = bytearray()

        # 預期長度表 (CMD -> payload_length)
        self.cmd_lengths = {
            CMD_PING: 0,
            CMD_DUAL_MOTOR: 4,
            CMD_SINGLE_MOTOR: 3,
            CMD_RELAY: 2,
            CMD_SET_AUTO_CORRECT: 1,
            CMD_RESET_IMU_YAW: 0,
            CMD_SET_HEADING: 4,
            CMD_SET_SWEEP: 5,
            CMD_STOP_MOTION: 0,
            CMD_SET_SAFETY_THRESH: 2,
            CMD_GET_SAFETY_THRESH: 0,
            CMD_SET_TURN_SPEED: 2,
            CMD_SAFETY_THRESH_DATA: 2,
            CMD_ULTRASONIC_FRONT: 4,
            CMD_ULTRASONIC_RIGHT: 4,
            CMD_IMU_DATA: 7,
        }

    def connect(self):
        try:
            print(f"[資訊] 正在連接 {self.port} (Baudrate: {self.baudrate})...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            # 等待 Arduino 重啟復位
            time.sleep(2.0)
            self.is_running = True
            self.rx_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.rx_thread.start()
            self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self.heartbeat_thread.start()
            print("[資訊] 連線成功！通訊協定解析器與巡航心跳已啟動。")
            return True
        except Exception as e:
            print(f"[錯誤] 無法開啟序列埠 {self.port}: {e}")
            return False

    def close(self):
        self.is_running = False
        self.active_heading = None
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[資訊] 序列埠已關閉。")

    def _heartbeat_loop(self):
        """背景定時發送巡航心跳，維持 Arduino 目標航向自主運動不超時"""
        while self.is_running:
            try:
                if self.active_heading is not None and self.ser and self.ser.is_open:
                    yaw, spd = self.active_heading
                    if spd != 0:
                        yaw_x10 = max(-1800, min(1800, int(round(yaw * 10.0))))
                        spd_val = max(-255, min(255, int(spd)))
                        payload = struct.pack('>hh', yaw_x10, spd_val)
                        packet = bytearray([START_BYTE, CMD_SET_HEADING])
                        packet.extend(payload)
                        packet.append(END_BYTE)
                        self.ser.write(packet)
                time.sleep(0.2)
            except Exception:
                time.sleep(0.2)

    def send_packet(self, cmd: int, payload: bytes = b''):
        """打包並發送協定封包: [START_BYTE] [CMD] [PAYLOAD...] [END_BYTE]"""
        if not self.ser or not self.ser.is_open:
            print("[警告] 尚未連線，無法發送封包。")
            return

        packet = bytearray([START_BYTE, cmd])
        packet.extend(payload)
        packet.append(END_BYTE)

        self.ser.write(packet)
        hex_str = ' '.join(f"{b:02X}" for b in packet)
        print(f"-> [TX] CMD: 0x{cmd:02X} | 原始封包: {hex_str}")

    def send_ping(self):
        """發送連線測試 Ping"""
        self.send_packet(CMD_PING, b'')

    def send_dual_motor(self, turn: int, speed: int):
        """
        控制雙馬達
        :param turn: 方向 (-255 ~ +255)，正值逆時針 (左轉)，負值順時針 (右轉)
        :param speed: 速度 (-255 ~ +255)，正值前進，負值後退
        """
        if turn == 0 and speed == 0:
            self.active_heading = None
        turn = max(-255, min(255, int(turn)))
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>hh', turn, speed)
        print(f"[控制雙馬達] Turn={turn} ({'逆時針' if turn>0 else '順時針' if turn<0 else '直行'}), Speed={speed}")
        self.send_packet(CMD_DUAL_MOTOR, payload)

    def send_single_motor(self, motor_id: int, speed: int):
        """控制單馬達"""
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>Bh', motor_id, speed)
        print(f"[控制單馬達] Motor={motor_id}, Speed={speed}")
        self.send_packet(CMD_SINGLE_MOTOR, payload)

    def send_relay(self, relay_id: int, state: bool):
        """控制繼電器"""
        payload = struct.pack('>BB', relay_id, 1 if state else 0)
        print(f"[控制繼電器] Relay={relay_id}, State={'ON' if state else 'OFF'}")
        self.send_packet(CMD_RELAY, payload)

    def send_set_heading(self, target_yaw: float, speed: int = 0):
        """指定目標航向導航 (0x06): target_yaw (deg), speed (-255~255)"""
        yaw_x10 = max(-1800, min(1800, int(round(target_yaw * 10.0))))
        speed = max(-255, min(255, int(speed)))
        payload = struct.pack('>hh', yaw_x10, speed)
        print(f"[設定目標航向] 目標角度={target_yaw:.1f}°, 行進速度={speed}")
        if speed != 0:
            self.active_heading = (target_yaw, speed)
        else:
            self.active_heading = None
        self.send_packet(CMD_SET_HEADING, payload)

    def send_set_sweep(self, enable: bool, amp: float = 20.0, period_ms: int = 2000):
        """設定或開關擺頭模式 (0x07): enable (bool), amp (deg), period_ms (ms)"""
        en_val = 1 if enable else 0
        amp_x10 = max(0, min(900, int(round(amp * 10.0))))
        period_ms = max(500, min(65535, int(period_ms)))
        payload = struct.pack('>BhH', en_val, amp_x10, period_ms)
        print(f"[動態擺頭設定] {'開啟 ON' if enable else '關閉 OFF'} (振幅=±{amp:.1f}°, 週期={period_ms}ms)")
        self.send_packet(CMD_SET_SWEEP, payload)

    def send_stop_motion(self):
        """煞停並停止任何航向/擺頭運動 (0x08)"""
        self.active_heading = None
        print("[停止運動] 解除航向/擺頭鎖定並煞停馬達")
        self.send_packet(CMD_STOP_MOTION, b'')

    def send_reset_imu(self):
        """重設 IMU 朝向歸零 (0x05)"""
        print("[重設 IMU] 朝向歸零 Yaw = 0°")
        self.send_packet(CMD_RESET_IMU_YAW, b'')

    def send_set_safety_threshold(self, threshold_mm: int):
        """設定超音波防撞距離門檻 (0x09)"""
        threshold_mm = max(0, min(65535, int(threshold_mm)))
        payload = struct.pack('>H', threshold_mm)
        print(f"[設定防撞門檻] 門檻={threshold_mm} mm ({threshold_mm/10.0:.1f} cm)")
        self.send_packet(CMD_SET_SAFETY_THRESH, payload)

    def send_get_safety_threshold(self):
        """查詢當前超音波防撞門檻 (0x0A)"""
        print("[查詢防撞門檻] 發送查詢請求...")
        self.send_packet(CMD_GET_SAFETY_THRESH, b'')

    def send_set_turn_speed(self, turn_speed: int):
        """設定原地旋轉與轉向轉速上限 (CMD 0x0B, 2 bytes int16_t, 範圍 40~220, 預設 90)"""
        clamped = max(40, min(220, int(turn_speed)))
        payload = struct.pack('>h', clamped)
        print(f"[設定旋轉速度上限] 轉向轉速={clamped} PWM (預設 90)")
        self.send_packet(CMD_SET_TURN_SPEED, payload)

    # ------------------ 接收與狀態機解析 ------------------

    def _read_loop(self):
        """背景執行緒：以狀態機持續讀取並解出 Arduino 傳來的封包"""
        while self.is_running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    raw_bytes = self.ser.read(self.ser.in_waiting)
                    for byte in raw_bytes:
                        self._process_byte(byte)
                else:
                    time.sleep(0.005)
            except Exception as e:
                if self.is_running:
                    print(f"\n[接收例外] {e}")
                break

    def _process_byte(self, b: int):
        """解析器狀態機處理單一 byte"""
        if self.rx_state == STATE_WAIT_START:
            if b == START_BYTE:
                self.rx_state = STATE_WAIT_CMD

        elif self.rx_state == STATE_WAIT_CMD:
            self.current_cmd = b
            self.rx_buffer.clear()

            # 檢查指令長度規則
            if b in self.cmd_lengths:
                self.expected_len = self.cmd_lengths[b]
                if self.expected_len == 0:
                    self.rx_state = STATE_WAIT_END
                else:
                    self.rx_state = STATE_WAIT_DATA
            elif b == CMD_ULTRASONIC:
                # 支援 4 bytes (mm)
                self.expected_len = 4
                self.rx_state = STATE_WAIT_DATA
            elif b == CMD_LOG_MSG:
                # 文字訊息可變長度，動態讀取直到遇上 END_BYTE
                self.expected_len = 32
                self.rx_state = STATE_WAIT_DATA
            else:
                # 未知指令，若收到 START_BYTE 則留在 WAIT_CMD，否則重置
                if b != START_BYTE:
                    self.rx_state = STATE_WAIT_START

        elif self.rx_state == STATE_WAIT_DATA:
            # 特殊處理：若是 CMD_LOG_MSG，只要遇到 END_BYTE 就提早結束
            if self.current_cmd == CMD_LOG_MSG and b == END_BYTE:
                self._dispatch_packet(self.current_cmd, bytes(self.rx_buffer))
                self.rx_state = STATE_WAIT_START
                return

            self.rx_buffer.append(b)
            if len(self.rx_buffer) >= self.expected_len:
                self.rx_state = STATE_WAIT_END

        elif self.rx_state == STATE_WAIT_END:
            if b == END_BYTE:
                # 完整校驗通過，派發指令處理
                self._dispatch_packet(self.current_cmd, bytes(self.rx_buffer))
            self.rx_state = STATE_WAIT_START

    def _dispatch_packet(self, cmd: int, payload: bytes):
        """依照 CMD 代碼解碼各類資料 (特別處理 4-byte 毫米整數)"""
        now = time.time()

        if cmd == CMD_PING:
            print("<- [RX 封包] [PING ACK] 收到 Arduino 回應 (連線正常)！")

        elif cmd == CMD_ULTRASONIC_FRONT:
            # 4 bytes int32 (Big-Endian 有號整數)
            if len(payload) >= 4:
                dist_mm = struct.unpack('>i', payload[:4])[0]
                self.latest_data['front_mm'] = dist_mm
                self.latest_data['last_seen'] = now

                if self.print_ultrasonic or self.print_telemetry:
                    if dist_mm < 0:
                        print("<- [前方超音波] 距離: 無效 / 超出範圍")
                    else:
                        print(f"<- [前方超音波] 距離: {dist_mm} mm ({dist_mm / 10.0:.1f} cm)")

        elif cmd == CMD_ULTRASONIC_RIGHT:
            # 4 bytes int32 (Big-Endian 有號整數)
            if len(payload) >= 4:
                dist_mm = struct.unpack('>i', payload[:4])[0]
                self.latest_data['right_mm'] = dist_mm
                self.latest_data['last_seen'] = now

                if self.print_ultrasonic or self.print_telemetry:
                    if dist_mm < 0:
                        print("<- [右方超音波] 距離: 無效 / 超出範圍")
                    else:
                        print(f"<- [右方超音波] 距離: {dist_mm} mm ({dist_mm / 10.0:.1f} cm)")

        elif cmd == CMD_ULTRASONIC:
            if len(payload) == 4:
                dist_mm = struct.unpack('>i', payload)[0]
                if self.print_ultrasonic or self.print_telemetry:
                    print(f"<- [超音波] 距離: {dist_mm} mm ({dist_mm / 10.0:.1f} cm)")
            elif len(payload) == 5:
                sensor_id, dist_mm = struct.unpack('>Bi', payload)
                name = "前方" if sensor_id == 0 else "右方" if sensor_id == 1 else f"ID {sensor_id}"
                if self.print_ultrasonic or self.print_telemetry:
                    print(f"<- [{name}超音波] 距離: {dist_mm} mm ({dist_mm / 10.0:.1f} cm)")

        elif cmd == CMD_IMU_DATA:
            if len(payload) >= 7:
                y10, p10, r10, mode = struct.unpack('>hhhB', payload[:7])
                yaw = y10 / 10.0
                pitch = p10 / 10.0
                roll = r10 / 10.0
                is_heading = bool(mode & 0x01)
                is_sweep = bool(mode & 0x02)
                base_str = "目標航向 (HEADING)" if is_heading else "手動 (MANUAL)"
                mode_str = f"{base_str}+擺頭(SWEEP)" if is_sweep else base_str

                self.latest_data['yaw'] = yaw
                self.latest_data['pitch'] = pitch
                self.latest_data['roll'] = roll
                self.latest_data['mode'] = mode
                self.latest_data['mode_str'] = mode_str
                self.latest_data['last_seen'] = now

                if self.print_ultrasonic or self.print_telemetry:
                    print(f"<- [IMU 姿態] Yaw: {yaw:6.1f}° | Pitch: {pitch:5.1f}° | Roll: {roll:5.1f}° | 狀態: {mode_str}")

        elif cmd == CMD_SAFETY_THRESH_DATA:
            if len(payload) >= 2:
                thresh_mm = struct.unpack('>H', payload[:2])[0]
                self.latest_data['safety_threshold_mm'] = thresh_mm
                print(f"<- [防撞安全門檻回應] 目前門檻值: {thresh_mm} mm ({thresh_mm/10.0:.1f} cm)")

        elif cmd == CMD_LOG_MSG:
            msg = payload.decode('utf-8', errors='ignore')
            print(f"<- [Arduino 系統訊息] {msg}")

        else:
            hex_str = ' '.join(f"{b:02X}" for b in payload)
            print(f"<- [RX 封包] 未知指令 0x{cmd:02X} | Payload: {hex_str}")


def auto_test_demo(controller: ArduinoController):
    """自動示範測試流程"""
    print("\n========== 開始自動測試示範 ==========")
    
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


def live_monitor(controller: ArduinoController):
    """進入即時單行監看儀表板，動態更新數值不洗屏，按 Ctrl+C 或 Enter 返回選單"""
    print("\n[進入即時監看模式] 數值將在單行即時更新，按 Ctrl+C 返回控制選單...\n")
    try:
        while True:
            f_mm = controller.latest_data.get('front_mm')
            r_mm = controller.latest_data.get('right_mm')
            yaw = controller.latest_data.get('yaw', 0.0)
            mode_str = controller.latest_data.get('mode_str', '手動 (MANUAL)')
            f_str = f"{f_mm/10.0:5.1f}cm" if f_mm is not None and f_mm >= 0 else "  -- cm"
            r_str = f"{r_mm/10.0:5.1f}cm" if r_mm is not None and r_mm >= 0 else "  -- cm"
            sys.stdout.write(f"\r>> 前方: {f_str} | 右方: {r_str} | Yaw: {yaw:6.1f}° | 模式: {mode_str:<18}")
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("\n\n[已返回控制選單]\n")


def main():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[警告] 找不到任何可用的 COM 序列埠！請檢查 Arduino 是否已連接。")
        target_port = input("請手動輸入 COM 埠名稱 (例如 COM20): ").strip()
    else:
        print("可用序列埠列表:")
        for idx, p in enumerate(ports):
            print(f"  [{idx + 1}] {p.device} - {p.description}")
        choice = input(f"請選擇序列埠 (1-{len(ports)}, 預設 1): ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            target_port = ports[int(choice) - 1].device
        else:
            target_port = ports[0].device

    baud = input("請輸入鮑率 Baudrate (預設 9600): ").strip()
    baudrate = int(baud) if baud.isdigit() else 9600

    controller = ArduinoController(target_port, baudrate)
    if not controller.connect():
        return

    try:
        while True:
            status_print = "開啟" if controller.print_ultrasonic else "關閉 (靜音模式)"
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
                    turn = int(input("請輸入方向 Turn (-255~+255，正為逆時針/左轉，負為順時針/右轉): "))
                    speed = int(input("請輸入速度 Speed (-255~+255，正為前進，負為後退): "))
                    controller.send_dual_motor(turn, speed)
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '3':
                controller.send_dual_motor(0, 0)
            elif opt == '4':
                try:
                    m_id = int(input("請輸入 Motor ID (0 為左，1 為右): "))
                    spd = int(input("請輸入速度 Speed (-255~+255): "))
                    controller.send_single_motor(m_id, spd)
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '5':
                try:
                    r_id = int(input("請輸入 Relay ID (例如 1): "))
                    st = input("請輸入狀態 (1: 開啟 / 0: 關閉): ").strip()
                    controller.send_relay(r_id, st == '1')
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '6':
                controller.send_ping()
            elif opt == '7':
                controller.print_ultrasonic = not controller.print_ultrasonic
                controller.print_telemetry = controller.print_ultrasonic
                print(f"[設定] 終端機即時洗頻顯示已改為: {'開啟' if controller.print_ultrasonic else '關閉 (已靜音)'}")
            elif opt == '8':
                f_mm = controller.latest_data.get('front_mm')
                r_mm = controller.latest_data.get('right_mm')
                yaw = controller.latest_data.get('yaw', 0.0)
                mode_str = controller.latest_data.get('mode_str', '手動 (MANUAL)')
                th_mm = controller.latest_data.get('safety_threshold_mm', 150)
                f_str = f"{f_mm} mm ({f_mm/10.0:.1f} cm)" if f_mm is not None and f_mm >= 0 else "無資料/超出範圍"
                r_str = f"{r_mm} mm ({r_mm/10.0:.1f} cm)" if r_mm is not None and r_mm >= 0 else "無資料/超出範圍"
                print(f"\n[即時感測器與狀態快照]")
                print(f"  - 前方超音波: {f_str}")
                print(f"  - 右方超音波: {r_str}")
                print(f"  - 當前朝向 Yaw: {yaw:6.1f}°")
                print(f"  - 運作模式: {mode_str}")
                print(f"  - 防撞門檻: {th_mm} mm ({th_mm/10.0:.1f} cm)\n")
            elif opt == '9':
                try:
                    target_yaw = float(input("請輸入目標角度 Target Yaw (-180.0 ~ +180.0度): "))
                    spd = int(input("請輸入行進速度 Speed (-255~+255，輸入 0 代表純原地旋轉對準): "))
                    controller.send_set_heading(target_yaw, spd)
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
            elif opt == '12':
                controller.send_reset_imu()
            elif opt == '13':
                try:
                    th_mm = int(input("請輸入前方防撞安全距離門檻 (毫米 mm，例如 150 代表 15cm，輸入 0 代表關閉防撞): "))
                    controller.send_set_safety_threshold(th_mm)
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '14':
                controller.send_get_safety_threshold()
            elif opt == '15':
                live_monitor(controller)
            elif opt == '16':
                try:
                    raw_spd = input("請輸入旋轉轉速上限 (40~220 PWM，建議 60~120，預設 90): ").strip()
                    spd = int(raw_spd) if raw_spd.isdigit() else 90
                    controller.send_set_turn_speed(spd)
                except ValueError:
                    print("[錯誤] 請輸入整數！")
            elif opt == '0':
                break
            else:
                print("[錯誤] 無效選項！")
    except KeyboardInterrupt:
        print("\n使用者中斷程式。")
    finally:
        controller.send_dual_motor(0, 0)
        controller.close()


if __name__ == "__main__":
    main()
