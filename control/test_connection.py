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
        CMD_ULTRASONIC, CMD_ULTRASONIC_FRONT, CMD_ULTRASONIC_RIGHT,
        CMD_IMU_DATA, CMD_LOG_MSG,
        STATE_WAIT_START, STATE_WAIT_CMD, STATE_WAIT_DATA, STATE_WAIT_END
    )
except ImportError:
    try:
        from server import (
            START_BYTE, END_BYTE,
            CMD_PING, CMD_DUAL_MOTOR, CMD_SINGLE_MOTOR, CMD_RELAY,
            CMD_SET_AUTO_CORRECT, CMD_RESET_IMU_YAW,
            CMD_ULTRASONIC, CMD_ULTRASONIC_FRONT, CMD_ULTRASONIC_RIGHT,
            CMD_IMU_DATA, CMD_LOG_MSG,
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
        CMD_ULTRASONIC        = 0x10
        CMD_ULTRASONIC_FRONT  = 0x11
        CMD_ULTRASONIC_RIGHT  = 0x12
        CMD_IMU_DATA          = 0x13
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

        # 是否在終端機即時列印接收到的超音波封包 (預設開，可動態切換)
        self.print_ultrasonic = True

        # 最新接收到的狀態數據
        self.latest_data = {
            'front_mm': None,
            'right_mm': None,
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
            print("[資訊] 連線成功！通訊協定解析器已啟動。")
            return True
        except Exception as e:
            print(f"[錯誤] 無法開啟序列埠 {self.port}: {e}")
            return False

    def close(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[資訊] 序列埠已關閉。")

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

                if self.print_ultrasonic:
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

                if self.print_ultrasonic:
                    if dist_mm < 0:
                        print("<- [右方超音波] 距離: 無效 / 超出範圍")
                    else:
                        print(f"<- [右方超音波] 距離: {dist_mm} mm ({dist_mm / 10.0:.1f} cm)")

        elif cmd == CMD_ULTRASONIC:
            if len(payload) == 4:
                dist_mm = struct.unpack('>i', payload)[0]
                if self.print_ultrasonic:
                    print(f"<- [超音波] 距離: {dist_mm} mm ({dist_mm / 10.0:.1f} cm)")
            elif len(payload) == 5:
                sensor_id, dist_mm = struct.unpack('>Bi', payload)
                name = "前方" if sensor_id == 0 else "右方" if sensor_id == 1 else f"ID {sensor_id}"
                if self.print_ultrasonic:
                    print(f"<- [{name}超音波] 距離: {dist_mm} mm ({dist_mm / 10.0:.1f} cm)")

        elif cmd == CMD_IMU_DATA:
            if len(payload) >= 7:
                y10, p10, r10, ac = struct.unpack('>hhhB', payload[:7])
                yaw = y10 / 10.0
                pitch = p10 / 10.0
                roll = r10 / 10.0
                if self.print_ultrasonic:
                    print(f"<- [IMU 姿態] Yaw(朝向): {yaw:.1f}° | Pitch: {pitch:.1f}° | Roll: {roll:.1f}° | 自動校正: {'ON' if ac else 'OFF'}")

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
            print(f"  7. 切換超音波即時洗頻顯示 (目前: {status_print})")
            print("  8. 檢視最新一次超音波數值")
            print("  0. 離開程式")
            print("==============================")
            opt = input("請輸入選項 (0-8): ").strip()

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
                print(f"[設定] 超音波即時洗頻顯示已改為: {'開啟' if controller.print_ultrasonic else '關閉'}")
            elif opt == '8':
                f_mm = controller.latest_data['front_mm']
                r_mm = controller.latest_data['right_mm']
                f_str = f"{f_mm} mm ({f_mm/10.0:.1f} cm)" if f_mm is not None and f_mm >= 0 else "無資料/超出範圍"
                r_str = f"{r_mm} mm ({r_mm/10.0:.1f} cm)" if r_mm is not None and r_mm >= 0 else "無資料/超出範圍"
                print(f"\n[即時超音波數據快照]")
                print(f"  - 前方距離: {f_str}")
                print(f"  - 右方距離: {r_str}\n")
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
