#ifndef CONNECTION_H
#define CONNECTION_H

#include <Arduino.h>

// 常見預設指令定義 (亦可自訂)
#define CMD_PING              0x00  // 測試連線 (Payload: 0 bytes)
#define CMD_DUAL_MOTOR        0x01  // 控制雙馬達 (Payload: 4 bytes -> turn: int16, speed: int16)
#define CMD_SINGLE_MOTOR      0x02  // 控制單馬達 (Payload: 3 bytes -> motor_id: uint8, speed: int16)
#define CMD_RELAY             0x03  // 控制繼電器 (Payload: 2 bytes -> relay_id: uint8, state: uint8)
#define CMD_SET_AUTO_CORRECT  0x04  // 自動校正開關 (Payload: 1 byte -> enable: uint8, 1:開, 0:關)
#define CMD_RESET_IMU_YAW     0x05  // 朝向歸零/重設 (Payload: 0 bytes)

// Arduino 發送給電腦的訊息指令
#define CMD_ULTRASONIC        0x10  // 超音波數據 (Payload: 4 bytes int32 毫米，或 5 bytes: id + mm)
#define CMD_ULTRASONIC_FRONT  0x11  // 前方超音波數據 (Payload: 4 bytes int32 毫米)
#define CMD_ULTRASONIC_RIGHT  0x12  // 右方超音波數據 (Payload: 4 bytes int32 毫米)
#define CMD_IMU_DATA          0x13  // IMU 姿態數據 (Payload: 7 bytes -> yaw_x10: int16, pitch_x10: int16, roll_x10: int16, auto_correct: uint8)
#define CMD_LOG_MSG           0x20  // Arduino 發送文字訊息給電腦 (Payload: 字串)

/**
 * @brief 指令回呼函式型別定義
 * @param payload 數值 bytes 緩衝區
 * @param length 數值 bytes 長度
 */
typedef void (*CommandHandler)(const uint8_t* payload, uint8_t length);

/**
 * @brief Arduino 與電腦通訊協議解析與傳輸類別
 *
 * 封包格式:
 * [START_BYTE] [CMD_BYTE] [VALUE_BYTES...] [END_BYTE]
 * - START_BYTE: 0xAA (預設)
 * - CMD_BYTE: 控制代碼 (1 byte)
 * - VALUE_BYTES: 依 CMD_BYTE 不同而有不同長度
 * - END_BYTE: 0x55 (預設)
 */
class SimpleConnection {
public:
    static const uint8_t DEFAULT_START_BYTE = 0xAA;
    static const uint8_t DEFAULT_END_BYTE   = 0x55;
    static const uint8_t MAX_PAYLOAD_SIZE   = 32;   // 最大支援 payload 長度
    static const uint8_t MAX_COMMANDS       = 16;   // 最多註冊指令數量

private:
    struct CommandEntry {
        uint8_t cmd;
        uint8_t payloadLength;
        CommandHandler handler;
    };

    enum RxState {
        STATE_WAIT_START,
        STATE_WAIT_CMD,
        STATE_WAIT_DATA,
        STATE_WAIT_END
    };

    Stream* _stream;
    uint8_t _startByte;
    uint8_t _endByte;

    CommandEntry _commands[MAX_COMMANDS];
    uint8_t _commandCount;

    RxState _state;
    uint8_t _currentCmd;
    uint8_t _expectedPayloadLen;
    uint8_t _receivedDataLen;
    uint8_t _rxBuffer[MAX_PAYLOAD_SIZE];
    CommandHandler _activeHandler;

public:
    /**
     * @brief 建構子
     * @param stream 序列埠串流 (預設 &Serial)
     * @param startByte 起始 byte (預設 0xAA)
     * @param endByte 結尾 byte (預設 0x55)
     */
    SimpleConnection(Stream* stream = &Serial, 
                     uint8_t startByte = DEFAULT_START_BYTE, 
                     uint8_t endByte = DEFAULT_END_BYTE)
        : _stream(stream), 
          _startByte(startByte), 
          _endByte(endByte), 
          _commandCount(0), 
          _state(STATE_WAIT_START), 
          _currentCmd(0), 
          _expectedPayloadLen(0), 
          _receivedDataLen(0), 
          _activeHandler(nullptr) {}

    /**
     * @brief 設定使用的 Serial 串流
     */
    void setStream(Stream* stream) {
        _stream = stream;
    }

    /**
     * @brief 註冊控制代碼及其預期的 payload 長度與回呼函式
     * @param cmd 控制代碼 (例如 0x01, 0x02)
     * @param payloadLength 該指令隨附的數值 bytes 長度
     * @param handler 收到該指令並驗證結束 byte 正確時呼叫的函式
     * @return true 註冊成功, false 註冊數量已達上限
     */
    bool registerCommand(uint8_t cmd, uint8_t payloadLength, CommandHandler handler) {
        if (_commandCount >= MAX_COMMANDS) {
            return false;
        }
        _commands[_commandCount].cmd = cmd;
        _commands[_commandCount].payloadLength = payloadLength;
        _commands[_commandCount].handler = handler;
        _commandCount++;
        return true;
    }

    /**
     * @brief 輪詢接收與解析序列埠封包 (放在 loop() 中呼叫)
     */
    void service() {
        if (!_stream) return;

        while (_stream->available() > 0) {
            uint8_t b = (uint8_t)_stream->read();
            processByte(b);
        }
    }

    /**
     * @brief 處理單一 byte 的狀態機轉換
     */
    void processByte(uint8_t b) {
        switch (_state) {
            case STATE_WAIT_START:
                if (b == _startByte) {
                    _state = STATE_WAIT_CMD;
                }
                break;

            case STATE_WAIT_CMD: {
                _currentCmd = b;
                int cmdIndex = findCommandIndex(b);
                if (cmdIndex >= 0) {
                    _expectedPayloadLen = _commands[cmdIndex].payloadLength;
                    _activeHandler = _commands[cmdIndex].handler;
                    _receivedDataLen = 0;

                    if (_expectedPayloadLen == 0) {
                        _state = STATE_WAIT_END;
                    } else if (_expectedPayloadLen <= MAX_PAYLOAD_SIZE) {
                        _state = STATE_WAIT_DATA;
                    } else {
                        // 超過支援長度，重置
                        resetRx();
                    }
                } else {
                    // 若收到未知的 cmd，但如果是 startByte，保持等待 cmd
                    if (b != _startByte) {
                        resetRx();
                    }
                }
                break;
            }

            case STATE_WAIT_DATA:
                _rxBuffer[_receivedDataLen++] = b;
                if (_receivedDataLen >= _expectedPayloadLen) {
                    _state = STATE_WAIT_END;
                }
                break;

            case STATE_WAIT_END:
                if (b == _endByte) {
                    // 封包完整正確，執行對應處理
                    if (_activeHandler) {
                        _activeHandler(_rxBuffer, _expectedPayloadLen);
                    }
                }
                resetRx();
                break;
        }
    }

    /**
     * @brief 發送標準封包給電腦: [START] [CMD] [DATA...] [END]
     * @param cmd 控制/回應代碼
     * @param payload 數值 bytes
     * @param length 數值 bytes 長度
     */
    void sendPacket(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t length = 0) {
        if (!_stream) return;
        _stream->write(_startByte);
        _stream->write(cmd);
        if (payload != nullptr && length > 0) {
            _stream->write(payload, length);
        }
        _stream->write(_endByte);
    }

    /**
     * @brief 發送包含 4-byte int32_t 的封包 (大端序 Big-Endian)
     * @param cmd 指令代碼
     * @param val 4-byte 整數數值 (例如超音波距離，毫米)
     */
    void sendInt32(uint8_t cmd, int32_t val) {
        uint8_t buf[4];
        buf[0] = (uint8_t)((val >> 24) & 0xFF);
        buf[1] = (uint8_t)((val >> 16) & 0xFF);
        buf[2] = (uint8_t)((val >> 8) & 0xFF);
        buf[3] = (uint8_t)(val & 0xFF);
        sendPacket(cmd, buf, 4);
    }

    /**
     * @brief 發送超音波數值 (4 bytes int32，單位：毫米 mm)
     * @param cmd 超音波指令代碼 (如 CMD_ULTRASONIC_FRONT 或 CMD_ULTRASONIC_RIGHT)
     * @param distance_mm 距離數值 (毫米)
     */
    void sendUltrasonic(uint8_t cmd, int32_t distance_mm) {
        sendInt32(cmd, distance_mm);
    }

    /**
     * @brief 發送帶有 ID 的超音波數值 (1 byte ID + 4 bytes int32 mm，共 5 bytes)
     * @param sensorId 感測器編號 (0: 前方, 1: 右方)
     * @param distance_mm 距離數值 (毫米)
     */
    void sendUltrasonicWithId(uint8_t sensorId, int32_t distance_mm) {
        uint8_t buf[5];
        buf[0] = sensorId;
        buf[1] = (uint8_t)((distance_mm >> 24) & 0xFF);
        buf[2] = (uint8_t)((distance_mm >> 16) & 0xFF);
        buf[3] = (uint8_t)((distance_mm >> 8) & 0xFF);
        buf[4] = (uint8_t)(distance_mm & 0xFF);
        sendPacket(CMD_ULTRASONIC, buf, 5);
    }

    /**
     * @brief 發送包含兩個 int16_t 的回應封包 (大端序 Big-Endian)
     */
    void sendResponseInt16(uint8_t cmd, int16_t val1, int16_t val2) {
        uint8_t buf[4];
        buf[0] = (uint8_t)((val1 >> 8) & 0xFF);
        buf[1] = (uint8_t)(val1 & 0xFF);
        buf[2] = (uint8_t)((val2 >> 8) & 0xFF);
        buf[3] = (uint8_t)(val2 & 0xFF);
        sendPacket(cmd, buf, 4);
    }

    /**
     * @brief 發送 IMU 姿態封包 (7 bytes: yaw_x10: int16, pitch_x10: int16, roll_x10: int16, autoCorrect: uint8)
     */
    void sendIMUData(int16_t yaw_x10, int16_t pitch_x10, int16_t roll_x10, uint8_t autoCorrect) {
        uint8_t buf[7];
        buf[0] = (uint8_t)((yaw_x10 >> 8) & 0xFF);
        buf[1] = (uint8_t)(yaw_x10 & 0xFF);
        buf[2] = (uint8_t)((pitch_x10 >> 8) & 0xFF);
        buf[3] = (uint8_t)(pitch_x10 & 0xFF);
        buf[4] = (uint8_t)((roll_x10 >> 8) & 0xFF);
        buf[5] = (uint8_t)(roll_x10 & 0xFF);
        buf[6] = autoCorrect;
        sendPacket(CMD_IMU_DATA, buf, 7);
    }

    /**
     * @brief 發送文字訊息給電腦 (CMD_LOG_MSG)
     */
    void sendLog(const char* msg) {
        if (!msg) return;
        uint8_t len = strlen(msg);
        if (len > MAX_PAYLOAD_SIZE) len = MAX_PAYLOAD_SIZE;
        sendPacket(CMD_LOG_MSG, (const uint8_t*)msg, len);
    }

    // --- 輔助靜態工具函式：從 buffer 讀取數值 (大端序 Big-Endian) ---

    static int32_t readInt32(const uint8_t* buffer, uint8_t offset = 0) {
        return (int32_t)(
            ((uint32_t)buffer[offset] << 24) |
            ((uint32_t)buffer[offset + 1] << 16) |
            ((uint32_t)buffer[offset + 2] << 8) |
            ((uint32_t)buffer[offset + 3])
        );
    }

    static int16_t readInt16(const uint8_t* buffer, uint8_t offset = 0) {
        return (int16_t)(((uint16_t)buffer[offset] << 8) | buffer[offset + 1]);
    }

    static uint16_t readUInt16(const uint8_t* buffer, uint8_t offset = 0) {
        return (uint16_t)(((uint16_t)buffer[offset] << 8) | buffer[offset + 1]);
    }

    static int8_t readInt8(const uint8_t* buffer, uint8_t offset = 0) {
        return (int8_t)buffer[offset];
    }

    static uint8_t readUInt8(const uint8_t* buffer, uint8_t offset = 0) {
        return buffer[offset];
    }

private:
    int findCommandIndex(uint8_t cmd) {
        for (uint8_t i = 0; i < _commandCount; i++) {
            if (_commands[i].cmd == cmd) {
                return i;
            }
        }
        return -1;
    }

    void resetRx() {
        _state = STATE_WAIT_START;
        _currentCmd = 0;
        _expectedPayloadLen = 0;
        _receivedDataLen = 0;
        _activeHandler = nullptr;
    }
};

#endif // CONNECTION_H
