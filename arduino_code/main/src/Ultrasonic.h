#ifndef ULTRASONIC_H
#define ULTRASONIC_H 1

#include <Arduino.h>

/**
 * @brief 超音波測距類別 (含 5-Frame 中位數濾波與掉幀零階保持)
 */
class Ultrasonic {
private:
    uint8_t _trigPin;
    uint8_t _echoPin;
    unsigned long _period;
    unsigned long _lastTrigTime;

    static const uint8_t BUF_SIZE = 5;
    float _history[BUF_SIZE];
    uint8_t _bufCount;
    uint8_t _bufHead;
    unsigned long _lastValidTime;

public:
    float distance;          // 最新一次原始量測（公分，-1.0代表逾時）
    float filteredDistance;  // 中位數濾波後平滑距離（公分，-1.0代表持續無效）

    // 建構子，預設量測週期為 80 毫秒 (兼顧快速響應與感測穩定)
    Ultrasonic(uint8_t trigPin, uint8_t echoPin, unsigned long period = 80)
        : _trigPin(trigPin), _echoPin(echoPin), _period(period), _lastTrigTime(0),
          _bufCount(0), _bufHead(0), _lastValidTime(0),
          distance(-1.0f), filteredDistance(-1.0f) {}

    void begin() {
        pinMode(_trigPin, OUTPUT);
        pinMode(_echoPin, INPUT);
        digitalWrite(_trigPin, LOW); // 預設讓 Trig 保持低電位
        _bufCount = 0;
        _bufHead = 0;
        _lastValidTime = millis();
        distance = -1.0f;
        filteredDistance = -1.0f;
    }

    float getDistance() {
        digitalWrite(_trigPin, LOW);
        delayMicroseconds(2);

        digitalWrite(_trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(_trigPin, LOW);

        // pulseIn timeout 30000 微秒 (約 500 公分)，避免長時間阻塞 loop
        long duration = pulseIn(_echoPin, HIGH, 30000); 

        if (duration == 0) return -1.0f;

        return float(duration * 0.034f) / 2.0f;
    }

    void service() {
        unsigned long currentTime = millis();
        // 檢查是否到達量測週期
        if (currentTime - _lastTrigTime >= _period) {
            _lastTrigTime = currentTime;
            distance = getDistance();

            if (distance > 1.0f && distance < 400.0f) {
                // 有效物理測距，推入環形緩衝區
                _history[_bufHead] = distance;
                _bufHead = (_bufHead + 1) % BUF_SIZE;
                if (_bufCount < BUF_SIZE) {
                    _bufCount++;
                }
                _lastValidTime = currentTime;

                // 取中位數 (複製至小陣列進行排序)
                float temp[BUF_SIZE];
                for (uint8_t i = 0; i < _bufCount; i++) {
                    temp[i] = _history[i];
                }
                for (uint8_t i = 0; i < _bufCount - 1; i++) {
                    for (uint8_t j = 0; j < _bufCount - 1 - i; j++) {
                        if (temp[j] > temp[j + 1]) {
                            float swapVal = temp[j];
                            temp[j] = temp[j + 1];
                            temp[j + 1] = swapVal;
                        }
                    }
                }
                filteredDistance = temp[_bufCount / 2];
            } else {
                // 讀取無效/逾時/開口穿透：
                // 若持續逾時超過 1000ms (約 12 幀以上)，確認為持續無反射(如開口/障礙過遠)，輸出 -1
                if (currentTime - _lastValidTime > 1000) {
                    filteredDistance = -1.0f;
                    _bufCount = 0;
                }
                // 若僅短暫 1~2 幀丟幀，保持前一次 filteredDistance (零階保持，防止控制劇烈跳動)
            }
        }
    }
};

#endif // ULTRASONIC_H
