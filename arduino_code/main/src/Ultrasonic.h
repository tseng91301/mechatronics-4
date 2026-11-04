#ifndef ULTRASONIC_H
#define ULTRASONIC_H 1

#include <Arduino.h> // 確保能讀取到 Arduino 的標準函式（如 pinMode, digitalWrite 等）

class Ultrasonic {
private:
    uint8_t _trigPin;
    uint8_t _echoPin;
    unsigned long _period;
    unsigned long _lastTrigTime;

public:
    float distance; // 儲存最新一次量測的距離（公分）

    // 建構子，預設週期為 200 毫秒
    Ultrasonic(uint8_t trigPin, uint8_t echoPin, unsigned long period = 200)
        : _trigPin(trigPin), _echoPin(echoPin), _period(period), _lastTrigTime(0), distance(0.0) {}

    void begin() {
        pinMode(_trigPin, OUTPUT);
        pinMode(_echoPin, INPUT);
        digitalWrite(_trigPin, LOW); // 預設讓 Trig 保持低電位
    }

    float getDistance() {
        digitalWrite(_trigPin, LOW);
        delayMicroseconds(2);

        digitalWrite(_trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(_trigPin, LOW);

        // 加上第三個參數 timeout (微秒)：30000 微秒相當於約 500 公分
        // 防止前方完全沒障礙物時，pulseIn 會阻斷（卡住）主程式高達 1 秒鐘
        long duration = pulseIn(_echoPin, HIGH, 30000); 

        // 如果超時沒收到回音，duration 會回傳 0，直接回傳 -1 代表超出範圍或錯誤
        if (duration == 0) return -1.0;

        return float(duration * 0.034) / 2;
    }

    void service() {
        unsigned long currentTime = millis();
        // 檢查是否到了該量測的時間
        if (currentTime - _lastTrigTime >= _period) {
            _lastTrigTime = currentTime;
            distance = getDistance(); // 更新 distance 變數
        }
    }
};

#endif
