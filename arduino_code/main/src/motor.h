#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/**
 * @brief 單馬達驅動類別 (不需編碼器)
 * 透過 2 個 PWM 腳位控制馬達的正反轉與速度。
 */
class SingleMotor {
private:
    uint8_t _pin1;
    uint8_t _pin2;
    bool _reverse;
    int _currentSpeed;

public:
    /**
     * @brief 建構子
     * @param pin1 PWM 輸出腳位 1
     * @param pin2 PWM 輸出腳位 2
     * @param reverse 是否將方向反轉 (預設 false)
     */
    SingleMotor(uint8_t pin1, uint8_t pin2, bool reverse = false)
        : _pin1(pin1), _pin2(pin2), _reverse(reverse), _currentSpeed(0) {}

    /**
     * @brief 初始化腳位
     */
    void begin() {
        pinMode(_pin1, OUTPUT);
        pinMode(_pin2, OUTPUT);
        stop();
    }

    /**
     * @brief 設定馬達速度與方向
     * @param speed 數值範圍 -255 到 +255。
     *              正值前進，負值後退，0 停止。
     */
    void setSpeed(int speed) {
        // 限制在 -255 ~ 255
        speed = constrain(speed, -255, 255);
        _currentSpeed = speed;

        if (_reverse) {
            speed = -speed;
        }

        if (speed > 0) {
            analogWrite(_pin1, speed);
            analogWrite(_pin2, 0);
        } else if (speed < 0) {
            analogWrite(_pin1, 0);
            analogWrite(_pin2, -speed);
        } else {
            analogWrite(_pin1, 0);
            analogWrite(_pin2, 0);
        }
    }

    /**
     * @brief 停止馬達 (空轉停止)
     */
    void stop() {
        setSpeed(0);
    }

    /**
     * @brief 煞車馬達 (將兩端 PWM 設為最大，視 H 橋驅動模組而定)
     */
    void brake() {
        _currentSpeed = 0;
        analogWrite(_pin1, 255);
        analogWrite(_pin2, 255);
    }

    /**
     * @brief 設定是否反轉馬達方向
     */
    void setReverse(bool reverse) {
        _reverse = reverse;
    }

    /**
     * @brief 取得當前設定的速度
     */
    int getSpeed() const {
        return _currentSpeed;
    }
};

/**
 * @brief 雙馬達差速驅動類別
 * 結合左、右兩顆單馬達，以方向 (Turn) 與速度 (Speed) 進行差速混控。
 */
class DualMotor {
private:
    SingleMotor& _leftMotor;
    SingleMotor& _rightMotor;

public:
    /**
     * @brief 建構子
     * @param leftMotor 左側馬達物件引用
     * @param rightMotor 右側馬達物件引用
     */
    DualMotor(SingleMotor& leftMotor, SingleMotor& rightMotor)
        : _leftMotor(leftMotor), _rightMotor(rightMotor) {}

    /**
     * @brief 初始化兩顆馬達
     */
    void begin() {
        _leftMotor.begin();
        _rightMotor.begin();
    }

    /**
     * @brief 控制雙馬達運動
     * @param turn 方向 (-255 ~ +255)：越正越逆時針 (左轉)，越負越順時針 (右轉)
     *             當 turn = +255 且 speed = 0 時，達到原地逆時針旋轉；
     *             當 turn = -255 且 speed = 0 時，達到原地順時針旋轉。
     * @param speed 速度 (-255 ~ +255)：正值前進，負值後退，0 原地
     */
    void drive(int turn, int speed) {
        turn = constrain(turn, -255, 255);
        speed = constrain(speed, -255, 255);

        // 混控演算法 (Differential Drive Mixing):
        // 逆時針 (turn > 0) -> 左輪減速/後退，右輪加速/前進
        // 順時針 (turn < 0) -> 左輪加速/前進，右輪減速/後退
        int leftSpeed = speed - turn;
        int rightSpeed = speed + turn;

        // 限制在 -255 ~ 255 範圍
        leftSpeed = constrain(leftSpeed, -255, 255);
        rightSpeed = constrain(rightSpeed, -255, 255);

        _leftMotor.setSpeed(leftSpeed);
        _rightMotor.setSpeed(rightSpeed);
    }

    /**
     * @brief 停止雙馬達
     */
    void stop() {
        _leftMotor.stop();
        _rightMotor.stop();
    }

    /**
     * @brief 煞車雙馬達
     */
    void brake() {
        _leftMotor.brake();
        _rightMotor.brake();
    }

    SingleMotor& getLeftMotor() { return _leftMotor; }
    SingleMotor& getRightMotor() { return _rightMotor; }
};

#endif // MOTOR_H
