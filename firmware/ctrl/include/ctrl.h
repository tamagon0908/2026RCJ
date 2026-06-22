#ifndef CTRL_H
#define CTRL_H

#include <Arduino.h>
#include <SCServo.h>

// サーボのID定義
#define MOTOR_LEFT_FRONT    1
#define MOTOR_LEFT_BACK     2
#define MOTOR_RIGHT_FRONT   3
#define MOTOR_RIGHT_BACK    4

// サーボ制御用グローバルインスタンスの宣言
extern SCSCL sts3032;

/**
 * @brief STS3032サーボ制御用のシリアル通信を初期化します
 */
inline void initStsServos(HardwareSerial* uart_port, uint32_t baud, int rx_pin, int tx_pin) {
    uart_port->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
    sts3032.pSerial = uart_port;
    delay(10);
}

/**
 * @brief ロボットを連続回転（ホイール駆動）モードで走行させます
 * @param leftSpeed  左輪の速度 (-1500 ～ 1500)
 * @param rightSpeed 右輪の速度 (-1500 ～ 1500)
 * * ※目標位置(第2引数)を 0 にすることで、連続回転モードとして速度のみを制御します。
 * ※ライブラリの引数構成：WritePos(ID, Position, Speed, ACC)
 */
inline void driveRobot(int leftSpeed, int rightSpeed) {
    // 限界値（-1500〜1500）を超えないように安全ガード
    leftSpeed  = constrain(leftSpeed, -1500, 1500);
    rightSpeed = constrain(rightSpeed, -1500, 1500);

    // 左側のモーター（正の値で前進方向と仮定）
    sts3032.WritePos(MOTOR_LEFT_FRONT,  0, leftSpeed,  0);
    sts3032.WritePos(MOTOR_LEFT_BACK,   0, leftSpeed,  0);

    // 右側のモーター（反転マウントに対応するため符号を反転）
    sts3032.WritePos(MOTOR_RIGHT_FRONT, 0, -rightSpeed, 0);
    sts3032.WritePos(MOTOR_RIGHT_BACK,  0, -rightSpeed, 0);
}

#endif // CTRL_H