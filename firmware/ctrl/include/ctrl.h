#ifndef CTRL_H
#define CTRL_H

#include <stdint.h>

//======================================================
// ロボットの状態（仕様10）
//======================================================
enum RobotState {
    IDLE,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_BACK,
    FORWARD_TILE,
    POSTURE
};

// 現在の状態（main.cppからも参照可能）
extern RobotState state;

//======================================================
// ctrl.cppで実装する関数（仕様15）
//======================================================

// 4輪の基本動作
void stop();
void forward();
void backward();
void turnLeft();
void turnRight();

// 姿勢補正
void posture();

// 旋回コマンド（呼ぶと状態遷移し、以後ctrlLoop()内で完了まで実行される）
void turnLeft90();
void turnRight90();
void turnBack180();

// 1マス前進コマンド（呼ぶと状態遷移し、以後ctrlLoop()内で完了まで実行される）
void forwardTile();

// メイン制御ループから毎回呼び出す
void ctrlLoop();
void ctrlInit();

#endif // CTRL_H