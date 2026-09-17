#include "ctrl.h"
#include <Arduino.h>
#include <SCServo.h>
#include <math.h>
#include "gyro.h"

//======================================================
// 定数
//======================================================

// サーボ回転方向（Time値）※仕様3・4より
static const uint16_t TIME_STOP = 0;
static const uint16_t TIME_FORWARD = 300;
static const uint16_t TIME_BACKWARD = 1300;

// 壁判定しきい値[cm]（仕様8）
static const uint16_t WALL_THRESHOLD_CM = 15;
static const uint16_t WALL_THRESHOLD_LEFT2 = 11;  // TOFだけ短め

// 前壁停止距離[cm]（仕様13）
static const uint16_t FRONT_STOP_DISTANCE_CM = 17;

// 迷路1マスのサイズ[cm]（仕様7）
// → 前壁が無い区間でも「進みすぎ」ないようにするフォールバック用
static const uint16_t TILE_SIZE_CM = 30;

// 旋回終了の許容誤差[deg]（仕様11）
static const float YAW_TOLERANCE_DEG = 2.0f;

// 姿勢補正の許容誤差（仕様14の error≈0 の範囲）
static const int POSTURE_ALLOW_ERROR = 1;
static const uint32_t POSTURE_TIMEOUT_MS = 1000;
static uint32_t postureStartMs = 0;

// 直進中の補正ゲイン・上限（仕様12「gyro補正→壁補正→前進」を実装するために追加）
// ※符号・ゲインは実機で要調整
static const float STRAIGHT_YAW_KP = 4.0f;   // 1度あたりの補正量[Time]
static const float STRAIGHT_WALL_KP = 3.0f;  // 距離差1cmあたりの補正量[Time]
static const uint16_t STRAIGHT_CORRECTION_MAX = 150;  // 補正量の上限[Time]

// サーボ用UARTのTXピン（RXは未使用のため-1固定）
static const bool DEBUG_MOTOR_OUTPUT = true;
static const uint32_t DEBUG_MOTOR_OUTPUT_INTERVAL_MS = 100;

static const int TX_PIN = 27;

static const uint32_t LEFT_OPEN_CONFIRM_MS =
    100;  // 左壁が無いと判断するための連続確認時間[ms]（仕様8）
static uint32_t leftOpenStartMs =
    0;  // 左壁が無いと判断するための連続確認開始時刻[ms]
//======================================================
// 外部参照（main.cppでLiDAR値を格納）
//======================================================
struct LidarPacketRaw {
    uint16_t front1, front2;
    uint16_t back1, back2;
    uint16_t right1, right2;
    uint16_t left1, left2;
};
extern LidarPacketRaw rcv_packet;

// SCS0009用
SCSCL servo;

//======================================================
// 初期化（main.cppのsetup()から呼ぶ）
//======================================================
void ctrlInit() {
    Serial1.begin(1000000, SERIAL_8N1, -1, TX_PIN);
    servo.pSerial = &Serial1;
}

//======================================================
// 状態
//======================================================
RobotState state = IDLE;
static float targetYaw = 0.0f;
static float straightYaw = 0.0f;  // 直進開始時のyaw（直進中に維持する基準角）
static float turnStartYaw = 0.0f;
static uint16_t startFrontDistance = 0;

// サーボID（仕様2）
static const uint8_t LEFT_ID = 1;
static const uint8_t RIGHT_ID = 2;

//======================================================
// ユーティリティ
//======================================================
static float angleError(float target, float current) {
    float error = target - current;
    while (error > 180.0f)
        error -= 360.0f;
    while (error < -180.0f)
        error += 360.0f;
    return error;
}

static float clockwiseDelta(float start, float current) {
    float delta = start - current;
    while (delta < 0.0f)
        delta += 360.0f;
    while (delta >= 360.0f)
        delta -= 360.0f;
    return delta;
}

static uint16_t avgFront() {
    return (rcv_packet.front1 + rcv_packet.front2) / 2;
}
static uint16_t avgLeft() {
    return (rcv_packet.left1 + rcv_packet.left2) / 2;
}
static uint16_t avgRight() {
    return (rcv_packet.right1 + rcv_packet.right2) / 2;
}

static uint16_t clampTime(int32_t t) {
    if (t < 0)
        t = 0;
    if (t > 4095)
        t = 4095;  // SCS0009のTime値レンジに合わせて要調整
    return (uint16_t)t;
}

//======================================================
// 4輪制御（生のサーボ出力）
//======================================================
static void writeAll(uint16_t leftTime, uint16_t rightTime) {
    servo.WritePos(LEFT_ID, 0, leftTime, 0);
    servo.WritePos(RIGHT_ID, 0, rightTime, 0);
    // デバッグ用に出力
    Serial.print("L:");
    Serial.print(leftTime);
    Serial.print("  R:");
    Serial.println(rightTime);
}

void stop() {
    writeAll(TIME_STOP, TIME_STOP);
}

void forward() {
    writeAll(TIME_FORWARD, TIME_BACKWARD);
}

void backward() {
    writeAll(TIME_BACKWARD, TIME_FORWARD);
}

void turnLeft() {
    // 左後退・右前進（仕様4）
    writeAll(TIME_BACKWARD, TIME_BACKWARD);
}

void turnRight() {
    // 左前進・右後退（仕様4）
    writeAll(TIME_FORWARD, TIME_FORWARD);
}

//======================================================
// 直進中の補正付き前進（仕様12：gyro補正 → 左右壁補正 → 前進）
//======================================================
static void forwardWithCorrection() {
    // 1) gyro補正量：直進開始時の角度(straightYaw)からのズレを補正
    float yawErr = angleError(straightYaw, yaw);
    float correction = STRAIGHT_YAW_KP * yawErr;

    // 2) 左右壁補正量
    //    両側に壁があればセンタリング、片側だけならその壁との距離を一定に保つ
    bool hasL = avgLeft() <= WALL_THRESHOLD_CM;
    bool hasR = avgRight() <= WALL_THRESHOLD_CM;

    if (hasL && hasR) {
        int wallErr = (int)avgLeft() - (int)avgRight();
        correction += STRAIGHT_WALL_KP * wallErr;
    } else if (hasL) {
        int wallErr = (int)avgLeft() - (int)(WALL_THRESHOLD_CM / 2);
        correction += STRAIGHT_WALL_KP * wallErr;
    } else if (hasR) {
        int wallErr = (int)(WALL_THRESHOLD_CM / 2) - (int)avgRight();
        correction += STRAIGHT_WALL_KP * wallErr;
    }
    // 両側とも壁なしの場合はgyro補正のみ

    if (correction > STRAIGHT_CORRECTION_MAX)
        correction = STRAIGHT_CORRECTION_MAX;
    if (correction < -STRAIGHT_CORRECTION_MAX)
        correction = -STRAIGHT_CORRECTION_MAX;

    uint16_t leftOutputTime =
        clampTime((int32_t)TIME_FORWARD + (int32_t)correction);
    uint16_t rightOutputTime =
        clampTime((int32_t)TIME_BACKWARD - (int32_t)correction);

    if (DEBUG_MOTOR_OUTPUT) {
        static uint32_t lastDebugMs = 0;
        uint32_t nowMs = millis();

        if (nowMs - lastDebugMs >= DEBUG_MOTOR_OUTPUT_INTERVAL_MS) {
            lastDebugMs = nowMs;
            Serial.printf(
                "[MOTOR] left=%u right=%u correction=%.1f yawErr=%.1f L=%u "
                "R=%u hasL=%d hasR=%d\n",
                leftOutputTime, rightOutputTime, correction, yawErr, avgLeft(),
                avgRight(), hasL, hasR);
        }
    }

    writeAll(leftOutputTime, rightOutputTime);
}

//======================================================
// 姿勢補正（仕様14）
//======================================================
void posture() {
    // 姿勢補正の時間制限
    if (millis() - postureStartMs >= POSTURE_TIMEOUT_MS) {
        stop();
        state = IDLE;
        return;
    }

    bool hasL = avgLeft() <= WALL_THRESHOLD_CM;

    if (!hasL) {
        // 左壁が無ければ距離差での補正はできないのでIDLEへ
        stop();
        state = IDLE;
        return;
    }

    int error = (int)rcv_packet.left1 - (int)rcv_packet.left2;

    if (error > POSTURE_ALLOW_ERROR) {
        turnLeft();
    } else if (error < -POSTURE_ALLOW_ERROR) {
        turnRight();
    } else {
        stop();
        state = IDLE;
    }
}

//======================================================
// 移動コマンド（状態遷移のトリガー）
//======================================================
void turnLeft90() {
    // 現在のyawを基準に、左へ90度
    targetYaw = yaw + 85.0f;

    // 0～360度に収める
    if (targetYaw >= 360.0f) {
        targetYaw -= 360.0f;
    }

    // 左旋回状態にする
    state = TURN_LEFT;
}

void turnRight90() {
    // 現在のyawを基準に、右へ90度
    targetYaw = yaw - 85.0f;

    // 0～360度に収める
    if (targetYaw < 0.0f) {
        targetYaw += 360.0f;
    }

    // 右旋回状態にする
    state = TURN_RIGHT;
}

void turnBack180() {
    turnStartYaw = yaw;
    targetYaw = yaw + 180.0f;
    if (targetYaw >= 360.0f)
        targetYaw -= 360.0f;
    state = TURN_BACK;
}

void forwardTile() {
    startFrontDistance = avgFront();
    straightYaw = yaw;  // 直進中はこの角度を維持する
    state = FORWARD_TILE;
}

//======================================================
// メイン制御ループ
//======================================================
void ctrlLoop() {
    if (avgFront() <= FRONT_STOP_DISTANCE_CM && state != TURN_LEFT &&
        state != TURN_RIGHT && state != TURN_BACK) {
        stop();
        state = IDLE;
        return;
    }

    switch (state) {
        case IDLE:
            break;

        case TURN_LEFT:

            turnLeft();

            // 目標角度との差が2度以内なら停止
            if (fabs(angleError(targetYaw, yaw)) <= YAW_TOLERANCE_DEG) {
                stop();
                state = IDLE;
            }

            break;

        case TURN_RIGHT:

            turnRight();

            // 目標角度との差が2度以内なら停止
            if (fabs(angleError(targetYaw, yaw)) <= YAW_TOLERANCE_DEG) {
                stop();
                state = IDLE;
            }

            break;

        case TURN_BACK:
            turnRight();  // 180°回転は右回りに統一
            if (clockwiseDelta(turnStartYaw, yaw) >=
                (180.0f - YAW_TOLERANCE_DEG)) {
                stop();
                state = IDLE;
            }
            break;

        case FORWARD_TILE: {
            uint16_t nowFront = avgFront();

            // 左側の壁を常時監視
            bool hasLeftWall = (rcv_packet.left1 < WALL_THRESHOLD_CM ||
                                rcv_packet.left2 < WALL_THRESHOLD_LEFT2);

            // 左壁がなくなったことを確認
            if (!hasLeftWall) {
                if (leftOpenStartMs == 0) {
                    leftOpenStartMs = millis();
                }

                // 100ms連続して左壁がなければ左折
                if (millis() - leftOpenStartMs >= LEFT_OPEN_CONFIRM_MS) {
                    leftOpenStartMs = 0;

                    stop();
                    turnLeft90();
                    break;
                }
            } else {
                // 壁が戻ったらリセット
                leftOpenStartMs = 0;
            }

            // 前壁停止
            bool frontWallClose = (nowFront <= FRONT_STOP_DISTANCE_CM);

            // 1マス分進んだ場合
            bool movedOneTile =
                (startFrontDistance > nowFront) &&
                ((startFrontDistance - nowFront) >= TILE_SIZE_CM);

            if (frontWallClose || movedOneTile) {
                stop();
                postureStartMs = millis();
                state = POSTURE;
                break;
            }

            forwardWithCorrection();
            break;
        }

        case POSTURE:
            posture();
            break;
    }
}
