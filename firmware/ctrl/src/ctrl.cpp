#include "ctrl.h"
#include "gyro.h"
#include <SCServo.h>
#include <math.h>

//======================================================
// 定数
//======================================================

// サーボ回転方向（Time値）※仕様3・4より
static const uint16_t TIME_STOP     = 0;
static const uint16_t TIME_FORWARD  = 300;
static const uint16_t TIME_BACKWARD = 1300;

// 壁判定しきい値[cm]（仕様8）
static const uint16_t WALL_THRESHOLD_CM = 15;

// 前壁停止距離[cm]（仕様13）
static const uint16_t FRONT_STOP_DISTANCE_CM = 10;

// 迷路1マスのサイズ[cm]（仕様7）
// → 前壁が無い区間でも「進みすぎ」ないようにするフォールバック用
static const uint16_t TILE_SIZE_CM = 30;

// 旋回終了の許容誤差[deg]（仕様11）
static const float YAW_TOLERANCE_DEG = 2.0f;

// 姿勢補正の許容誤差（仕様14の error≈0 の範囲）
static const int POSTURE_ALLOW_ERROR = 5;

// 直進中の補正ゲイン・上限（仕様12「gyro補正→壁補正→前進」を実装するために追加）
// ※符号・ゲインは実機で要調整
static const float STRAIGHT_YAW_KP            = 4.0f;  // 1度あたりの補正量[Time]
static const float STRAIGHT_WALL_KP           = 3.0f;  // 距離差1cmあたりの補正量[Time]
static const uint16_t STRAIGHT_CORRECTION_MAX = 150;   // 補正量の上限[Time]

// サーボ用UARTのTXピン（RXは未使用のため-1固定）
static const int TX_PIN = 27;

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
void ctrlInit()
{
    Serial1.begin(1000000, SERIAL_8N1, -1, TX_PIN);
    servo.pSerial = &Serial1;
}

//======================================================
// 状態
//======================================================
RobotState state = IDLE;
static float targetYaw   = 0.0f;
static float straightYaw = 0.0f;   // 直進開始時のyaw（直進中に維持する基準角）
static uint16_t startFrontDistance = 0;

// サーボID（仕様2）
static const uint8_t LEFT_ID  = 1;
static const uint8_t RIGHT_ID = 2;

//======================================================
// ユーティリティ
//======================================================
static float angleError(float target, float current)
{
    float error = target - current;
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static uint16_t avgFront() { return (rcv_packet.front1 + rcv_packet.front2) / 2; }
static uint16_t avgLeft()  { return (rcv_packet.left1  + rcv_packet.left2)  / 2; }
static uint16_t avgRight() { return (rcv_packet.right1 + rcv_packet.right2) / 2; }

static uint16_t clampTime(int32_t t)
{
    if (t < 0)    t = 0;
    if (t > 4095) t = 4095;  // SCS0009のTime値レンジに合わせて要調整
    return (uint16_t)t;
}

//======================================================
// 4輪制御（生のサーボ出力）
//======================================================
static void writeAll(uint16_t leftTime, uint16_t rightTime)
{
    servo.WritePos(LEFT_ID,  0, leftTime,  0);
    servo.WritePos(RIGHT_ID, 0, rightTime, 0);
}

void stop()
{
    writeAll(TIME_STOP, TIME_STOP);
}

void forward()
{
    writeAll(TIME_FORWARD, TIME_BACKWARD);
}

void backward()
{
    writeAll(TIME_BACKWARD, TIME_FORWARD);
}

void turnLeft()
{
    // 左後退・右前進（仕様4）
    writeAll(TIME_BACKWARD, TIME_BACKWARD);
}

void turnRight()
{
    // 左前進・右後退（仕様4）
    writeAll(TIME_FORWARD, TIME_FORWARD);
}

//======================================================
// 直進中の補正付き前進（仕様12：gyro補正 → 左右壁補正 → 前進）
//======================================================
static void forwardWithCorrection()
{
    // 1) gyro補正量：直進開始時の角度(straightYaw)からのズレを補正
    float yawErr = angleError(straightYaw, yaw);
    float correction = STRAIGHT_YAW_KP * yawErr;

    // 2) 左右壁補正量
    //    両側に壁があればセンタリング、片側だけならその壁との距離を一定に保つ
    bool hasL = avgLeft()  <= WALL_THRESHOLD_CM;
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

    if (correction > STRAIGHT_CORRECTION_MAX)  correction = STRAIGHT_CORRECTION_MAX;
    if (correction < -STRAIGHT_CORRECTION_MAX) correction = -STRAIGHT_CORRECTION_MAX;

    uint16_t leftTime  = clampTime((int32_t)TIME_FORWARD + (int32_t)correction);
    uint16_t rightTime = clampTime((int32_t)TIME_FORWARD - (int32_t)correction);

    writeAll(leftTime, rightTime);
}

//======================================================
// 姿勢補正（仕様14）
//======================================================
void posture()
{
    bool hasL = avgLeft() <= WALL_THRESHOLD_CM;

    if (!hasL) {
        // 左壁が無ければ距離差での補正はできないのでそのままIDLEへ
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
void turnLeft90()
{
    // 現在のyawを基準に、左へ90度
    targetYaw = yaw + 90.0f;

    // 0～360度に収める
    if (targetYaw >= 360.0f) {
        targetYaw -= 360.0f;
    }

    // 左旋回状態にする
    state = TURN_LEFT;
}

void turnRight90()
{
    // 現在のyawを基準に、右へ90度
    targetYaw = yaw - 90.0f;

    // 0～360度に収める
    if (targetYaw < 0.0f) {
        targetYaw += 360.0f;
    }

    // 右旋回状態にする
    state = TURN_RIGHT;
}

void turnBack180()
{
    targetYaw = yaw + 180.0f;
    if (targetYaw >= 360.0f) targetYaw -= 360.0f;
    state = TURN_BACK;
}

void forwardTile()
{
    startFrontDistance = avgFront();
    straightYaw = yaw;   // 直進中はこの角度を維持する
    state = FORWARD_TILE;
}

//======================================================
// メイン制御ループ
//======================================================
void ctrlLoop()
{
        if (avgFront() <= FRONT_STOP_DISTANCE_CM &&
        state != TURN_LEFT &&
        state != TURN_RIGHT &&
        state != TURN_BACK)
    {
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
        turnRight();   // 180°回転は右回りに統一
        if (fabs(angleError(targetYaw, yaw)) < YAW_TOLERANCE_DEG) {
            stop();
            state = IDLE;
        }
        break;

    case FORWARD_TILE: {
        uint16_t nowFront = avgFront();

        // 停止条件1：前壁が約10cmまで近づいた（仕様13）
        bool frontWallClose = (nowFront <= FRONT_STOP_DISTANCE_CM);

        // 停止条件2：前壁が無い区間のフォールバック（1マス分=約30cm進んだ）
        bool movedOneTile =
            (startFrontDistance > nowFront) &&
            ((startFrontDistance - nowFront) >= TILE_SIZE_CM);

        if (frontWallClose || movedOneTile) {
            stop();
            state = POSTURE;
            break;
        }

        // gyro補正 → 壁補正 → 前進（仕様12）
        forwardWithCorrection();
        break;
    }

    case POSTURE:
        posture();
        break;
    }
}