#include <Arduino.h>
#include <Wire.h>
#include "ctrl.h"
#include "gyro.h"
#include "SCServo.h"

//======================================================
// I2Cピン設定
//======================================================
// LiDAR用（Wireバス）
#define LIDAR_SDA_PIN 12
#define LIDAR_SCL_PIN 14
#define LIDAR_I2C_ADDR 0x08

// ジャイロ(MPU6050)用（Wire1バス）
#define GYRO_SDA_PIN 21
#define GYRO_SCL_PIN 22
#define MPU_ADDR 0x68

// UnitV2 UART設定
#define UNITV2_RX_PIN 16
#define UNITV2_TX_PIN 17
HardwareSerial UnitV2(2);

//======================================================
// LiDAR受信データ構造体
//======================================================
struct LidarPacketRaw {
    uint16_t front1;
    uint16_t front2;
    uint16_t back1;
    uint16_t back2;
    uint16_t right1;
    uint16_t right2;
    uint16_t left1;
    uint16_t left2;
};

LidarPacketRaw rcv_packet;

// 壁があると判定する距離の閾値
const uint16_t WALL_THRESHOLD = 16;

//======================================================
// ジャイロ関連
//======================================================
float yaw = 0.0f;              // gyro.hでextern宣言、ctrl.cppからも参照される
static float gyroZOffset = 0.0f;
static unsigned long lastMicros = 0;

//======================================================
// センサ接続フラグ
//======================================================
bool gyroAvailable = false;
bool lidarAvailable = false;

// UnitV2による停止処理用
const int TARGET_REPEAT_COUNT = 5;  // 何回連続で同じ文字なら停止するか
String last_unitv2_msg = "";
int unitv2_repeat_count = 0;        // 同一文字の連続受信回数
unsigned long motor_stop_until = 0; // 停止が解除されるまでの時間

//======================================================
// ジャイロ(MPU6050)関連関数
//======================================================
bool gyroBegin() {
    Wire1.begin(GYRO_SDA_PIN, GYRO_SCL_PIN, 100000);
    Wire1.setTimeOut(50);  // 未接続時に長時間ブロックしないようにする

    Wire1.beginTransmission(MPU_ADDR);
    Wire1.write(0x6B);  // スリープ解除
    Wire1.write(0x00);
    bool ok = (Wire1.endTransmission(true) == 0);
    if (!ok) {
        Serial.println("【警告】21/22ピンに MPU6050 が見つかりません。");
    }
    lastMicros = micros();
    return ok;
}

void gyroCalibrate() {
    long sum = 0;
    int validSamples = 0;

    Serial.println("ジャイロ調整中...機体を動かさないでください...");

    for (int i = 0; i < 500; i++) {
        Wire1.beginTransmission(MPU_ADDR);
        Wire1.write(0x47);
        if (Wire1.endTransmission(false) == 0) {
            uint8_t bytes = Wire1.requestFrom((uint8_t)MPU_ADDR, (size_t)2, (bool)true);
            if (bytes == 2 && Wire1.available() >= 2) {
                int16_t gz = Wire1.read() << 8 | Wire1.read();
                sum += gz;
                validSamples++;
            }
        }
        delay(2);
    }

    if (validSamples > 0) {
        gyroZOffset = (float)sum / validSamples;
        Serial.printf("ジャイロ調整完了 (有効: %d/500)\n", validSamples);
    } else {
        gyroZOffset = 0.0f;
        Serial.println("【エラー】ジャイロからデータを読めませんでした。");
    }
}

void gyroUpdate() {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;

    if (dt <= 0.0f || dt > 0.5f) return;

    Wire1.beginTransmission(MPU_ADDR);
    Wire1.write(0x47);
    if (Wire1.endTransmission(false) != 0) return;

    uint8_t bytes = Wire1.requestFrom((uint8_t)MPU_ADDR, (size_t)2, (bool)true);
    if (bytes == 2 && Wire1.available() >= 2) {
        int16_t gz = Wire1.read() << 8 | Wire1.read();
        float rate = (gz - gyroZOffset) / 131.0f;

        if (fabs(rate) > 0.2f) {  // ドリフトノイズカット
            yaw += rate * dt;
        }

        if (yaw >= 360.0f) yaw -= 360.0f;
        if (yaw < 0.0f)    yaw += 360.0f;
    }
}

//======================================================
// モーター停止関数
//======================================================
void stopMotors() {
    Serial.println("モーター停止");
    stop();  // ctrl.cppのstop()を使用
}

void displaySensorData() {
    Serial.print("【SENSOR】 ");
    if (lidarAvailable) {
        Serial.printf("FRONT: %3d, %3d | ", rcv_packet.front1, rcv_packet.front2);
        Serial.printf("RIGHT: %3d, %3d | ", rcv_packet.right1, rcv_packet.right2);
        Serial.printf("BACK:  %3d, %3d | ", rcv_packet.back1, rcv_packet.back2);
        Serial.printf("LEFT:  %3d, %3d | ", rcv_packet.left1, rcv_packet.left2);
    } else {
        Serial.print("LiDAR未接続 | ");
    }

    if (gyroAvailable) {
        Serial.printf("YAW: %5.1f deg\n", yaw);
    } else {
        Serial.println("YAW: N/A (ジャイロ未接続)");
    }
}

bool isLidarPacketReady() {
    return lidarAvailable &&
           rcv_packet.front1 != 0 &&
           rcv_packet.front2 != 0 &&
           rcv_packet.back1  != 0 &&
           rcv_packet.back2  != 0 &&
           rcv_packet.right1 != 0 &&
           rcv_packet.right2 != 0 &&
           rcv_packet.left1  != 0 &&
           rcv_packet.left2  != 0;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // 【修正】ctrlLoop() ではなく ctrlInit() を呼び出して Serial1 とサーボを初期化
    ctrlInit();

    // I2C開始（LiDAR用：SDA=12, SCL=14）
    Wire.begin(LIDAR_SDA_PIN, LIDAR_SCL_PIN, 100000);
    Wire.setTimeOut(50);

    // UART2開始（UnitV2）
    UnitV2.begin(115200, SERIAL_8N1, UNITV2_RX_PIN, UNITV2_TX_PIN);

    rcv_packet = {0, 0, 0, 0, 0, 0, 0, 0};

    // ジャイロ初期化・接続確認
    gyroAvailable = gyroBegin();
    if (gyroAvailable) {
        stopMotors();   // ジャイロキャリブレーション中はモーターを停止
        Serial.println("ジャイロ検出：OK");
        Serial.println("キャリブレーション中");
        gyroCalibrate();
        Serial.println("完了");
    } else {
        Serial.println("ジャイロ未検出：ジャイロ関連の処理をスキップします");
    }
}

void loop() {
    // 1. 最優先：ジャイロが接続されていれば最新の「yaw」をキープする
    if (gyroAvailable) {
        gyroUpdate();
    }

     // LiDARデータの読み込み (I2C)
    {
        uint8_t bytesReceived =
            Wire.requestFrom(LIDAR_I2C_ADDR, sizeof(rcv_packet));
        if (bytesReceived == sizeof(rcv_packet)) {
            uint8_t* p = (uint8_t*)&rcv_packet;
            for (size_t i = 0; i < sizeof(rcv_packet); i++) {
                p[i] = Wire.read();
            }
            lidarAvailable = true;
        } else {
            // 通信エラー時
            Serial.println("I2C 通信エラー: データ未受信またはピンの接続不良");
            rcv_packet = {999, 999, 999, 999, 999, 999, 999, 999};
            lidarAvailable = false;
        }
    }

    // 2. モーターが停止中or走行中判定
    bool is_suspended = (millis() < motor_stop_until);
    bool lidarReady = isLidarPacketReady();

    if (is_suspended) {
        stopMotors();
    } else if (!lidarReady) {
        stopMotors();
    } else {
        // 3. 最優先：旋回や移動のモーター監視処理を回す
        ctrlLoop();
    }


    displaySensorData();

    // 【アルゴリズム】左手法の実装
    // 【修正】ctrl.cppのステートマシン関数（turnLeft90等）を呼ぶように変更
    if (state == IDLE && !is_suspended && lidarReady) {
        bool hasLeftWall =
            ((rcv_packet.left1 + rcv_packet.left2) / 2) < WALL_THRESHOLD;
        bool hasFrontWall =
            ((rcv_packet.front1 + rcv_packet.front2) / 2) < WALL_THRESHOLD;
        bool hasRightWall =
            ((rcv_packet.right1 + rcv_packet.right2) / 2) < WALL_THRESHOLD;

        if (!hasLeftWall) {
            Serial.println("[左手法] 左に壁がないので90度左旋回します。");
            turnLeft90();
        } else if (!hasFrontWall) {
            Serial.println("[左手法] 前が進めるので1マス前進します。");
            forwardTile();
        } else if (!hasRightWall) {
            Serial.println("[左手法] 右しか空いていないので90度右旋回します。");
            turnRight90();
        } else {
            Serial.println("[左手法] 行き止まりなので180度反転します。");
            turnBack180();
        }
    }

    // UnitV2の処理 (UART) - 非ブロッキング文字受信方式
    static String unitv2_buffer = "";

    while (UnitV2.available()) {
        char c = (char)UnitV2.read();
        if (c == '\r') continue;

        if (c == '\n') {
            unitv2_buffer.trim();
            if (unitv2_buffer.length() > 0) {
                Serial.print("UnitV2 : ");
                Serial.println(unitv2_buffer);

                // 同一メッセージの連続受信判定
                if (unitv2_buffer == last_unitv2_msg) {
                    unitv2_repeat_count++;
                } else {
                    last_unitv2_msg = unitv2_buffer;
                    unitv2_repeat_count = 1;
                }

                // 指定回数(n回)連続で一致した場合
                if (unitv2_repeat_count >= TARGET_REPEAT_COUNT) {
                    Serial.printf(
                        "★ [ALERT] UnitV2から '%s' "
                        "が%d回連続で届きました。5秒間モーターを停止します。\n",
                        unitv2_buffer.c_str(), TARGET_REPEAT_COUNT);

                    motor_stop_until = millis() + 5000;

                    unitv2_repeat_count = 0;
                    last_unitv2_msg = "";

                    stopMotors();
                }
            }
            unitv2_buffer = "";
        } else {
            unitv2_buffer += c;
            if (unitv2_buffer.length() > 128) {
                unitv2_buffer = "";
            }
        }
    }

    delay(10);
}
