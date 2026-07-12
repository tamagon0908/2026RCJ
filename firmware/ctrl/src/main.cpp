#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include "ctrl.h"  // サーボ制御用ヘッダファイル(ctrl.h)の読み込み

// ====================================================================
// 1. ピン・通信の設定
// ====================================================================

// 【系統2】MPU6050姿勢計測（I2Cマスタ） -> Wire1 を使用
#define MPU_SDA_PIN 26
#define MPU_SCL_PIN 25
#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6b

// MPU接続フラグ
bool mpuConnected = false;

// 【系統3】STS3032 サーボ制御（UART1）
#define STS_RX_PIN 4
#define STS_TX_PIN 2

// 【系統4】UnitV2通信用（UART2）
#define RXD2 16
#define TXD2 17

// ====================================================================
// 2. データ構造体・グローバル変数の定義
// ====================================================================
// ctrl.h側でextern宣言した実体
SCSCL sts3032;

// 受信用の生のデータ構造体（16バイト）
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

// ロボット走行用（cm単位）のデータ構造体
struct RobotLidarData {
    float front1;
    float front2;  // 前方 (355° / 5°)
    float back1;
    float back2;  // 後方 (175° / 185°)
    float right1;
    float right2;  // 右側 (85° / 95°)
    float left1;
    float left2;  // 左側 (265° / 275°)
};

// 各種センサー用変数
volatile LidarPacketRaw volatile_raw_packet;
volatile bool new_data_flag = false;
RobotLidarData lidar;

double offsetX = 0, offsetY = 0, offsetZ = 0;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

// UnitV2受信用の非同期バッファ
String v2_buffer = "";

// --- 【追加】UnitV2 連続受信判定用の変数 ---
char last_received_cmd = '\0';   // 最後に確定した文字を記憶
int cmd_repeat_count = 0;        // 連続で受信した回数のカウンタ
bool should_stop_by_v2 = false;  // UnitV2の条件達成による停止フラグ

// ====================================================================
// 3. MPU6050低レイヤ関数群
// ====================================================================
void writeMPU6050(byte reg, byte data) {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(reg);
    Wire1.write(data);
    Wire1.endTransmission();
}

byte readMPU6050(byte reg) {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission(true);
    Wire1.requestFrom((uint8_t)MPU6050_ADDR, (size_t)1, true);
    return Wire1.read();
}

// 加速度、ジャイロから角度を計算する関数
void calcRotation() {

    if(!mpuConnected){
        return;
    }
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y,
        raw_gyro_z;

    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(0x3B);
    Wire1.endTransmission(false);

    Wire1.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);

    raw_acc_x = Wire1.read() << 8 | Wire1.read();
    raw_acc_y = Wire1.read() << 8 | Wire1.read();
    raw_acc_z = Wire1.read() << 8 | Wire1.read();
    raw_t = Wire1.read() << 8 | Wire1.read();
    raw_gyro_x = Wire1.read() << 8 | Wire1.read();
    raw_gyro_y = Wire1.read() << 8 | Wire1.read();
    raw_gyro_z = Wire1.read() << 8 | Wire1.read();

    acc_x = ((float)raw_acc_x) / 16384.0;
    acc_y = ((float)raw_acc_y) / 16384.0;
    acc_z = ((float)raw_acc_z) / 16384.0;

    acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
    acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

    dpsX = ((float)raw_gyro_x) / 65.5;
    dpsY = ((float)raw_gyro_y) / 65.5;
    dpsZ = ((float)raw_gyro_z) / 65.5;

    interval = millis() - preInterval;
    preInterval = millis();

    gyro_angle_x += (dpsX - offsetX) * (interval * 0.001);
    gyro_angle_y += (dpsY - offsetY) * (interval * 0.001);
    gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.001);

    angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
    angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
    angleZ = gyro_angle_z;

    gyro_angle_x = angleX;
    gyro_angle_y = angleY;
    gyro_angle_z = angleZ;
}

// ====================================================================
// 3. LiDARデータ受信コールバック関数（I2Cスレーブ）
// ====================================================================
void OnDataRecv(const uint8_t *mac,
                const uint8_t *incomingData,
                int len)
{
    if (len != sizeof(LidarPacketRaw))
        return;

    memcpy((void *)&volatile_raw_packet,
           incomingData,
           sizeof(LidarPacketRaw));

    new_data_flag = true;
}

// ====================================================================
// 4 UnitV2 シリアル受信関数 (ノンブロッキング)
// ====================================================================
void checkUnitV2() {
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        if (c == '\n') {
            v2_buffer.trim();
            if (v2_buffer.length() > 0) {
                Serial.print("[UnitV2] ");
                Serial.println(v2_buffer);

                // 文字列の先頭の1文字を取り出す（例: "F" や "S\r" などに対応）
                char cmd = v2_buffer.charAt(0);

                // 対象 of 文字（F, S, O）のいずれかであるかチェック
                if (cmd == 'F' || cmd == 'S' || cmd == 'O') {
                    if (cmd == last_received_cmd) {
                        // 前回と同じ文字ならカウントアップ
                        cmd_repeat_count++;
                    } else {
                        // 新しい文字が来たらカウンタを1にリセットして文字を更新
                        last_received_cmd = cmd;
                        cmd_repeat_count = 1;
                    }

                    // 連続5回一致したら停止フラグを真にする
                    if (cmd_repeat_count >= 5) {
                        should_stop_by_v2 = true;
                        Serial.print(
                            ">> UnitV2 Stop triggered by 5 consecutive '");
                        Serial.print(cmd);
                        Serial.println("'s <<");
                    }
                } else {
                    // F, S, O
                    // 以外のデータが挟まった場合は連続が途切れたとみなしてリセット
                    last_received_cmd = '\0';
                    cmd_repeat_count = 0;
                }
            }
            v2_buffer = "";
        } else {
            v2_buffer += c;
        }
    }
}

// ====================================================================
// 5. ロボット走行制御メインロジック（連続時間・スムーズ走行版）
// ====================================================================
void controlRobot() {
    // MPU6050が接続されている時だけ転倒判定
    if (mpuConnected) {
        if (abs(angleX) > 25.0 || abs(angleY) > 25.0) {
            driveRobot(0, 0);
            return;
        }
    }

    // UnitV2による停止
    if (should_stop_by_v2) {
        driveRobot(0, 0);
        return;
    }

    // 前方距離による減速
    int base_speed = 600;
    float min_front_dist = min(lidar.front1, lidar.front2);

    if (min_front_dist < 40.0) {
        float speed_rate = (min_front_dist - 15.0) / (40.0 - 15.0);
        if (speed_rate < 0.0)
            speed_rate = 0.0;
        base_speed = (int)(base_speed * speed_rate);
    }

    if (min_front_dist <= 15.0) {
        driveRobot(0, 0);
        return;
    }

    // 壁との平行を保つ
    float right_error = lidar.right1 - lidar.right2;
    float Kp = 40.0;
    int steering_output = (int)(right_error * Kp);

    int left_motor_speed = base_speed + steering_output;
    int right_motor_speed = base_speed - steering_output;

    driveRobot(left_motor_speed, right_motor_speed);
}

// ====================================================================
// 6. セットアップ
// ====================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Serial2 (UnitV2) initialized.");

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        while (1)
            ;
    }

    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("ESP-NOW Ready");

    Wire1.begin(MPU_SDA_PIN, MPU_SCL_PIN);

    Wire1.beginTransmission(MPU6050_ADDR);

    if (Wire1.endTransmission() == 0 && readMPU6050(MPU6050_WHO_AM_I) == 0x68) {
        mpuConnected = true;
        Serial.println("MPU6050 Found");

        writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
        writeMPU6050(MPU6050_CONFIG, 0x00);
        writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
        writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
        writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
    } else {
        Serial.println("MPU6050 Not Found");
    }

    initStsServos(&Serial1, 1000000, STS_RX_PIN, STS_TX_PIN);

    delay(100);

    if (mpuConnected) {
        Serial.print("Calculate Calibration");
        for (int i = 0; i < 3000; i++) {
            int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x,
                raw_gyro_y, raw_gyro_z;

            Wire1.beginTransmission(MPU6050_ADDR);
            Wire1.write(0x3B);
            Wire1.endTransmission(false);
            Wire1.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);

            raw_acc_x = Wire1.read() << 8 | Wire1.read();
            raw_acc_y = Wire1.read() << 8 | Wire1.read();
            raw_acc_z = Wire1.read() << 8 | Wire1.read();
            raw_t = Wire1.read() << 8 | Wire1.read();
            raw_gyro_x = Wire1.read() << 8 | Wire1.read();
            raw_gyro_y = Wire1.read() << 8 | Wire1.read();
            raw_gyro_z = Wire1.read() << 8 | Wire1.read();

            offsetX += ((float)raw_gyro_x) / 65.5;
            offsetY += ((float)raw_gyro_y) / 65.5;
            offsetZ += ((float)raw_gyro_z) / 65.5;
            if (i % 1000 == 0)
                Serial.print(".");
        }
        Serial.println();

        offsetX /= 3000;
        offsetY /= 3000;
        offsetZ /= 3000;
    }

    lidar = {400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0};
    preInterval = millis();

    Serial.println(
        "システム起動成功 (LiDAR I2C + MPU6050 I2C + STS3032 車輪制御 + UnitV2 "
        "UART)");
}

// ====================================================================
// 7. メインループ
// ====================================================================
void loop() {

    if (mpuConnected) {
        calcRotation();
    }

    if (new_data_flag) {
        LidarPacketRaw raw;

        noInterrupts();
        memcpy(&raw, (const void*)&volatile_raw_packet, sizeof(LidarPacketRaw));
        new_data_flag = false;
        interrupts();

        lidar.front1 = (raw.front1 <= 10) ? 400.0 : (float)raw.front1 / 10.0;
        lidar.front2 = (raw.front2 <= 10) ? 400.0 : (float)raw.front2 / 10.0;
        lidar.back1  = (raw.back1  <= 10) ? 400.0 : (float)raw.back1  / 10.0;
        lidar.back2  = (raw.back2  <= 10) ? 400.0 : (float)raw.back2  / 10.0;
        lidar.right1 = (raw.right1 <= 10) ? 400.0 : (float)raw.right1 / 10.0;
        lidar.right2 = (raw.right2 <= 10) ? 400.0 : (float)raw.right2 / 10.0;
        lidar.left1  = (raw.left1  <= 10) ? 400.0 : (float)raw.left1  / 10.0;
        lidar.left2  = (raw.left2  <= 10) ? 400.0 : (float)raw.left2  / 10.0;
    }

    checkUnitV2();
    controlRobot();

    delay(10);
}