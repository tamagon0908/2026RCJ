#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "ctrl.h" // サーボ制御用ヘッダファイル(ctrl.h)の読み込み

// ====================================================================
// 1. ピン・通信の設定
// ====================================================================
// 【系統1】LiDARデータ受信（I2Cスレーブ） -> Wire を使用
#define I2C_SLAVE_ADDR      0x24
#define LIDAR_SDA_PIN       21  
#define LIDAR_SCL_PIN       22  

// 【系統2】MPU6050姿勢計測（I2Cマスタ） -> Wire1 を使用
#define MPU_SDA_PIN         26  
#define MPU_SCL_PIN         25  
#define MPU6050_ADDR        0x68 
#define MPU6050_SMPLRT_DIV  0x19 
#define MPU6050_CONFIG      0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6b

// 【系統3】STS3032 サーボ制御（UART1）
#define STS_RX_PIN          4
#define STS_TX_PIN          2

// ====================================================================
// 2. データ構造体・グローバル変数の定義
// ====================================================================
// ctrl.h側でextern宣言した実体をここで定義
SCSCL sts3032; 

// 受信用の生のデータ構造体（16バイト）
struct LidarPacketRaw {
  uint16_t front1; uint16_t front2;
  uint16_t back1;  uint16_t back2;
  uint16_t right1; uint16_t right2;
  uint16_t left1;  uint16_t left2;
};

// ロボット走行用（cm単位）のデータ構造体
struct RobotLidarData {
  float front1; float front2;  // 前方 (355° / 5°)
  float back1;  float back2;   // 後方 (175° / 185°)
  float right1; float right2;  // 右側 (85° / 95°)
  float left1;  float left2;   // 左側 (265° / 275°)
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
void calcRotation(){
  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z;
  
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(0x3B);
  Wire1.endTransmission(false);
  
  // 型の曖昧さを完全に排除するため (uint8_t, size_t, bool) の組み合わせに固定
  Wire1.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);
  
  raw_acc_x = Wire1.read() << 8 | Wire1.read();
  raw_acc_y = Wire1.read() << 8 | Wire1.read();
  raw_acc_z = Wire1.read() << 8 | Wire1.read();
  raw_t     = Wire1.read() << 8 | Wire1.read();
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
// 4. LiDAR I2C受信割り込みハンドラ (Wireを使用)
// ====================================================================
void receiveEvent(int howMany) {
  if (howMany == sizeof(LidarPacketRaw)) {
    uint8_t* p = (uint8_t*)&volatile_raw_packet;
    for (int i = 0; i < howMany; i++) {
      p[i] = Wire.read();
    }
    new_data_flag = true; 
  } else {
    while (Wire.available()) Wire.read(); 
  }
}

// ====================================================================
// 5. ロボット走行制御メインロジック（連続時間・スムーズ走行版）
// ====================================================================
void controlRobot() {
  // ① 異常傾斜検知時の安全停止
  if (abs(angleX) > 25.0 || abs(angleY) > 25.0) {
    driveRobot(0, 0);
    return;
  }

  // ---- 基準となる前進ベース速度の設定 ----
  int base_speed = 600; // 通常時巡航速度 (範囲: -1500 ～ 1500)

  // ② 【減速アプローチ】前方の壁に近づくほど、滑らかにベース速度を下げる
  float min_front_dist = min(lidar.front1, lidar.front2);
  
  if (min_front_dist < 40.0) { // 40cm以内に入ったらクッションブレーキ開始
    // 15cmに近づくほど減速倍率が 1.0 から 0.0 へ直線的に落ちる
    float speed_rate = (min_front_dist - 15.0) / (40.0 - 15.0);
    if (speed_rate < 0.0) speed_rate = 0.0;
    
    base_speed = (int)(base_speed * speed_rate);
  }

  // 前方が完全に詰まっている（15cm以下）なら安全のため停止
  if (min_front_dist <= 15.0) {
    driveRobot(0, 0);
    return;
  }

  // ③ 【P制御】右側の壁との並行度（アライメント）調整
  // 右前と右後の距離の差（ズレ量）を算出
  float right_error = lidar.right1 - lidar.right2; 
  
  // 比例ゲイン（感度）。蛇行する場合は小さく、壁への反応を強める場合は大きく調整します
  float Kp = 40.0; 
  int steering_output = (int)(right_error * Kp);

  // 左右のモーターに差分を与えて曲がりながら進む（自動車のハンドルのような滑らかな挙動）
  int left_motor_speed  = base_speed + steering_output;
  int right_motor_speed = base_speed - steering_output;

  // 実際にctrl.hの制御関数を呼び出し、サーボを駆動
  driveRobot(left_motor_speed, right_motor_speed);
}

// ====================================================================
// 6. セットアップ
// ====================================================================
void setup() {
  Serial.begin(115200);
  
  // --- 【系統1】LiDARデータ受信ポート（スレーブ）の初期化 ---
  Wire.begin(I2C_SLAVE_ADDR, LIDAR_SDA_PIN, LIDAR_SCL_PIN, 0); 
  Wire.onReceive(receiveEvent);
  
  // --- 【系統2】MPU6050姿勢ポート（マスタ）の初期化 ---
  Wire1.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(0x6B); 
  Wire1.write(0); 
  Wire1.endTransmission(true);
  
  // --- 【系統3】STS3032用シリアルとライブラリの一括初期化（ctrl.hの関数） ---
  initStsServos(&Serial1, 1000000, STS_RX_PIN, STS_TX_PIN);
  
  delay(100);
  
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("MPU6050が見つかりません。プログラムを停止します。");
    while (true);
  }

  // MPU6050構成書き込み
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   
  writeMPU6050(MPU6050_CONFIG, 0x00);       
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); 
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   

  // MPU6050 キャリブレーション
  Serial.print("Calculate Calibration");
  for(int i = 0; i < 3000; i++){
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z;
    
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(0x3B);
    Wire1.endTransmission(false);
    
    Wire1.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);
  
    raw_acc_x = Wire1.read() << 8 | Wire1.read();
    raw_acc_y = Wire1.read() << 8 | Wire1.read();
    raw_acc_z = Wire1.read() << 8 | Wire1.read();
    raw_t     = Wire1.read() << 8 | Wire1.read();
    raw_gyro_x = Wire1.read() << 8 | Wire1.read();
    raw_gyro_y = Wire1.read() << 8 | Wire1.read();
    raw_gyro_z = Wire1.read() << 8 | Wire1.read();
    
    offsetX += ((float)raw_gyro_x) / 65.5;
    offsetY += ((float)raw_gyro_y) / 65.5;
    offsetZ += ((float)raw_gyro_z) / 65.5;
    if(i % 1000 == 0) Serial.print(".");
  }
  Serial.println();

  offsetX /= 3000;
  offsetY /= 3000;
  offsetZ /= 3000;

  // LiDAR構造体の初期設定
  lidar = {400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0};
  preInterval = millis();
  
  Serial.println("システム起動成功 (LiDAR I2C + MPU6050 I2C + STS3032 車輪制御)");
}

// ====================================================================
// 7. メインループ
// ====================================================================
void loop() {
  // 1. 姿勢角度計算を実行
  calcRotation();

  // 2. 新しいLiDARデータのチェックとcm変換
  if (new_data_flag) {
    LidarPacketRaw raw;

    noInterrupts();
    memcpy(&raw, (const void*)&volatile_raw_packet, sizeof(LidarPacketRaw));
    new_data_flag = false;
    interrupts();

    lidar.front1 = (raw.front1 <= 10) ? 400.0 : (float)raw.front1 / 10.0;
    lidar.front2 = (raw.front2 <= 10) ? 400.0 : (float)raw.front2 / 10.0;
    lidar.back1  = (raw.back1 <= 10)  ? 400.0 : (float)raw.back1 / 10.0;
    lidar.back2  = (raw.back2 <= 10)  ? 400.0 : (float)raw.back2 / 10.0;
    lidar.right1 = (raw.right1 <= 10) ? 400.0 : (float)raw.right1 / 10.0;
    lidar.right2 = (raw.right2 <= 10) ? 400.0 : (float)raw.right2 / 10.0;
    lidar.left1  = (raw.left1 <= 10)  ? 400.0 : (float)raw.left1 / 10.0;
    lidar.left2  = (raw.left2 <= 10)  ? 400.0 : (float)raw.left2 / 10.0;
  }

  // 3. ロボット走行制御判断（常に速度を計算し、シームレスに速度命令を送り続ける）
  controlRobot();

  // 制御周期を約100Hz(10ms)に維持
  delay(10); 
}