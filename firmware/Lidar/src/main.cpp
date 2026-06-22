// #include <Arduino.h>
// #include <HardwareSerial.h>
// #include <math.h>
// #include <Wire.h>

// #define LIDAR_RX_PIN 16  // ESP32のRX2 -> T-miniのTX
// #define LIDAR_TX_PIN 17  // ESP32のTX2 -> T-miniのRX

// HardwareSerial LidarSerial(2);

// // 解析用ステート定義
// enum ParseState {
//   WAIT_PH1, WAIT_PH2, WAIT_CT, WAIT_LSN, WAIT_FSA1, WAIT_FSA2, WAIT_LSA1, WAIT_LSA2, WAIT_CS1, WAIT_CS2, READ_DATA
// };

// ParseState state = WAIT_PH1;
// uint8_t ct;
// uint8_t lsn;
// uint16_t fsa;
// uint16_t lsa;
// uint16_t cs;
// uint8_t data_buf[120]; 
// uint8_t data_idx = 0;

// // 1回転分（360度）の距離データを保持するバッファ（mm単位）
// float scan_distances[360]; 

// // 画面表示の間隔調整用（ミリ秒単位）
// unsigned long last_display_time = 0;
// const unsigned long DISPLAY_INTERVAL = 500; // 500ms(0.5秒)ごとに表示を更新

// void setup() {
//   Serial.begin(115200);   
//   LidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
//   Serial.println("YDLIDAR T-mini Plus 起動準備完了");
  
//   for (int i = 0; i < 360; i++) scan_distances[i] = 0.0;

//   delay(1000);
//   startScan();
// }

// void loop() {
//   // Lidarのデータ受信と解析
//   while (LidarSerial.available()) {
//     uint8_t b = LidarSerial.read();
//     parseLidarByte(b);
//   }
// }

// // 点群パケットのステートマシン解析
// void parseLidarByte(uint8_t b) {
//   switch (state) {
//     case WAIT_PH1: if (b == 0xAA) state = WAIT_PH2; break;
//     case WAIT_PH2: if (b == 0x55) state = WAIT_CT; else state = WAIT_PH1; break;
//     case WAIT_CT:  ct = b; state = WAIT_LSN; break;
//     case WAIT_LSN: lsn = b; state = WAIT_FSA1; break;
//     case WAIT_FSA1: fsa = b; state = WAIT_FSA2; break;
//     case WAIT_FSA2: fsa |= ((uint16_t)b << 8); state = WAIT_LSA1; break;
//     case WAIT_LSA1: lsa = b; state = WAIT_LSA2; break;
//     case WAIT_LSA2: lsa |= ((uint16_t)b << 8); state = WAIT_CS1; break;
//     case WAIT_CS1:  cs = b; state = WAIT_CS2; break;
//     case WAIT_CS2:  cs |= ((uint16_t)b << 8); data_idx = 0; state = READ_DATA; break;
    
//     case READ_DATA:
//       data_buf[data_idx++] = b;
//       if (data_idx >= lsn * 3) {
//         processPacket();
//         state = WAIT_PH1;
//       }
//       break;
//   }
// }

// // パケットから距離と角度を計算してバッファに格納
// void processPacket() {
//   float angle_fsa = (float)(fsa >> 1) / 64.0;
//   float angle_lsa = (float)(lsa >> 1) / 64.0;
  
//   float angle_diff = angle_lsa - angle_fsa;
//   if (angle_diff < 0) angle_diff += 360.0;

//   for (int i = 0; i < lsn; i++) {
//     uint8_t si2 = data_buf[i * 3 + 1]; 
//     uint8_t si3 = data_buf[i * 3 + 2]; 

//     uint16_t dist_raw = (si2 >> 2) + (si3 << 6); 
//     float distance = (float)dist_raw; 

//     float angle = angle_fsa;
//     if (lsn > 1) {
//       angle = angle_fsa + (angle_diff / (lsn - 1)) * i;
//     }
//     while (angle >= 360.0) angle -= 360.0;

//     int angle_idx = (int)round(angle) % 360;
//     scan_distances[angle_idx] = distance;
//   }

//   // 1回転の終わり(CTの最下位ビットが1)かつ、指定時間が経過していたら2点ずつの距離を表示
//   if ((ct & 0x01) && (millis() - last_display_time >= DISPLAY_INTERVAL)) { 
//     last_display_time = millis();
//     displayTwoPoints();
//   }
// }

// // 指定した角度の距離（cm）を取得するヘルパー関数（無効値0は「--」表示用）
// void printDistance(const char* label, int angle) {
//   float d = scan_distances[angle];
//   if (d <= 10.0) {
//     Serial.printf("%s(%d°): ---- cm", label, angle);
//   } else {
//     Serial.printf("%s(%d°): %5.1f cm", label, angle, d / 10.0);
//   }
// }

// // 前後左右、それぞれ2点ずつの距離を出力する関数
// void displayTwoPoints() {
//   Serial.println("\n--- 2-Point Alignment Monitor ---");

//   // 【前 方】 355度 と 5度
//   printDistance("前方1", 355); Serial.print("  |  "); printDistance("前方2", 5);   Serial.println("");

//   // 【後 方】 175度 と 185度
//   printDistance("後方1", 175); Serial.print("  |  "); printDistance("後方2", 185);  Serial.println("");

//   // 【右 側】 85度 と 95度
//   printDistance("右側1", 85);  Serial.print("  |  "); printDistance("右側2", 95);   Serial.println("");

//   // 【左 側】 265度 と 275度
//   printDistance("左側1", 265); Serial.print("  |  "); printDistance("左側2", 275);  Serial.println("");
// }

// void startScan() {
//   uint8_t start_cmd[] = {0xA5, 0x60};
//   LidarSerial.write(start_cmd, sizeof(start_cmd));
//   Serial.println(">>> スキャン開始コマンドを送信（モーターON）");
// }

// void stopScan() {
//   uint8_t stop_cmd[] = {0xA5, 0x65};
//   LidarSerial.write(stop_cmd, sizeof(stop_cmd));
//   Serial.println(">>> スキャン停止コマンドを送信（モーターOFF）");
// }


#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <Wire.h>

// ピン配置の設定
#define LIDAR_RX_PIN 16  // ESP32のRX2 -> T-miniのTX
#define LIDAR_TX_PIN 17  // ESP32のTX2 -> T-miniのRX
#define SDA_PIN 21       // I2C SDA
#define SCL_PIN 22       // I2C SCL

// I2C通信設定
#define I2C_SLAVE_ADDR 0x24  // 受信側ESP32のI2Cアドレス
#define I2C_CLOCK_SPEED 400000 // 400kHz (Fast Mode)

// 送信用データ構造体（2バイト整数 × 8点 = 16バイト）
struct LidarPacketRaw {
  uint16_t front1; // 355°
  uint16_t front2; // 5°
  uint16_t back1;  // 175°
  uint16_t back2;  // 185°
  uint16_t right1; // 85°
  uint16_t right2; // 95°
  uint16_t left1;  // 265°
  uint16_t left2;  // 275°
};

LidarPacketRaw send_packet;
HardwareSerial LidarSerial(2);

// LiDAR解析用ステートマシン
enum ParseState {
  WAIT_PH1, WAIT_PH2, WAIT_CT, WAIT_LSN, WAIT_FSA1, WAIT_FSA2, WAIT_LSA1, WAIT_LSA2, WAIT_CS1, WAIT_CS2, READ_DATA
};

ParseState state = WAIT_PH1;
uint8_t ct;
uint8_t lsn;
uint16_t fsa;
uint16_t lsa;
uint16_t cs;
uint8_t data_buf[120]; 
uint8_t data_idx = 0;

float scan_distances[360]; 

// 送信インターバル制御用（500msごとに送信・表示）
unsigned long last_display_time = 0;
const unsigned long DISPLAY_INTERVAL = 500; 

// プロトタイプ宣言
void startScan();
void parseLidarByte(uint8_t b);
void processPacket();
void sendDataToI2C();

void setup() {
  Serial.begin(115200);   
  
  // I2Cマスタとして初期化
  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_SPEED);
  
  // LiDAR用シリアル初期化
  LidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.println("YDLIDAR T-mini Plus 起動準備完了 (Master 送信側)");
  
  for (int i = 0; i < 360; i++) scan_distances[i] = 0.0;

  delay(1000);
  startScan();
}

void loop() {
  // 常時LiDARのデータを受信・解析
  while (LidarSerial.available()) {
    uint8_t b = LidarSerial.read();
    parseLidarByte(b);
  }
}

void parseLidarByte(uint8_t b) {
  switch (state) {
    case WAIT_PH1: if (b == 0xAA) state = WAIT_PH2; break;
    case WAIT_PH2: if (b == 0x55) state = WAIT_CT; else state = WAIT_PH1; break;
    case WAIT_CT:  ct = b; state = WAIT_LSN; break;
    case WAIT_LSN: lsn = b; state = WAIT_FSA1; break;
    case WAIT_FSA1: fsa = b; state = WAIT_FSA2; break;
    case WAIT_FSA2: fsa |= ((uint16_t)b << 8); state = WAIT_LSA1; break;
    case WAIT_LSA1: lsa = b; state = WAIT_LSA2; break;
    case WAIT_LSA2: lsa |= ((uint16_t)b << 8); state = WAIT_CS1; break;
    case WAIT_CS1:  cs = b; state = WAIT_CS2; break;
    case WAIT_CS2:  cs |= ((uint16_t)b << 8); data_idx = 0; state = READ_DATA; break;
    
    case READ_DATA:
      data_buf[data_idx++] = b;
      if (data_idx >= lsn * 3) {
        processPacket();
        state = WAIT_PH1;
      }
      break;
  }
}

void processPacket() {
  float angle_fsa = (float)(fsa >> 1) / 64.0;
  float angle_lsa = (float)(lsa >> 1) / 64.0;
  
  float angle_diff = angle_lsa - angle_fsa;
  if (angle_diff < 0) angle_diff += 360.0;

  for (int i = 0; i < lsn; i++) {
    uint8_t si2 = data_buf[i * 3 + 1]; 
    uint8_t si3 = data_buf[i * 3 + 2]; 

    uint16_t dist_raw = (si2 >> 2) + (si3 << 6); 
    float distance = (float)dist_raw; 

    float angle = angle_fsa;
    if (lsn > 1) {
      angle = angle_fsa + (angle_diff / (lsn - 1)) * i;
    }
    while (angle >= 360.0) angle -= 360.0;

    int angle_idx = (int)round(angle) % 360;
    scan_distances[angle_idx] = distance;
  }

  // 1回転の終わり(CTの最下位ビットが1) かつ 指定時間が経過していたら送信
  if ((ct & 0x01) && (millis() - last_display_time >= DISPLAY_INTERVAL)) { 
    last_display_time = millis();
    
    // 生のミリメートル値（整数）を構造体にセット
    send_packet.front1 = (uint16_t)scan_distances[355];
    send_packet.front2 = (uint16_t)scan_distances[5];
    send_packet.back1  = (uint16_t)scan_distances[175];
    send_packet.back2  = (uint16_t)scan_distances[185];
    send_packet.right1 = (uint16_t)scan_distances[85];
    send_packet.right2 = (uint16_t)scan_distances[95];
    send_packet.left1  = (uint16_t)scan_distances[265];
    send_packet.left2  = (uint16_t)scan_distances[275];

    sendDataToI2C();
  }
}

void sendDataToI2C() {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((uint8_t*)&send_packet, sizeof(send_packet));
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("[I2C Master] データ送信成功");
  } else {
    Serial.printf("[I2C Master] 送信エラー: %d\n", error);
  }
}

void startScan() {
  uint8_t start_cmd[] = {0xA5, 0x60};
  LidarSerial.write(start_cmd, sizeof(start_cmd));
  Serial.println(">>> スキャン開始コマンドを送信（モーターON）");
}