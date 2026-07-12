// #include <Arduino.h>
// #include <HardwareSerial.h>
// #include <math.h>
// #include <Wire.h>

// // ==========================================
// // ピン配置の設定（ESP32環境用に最適化）
// // ==========================================
// #define LIDAR_RX_PIN 16   // ESP32のRX2 ➔ LiDARのTXへ配線
// #define LIDAR_TX_PIN 17   // ESP32のTX2 ➔ LiDARのRXへ配線
// #define SDA_PIN 21        // I2C SDA
// #define SCL_PIN 22        // I2C SCL

// // I2C通信設定
// #define I2C_SLAVE_ADDR 0x24    // 受信側ESP32のI2Cアドレス
// #define I2C_CLOCK_SPEED 400000 // 400kHz (Fast Mode)

// // 競合のないSerial2を使用してLidarSerialオブジェクトを作成
// HardwareSerial LidarSerial(2);

// // 送信用データ構造体（2バイト整数 × 8点 = 16バイト）
// struct LidarPacketRaw {
//   uint16_t front1; // 355°
//   uint16_t front2; // 5°
//   uint16_t back1;  // 175°
//   uint16_t back2;  // 185°
//   uint16_t right1; // 85°
//   uint16_t right2; // 95°
//   uint16_t left1;  // 265°
//   uint16_t left2;  // 275°
// };

// LidarPacketRaw send_packet;

// // 解析用ステート定義
// enum ParseState {
//   WAIT_PH1, WAIT_PH2, WAIT_CT, WAIT_LSN, WAIT_FSA1, WAIT_FSA2, WAIT_LSA1, WAIT_LSA2, WAIT_CS1, WAIT_CS2, READ_DATA
// };

// // ==========================================
// // プロトタイプ宣言（VS Code/PlatformIOで必須）
// // ==========================================
// void startScan();
// void stopScan();
// void parseLidarByte(uint8_t b);
// void processPacket();
// void displayTwoPoints();
// void printDistance(const char* label, int angle);
// void sendDataToI2C();

// // グローバル変数
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

// // 画面表示・I2C送信の間隔調整用（ミリ秒単位）
// unsigned long last_display_time = 0;
// const unsigned long DISPLAY_INTERVAL = 500; // 500ms(0.5秒)ごとに更新

// void setup() {
//   Serial.begin(115200); // PCへのデバッグ用(USBシリアル)
//   delay(1000);
  
//   // I2Cマスタとして初期化
//   Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_SPEED);
  
//   // T-mini Plusの通信速度は 230400 bps 固定
//   // ESP32のSerial2を、指定したRX/TXピンで確実に開始
//   LidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
//   Serial.println("YDLIDAR T-mini Plus 起動準備完了 (ESP32 - VS Code環境)");
  
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
//       if (data_idx < sizeof(data_buf)) {
//         data_buf[data_idx++] = b;
//       }
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
//     int sample_base = i * 3;
//     uint8_t si2 = data_buf[sample_base + 1]; 
//     uint8_t si3 = data_buf[sample_base + 2]; 

//     // YDLIDAR T-miniのデータ構造（2バイトを結合して4で割ることでmm単位に変換）
//     uint16_t dist_raw = si2 | ((uint16_t)si3 << 8);
//     float distance = (float)dist_raw / 4.0; 

//     float angle = angle_fsa;
//     if (lsn > 1) {
//       angle = angle_fsa + (angle_diff / (lsn - 1)) * i;
//     }
//     while (angle >= 360.0) angle -= 360.0;

//     int angle_idx = (int)round(angle) % 360;
//     scan_distances[angle_idx] = distance;
//   }

//   // 1回転の終わりかつ、指定時間が経過していたら表示とI2C送信を実行
//   if ((ct & 0x01) && (millis() - last_display_time >= DISPLAY_INTERVAL)) { 
//     last_display_time = millis();
    
//     // シリアルモニターへ表示
//     displayTwoPoints();

//     // 生のミリメートル値（整数）を構造体にセットしてI2C送信
//     send_packet.front1 = (uint16_t)scan_distances[355];
//     send_packet.front2 = (uint16_t)scan_distances[5];
//     send_packet.back1  = (uint16_t)scan_distances[175];
//     send_packet.back2  = (uint16_t)scan_distances[185];
//     send_packet.right1 = (uint16_t)scan_distances[85];
//     send_packet.right2 = (uint16_t)scan_distances[95];
//     send_packet.left1  = (uint16_t)scan_distances[265];
//     send_packet.left2  = (uint16_t)scan_distances[275];

//     sendDataToI2C();
//   }
// }

// // 指定した角度の距離（cm）を取得するヘルパー関数
// void printDistance(const char* label, int angle) {
//   float d = scan_distances[angle];
//   char buf[64];
//   if (d <= 10.0) {
//     sprintf(buf, "%s(%d°): ---- cm", label, angle);
//   } else {
//     sprintf(buf, "%s(%d°): %5.1f cm", label, angle, d / 10.0);
//   }
//   Serial.print(buf);
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

// // I2Cデータ送信関数
// void sendDataToI2C() {
//   Wire.beginTransmission(I2C_SLAVE_ADDR);
//   Wire.write((uint8_t*)&send_packet, sizeof(send_packet));
//   uint8_t error = Wire.endTransmission();
  
//   if (error == 0) {
//     Serial.println("[I2C Master] データ送信成功");
//   } else {
//     Serial.printf("[I2C Master] 送信エラー: %d\n", error);
//   }
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
#include <WiFi.h>
#include <esp_now.h>

// ==========================================
// ピン配置
// ==========================================
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17

HardwareSerial LidarSerial(2);

//==============================
// 受信側ESP32のMACアドレス
// 書き換えて使用してください
//==============================
uint8_t receiverMac[] = {
    0x8C, 0x94, 0xDF, 0xC4, 0x0E, 0xD0
};

//==========================================
// 送信データ（変更なし）
//==========================================
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

LidarPacketRaw send_packet;

enum ParseState {
  WAIT_PH1, WAIT_PH2, WAIT_CT, WAIT_LSN,
  WAIT_FSA1, WAIT_FSA2,
  WAIT_LSA1, WAIT_LSA2,
  WAIT_CS1, WAIT_CS2,
  READ_DATA
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

unsigned long last_display_time = 0;
const unsigned long DISPLAY_INTERVAL = 500;

//==========================================

void startScan();
void stopScan();
void parseLidarByte(uint8_t b);
void processPacket();
void displayTwoPoints();
void printDistance(const char *label, int angle);
void sendDataESPNow();

//==========================================
// ESP-NOW送信結果
//==========================================
void OnDataSent(const uint8_t *mac_addr,
                esp_now_send_status_t status)
{
    Serial.print("ESP-NOW : ");

    if (status == ESP_NOW_SEND_SUCCESS)
        Serial.println("送信成功");
    else
        Serial.println("送信失敗");
}

//==========================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW初期化失敗");
        while (1);
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Peer追加失敗");
        while (1);
    }

    LidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

    Serial.println("YDLIDAR T-mini Plus");

    for(int i=0;i<360;i++)
        scan_distances[i]=0;

    delay(1000);

    startScan();
}

void loop()
{
    while(LidarSerial.available())
    {
        parseLidarByte(LidarSerial.read());
    }
}

//==========================================

void parseLidarByte(uint8_t b)
{
    switch(state)
    {
        case WAIT_PH1:
            if(b==0xAA) state=WAIT_PH2;
            break;

        case WAIT_PH2:
            if(b==0x55) state=WAIT_CT;
            else state=WAIT_PH1;
            break;

        case WAIT_CT:
            ct=b;
            state=WAIT_LSN;
            break;

        case WAIT_LSN:
            lsn=b;
            state=WAIT_FSA1;
            break;

        case WAIT_FSA1:
            fsa=b;
            state=WAIT_FSA2;
            break;

        case WAIT_FSA2:
            fsa|=((uint16_t)b<<8);
            state=WAIT_LSA1;
            break;

        case WAIT_LSA1:
            lsa=b;
            state=WAIT_LSA2;
            break;

        case WAIT_LSA2:
            lsa|=((uint16_t)b<<8);
            state=WAIT_CS1;
            break;

        case WAIT_CS1:
            cs=b;
            state=WAIT_CS2;
            break;

        case WAIT_CS2:
            cs|=((uint16_t)b<<8);
            data_idx=0;
            state=READ_DATA;
            break;

        case READ_DATA:

            if(data_idx<sizeof(data_buf))
                data_buf[data_idx++]=b;

            if(data_idx>=lsn*3)
            {
                processPacket();
                state=WAIT_PH1;
            }

            break;
    }
}

//==========================================

void processPacket()
{
    float angle_fsa=(float)(fsa>>1)/64.0;
    float angle_lsa=(float)(lsa>>1)/64.0;

    float angle_diff=angle_lsa-angle_fsa;

    if(angle_diff<0)
        angle_diff+=360.0;

    for(int i=0;i<lsn;i++)
    {
        int base=i*3;

        uint16_t dist_raw=
            data_buf[base+1]|
            ((uint16_t)data_buf[base+2]<<8);

        float distance=dist_raw/4.0;

        float angle=angle_fsa;

        if(lsn>1)
            angle+=angle_diff/(lsn-1)*i;

        while(angle>=360)
            angle-=360;

        scan_distances[(int)round(angle)%360]=distance;
    }

    if((ct&1) && millis()-last_display_time>=DISPLAY_INTERVAL)
    {
        last_display_time=millis();

        displayTwoPoints();

        send_packet.front1=(uint16_t)scan_distances[355];
        send_packet.front2=(uint16_t)scan_distances[5];
        send_packet.back1=(uint16_t)scan_distances[175];
        send_packet.back2=(uint16_t)scan_distances[185];
        send_packet.right1=(uint16_t)scan_distances[85];
        send_packet.right2=(uint16_t)scan_distances[95];
        send_packet.left1=(uint16_t)scan_distances[265];
        send_packet.left2=(uint16_t)scan_distances[275];

        sendDataESPNow();
    }
}

//==========================================

void sendDataESPNow()
{
    esp_now_send(
        receiverMac,
        (uint8_t*)&send_packet,
        sizeof(send_packet)
    );
}

//==========================================

void printDistance(const char* label,int angle)
{
    float d=scan_distances[angle];

    Serial.print(label);
    Serial.print(" ");

    if(d<=10)
        Serial.println("----");
    else
    {
        Serial.print(d/10.0);
        Serial.println(" cm");
    }
}

void displayTwoPoints()
{
    Serial.println();

    printDistance("Front355",355);
    printDistance("Front5",5);

    printDistance("Back175",175);
    printDistance("Back185",185);

    printDistance("Right85",85);
    printDistance("Right95",95);

    printDistance("Left265",265);
    printDistance("Left275",275);
}

//==========================================

void startScan()
{
    uint8_t cmd[]={0xA5,0x60};
    LidarSerial.write(cmd,sizeof(cmd));
}

void stopScan()
{
    uint8_t cmd[]={0xA5,0x65};
    LidarSerial.write(cmd,sizeof(cmd));
}