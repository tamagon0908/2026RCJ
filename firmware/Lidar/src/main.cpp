#include <Arduino.h>
//ピン変更　MTOF_SDA_PIN = 25, MTOF_SCL_PIN = 26 / 

#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>

//======================================================
// YDLIDAR
//======================================================
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17

//======================================================
// I2C
//======================================================
#define I2C_SDA_PIN 12 //21にするかも   (その場合はctrl側も変更)
#define I2C_SCL_PIN 14 //22にするかも

// 外部ESP32などから読まれるI2Cスレーブアドレス
#define I2C_SLAVE_ADDR 0x08

//======================================================
// MTOF171000C0
//======================================================
#define MTOF_ADDRESS 0x52

// MTOF制御用ピン
#define CONTROL_PIN 18

//MTOF_SDA SCLピン
#define MTOF_SDA_PIN 25
#define MTOF_SCL_PIN 26

//======================================================
// Serial
//======================================================
HardwareSerial LidarSerial(2);

//======================================================
// 送信データ構造体
//
// left1  = LiDAR左
// left2  = MTOF
//======================================================
struct LidarPacketRaw {
    uint16_t front1;
    uint16_t front2;

    uint16_t back1;
    uint16_t back2;

    uint16_t right1;
    uint16_t right2;

    uint16_t left1;
    uint16_t left2;  // ← MTOFの値
};

LidarPacketRaw send_packet;
volatile LidarPacketRaw i2c_regs;

//======================================================
// LiDAR用変数
//======================================================
int count_front = 0;
int count_right = 0;
int count_back = 0;
int count_left = 0;

const float ANGLE_TOLERANCE = 5.0;

//======================================================
// LiDAR解析ステート
//======================================================
enum ParseState {
    WAIT_PH1,
    WAIT_PH2,
    WAIT_CT,
    WAIT_LSN,
    WAIT_FSA1,
    WAIT_FSA2,
    WAIT_LSA1,
    WAIT_LSA2,
    WAIT_CS1,
    WAIT_CS2,
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

//======================================================
// 関数宣言
//======================================================
void startScan();
void parseLidarByte(uint8_t b);
void processPacket();
void updateI2CBuffer();

uint16_t readMTOFDistance(byte reg);

//======================================================
// I2Cスレーブ：外部から要求されたとき
//======================================================
void onRequest() {
    Wire.write((uint8_t*)&i2c_regs, sizeof(i2c_regs));
}

//======================================================
// SETUP
//======================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    //==================================================
    // 外部通信側
    //
    // ESP32自身はI2Cスレーブ
    // SDA = GPIO12
    // SCL = GPIO14
    //==================================================
    Wire.begin(I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, 100000);

    Wire.onRequest(onRequest);

    //==================================================
    // MTOF用I2C
    //
    // Wire1を使ってマスターとして動作
    // SDA = GPIO21
    // SCL = GPIO22
    //==================================================
    Wire1.begin(MTOF_SDA_PIN, MTOF_SCL_PIN, 100000);

    //タイムアウト
    Wire1.setTimeout(50);

    //==================================================
    // MTOF制御ピン
    //==================================================
    pinMode(CONTROL_PIN, OUTPUT);

    // 最初はHIGH
    digitalWrite(CONTROL_PIN, HIGH);

    //==================================================
    // LiDAR
    //==================================================
    LidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

    Serial.printf("I2C Slave : 0x%02X\n", I2C_SLAVE_ADDR);

    Serial.printf("I2C SDA   : GPIO%d\n", I2C_SDA_PIN);

    Serial.printf("I2C SCL   : GPIO%d\n", I2C_SCL_PIN);

    Serial.printf("MTOF Addr : 0x%02X\n", MTOF_ADDRESS);

    Serial.printf("MTOF CTRL : GPIO%d\n", CONTROL_PIN);

    //Serial.println("================================");

    //==================================================
    // 初期値
    //==================================================
    send_packet = {0, 0, 0, 0, 0, 0, 0, 0};

    memset((void*)&i2c_regs, 0, sizeof(i2c_regs));

    delay(1000);

    //==================================================
    // LiDAR開始
    //==================================================
    startScan();
}

//======================================================
// LOOP
//======================================================
void loop() {
    //==================================================
    // LiDAR受信
    //==================================================
    while (LidarSerial.available()) {
        parseLidarByte(LidarSerial.read());
    }

    //==================================================
    // MTOF読み取り
    //
    // 約50msごと
    //==================================================
    static unsigned long lastMTOF = 0;

    if (millis() - lastMTOF >= 50) {
        lastMTOF = millis();

        // MTOF制御
        digitalWrite(CONTROL_PIN, LOW);

        delay(5);

        // D3レジスタから距離読み取り
        uint16_t distance = readMTOFDistance(0xD3);

        // MTOFの値をmm → cm
        uint16_t distance_cm = distance / 10;

        // 左側2個目として保存
        send_packet.left2 = distance_cm;

        // 制御終了
        digitalWrite(CONTROL_PIN, HIGH);
    }
}

//======================================================
// LiDARパケット解析
//======================================================
void parseLidarByte(uint8_t b) {
    switch (state) {
        case WAIT_PH1:
            if (b == 0xAA)
                state = WAIT_PH2;
            break;

        case WAIT_PH2:
            if (b == 0x55)
                state = WAIT_CT;
            else
                state = WAIT_PH1;
            break;

        case WAIT_CT:
            ct = b;
            state = WAIT_LSN;
            break;

        case WAIT_LSN:
            lsn = b;
            state = WAIT_FSA1;
            break;

        case WAIT_FSA1:
            fsa = b;
            state = WAIT_FSA2;
            break;

        case WAIT_FSA2:
            fsa |= ((uint16_t)b << 8);
            state = WAIT_LSA1;
            break;

        case WAIT_LSA1:
            lsa = b;
            state = WAIT_LSA2;
            break;

        case WAIT_LSA2:
            lsa |= ((uint16_t)b << 8);
            state = WAIT_CS1;
            break;

        case WAIT_CS1:
            cs = b;
            state = WAIT_CS2;
            break;

        case WAIT_CS2:
            cs |= ((uint16_t)b << 8);

            data_idx = 0;

            state = READ_DATA;
            break;

        case READ_DATA:

            if (data_idx < sizeof(data_buf)) {
                data_buf[data_idx++] = b;
            }

            if (data_idx >= lsn * 3) {
                processPacket();

                state = WAIT_PH1;
            }

            break;
    }
}

//======================================================
// LiDARパケット処理
//======================================================
void processPacket() {
    if ((ct & 0x01) == 1) {
        updateI2CBuffer();
    }

    float angle_fsa = (float)(fsa >> 1) / 64.0;

    float angle_lsa = (float)(lsa >> 1) / 64.0;

    float angle_diff = angle_lsa - angle_fsa;

    if (angle_diff < 0)
        angle_diff += 360.0;

    for (int i = 0; i < lsn; i++) {
        int base = i * 3;

        uint32_t distance_mm =
            ((uint32_t)data_buf[base + 2] << 6) | (data_buf[base + 1] >> 2);

        if (distance_mm == 0)
            continue;

        uint16_t distance_cm = (uint16_t)(distance_mm / 10);

        float current_angle = angle_fsa;

        if (lsn > 1) {
            current_angle += (angle_diff / (lsn - 1)) * i;
        }

        while (current_angle >= 360.0)
            current_angle -= 360.0;

        //================================================
        // FRONT
        //================================================
        float diff_f = abs(current_angle - 0.0);

        if (diff_f > 180.0)
            diff_f = 360.0 - diff_f;

        if (diff_f <= ANGLE_TOLERANCE) {
            if (count_front == 0) {
                send_packet.front1 = distance_cm;

                count_front++;
            }

            else if (count_front == 1) {
                send_packet.front2 = distance_cm;

                count_front++;
            }
        }

        //================================================
        // RIGHT
        //================================================
        float diff_r = abs(current_angle - 90.0);

        if (diff_r > 180.0)
            diff_r = 360.0 - diff_r;

        if (diff_r <= ANGLE_TOLERANCE) {
            if (count_right == 0) {
                send_packet.right1 = distance_cm;

                count_right++;
            }

            else if (count_right == 1) {
                send_packet.right2 = distance_cm;

                count_right++;
            }
        }

        //================================================
        // BACK
        //================================================
        float diff_b = abs(current_angle - 180.0);

        if (diff_b > 180.0)
            diff_b = 360.0 - diff_b;

        if (diff_b <= ANGLE_TOLERANCE) {
            if (count_back == 0) {
                send_packet.back1 = distance_cm;

                count_back++;
            }

            else if (count_back == 1) {
                send_packet.back2 = distance_cm;

                count_back++;
            }
        }

        //================================================
        // LEFT
        //
        // LiDARでは1個だけ取得
        //================================================
        float diff_l = abs(current_angle - 270.0);

        if (diff_l > 180.0)
            diff_l = 360.0 - diff_l;

        if (diff_l <= ANGLE_TOLERANCE) {
            if (count_left == 0) {
                send_packet.left1 = distance_cm;

                count_left++;
            }
        }
    }
}

//======================================================
// I2C送信用バッファ更新
//======================================================
void updateI2CBuffer() {
    // MTOFのleft2は保持したまま
    uint16_t mtof_value = send_packet.left2;

    noInterrupts();

    memcpy((void*)&i2c_regs, &send_packet, sizeof(send_packet));

    interrupts();

    //==================================================
    // シリアル表示
    //==================================================
    Serial.printf("Front : %4d cm | %4d cm\n", i2c_regs.front1,
                  i2c_regs.front2);

    Serial.printf("Right : %4d cm | %4d cm\n", i2c_regs.right1,
                  i2c_regs.right2);

    Serial.printf("Back  : %4d cm | %4d cm\n", i2c_regs.back1, i2c_regs.back2);

    Serial.printf("Left  : %4d cm | %4d cm\n", i2c_regs.left1,
                  i2c_regs.left2);

    Serial.println("-----------------------------");

    //==================================================
    // 次のLiDARスキャン用
    //
    // left2(MTOF)は消さない
    //==================================================
    send_packet.front1 = 0;
    send_packet.front2 = 0;

    send_packet.back1 = 0;
    send_packet.back2 = 0;

    send_packet.right1 = 0;
    send_packet.right2 = 0;

    send_packet.left1 = 0;

    send_packet.left2 = mtof_value;

    count_front = 0;
    count_right = 0;
    count_back = 0;
    count_left = 0;
}

//======================================================
// MTOF171000C0 距離読み取り
//======================================================
uint16_t readMTOFDistance(byte reg) {
    uint16_t result = 0;

    // レジスタ指定
    Wire1.beginTransmission(MTOF_ADDRESS);

    Wire1.write(reg);

    // STOPを出さずRepeated START
    if (Wire1.endTransmission(false) != 0) {
        Serial.println("MTOF I2C transmission error");

        return 0;
    }

    // 2バイト読み取り
    uint8_t received = Wire1.requestFrom(MTOF_ADDRESS, (uint8_t)2);

    if (received != 2) {
        Serial.println("MTOF I2C receive error");

        return 0;
    }

    result = ((uint16_t)Wire1.read() << 8);

    result |= Wire1.read();

    return result;
}

//======================================================
// LiDARスキャン開始
//======================================================
void startScan() {
    uint8_t cmd[] = {0xA5, 0x60};

    LidarSerial.write(cmd, sizeof(cmd));
}