#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>

#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17

// === 使用するI2Cピンをここで自由に指定 ===
#define I2C_SDA_PIN 12   // SDAにするGPIOピン番号
#define I2C_SCL_PIN 14   // SCLにするGPIOピン番号
#define I2C_SLAVE_ADDR 0x08 

HardwareSerial LidarSerial(2);

//==========================================
// 送信データ構造体
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
volatile LidarPacketRaw i2c_regs; 

int count_front = 0, count_right = 0, count_back = 0, count_left = 0;
const float ANGLE_TOLERANCE = 5.0;

enum ParseState {
    WAIT_PH1, WAIT_PH2, WAIT_CT, WAIT_LSN,
    WAIT_FSA1, WAIT_FSA2, WAIT_LSA1, WAIT_LSA2,
    WAIT_CS1, WAIT_CS2, READ_DATA
};
ParseState state = WAIT_PH1;

uint8_t ct, lsn;
uint16_t fsa, lsa, cs;
uint8_t data_buf[120];
uint8_t data_idx = 0;

void startScan();
void parseLidarByte(uint8_t b);
void processPacket();
void updateI2CBuffer();

void onRequest() {
    Wire.write((uint8_t*)&i2c_regs, sizeof(i2c_regs));
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // 指定したピンでI2Cスレーブを初期化 (通信速度: 100kHz)
    Wire.begin(I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, 100000);
    Wire.onRequest(onRequest);

    LidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    Serial.printf("YDLIDAR I2C Slave (SDA:%d, SCL:%d) Start\n", I2C_SDA_PIN, I2C_SCL_PIN);

    send_packet = {0, 0, 0, 0, 0, 0, 0, 0};
    memset((void*)&i2c_regs, 0, sizeof(i2c_regs));

    delay(1000);
    startScan();
}

void loop() {
    while (LidarSerial.available()) {
        parseLidarByte(LidarSerial.read());
    }
}

void parseLidarByte(uint8_t b) {
    switch (state) {
        case WAIT_PH1:  if (b == 0xAA) state = WAIT_PH2; break;
        case WAIT_PH2:  if (b == 0x55) state = WAIT_CT; else state = WAIT_PH1; break;
        case WAIT_CT:   ct = b; state = WAIT_LSN; break;
        case WAIT_LSN:  lsn = b; state = WAIT_FSA1; break;
        case WAIT_FSA1: fsa = b; state = WAIT_FSA2; break;
        case WAIT_FSA2: fsa |= ((uint16_t)b << 8); state = WAIT_LSA1; break;
        case WAIT_LSA1: lsa = b; state = WAIT_LSA2; break;
        case WAIT_LSA2: lsa |= ((uint16_t)b << 8); state = WAIT_CS1; break;
        case WAIT_CS1:  cs = b; state = WAIT_CS2; break;
        case WAIT_CS2:  cs |= ((uint16_t)b << 8); data_idx = 0; state = READ_DATA; break;
        case READ_DATA:
            if (data_idx < sizeof(data_buf)) data_buf[data_idx++] = b;
            if (data_idx >= lsn * 3) {
                processPacket();
                state = WAIT_PH1;
            }
            break;
    }
}

void processPacket() {
    if ((ct & 0x01) == 1) {
        updateI2CBuffer();
    }

    float angle_fsa = (float)(fsa >> 1) / 64.0;
    float angle_lsa = (float)(lsa >> 1) / 64.0;
    float angle_diff = angle_lsa - angle_fsa;
    if (angle_diff < 0) angle_diff += 360.0;

    for (int i = 0; i < lsn; i++) {
        int base = i * 3;
        uint32_t distance_mm = ((uint32_t)data_buf[base + 2] << 6) | (data_buf[base + 1] >> 2);
        if (distance_mm == 0) continue; 
        
        uint16_t distance_cm = (uint16_t)(distance_mm / 10);

        float current_angle = angle_fsa;
        if (lsn > 1) current_angle += (angle_diff / (lsn - 1)) * i;
        while (current_angle >= 360.0) current_angle -= 360.0;

        // FRONT
        float diff_f = abs(current_angle - 0.0);
        if (diff_f > 180.0) diff_f = 360.0 - diff_f;
        if (diff_f <= ANGLE_TOLERANCE) {
            if (count_front == 0)      { send_packet.front1 = distance_cm; count_front++; }
            else if (count_front == 1) { send_packet.front2 = distance_cm; count_front++; }
        }
        // RIGHT
        float diff_r = abs(current_angle - 90.0);
        if (diff_r > 180.0) diff_r = 360.0 - diff_r;
        if (diff_r <= ANGLE_TOLERANCE) {
            if (count_right == 0)      { send_packet.right1 = distance_cm; count_right++; }
            else if (count_right == 1) { send_packet.right2 = distance_cm; count_right++; }
        }
        // BACK
        float diff_b = abs(current_angle - 180.0);
        if (diff_b <= ANGLE_TOLERANCE) {
            if (count_back == 0)      { send_packet.back1 = distance_cm; count_back++; }
            else if (count_back == 1) { send_packet.back2 = distance_cm; count_back++; }
        }
        // LEFT
        float diff_l = abs(current_angle - 270.0);
        if (diff_l > 180.0) diff_l = 360.0 - diff_l;
        if (diff_l <= ANGLE_TOLERANCE) {
            if (count_left == 0)      { send_packet.left1 = distance_cm; count_left++; }
            else if (count_left == 1) { send_packet.left2 = distance_cm; count_left++; }
        }
    }
}

void updateI2CBuffer() {
    noInterrupts();
    memcpy((void*)&i2c_regs, &send_packet, sizeof(send_packet));
    interrupts();

    Serial.printf("Buffer Updated -> F:%d,%d | R:%d,%d\n", i2c_regs.front1, i2c_regs.front2, i2c_regs.right1, i2c_regs.right2);

    count_front = count_right = count_back = count_left = 0;
    send_packet = {0, 0, 0, 0, 0, 0, 0, 0};
}

void startScan() {
    uint8_t cmd[] = {0xA5, 0x60};
    LidarSerial.write(cmd, sizeof(cmd));
}