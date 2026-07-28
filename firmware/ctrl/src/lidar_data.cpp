#ifndef LIDAR_DATA_H
#define LIDAR_DATA_H

#include <Arduino.h>
#include "lidar_data.h" // lidarデータ受ける
#include "ctrl.h" // SCS0009

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

extern LidarPacketRaw rcv_packet;

#endif