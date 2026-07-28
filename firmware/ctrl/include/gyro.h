#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>

// main.cppから変数「yaw」を共有するための宣言
extern float yaw;

void gyro_begin();
void gyro_calibrate();
void gyro_update();

#endif // GYRO_H