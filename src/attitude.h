// attitude.h – PayloadX
// Stima di assetto (quaternione w,x,y,z) tramite filtro Madgwick.
// Fonde gyro + accel + mag dal modulo imu.

#pragma once

#include "imu.h"

struct attitudeData {
    float qw;
    float qx;
    float qy;
    float qz;
};

void initAttitude();
void updateAttitude(const sensorData &imu, attitudeData &att);
