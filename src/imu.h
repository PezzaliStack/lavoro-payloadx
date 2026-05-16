// imu.h – PayloadX

#pragma once

struct sensorData {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
};

void initIMU();
void readIMUData(sensorData &data);
