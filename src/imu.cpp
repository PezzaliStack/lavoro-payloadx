// imu.cpp – PayloadX
// Driver reale per MPU-9250 basato sulla libreria
// "SparkFun MPU-9250 Digital Motion Sensor" (gia' in platformio.ini).

#include "imu.h"
#include <SparkFunMPU9250-DMP.h>

static MPU9250_DMP imu;
static bool imuOk = false;

void initIMU() {
    if (imu.begin() != INV_SUCCESS) {
        imuOk = false;
        Serial.println(F("[IMU] MPU-9250 non rilevato"));
        return;
    }
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    imu.setAccelFSR(2);
    imu.setGyroFSR(2000);
    imu.setSampleRate(100);
    imu.setCompassSampleRate(100);
    imuOk = true;
    Serial.println(F("[IMU] MPU-9250 inizializzato"));
}

void readIMUData(sensorData &data) {
    if (!imuOk) {
        data.ax = data.ay = data.az = 0.0f;
        data.gx = data.gy = data.gz = 0.0f;
        data.mx = data.my = data.mz = 0.0f;
        return;
    }
    if (imu.dataReady()) {
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
        data.ax = imu.calcAccel(imu.ax);
        data.ay = imu.calcAccel(imu.ay);
        data.az = imu.calcAccel(imu.az);
        data.gx = imu.calcGyro(imu.gx);
        data.gy = imu.calcGyro(imu.gy);
        data.gz = imu.calcGyro(imu.gz);
        data.mx = imu.calcMag(imu.mx);
        data.my = imu.calcMag(imu.my);
        data.mz = imu.calcMag(imu.mz);
    }
}
