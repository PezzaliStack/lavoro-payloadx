// main.cpp – PayloadX

#include "imu.h"
#include "gps.h"
#include "radio.h"
#include "bms.h"

gpsData gps;
sensorData imu;
bmsData bms;

void setup() {
    Serial.begin(115200);
    initIMU();
    initGPS();
    initRadio();
}

void loop() {
    readIMUData(imu);
    readGPSData(gps);
    sendTelemetry(gps, imu);

    delay(1000);  // trasmissione ogni 1 secondo
}
