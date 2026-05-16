// radio.h – PayloadX

#pragma once

#include "gps.h"
#include "imu.h"

void initRadio();
void sendTelemetry(const gpsData &gps, const sensorData &imu);
