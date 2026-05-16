// radio.h – PayloadX

#pragma once

#include "gps.h"
#include "imu.h"
#include "attitude.h"
#include "payload.h"

void initRadio();
void sendTelemetry(const gpsData &gps, const sensorData &imu, const attitudeData &att);
void sendBeaconRadio(const payloadData &pl);
void sendRawImu(const sensorData &imu);
