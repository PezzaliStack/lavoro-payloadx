// gps.h – PayloadX

#pragma once

struct gpsData {
    double lat;
    double lng;
    float alt;
    float speed;
    float heading;
    int sats;
};

void initGPS();
void readGPSData(gpsData &data);
