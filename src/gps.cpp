// gps.cpp – PayloadX
// Driver reale per ricevitore GPS NMEA basato su Adafruit_GPS
// (gia' in platformio.ini). Sull'ESP32 usiamo la UART hardware Serial2.

#include "gps.h"
#include <Adafruit_GPS.h>

#define GPS_SERIAL Serial2
static Adafruit_GPS GPS(&GPS_SERIAL);

void initGPS() {
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    Serial.println(F("[GPS] Inizializzato (in attesa di fix...)"));
}

void readGPSData(gpsData &data) {
    while (GPS.read()) { }

    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) {
            return;
        }
    }

    if (GPS.fix) {
        data.lat     = GPS.latitudeDegrees;
        data.lng     = GPS.longitudeDegrees;
        data.alt     = GPS.altitude;
        data.speed   = GPS.speed * 0.514444f;
        data.heading = GPS.angle;
        data.sats    = GPS.satellites;
    } else {
        data.sats = 0;
    }
}
