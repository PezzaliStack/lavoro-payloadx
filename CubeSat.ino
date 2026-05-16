// CubeSat.ino – PayloadX entry point (versione integrata)
//
// Novita' rispetto alla versione precedente:
//  - Aggiunto il modulo payload + beacon
//  - Scheduler NON bloccante con millis() al posto di delay():
//    un satellite non puo' "addormentarsi" 1 secondo, deve restare
//    reattivo (watchdog, comandi, lettura sensori continua).

#include "src/imu.h"
#include "src/gps.h"
#include "src/radio.h"
#include "src/bms.h"
#include "src/payload.h"

gpsData     gps;
sensorData  imu;
bmsData     bms;
payloadData pl;

// Periodi dei task (millisecondi)
static const uint32_t T_SENSORS   = 200;    // 5 Hz: lettura IMU/GPS
static const uint32_t T_TELEMETRY = 1000;   // 1 Hz: invio telemetria
static const uint32_t T_BEACON    = 10000;  // ogni 10 s: beacon identificativo

static uint32_t lastSensors = 0, lastTelem = 0, lastBeacon = 0;

void setup() {
    Serial.begin(115200);
    initIMU();
    initGPS();
    initRadio();
    initPayload();
    Serial.println(F("[SYS] PayloadX avviato"));
}

void loop() {
    uint32_t now = millis();

    if (now - lastSensors >= T_SENSORS) {
        lastSensors = now;
        readIMUData(imu);
        readGPSData(gps);
        readPayload(pl);
    }

    if (now - lastTelem >= T_TELEMETRY) {
        lastTelem = now;
        sendTelemetry(gps, imu);
    }

    if (now - lastBeacon >= T_BEACON) {
        lastBeacon = now;
        char beacon[32];
        size_t len = buildBeacon(beacon, sizeof(beacon), pl);
        Serial.print(F("[BEACON] "));
        Serial.println(beacon);
        (void)len;
    }
}
