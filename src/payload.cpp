// payload.cpp – PayloadX
// Implementazione del payload + beacon. L'esperimento e' generico:
// legge un sensore analogico su pin ADC. Sostituisci readPayload()
// con il tuo strumento reale.

#include "payload.h"

#define PAYLOAD_SENSOR_PIN 34

// Per un satellite reale BEACON_ID deve essere il tuo nominativo
// radioamatore. Senza licenza e coordinazione di frequenza IARU
// non si trasmette legalmente dallo spazio (vedi note finali).
static const char *BEACON_ID = "PAYLOADX-1";

static uint32_t measurementCount = 0;

void initPayload() {
    pinMode(PAYLOAD_SENSOR_PIN, INPUT);
    analogReadResolution(12);
    measurementCount = 0;
    Serial.println(F("[PAYLOAD] Inizializzato"));
}

void readPayload(payloadData &data) {
    int raw = analogRead(PAYLOAD_SENSOR_PIN);
    data.expValue = (raw / 4095.0f) * 3.3f;
    data.expCount = ++measurementCount;
    data.valid    = true;
}

const char *payloadBeaconId() {
    return BEACON_ID;
}

size_t buildBeacon(char *out, size_t outSize, const payloadData &p) {
    int n = snprintf(out, outSize, "%s,%lu,%.3f",
                     BEACON_ID,
                     (unsigned long)p.expCount,
                     p.expValue);
    if (n < 0) { out[0] = '\0'; return 0; }
    if ((size_t)n >= outSize) return outSize - 1;
    return (size_t)n;
}
