// payload.h – PayloadX
// Il payload (carico utile) e' lo strumento scientifico del satellite.

#pragma once
#include <Arduino.h>

struct payloadData {
    float    expValue;
    uint32_t expCount;
    bool     valid;
};

void initPayload();
void readPayload(payloadData &data);
size_t buildBeacon(char *out, size_t outSize, const payloadData &p);
const char *payloadBeaconId();
