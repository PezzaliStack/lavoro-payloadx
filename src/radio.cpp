// radio.cpp – PayloadX
// FIX del bug telemetria: il pacchetto ASCII via snprintf supera i 32 byte
// del payload NRF24, quindi la stringa veniva troncata in volo.
// Soluzione: pacchetto BINARIO compatto a dimensione fissa (28 byte).

#include "radio.h"
#include <SPI.h>
#include <RF24.h>

#define CE_PIN  7
#define CSN_PIN 8
static RF24 radio(CE_PIN, CSN_PIN);

#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  magic;
    uint16_t seq;
    int32_t  lat_1e7;
    int32_t  lng_1e7;
    int32_t  alt_cm;
    int16_t  ax_mg;
    int16_t  ay_mg;
    int16_t  az_mg;
    uint8_t  sats;
    uint8_t  flags;
    uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) <= 32,
              "Il pacchetto deve stare nei 32 byte del payload NRF24");

static uint16_t seqCounter = 0;

static uint16_t crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void initRadio() {
    if (!radio.begin()) {
        Serial.println(F("[RADIO] NRF24 non rilevato"));
        return;
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(62);
    radio.setDataRate(RF24_250KBPS);
    radio.setPayloadSize(sizeof(TelemetryPacket));
    radio.setRetries(5, 15);
    radio.openWritingPipe(0xE8E8F0F0E1LL);
    radio.stopListening();
    Serial.println(F("[RADIO] Inizializzata"));
}

void sendTelemetry(const gpsData &gps, const sensorData &imu) {
    TelemetryPacket pkt;
    pkt.magic   = 0x54;
    pkt.seq     = seqCounter++;
    pkt.lat_1e7 = (int32_t)(gps.lat * 1e7);
    pkt.lng_1e7 = (int32_t)(gps.lng * 1e7);
    pkt.alt_cm  = (int32_t)(gps.alt * 100.0f);
    pkt.ax_mg   = (int16_t)(imu.ax * 1000.0f);
    pkt.ay_mg   = (int16_t)(imu.ay * 1000.0f);
    pkt.az_mg   = (int16_t)(imu.az * 1000.0f);
    pkt.sats    = (uint8_t)gps.sats;
    pkt.flags   = (gps.sats > 0) ? 0x01 : 0x00;
    pkt.crc     = crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));
    bool ok = radio.write(&pkt, sizeof(pkt));
    if (!ok) {
        Serial.println(F("[RADIO] TX fallita (nessun ACK)"));
    }
}
