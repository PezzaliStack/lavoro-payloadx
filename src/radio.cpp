// radio.cpp – PayloadX
// FIX del bug telemetria: il pacchetto ASCII via snprintf supera i 32 byte
// del payload NRF24, quindi la stringa veniva troncata in volo.
// Soluzione: pacchetto BINARIO compatto a dimensione fissa.
//
// Obiettivo 3: aggiunto il quaternione di assetto (Madgwick) come 4 x int16
// scalati a [-1, 1] -> [-32767, 32767]. I 6 byte di accel cruda sono stati
// rimossi (l'attitudine ne e' il prodotto a valle); netto la struct cresce
// di 2 byte e resta < 32.

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
    int16_t  qw_i16;     // quaternione * 32767
    int16_t  qx_i16;
    int16_t  qy_i16;
    int16_t  qz_i16;
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

// Converte un componente del quaternione [-1, 1] in int16 [-32767, 32767]
// con saturazione (un Madgwick numericamente sano non dovrebbe mai
// sforare, ma rinforziamo l'invariante per evitare overflow di cast).
static int16_t quatToI16(float q) {
    float v = q * 32767.0f;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32767.0f) v = -32767.0f;
    return (int16_t)v;
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

void sendTelemetry(const gpsData &gps, const sensorData &imu, const attitudeData &att) {
    (void)imu;  // accel/gyro/mag cruda non e' piu' in pacchetto, ma teniamo
                // il parametro per estensioni future (es. g-load di backup).
    TelemetryPacket pkt;
    pkt.magic   = 0x54;
    pkt.seq     = seqCounter++;
    pkt.lat_1e7 = (int32_t)(gps.lat * 1e7);
    pkt.lng_1e7 = (int32_t)(gps.lng * 1e7);
    pkt.alt_cm  = (int32_t)(gps.alt * 100.0f);
    pkt.qw_i16  = quatToI16(att.qw);
    pkt.qx_i16  = quatToI16(att.qx);
    pkt.qy_i16  = quatToI16(att.qy);
    pkt.qz_i16  = quatToI16(att.qz);
    pkt.sats    = (uint8_t)gps.sats;
    pkt.flags   = (gps.sats > 0) ? 0x01 : 0x00;
    pkt.crc     = crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));
    bool ok = radio.write(&pkt, sizeof(pkt));
    if (!ok) {
        Serial.println(F("[RADIO] TX fallita (nessun ACK)"));
    }
}
