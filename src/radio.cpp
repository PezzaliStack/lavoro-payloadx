// radio.cpp – PayloadX
// FIX del bug telemetria: il pacchetto ASCII via snprintf supera i 32 byte
// del payload NRF24, quindi la stringa veniva troncata in volo.
// Soluzione: pacchetto BINARIO compatto a dimensione fissa.
//
// Layout dei tre pacchetti (TelemetryPacket, RawImuPacket, BeaconPacket),
// magic byte e funzione CRC vivono in src/packet_format.h, condiviso con
// GS/GS.ino e con eventuali ricevitori di banco: un'unica fonte di
// verita' del protocollo (vedi docs/BUILD.md sez. 4).

#include "radio.h"
#include "packet_format.h"
#include <string.h>
#include <SPI.h>
#include <RF24.h>

#define CE_PIN  7
#define CSN_PIN 8
static RF24 radio(CE_PIN, CSN_PIN);

static uint16_t seqCounter    = 0;
static uint16_t seqCounterRaw = 0;

// Converte un componente del quaternione [-1, 1] in int16 [-32767, 32767]
// con saturazione (un Madgwick numericamente sano non dovrebbe mai
// sforare, ma rinforziamo l'invariante per evitare overflow di cast).
static int16_t quatToI16(float q) {
    float v = q * 32767.0f;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32767.0f) v = -32767.0f;
    return (int16_t)v;
}

// Saturazione generica float -> int16 (per accel/gyro/mag): un fondo scala
// troppo aggressivo o un sensore in errore potrebbe produrre valori fuori
// range, e (int16_t)cast su un float fuori range e' undefined behavior.
static int16_t toI16Sat(float v) {
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
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
    (void)imu;  // i valori IMU grezzi viaggiano nel proprio pacchetto
                // (sendRawImu / magic 'R'); qui resta solo l'attitudine.
    TelemetryPacket pkt;
    pkt.magic   = PKT_MAGIC_TELEMETRY;
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
    pkt.crc     = pkt_crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));
    bool ok = radio.write(&pkt, sizeof(pkt));
    if (!ok) {
        Serial.println(F("[RADIO] TX fallita (nessun ACK)"));
    }
}

// Pacchetto IMU grezzo via NRF24. Stessa cadenza della telemetria (1 Hz).
// Scale: accel in mg, gyro in 0.1 deg/s, mag in uT (vedi RawImuPacket).
void sendRawImu(const sensorData &imu) {
    RawImuPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic    = PKT_MAGIC_RAW_IMU;
    pkt.seq      = seqCounterRaw++;
    pkt.ax_mg    = toI16Sat(imu.ax * 1000.0f);
    pkt.ay_mg    = toI16Sat(imu.ay * 1000.0f);
    pkt.az_mg    = toI16Sat(imu.az * 1000.0f);
    pkt.gx_dps10 = toI16Sat(imu.gx * 10.0f);
    pkt.gy_dps10 = toI16Sat(imu.gy * 10.0f);
    pkt.gz_dps10 = toI16Sat(imu.gz * 10.0f);
    pkt.mx_uT    = toI16Sat(imu.mx);
    pkt.my_uT    = toI16Sat(imu.my);
    pkt.mz_uT    = toI16Sat(imu.mz);
    pkt.crc      = pkt_crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));
    bool ok = radio.write(&pkt, sizeof(pkt));
    if (!ok) {
        Serial.println(F("[RADIO] raw IMU TX fallita (nessun ACK)"));
    }
}

// Beacon binario via NRF24.
// Promemoria: questa trasmissione, su un satellite REALE in volo, richiede
// licenza radioamatore + coordinazione di frequenza IARU. Finche'
// payloadBeaconId() restituisce un placeholder ("PAYLOADX-1"), il beacon
// e' adatto solo a test in laboratorio / banco prova / payload simulato.
void sendBeaconRadio(const payloadData &pl) {
    BeaconPacket pkt;
    memset(&pkt, 0, sizeof(pkt));        // azzera anche il padding di id[]
    pkt.magic    = PKT_MAGIC_BEACON;
    pkt.expCount = pl.expCount;
    pkt.expValue = pl.expValue;
    // strncpy null-padda l'id se piu' corto del campo; rinforziamo
    // l'ultimo byte a 0 per garantire stringa C-terminata anche se
    // l'id raggiunge la dimensione massima.
    strncpy(pkt.id, payloadBeaconId(), sizeof(pkt.id));
    pkt.id[sizeof(pkt.id) - 1] = '\0';
    pkt.crc      = pkt_crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));
    bool ok = radio.write(&pkt, sizeof(pkt));
    if (!ok) {
        Serial.println(F("[RADIO] beacon TX fallita (nessun ACK)"));
    }
}
