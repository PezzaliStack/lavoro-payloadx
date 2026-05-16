// packet_format.h — PayloadX downlink protocol
//
// Fonte di verita' UNICA per il layout dei tre pacchetti binari trasmessi
// dal CubeSat alla Ground Station via NRF24. Ogni TU che fa encoding o
// decoding (src/radio.cpp, GS/GS.ino, eventuali ricevitori di banco)
// DEVE includere questo file invece di ridichiarare le struct, per
// evitare deriva silenziosa del protocollo (esattamente il problema che
// aveva la doppia copia di Madgwick prima del commit 1a4219a).
//
// Endianness: ESP32 e ATmega328PB sono entrambi little-endian; con
// #pragma pack(1) non c'e' padding, quindi il blob in memoria viaggia
// trasparentemente sui due archi.
//
// Per i dettagli di alto livello (cadenze, semantica dei campi, vincoli
// legali sul beacon) vedi docs/BUILD.md sezione 4.

#pragma once

#include <stdint.h>
#include <stddef.h>

// --- Magic byte (primo byte di ogni pacchetto) ----------------------------

static const uint8_t PKT_MAGIC_TELEMETRY = 0x54;  // 'T'
static const uint8_t PKT_MAGIC_RAW_IMU   = 0x52;  // 'R'
static const uint8_t PKT_MAGIC_BEACON    = 0x42;  // 'B'

// --- Pacchetti ------------------------------------------------------------

#pragma pack(push, 1)

// Posizione + assetto. 1 Hz.
struct TelemetryPacket {
    uint8_t  magic;        // PKT_MAGIC_TELEMETRY
    uint16_t seq;
    int32_t  lat_1e7;      // gradi * 1e7
    int32_t  lng_1e7;      // gradi * 1e7
    int32_t  alt_cm;       // altitudine in cm
    int16_t  qw_i16;       // quaternione * 32767
    int16_t  qx_i16;
    int16_t  qy_i16;
    int16_t  qz_i16;
    uint8_t  sats;         // satelliti in fix
    uint8_t  flags;        // bit0 = gps_fix
    uint16_t crc;          // CRC-16 sui byte 0..size-3
};

// Ingressi grezzi del filtro Madgwick. 1 Hz, piggyback su TelemetryPacket.
struct RawImuPacket {
    uint8_t  magic;        // PKT_MAGIC_RAW_IMU
    uint16_t seq;
    int16_t  ax_mg;        // accel mg (1 LSB = 1 mg)
    int16_t  ay_mg;
    int16_t  az_mg;
    int16_t  gx_dps10;     // gyro deg/s * 10
    int16_t  gy_dps10;
    int16_t  gz_dps10;
    int16_t  mx_uT;        // mag in uT
    int16_t  my_uT;
    int16_t  mz_uT;
    uint32_t reserved;     // 0 (futuro: temp, status)
    uint16_t crc;
};

// Beacon identificativo. 0.1 Hz.
struct BeaconPacket {
    uint8_t  magic;        // PKT_MAGIC_BEACON
    uint32_t expCount;
    float    expValue;     // tensione del sensore payload (V)
    char     id[16];       // callsign / placeholder, null-padded
    uint16_t crc;
};

#pragma pack(pop)

// Tutti i pacchetti devono avere la stessa dimensione: con NRF24 in
// fixed-payload-size, TX e RX condividono il pipe e devono leggere e
// scrivere lo stesso numero di byte. La GS dispatcha sul primo byte
// (magic) dopo aver letto un buffer della size comune.
static_assert(sizeof(TelemetryPacket) <= 32,
              "PayloadX: pacchetto sopra i 32 byte del payload NRF24");
static_assert(sizeof(BeaconPacket) == sizeof(TelemetryPacket),
              "PayloadX: BeaconPacket size != TelemetryPacket size");
static_assert(sizeof(RawImuPacket) == sizeof(TelemetryPacket),
              "PayloadX: RawImuPacket size != TelemetryPacket size");

// --- CRC-16 / CCITT-FALSE (poly 0x1021, init 0xFFFF, MSB-first) -----------
//
// Definita static inline per evitare doppie definizioni quando il header
// viene incluso da piu' TU: ciascuno ottiene una copia con linkage
// interno, il compilatore puo' inlineare nei punti caldi.

static inline uint16_t pkt_crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
