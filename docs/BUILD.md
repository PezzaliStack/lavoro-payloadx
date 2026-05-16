# PayloadX — Build, upload, test

Questo documento e' la guida operativa per chi deve compilare il firmware,
caricarlo sull'ESP32 e verificare la catena radio fino alla Ground Station.

---

## 1. Toolchain: PlatformIO vs Arduino IDE

### PlatformIO (UFFICIALE)

PlatformIO e' la toolchain raccomandata e l'unica supportata dai vincoli
del progetto (CLAUDE.md). E' quella che compila i file in `src/` —
configurazione in `platformio.ini`:

- `src_dir = src` (default)
- `env:esp32dev` con board `esp32dev`, framework `arduino`, libreria
  MPU-9250 nel fork `kwokkayan/...(For ESP32)` (l'originale SparkFun ha un
  link-error su `min` in `inv_mpu.c` sul toolchain xtensa).

Comandi tipici:

```sh
pio run                 # compila
pio run -t upload       # compila e carica sull'ESP32
pio device monitor      # apre il monitor seriale (115200 8N1)
pio run -t clean        # pulisce .pio/build
```

### Arduino IDE (NON SUPPORTATO)

Il file `CubeSat.ino` alla root e' un marker documentale: NON viene
compilato da PlatformIO (sta fuori da `src_dir`) e NON contiene logica
di entry point (solo un blocco di commenti che rimanda a `src/main.cpp`).
Vedere il commit `c66054c` per la motivazione: includere `src/main.cpp`
da `CubeSat.ino` produrrebbe simboli `setup/loop` duplicati su Arduino
IDE (che scansiona anche `src/`).

Risultato: l'entry point REALE e' `src/main.cpp`, l'unica fonte di
verita' del firmware. Usare PlatformIO.

---

## 2. Quick start

Prerequisiti: PlatformIO Core (`pip install platformio`) o l'estensione
VS Code "PlatformIO IDE".

```sh
cd PayloadX
pio run                 # prima build: scarica anche le librerie
# collegare l'ESP32 via USB
pio device list         # individua la porta (es. /dev/cu.SLAB_USBtoUART)
pio run -t upload
pio device monitor
```

Output atteso al boot sul monitor seriale:

```
[IMU] MPU-9250 ... (rilevato o non rilevato)
[GPS] Inizializzato (in attesa di fix...)
[RADIO] ... (rilevata o no)
[PAYLOAD] Inizializzato
[SYS] PayloadX avviato
```

Poi, ciclicamente:
- ogni 1 s: trasmissione `TelemetryPacket` + `RawImuPacket` via NRF24
- ogni 10 s: trasmissione `BeaconPacket` via NRF24 e stampa `[BEACON] ...`
  su seriale (testo libero, per debug).

---

## 3. Pinout / hardware assumption

I pin di default in `src/radio.cpp`:

```c
#define CE_PIN  7
#define CSN_PIN 8
```

ATTENZIONE: su un ESP32-WROOM-32 standard, GPIO 6-11 sono dedicati alla
flash SPI interna e NON sono esposti sui pin esterni. Se la build deve
girare su una dev-board WROOM, modificare `CE_PIN`/`CSN_PIN` in
`src/radio.cpp` con GPIO disponibili (es. CE=4, CSN=5) e ricompilare.
La scelta dei pin in repo riflette il PCB del progetto originale; la
documentazione PCB non e' (ancora) in questo branch.

Altri vincoli hardware:
- GPS NMEA su `Serial2` (UART2). Su ESP32 di default: RX2=GPIO16, TX2=GPIO17.
- MPU-9250 via I2C (SDA=GPIO21, SCL=GPIO22 — default Wire ESP32).
- Payload sensor (analog): GPIO34, ADC1_CH6.

---

## 4. Pacchetti binari over NRF24

Tutto il downlink usa pacchetti binari a dimensione **fissa di 27 byte**
(definita da `sizeof(TelemetryPacket)` in `src/radio.cpp`; gli altri tipi
hanno `static_assert` di parita'). La dimensione fissa e' richiesta dalla
modalita' fixed-payload del modulo NRF24L01.

Endianness: ESP32 e ATmega328PB sono entrambi little-endian e usano
`#pragma pack(1)` per evitare padding, quindi il layout in memoria e'
trasportabile bit-per-bit. La GS riceve un buffer di 27 byte, legge il
**primo byte** (magic) e fa dispatch sui tre tipi.

| Magic | Char | Tipo               | Cadenza      |
|------:|:----:|--------------------|--------------|
|  0x54 | 'T'  | TelemetryPacket    | 1 Hz         |
|  0x52 | 'R'  | RawImuPacket       | 1 Hz         |
|  0x42 | 'B'  | BeaconPacket       | 0.1 Hz       |

CRC: CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`, MSB-first),
calcolato su tutti i byte del pacchetto **eccetto** i due finali del
campo `crc`. Implementazione speculare in `src/radio.cpp` e `GS/GS.ino`
(funzione `crc16`).

### 4.1 TelemetryPacket (magic 0x54 'T') — 27 byte

| Off | Size | Tipo     | Campo      | Note                             |
|----:|-----:|----------|------------|----------------------------------|
|   0 |    1 | uint8_t  | magic      | `0x54`                           |
|   1 |    2 | uint16_t | seq        | contatore proprio                |
|   3 |    4 | int32_t  | lat_1e7    | latitudine `* 1e7`               |
|   7 |    4 | int32_t  | lng_1e7    | longitudine `* 1e7`              |
|  11 |    4 | int32_t  | alt_cm     | altitudine in cm                 |
|  15 |    2 | int16_t  | qw_i16     | quaternione w `* 32767`          |
|  17 |    2 | int16_t  | qx_i16     | quaternione x `* 32767`          |
|  19 |    2 | int16_t  | qy_i16     | quaternione y `* 32767`          |
|  21 |    2 | int16_t  | qz_i16     | quaternione z `* 32767`          |
|  23 |    1 | uint8_t  | sats       | satelliti in fix                 |
|  24 |    1 | uint8_t  | flags      | bit0 = `gps_fix`                 |
|  25 |    2 | uint16_t | crc        | CRC-16 sui byte 0..24            |

### 4.2 RawImuPacket (magic 0x52 'R') — 27 byte

Complementare a Telemetry: contiene gli ingressi a monte del filtro
Madgwick (necessari per debug del filtro, validazione, rilevazione
vibrazioni/g-load) a piena risoluzione.

| Off | Size | Tipo     | Campo      | Note                                  |
|----:|-----:|----------|------------|---------------------------------------|
|   0 |    1 | uint8_t  | magic      | `0x52`                                |
|   1 |    2 | uint16_t | seq        | contatore proprio                     |
|   3 |    2 | int16_t  | ax_mg      | accel X in mg (1 LSB = 1 mg)          |
|   5 |    2 | int16_t  | ay_mg      |                                       |
|   7 |    2 | int16_t  | az_mg      |                                       |
|   9 |    2 | int16_t  | gx_dps10   | gyro X `* 10` (1 LSB = 0.1 deg/s)     |
|  11 |    2 | int16_t  | gy_dps10   |                                       |
|  13 |    2 | int16_t  | gz_dps10   |                                       |
|  15 |    2 | int16_t  | mx_uT      | campo magnetico in uT                 |
|  17 |    2 | int16_t  | my_uT      |                                       |
|  19 |    2 | int16_t  | mz_uT      |                                       |
|  21 |    4 | uint32_t | reserved   | 0 (riservato: temp, status, ...)      |
|  25 |    2 | uint16_t | crc        | CRC-16 sui byte 0..24                 |

### 4.3 BeaconPacket (magic 0x42 'B') — 27 byte

Beacon identificativo: serve a far sapere a chi e' in ascolto "io sono
qui, sto facendo X misurazioni e l'ultima vale Y volt". Trasmesso ogni
10 s. Il campo `id` e' null-padded; vedi sezione 6 sui vincoli legali.

| Off | Size | Tipo     | Campo      | Note                                  |
|----:|-----:|----------|------------|---------------------------------------|
|   0 |    1 | uint8_t  | magic      | `0x42`                                |
|   1 |    4 | uint32_t | expCount   | numero misurazioni payload            |
|   5 |    4 | float    | expValue   | tensione del sensore payload (V)      |
|   9 |   16 | char[16] | id         | callsign / placeholder, null-padded   |
|  25 |    2 | uint16_t | crc        | CRC-16 sui byte 0..24                 |

---

## 5. Test hardware-in-the-loop con due ESP32

Setup di banco: una ESP32 nel ruolo CubeSat (TX) e una seconda ESP32 nel
ruolo Ground Station (RX). Pratico perche' evita di programmare la GS
ATmega328PB reale per testare la catena radio + parsing.

### 5.1 Hardware necessario

- 2x ESP32 dev-board (qualunque variante, anche miste).
- 2x modulo NRF24L01 (preferibilmente con cappuccio di alimentazione
  a 10-100 uF tra VCC e GND: i picchi di TX fanno svenire il regolatore
  delle dev-board USB).
- Cablaggio SPI dell'NRF24 verso ognuna delle ESP32 (CE/CSN configurabili
  via `CE_PIN`/`CSN_PIN`, vedi sezione 3).
- 2 PC, o 1 PC con due porte USB e due monitor seriali distinti.

### 5.2 Procedura

1. **CubeSat (TX)**: clonare il repo, modificare `CE_PIN`/`CSN_PIN` in
   `src/radio.cpp` se i pin di default non sono accessibili sulla
   dev-board, e flashare:
   ```sh
   pio run -t upload
   pio device monitor
   ```
   Verificare che compaiano i log `[RADIO] Inizializzata`,
   `[SYS] PayloadX avviato` e, ogni 10 s, `[BEACON] ...`.

2. **Ground Station (RX)**: la GS ufficiale gira su ATmega328PB (vedi
   `GS/GS.ino` e `GS/README.md`); per il test bench, sulla seconda
   ESP32, caricare lo sketch ricevitore minimo riportato in Appendice A.
   Questo sketch usa **gli stessi** `struct` (TelemetryPacket,
   RawImuPacket, BeaconPacket) e funzioni (`crc16`, dispatch sui magic)
   di `GS/GS.ino`. Quando viene aggiornato un layout, vanno aggiornati
   anche `src/radio.cpp`, `GS/GS.ino` e l'Appendice A. (TODO: estrarre
   in un header condiviso per evitare deriva — al momento la sicurezza
   e' un `diff` periodico.)

3. **Monitor**: aprire due monitor seriali distinti, uno per board, a
   115200 8N1.

4. **Esito atteso** sul ricevitore (RX), in CSV:
   ```
   T,12,45.4642510,9.1899580,127.50,0.9998,0.0050,-0.0123,0.0050,7,1
   R,12,3,-7,1023,0.0,-0.1,0.0,12,-5,-31
   B,PAYLOADX-1,3,1.652
   ```
   Le righe `T` arrivano a 1 Hz, `R` a 1 Hz (piggyback su `T`),
   `B` ogni 10 s.

5. **Diagnostica**: se non arriva nulla, ricontrollare nell'ordine:
   - Alimentazione NRF24 stabile (mancante ~100 uF e' la causa #1).
   - **Channel** identico (default 62 in `src/radio.cpp`).
   - **Data rate** identico (default `RF24_250KBPS` in `src/radio.cpp`).
   - **Payload size** identico (`sizeof(TelemetryPacket)` = 27).
   - **Indirizzo** del pipe identico (default `0xE8E8F0F0E1LL`).
   - `radio.begin()` deve restituire `true` su entrambi i lati.
   - Output `%%E,CRC,T|R|B` sul ricevitore: pacchetti corrotti o
     layout fuori sync — verificare i `struct` con `diff`.
   - Output `%%E,MAGIC`: il primo byte non e' nessuno dei tre attesi —
     molto probabile mismatch di indirizzo / payload size / endianness
     (su due chip little-endian non dovrebbe mai capitare).

### 5.3 Known issues / configuration drift

Differenze pre-esistenti tra `src/radio.cpp` (TX ESP32) e `GS/GS.ino`
(RX ATmega328PB ufficiale) che impediscono la catena radio di funzionare
con i default:

- **Indirizzo del pipe**: TX usa `0xE8E8F0F0E1LL`, la GS ufficiale usa
  `"EFEF0"` (5 byte ASCII). Da allineare prima di un test reale.
- **Data rate**: TX `RF24_250KBPS`, GS `RF24_1MBPS`.

Questi mismatch sono **fuori scope** dei commit di obiettivi 1-4 (che
hanno trattato solo il formato di pacchetto e il dispatch) e vanno
risolti in un commit dedicato. Lo sketch ricevitore di Appendice A
usa i valori corretti (allineati al TX), quindi il bench HIL con due
ESP32 funziona out-of-the-box.

---

## 6. Trasmissione RF reale: vincoli legali

Il modulo NRF24L01 trasmette nella banda ISM 2.4 GHz e per uso a corto
raggio in lab non richiede licenza specifica. Tuttavia, su un satellite
REALE in orbita la trasmissione RF e' soggetta a normative stringenti:

- **Licenza radioamatore** valida del personale che opera l'apparato.
- **Coordinazione di frequenza IARU** (International Amateur Radio Union)
  per assegnazione e protezione della banda dello spazio-amatoriale.
- Conformita' alle norme nazionali (in Italia: MISE/MIMIT) e ITU.

In particolare il campo `BeaconPacket.id` deve contenere un **nominativo
radioamatore valido** del responsabile della trasmissione, non un
placeholder. Allo stato attuale `src/payload.cpp` espone
`payloadBeaconId()` che restituisce la stringa **`"PAYLOADX-1"`**: e' un
placeholder esplicito, utilizzabile solo a banco / nelle condizioni ISM
brevi. **Prima di mettere in volo qualsiasi cosa che irradi, sostituire
`BEACON_ID` in `src/payload.cpp` con il proprio callsign e ottenere la
coordinazione IARU.**

Il commento sopra `sendBeaconRadio()` in `src/radio.cpp` riporta lo
stesso promemoria (vincolo 3 di CLAUDE.md).

---

## Appendice A — Sketch ricevitore ESP32 per HIL test

Sketch minimale, da caricare sulla seconda ESP32 nel test di sezione 5.
**Non duplica il layout dei pacchetti**: include `src/packet_format.h`
(fonte di verita' unica del protocollo) come fa `GS/GS.ino`. Cosi' se
il protocollo cambia, basta cambiare il file in `src/` e la GS bench
rimane allineata gratis — niente piu' tre copie speculari.

Dove mettere il file dipende dalla toolchain:

- **Arduino IDE**: crea una cartella sketch (es. `tools/esp32_gs/`)
  dentro al repo e salva li' `esp32_gs.ino`. Il path
  `#include "../../src/packet_format.h"` raggiunge l'header dal sketch.
- **Bench fuori repo**: copia `src/packet_format.h` accanto allo
  sketch e includi `"packet_format.h"`. Diventa un'altra copia da
  tenere allineata: meglio tenere lo sketch in-repo.
- **PlatformIO env dedicato** (opzionale, robusto): aggiungere a
  `platformio.ini`
  ```
  [env:esp32_gs]
  platform = espressif32
  board = esp32dev
  framework = arduino
  build_src_filter = -<*> +<../tools/esp32_gs/>
  build_flags = -I src
  ```
  e includere semplicemente `"packet_format.h"`.

```cpp
// esp32_gs.ino — ricevitore minimale per HIL test con PayloadX TX.
// Stampa CSV su Serial a 115200. Layout e CRC vengono dall'header
// condiviso: niente ridichiarazione locale, niente deriva.

#include <SPI.h>
#include <RF24.h>
#include <string.h>
#include "../../src/packet_format.h"   // adattare il path al setup scelto

#define CE_PIN  4    // adattare ai pin disponibili sulla dev-board
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);

void setup() {
    Serial.begin(115200);
    if (!radio.begin()) {
        Serial.println("[GS] NRF24 non rilevato");
        while (1) delay(1000);
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(62);
    radio.setDataRate(RF24_250KBPS);
    radio.setPayloadSize(sizeof(TelemetryPacket));
    radio.openReadingPipe(1, 0xE8E8F0F0E1LL);
    radio.startListening();
    Serial.println("[GS] in ascolto");
}

void loop() {
    if (!radio.available()) return;
    uint8_t buf[sizeof(TelemetryPacket)];
    radio.read(buf, sizeof(buf));
    switch (buf[0]) {
        case PKT_MAGIC_TELEMETRY: {
            TelemetryPacket p; memcpy(&p, buf, sizeof(p));
            if (pkt_crc16(buf, sizeof(p) - 2) != p.crc) { Serial.println("%%E,CRC,T"); break; }
            Serial.print("T,"); Serial.print(p.seq); Serial.print(',');
            Serial.print(p.lat_1e7 / 1.0e7, 7); Serial.print(',');
            Serial.print(p.lng_1e7 / 1.0e7, 7); Serial.print(',');
            Serial.print(p.alt_cm / 100.0f, 2); Serial.print(',');
            Serial.print(p.qw_i16 / 32767.0f, 4); Serial.print(',');
            Serial.print(p.qx_i16 / 32767.0f, 4); Serial.print(',');
            Serial.print(p.qy_i16 / 32767.0f, 4); Serial.print(',');
            Serial.print(p.qz_i16 / 32767.0f, 4); Serial.print(',');
            Serial.print(p.sats); Serial.print(',');
            Serial.println(p.flags, HEX);
            break;
        }
        case PKT_MAGIC_RAW_IMU: {
            RawImuPacket p; memcpy(&p, buf, sizeof(p));
            if (pkt_crc16(buf, sizeof(p) - 2) != p.crc) { Serial.println("%%E,CRC,R"); break; }
            Serial.print("R,"); Serial.print(p.seq); Serial.print(',');
            Serial.print(p.ax_mg / 1000.0f, 3); Serial.print(',');
            Serial.print(p.ay_mg / 1000.0f, 3); Serial.print(',');
            Serial.print(p.az_mg / 1000.0f, 3); Serial.print(',');
            Serial.print(p.gx_dps10 / 10.0f, 1); Serial.print(',');
            Serial.print(p.gy_dps10 / 10.0f, 1); Serial.print(',');
            Serial.print(p.gz_dps10 / 10.0f, 1); Serial.print(',');
            Serial.print(p.mx_uT); Serial.print(',');
            Serial.print(p.my_uT); Serial.print(',');
            Serial.println(p.mz_uT);
            break;
        }
        case PKT_MAGIC_BEACON: {
            BeaconPacket p; memcpy(&p, buf, sizeof(p));
            if (pkt_crc16(buf, sizeof(p) - 2) != p.crc) { Serial.println("%%E,CRC,B"); break; }
            p.id[sizeof(p.id) - 1] = '\0';
            Serial.print("B,"); Serial.print(p.id); Serial.print(',');
            Serial.print(p.expCount); Serial.print(',');
            Serial.println(p.expValue, 3);
            break;
        }
        default:
            Serial.println("%%E,MAGIC");
            break;
    }
}
```
