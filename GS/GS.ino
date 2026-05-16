// EdgeFlyte 1U CubeSat Ground Station v2
// Copyright ©2025 EdgeFlyte.

// This code is licensed for use in non-commercial applications only.
// Redistribution and modification are permitted for personal, educational,
// or research purposes, provided that proper credit is given.

// THIS SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
// FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// PayloadX (fork GPL v3) — modifiche aggiunte:
//   - Decoder dei pacchetti binari emessi da src/radio.cpp (Telemetry,
//     RawImu, Beacon). Layout e CRC NON sono piu' ridichiarati qui:
//     vengono dalla fonte di verita' unica ../src/packet_format.h.
//   - Le sole sezioni con commenti in italiano (e questo blocco) sono coperte
//     da GPL v3; il resto della GS resta sotto la licenza EdgeFlyte sopra.

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// Layout e magic byte dei pacchetti vivono in src/packet_format.h.
// Il path relativo funziona perche' il preprocessore C risolve il "..."
// rispetto al file che fa l'include (gcc / avr-gcc / arduino-builder).
// Se in futuro questo non bastasse per Arduino IDE su Windows, valutare
// un PlatformIO env dedicato alla GS con build_flags = -I ../src.
#include "../src/packet_format.h"

#define CE_PIN 7
#define CSN_PIN 8
RF24 radio(CE_PIN, CSN_PIN);

uint16_t runtimeSerialNumber __attribute__((section(".noinit")));
uint16_t EEMEM serialNumber;

uint64_t rxPackets = 0;
uint64_t txPackets = 0;
uint64_t errPackets = 0;

uint8_t radioChannel = 62;
uint8_t radioPowerLevel = RF24_PA_LOW;
// PayloadX: data rate ALLINEATO al TX in src/radio.cpp (RF24_250KBPS).
// Era RF24_1MBPS nel codice EdgeFlyte v2, ma TX e RX devono usare
// LO STESSO data rate o nessun pacchetto viene riconosciuto.
uint8_t radioDataRate = RF24_250KBPS;

// PayloadX: indirizzo del pipe ALLINEATO al TX in src/radio.cpp
// (radio.openWritingPipe(0xE8E8F0F0E1LL)). Il vecchio "EFEF0" appartiene
// al protocollo ASCII pre-binarizzazione e veniva mantenuto per inerzia.
// La forma uint64_t corrisponde, come bytes-on-wire, a 0xE1 0xF0 0xF0
// 0xE8 0xE8 (RF24 trasmette LSB first).
static const uint64_t PIPE_ADDRESS = 0xE8E8F0F0E1ULL;

// Stampa CSV beacon: B,id,count,value
static void printBeacon(const BeaconPacket &pkt) {
    Serial.print(F("B,"));
    Serial.print(pkt.id);
    Serial.print(',');
    Serial.print(pkt.expCount);
    Serial.print(',');
    Serial.println(pkt.expValue, 3);
}

// Stampa CSV raw IMU: R,seq,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT
static void printRawImu(const RawImuPacket &pkt) {
    Serial.print(F("R,"));
    Serial.print(pkt.seq);
    Serial.print(',');
    Serial.print(pkt.ax_mg / 1000.0f, 3);
    Serial.print(',');
    Serial.print(pkt.ay_mg / 1000.0f, 3);
    Serial.print(',');
    Serial.print(pkt.az_mg / 1000.0f, 3);
    Serial.print(',');
    Serial.print(pkt.gx_dps10 / 10.0f, 1);
    Serial.print(',');
    Serial.print(pkt.gy_dps10 / 10.0f, 1);
    Serial.print(',');
    Serial.print(pkt.gz_dps10 / 10.0f, 1);
    Serial.print(',');
    Serial.print(pkt.mx_uT);
    Serial.print(',');
    Serial.print(pkt.my_uT);
    Serial.print(',');
    Serial.println(pkt.mz_uT);
}

// Stampa una riga CSV per il client a valle (es. logger su PC).
// Formato: T,seq,lat,lng,alt_m,qw,qx,qy,qz,sats,flags_hex
static void printTelemetry(const TelemetryPacket &pkt) {
    Serial.print(F("T,"));
    Serial.print(pkt.seq);
    Serial.print(',');
    Serial.print(pkt.lat_1e7 / 1.0e7, 7);
    Serial.print(',');
    Serial.print(pkt.lng_1e7 / 1.0e7, 7);
    Serial.print(',');
    Serial.print(pkt.alt_cm / 100.0f, 2);
    Serial.print(',');
    Serial.print(pkt.qw_i16 / 32767.0f, 4);
    Serial.print(',');
    Serial.print(pkt.qx_i16 / 32767.0f, 4);
    Serial.print(',');
    Serial.print(pkt.qy_i16 / 32767.0f, 4);
    Serial.print(',');
    Serial.print(pkt.qz_i16 / 32767.0f, 4);
    Serial.print(',');
    Serial.print(pkt.sats);
    Serial.print(',');
    Serial.println(pkt.flags, HEX);
}
// -------------------------------------------------------------------------

void setup() {
  if (!(MCUSR & (1 << WDRF))) runtimeSerialNumber = eeprom_read_byte(&serialNumber);
  MCUSR = 0;
  delay(1000);
  Serial.begin(9600);
  SPI.begin();
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  if (!radio.begin()) {
    while(1){
      digitalWrite(2, 1);
      delay(1000);
      digitalWrite(2, 0);
      delay(1000);
      Serial.println("%%99 RADIO ERROR");
    }
  }

  radio.setChannel(radioChannel);
  radio.setPALevel(RF24_PA_LOW);
  // PayloadX: dimensione fissa allineata al TX dell'ESP32
  // (src/radio.cpp -> radio.setPayloadSize(sizeof(TelemetryPacket))).
  radio.setPayloadSize(sizeof(TelemetryPacket));
  radio.openWritingPipe(PIPE_ADDRESS);
  radio.openReadingPipe(1, PIPE_ADDRESS);
  radio.setDataRate(radioDataRate);
  radio.startListening();
  digitalWrite(2, 1);
}


char txMSG[32];

uint8_t pipe;

void loop() {
  if (radio.available()) {
    digitalWrite(3, 1);

    // PayloadX: lettura BINARIA. TX e RX condividono la stessa
    // setPayloadSize, quindi leggiamo un blocco fisso e dispatchiamo
    // sul primo byte (magic). Telemetry e Beacon hanno size identica
    // proprio per coesistere sullo stesso pipe.
    uint8_t buf[sizeof(TelemetryPacket)];
    radio.read(buf, sizeof(buf));
    uint8_t magic = buf[0];

    if (magic == PKT_MAGIC_TELEMETRY) {
      TelemetryPacket pkt;
      memcpy(&pkt, buf, sizeof(pkt));
      uint16_t expected = pkt_crc16((const uint8_t *)&pkt,
                                    sizeof(pkt) - sizeof(pkt.crc));
      if (expected != pkt.crc) {
        errPackets++;
        Serial.println(F("%%E,CRC,T"));
      } else {
        rxPackets++;
        printTelemetry(pkt);
      }
    } else if (magic == PKT_MAGIC_BEACON) {
      BeaconPacket pkt;
      memcpy(&pkt, buf, sizeof(pkt));
      uint16_t expected = pkt_crc16((const uint8_t *)&pkt,
                                    sizeof(pkt) - sizeof(pkt.crc));
      if (expected != pkt.crc) {
        errPackets++;
        Serial.println(F("%%E,CRC,B"));
      } else {
        rxPackets++;
        // Sicurezza: garantiamo terminazione anche se il TX ha mancato
        // di azzerare l'ultimo byte di id.
        pkt.id[sizeof(pkt.id) - 1] = '\0';
        printBeacon(pkt);
      }
    } else if (magic == PKT_MAGIC_RAW_IMU) {
      RawImuPacket pkt;
      memcpy(&pkt, buf, sizeof(pkt));
      uint16_t expected = pkt_crc16((const uint8_t *)&pkt,
                                    sizeof(pkt) - sizeof(pkt.crc));
      if (expected != pkt.crc) {
        errPackets++;
        Serial.println(F("%%E,CRC,R"));
      } else {
        rxPackets++;
        printRawImu(pkt);
      }
    } else {
      errPackets++;
      Serial.println(F("%%E,MAGIC"));
    }

    digitalWrite(3, 0);
  }
  parseSerial();
}


void parseSerial(){
  if (Serial.available()) {
    for(int a=0; a<32; a++){
      txMSG[a] = 0x20;
    }

    if(txMSG[0] == '%' && txMSG[1] == '^'){   // Prefix for a command code
      // Command Code
      digitalWrite(4, 1);
      digitalWrite(2, 0);

      if(txMSG[2] == '0' && txMSG[3] == '0'){ // General Ping
        Serial.println("%%00,OK");
        return;
      }

      if(txMSG[2] == '0' && txMSG[3] == '1'){ // System Status Request
        Serial.print("%%01,");
        Serial.print(runtimeSerialNumber);
        Serial.print(',');
        Serial.println("");
        return;
      }

      if(txMSG[2] == '9' && txMSG[3] == '0'){ // Write Serial Number
        if(txMSG[5] == 'G' && txMSG[6] == 'R' && txMSG[7] == 'S'){ // Confirm Serial Number
          const char* hexStr = &txMSG[9];
          uint16_t number = strtoul(hexStr, NULL, 16);
          Serial.print("Converted number: ");
          Serial.println(number, HEX);
          Serial.println("%%90,OK");
          return;
        }
        Serial.print("%%90,?");
        return;
      }

      if(txMSG[2] == '0' && txMSG[3] == '2'){ // Set Radio Channel

        char c1 = txMSG[5];
        char c2 = txMSG[6];
        uint8_t c = (uint8_t)((c1 - '0') * 10) + (uint8_t)(c2 - '0');
        radio.stopListening();
        radio.setChannel(radioChannel);
        radio.startListening();
        Serial.println("%%02,OK");
        return;
      }

      digitalWrite(4, 0);
      digitalWrite(2, 1);
      return;
    }

    digitalWrite(4, 1);
    delay(100);
    int rxB = Serial.available();
    for(int i=0; i<rxB; i++){
      txMSG[i] = Serial.read();
    }
    Serial.println(txMSG);
    radio.stopListening();
    transmitPacket();
    radio.startListening();
    digitalWrite(4, 0);
  }
}



void saveSerialNumber(uint16_t number){
  eeprom_update_byte(&serialNumber, number);
  runtimeSerialNumber = number;
}


void setRadioAddress(uint8_t *addr){
  // In Process
}


void transmitPacket(){
  unsigned long start_timer = micros();
  bool r = radio.write(&txMSG, 32);
  unsigned long end_timer = micros();

  if (r) {
    Serial.print(F("Transmission successful! "));
    Serial.print(F("Time to transmit = "));
    Serial.print(end_timer - start_timer);
    Serial.print(F(" us. Sent: "));
    Serial.println(txMSG);
  } else {
    Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
  }
}
