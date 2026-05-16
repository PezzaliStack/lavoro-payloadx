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
//   - Decoder del pacchetto binario TelemetryPacket emesso da src/radio.cpp
//     (magic 0x54, seq, lat/lng 1e7, alt cm, accel mg, sats, flags, CRC-16).
//   - Le sole sezioni con commenti in italiano (e questo blocco) sono coperte
//     da GPL v3; il resto della GS resta sotto la licenza EdgeFlyte sopra.

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

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
uint8_t radioDataRate = RF24_1MBPS;

uint8_t address[6] = {"EFEF0"};

// --- PayloadX: definizione SPECULARE del pacchetto telemetria -------------
// Deve restare bit-per-bit identica a quella in src/radio.cpp sull'ESP32.
// #pragma pack(1) garantisce zero padding tra i campi.
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

static const uint8_t PKT_MAGIC_TELEMETRY = 0x54;

// CRC-16 con polinomio 0x1021, init 0xFFFF (uguale a src/radio.cpp).
// Calcolato sui byte del pacchetto ESCLUSO il campo crc finale.
static uint16_t crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
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
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.setDataRate(radioDataRate);
  radio.startListening();
  digitalWrite(2, 1);
}


char txMSG[32];

uint8_t pipe;

void loop() {
  if (radio.available()) {
    digitalWrite(3, 1);

    // PayloadX: lettura BINARIA. Il vecchio percorso ASCII (Serial.println su
    // char[32]) e' stato rimosso: il TX ora emette TelemetryPacket binario.
    TelemetryPacket pkt;
    radio.read(&pkt, sizeof(pkt));

    if (pkt.magic != PKT_MAGIC_TELEMETRY) {
      errPackets++;
      Serial.println(F("%%E,MAGIC"));
    } else {
      uint16_t expected = crc16((const uint8_t *)&pkt,
                                sizeof(pkt) - sizeof(pkt.crc));
      if (expected != pkt.crc) {
        errPackets++;
        Serial.println(F("%%E,CRC"));
      } else {
        rxPackets++;
        printTelemetry(pkt);
      }
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
