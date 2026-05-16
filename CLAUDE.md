# PayloadX — Guida per Claude Code

> Questo file viene letto automaticamente da Claude Code all'avvio nella
> cartella del progetto. Definisce contesto, vincoli e obiettivi.

## Cos'è il progetto

Firmware di un nanosatellite CubeSat (fork GPL v3 di EdgeFlyte CubeSat v1,
iniziativa "PayloadX"). Target hardware: ESP32. Toolchain: PlatformIO
(vedi platformio.ini). Scopo: educativo / prototipazione / simulazione di
volo. "Payload" = carico utile scientifico del satellite (terminologia
aerospaziale standard).

## Stato attuale

- src/main.cpp: entry point reale (setup/loop, scheduler non bloccante
  con millis(), modulo payload + beacon). E' l'unica fonte di verita'
  per il firmware ed e' cio' che PlatformIO compila (src_dir = src).
- src/imu.cpp, src/gps.cpp, src/radio.cpp: driver implementati
  (MPU-9250 SparkFun, Adafruit_GPS, NRF24 con pacchetto binario 28 byte).
- src/payload.h, src/payload.cpp: modulo payload + beacon.
- CubeSat.ino (alla root): solo marker documentale, NON compilato da
  PlatformIO (sta fuori da src_dir). Non modificarlo aspettandosi
  effetti sul firmware.
- src/Madgwick.{h,cpp}: FONTE DI VERITA' UNICA del filtro Madgwick.
  Esposto da src/attitude.{h,cpp}, che fonde gyro+accel+mag e produce
  il quaternione (qw,qx,qy,qz).
- MCB/: codice EdgeFlyte v1 originario, DEPRECATO. Non compilato dalla
  toolchain ufficiale. MCB/Madgwick.{h,cpp} e' stato rimosso per
  eliminare la duplicazione; ogni modifica al filtro va fatta in src/.
- GS/: ground station (ATmega328PB) — non ancora aggiornata al nuovo
  formato di pacchetto binario.

## Vincoli (non negoziabili)

1. Licenza GPL v3: ogni modifica resta sotto GPL v3, non rimuovere
   intestazioni di licenza, non introdurre codice con licenza incompatibile.
2. Il progetto deve compilare con PlatformIO senza errori:
   pio run deve terminare con successo. Questo è il criterio di verifica.
3. Non aggiungere funzioni di rete/telemetria che trasmettano su frequenze
   reali senza un commento esplicito che ricordi: serve licenza radioamatore
   + coordinazione di frequenza IARU. Il beacon usa un identificativo
   placeholder finché l'utente non inserisce il proprio nominativo.
4. Codice in C++ Arduino, commenti in italiano coerenti con i file esistenti.
5. Lavorare su un branch git, mai direttamente su main. Commit piccoli e
   descrittivi.

## Obiettivi, in ordine (ognuno con criterio di verifica)

1. Verifica build base: esegui pio run, risolvi errori di compilazione.
   Verifica: pio run termina senza errori.
2. Decoder Ground Station: aggiorna GS/ per decodificare il pacchetto
   binario TelemetryPacket di src/radio.cpp (magic 0x54, seq, lat/lng 1e7,
   alt cm, accel mg, CRC-16). Verifica: compila, parsing speculare.
3. Integrazione filtro Madgwick: porta MCB/Madgwick nel loop modulare,
   calcola il quaternione di assetto. Verifica: compila, sta nei 32 byte.
4. Beacon via radio: invia il beacon anche via NRF24 con magic diverso.
   Verifica: compila, GS distingue i due tipi.
5. docs/BUILD.md: come compilare, caricare, test con due ESP32.

## Come lavorare

- Un obiettivo alla volta, in ordine.
- Dopo ogni obiettivo: compila, poi commit git con messaggio chiaro.
- Se serve hardware fisico, documenta l'assunzione e verifica la compilazione.
- Non reintrodurre la telemetria ASCII vecchia (bug di troncamento >32 byte).
