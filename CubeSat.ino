// CubeSat.ino – PayloadX
//
// IMPORTANTE: questo file NON e' l'entry point compilato.
// E' un marker documentale che resta alla root del progetto per
// abitudine (chi apre la cartella in Arduino IDE cerca un .ino qui).
//
// L'entry point REALE (setup/loop, scheduler non bloccante con
// millis(), modulo payload + beacon) e' in src/main.cpp ed e' quello
// che PlatformIO compila (src_dir = src in platformio.ini).
//
// Toolchain ufficiale: PlatformIO. Comandi rapidi:
//   pio run                 # compilazione
//   pio run -t upload       # upload sull'ESP32
//   pio device monitor      # monitor seriale
//
// Per chi vuole davvero usare Arduino IDE: il layout con src/
// separato non e' supportato in modo pulito dal core Arduino,
// quindi PlatformIO resta la via raccomandata. Vedi docs/BUILD.md.
//
// Questo file e' fuori da src_dir e quindi NON viene compilato
// da PlatformIO: modificarlo non ha effetto sul firmware.
