#!/usr/bin/env python3
# payloadx_sim.py - Simulatore software del link CubeSat -> Ground Station
# Riproduce cio' che farebbe il banco con DUE ESP32, senza hardware.
# Protocollo come docs/BUILD.md sez.4: magic T=0x54 R=0x52 B=0x42,
# CRC-16/CCITT-FALSE, coord *1e7, alt in cm, quaternione int16 *32767.

import struct, time, math, random

MAGIC_T = 0x54
MAGIC_R = 0x52
MAGIC_B = 0x42

def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def _c16(x):
    return max(-32768, min(32767, int(x)))

class CubeSat:
    def __init__(self):
        self.seq = 0
        self.exp_count = 0
        self.t0 = time.time()
    def _orbit(self):
        t = time.time() - self.t0
        ang = (t / 92.0) * 2 * math.pi * 6
        lat = math.degrees(math.asin(math.sin(ang) * math.sin(0.9)))
        lng = ((math.degrees(ang) + 180) % 360) - 180
        alt = 420.0 + 15.0 * math.sin(t * 0.3)
        a = t * 0.2
        qw, qx, qy, qz = math.cos(a/2), math.sin(a/2)*0.3, math.sin(a/2)*0.5, math.sin(a/2)*0.8
        n = math.sqrt(qw*qw+qx*qx+qy*qy+qz*qz) or 1.0
        return lat, lng, alt, (qw/n, qx/n, qy/n, qz/n)
    def packet_T(self):
        lat, lng, alt, q = self._orbit()
        sats = 4 + int(4 + 3 * math.sin((time.time()-self.t0)*0.4))
        flags = 0x01 if sats > 0 else 0x00
        body = struct.pack("<BHiiihhhhBB", MAGIC_T, self.seq & 0xFFFF,
            int(lat*1e7), int(lng*1e7), int(alt*100000),
            _c16(q[0]*32767), _c16(q[1]*32767), _c16(q[2]*32767), _c16(q[3]*32767),
            sats, flags)
        self.seq += 1
        return body + struct.pack("<H", crc16_ccitt(body))
    def packet_R(self):
        t = time.time() - self.t0
        ax, ay, az = 30*math.sin(t), 25*math.cos(t), 1000+20*math.sin(t*2)
        gx, gy, gz = 12*math.sin(t*1.3), 8*math.cos(t), 5*math.sin(t*0.7)
        mx, my, mz = 21, -38, 14
        body = struct.pack("<BHhhhhhhhhh", MAGIC_R, self.seq & 0xFFFF,
            _c16(ax), _c16(ay), _c16(az),
            _c16(gx*10), _c16(gy*10), _c16(gz*10), _c16(mx), _c16(my), _c16(mz))
        return body + struct.pack("<H", crc16_ccitt(body))
    def packet_B(self):
        self.exp_count += 1
        ev = 1.2 + 0.9 * math.sin((time.time()-self.t0)*2.3)
        bid = b"PAYLOADX-1".ljust(16, b"\x00")
        body = struct.pack("<BHIf", MAGIC_B, self.seq & 0xFFFF, self.exp_count, ev) + bid
        return body + struct.pack("<H", crc16_ccitt(body))

def radio_channel(packet, loss=0.05, corrupt=0.03):
    if random.random() < loss:
        return None
    pkt = bytearray(packet)
    if random.random() < corrupt:
        i = random.randrange(len(pkt))
        pkt[i] ^= 1 << random.randrange(8)
    return bytes(pkt)

class GroundStation:
    def __init__(self):
        self.last_seq = {}
        self.lost = 0; self.bad_crc = 0; self.ok = 0
    def receive(self, raw):
        if raw is None:
            self.lost += 1
            print("  [..]  pacchetto perso (nessun segnale)")
            return
        body, rx_crc = raw[:-2], struct.unpack("<H", raw[-2:])[0]
        if crc16_ccitt(body) != rx_crc:
            self.bad_crc += 1
            print("  [!!]  CRC errato -> pacchetto scartato")
            return
        self.ok += 1
        magic = raw[0]
        seq = struct.unpack("<H", raw[1:3])[0]
        gap = ""
        if magic in self.last_seq and seq > self.last_seq[magic] + 1:
            gap = f"  (saltati {seq - self.last_seq[magic] - 1})"
        self.last_seq[magic] = seq
        if magic == MAGIC_T:
            _,_,lat,lng,alt,qw,qx,qy,qz,sats,flags = struct.unpack("<BHiiihhhhBB", body)
            print(f"  [T] seq={seq:<4} lat={lat/1e7:8.4f} lng={lng/1e7:9.4f} "
                  f"alt={alt/100000:6.1f}km  q=({qw/32767:+.3f},{qx/32767:+.3f},"
                  f"{qy/32767:+.3f},{qz/32767:+.3f}) sats={sats} fix={flags}{gap}")
        elif magic == MAGIC_R:
            v = struct.unpack("<BHhhhhhhhhh", body)
            print(f"  [R] seq={seq:<4} accel(mg)=({v[2]},{v[3]},{v[4]})  "
                  f"gyro(dps)=({v[5]/10:.1f},{v[6]/10:.1f},{v[7]/10:.1f})  "
                  f"mag(uT)=({v[8]},{v[9]},{v[10]}){gap}")
        elif magic == MAGIC_B:
            _,_,cnt,val = struct.unpack("<BHIf", body[:11])
            bid = body[11:27].split(b"\x00")[0].decode(errors="replace")
            print(f"  [B] BEACON  id={bid}  misura#{cnt}  valore={val:.3f}{gap}")
    def summary(self):
        tot = self.ok + self.bad_crc + self.lost
        print("\n" + "-"*60)
        print(f"  Ricevuti OK : {self.ok}/{tot}")
        print(f"  CRC errati  : {self.bad_crc}")
        print(f"  Persi       : {self.lost}")
        if tot:
            print(f"  Affidabilita' link: {100*self.ok/tot:.1f}%")
        print("-"*60)

def main():
    print("="*60)
    print("  SIMULATORE LINK PayloadX  (CubeSat -> Ground Station)")
    print("  Nessun hardware. Ctrl+C per fermare.")
    print("="*60 + "\n")
    sat = CubeSat(); gs = GroundStation(); tick = 0
    try:
        while True:
            tick += 1
            gs.receive(radio_channel(sat.packet_T()))
            gs.receive(radio_channel(sat.packet_R()))
            if tick % 10 == 0:
                gs.receive(radio_channel(sat.packet_B()))
            time.sleep(1.0)
    except KeyboardInterrupt:
        gs.summary()

if __name__ == "__main__":
    main()
