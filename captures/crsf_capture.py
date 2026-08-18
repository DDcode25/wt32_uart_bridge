#!/usr/bin/env python3
"""Запись эталонного трафика CRSF с оригинальной платы.

Слушает UDP, куда .51 отдаёт копию всего, что получает из сети (см.
/api/mirror). Пишет два файла:

  <имя>.bin   — сырые байты подряд, как пришли: поток для повторного
                разбора любым инструментом;
  <имя>.jsonl — по строке на датаграмму: время от начала записи, длина,
                hex. Нужен, потому что в сыром потоке границы датаграмм
                теряются, а для CRSF важно, чем именно устройство режет
                поток на посылки.

Разбор идёт параллельно записи и печатает сводку: какие типы кадров,
с каким темпом, сколько ошибок CRC. Разбор ничего не меняет в записи —
если он ошибается, файлы всё равно верны.
"""
import argparse, json, socket, sys, time

CRSF_ADDR = {0xC8: "FC", 0xEE: "MODULE", 0xEA: "RADIO", 0xEC: "RX", 0x00: "BROADCAST"}
TYPES = {0x02: "GPS", 0x07: "VARIO", 0x08: "BATTERY", 0x09: "BARO_ALT",
         0x0B: "HEARTBEAT", 0x14: "LINK_STATS", 0x16: "RC_CHANNELS",
         0x1E: "ATTITUDE", 0x21: "FLIGHT_MODE", 0x28: "DEVICE_PING",
         0x29: "DEVICE_INFO", 0x2B: "PARAM_ENTRY", 0x2C: "PARAM_READ",
         0x2D: "PARAM_WRITE", 0x32: "COMMAND", 0x3A: "RADIO_ID"}


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5055)
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--out", default="crsf_reference")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", args.port))
    s.settimeout(1.0)

    raw = open(args.out + ".bin", "wb")
    meta = open(args.out + ".jsonl", "w")

    buf = bytearray()
    types, bad_crc, frames, datagrams, total = {}, 0, 0, 0, 0
    t0 = time.time()
    print("слушаю UDP :%d, запись %.0f с -> %s.bin / .jsonl"
          % (args.port, args.seconds, args.out), flush=True)

    while time.time() - t0 < args.seconds:
        try:
            pkt, src = s.recvfrom(4096)
        except socket.timeout:
            continue
        now = time.time() - t0
        datagrams += 1
        total += len(pkt)
        raw.write(pkt)
        meta.write(json.dumps({"t": round(now, 6), "src": src[0],
                               "len": len(pkt), "hex": pkt.hex()}) + "\n")

        buf += pkt
        while len(buf) >= 4:
            if buf[0] not in CRSF_ADDR:
                del buf[:1]
                continue
            ln = buf[1]
            if ln < 2 or ln > 62:
                del buf[:1]
                continue
            if len(buf) < ln + 2:
                break
            frame = bytes(buf[:ln + 2])
            if crc8(frame[2:ln + 1]) == frame[ln + 1]:
                t = frame[2]
                types[t] = types.get(t, 0) + 1
                frames += 1
                del buf[:ln + 2]
            else:
                bad_crc += 1
                del buf[:1]

    raw.close()
    meta.close()
    dur = time.time() - t0
    print("\nзаписано за %.1f с: %d датаграмм, %d байт (%.0f Б/с)"
          % (dur, datagrams, total, total / dur if dur else 0))
    print("разобрано кадров %d (%.1f/с), CRC-ошибок %d" % (frames, frames / dur if dur else 0, bad_crc))
    for t, n in sorted(types.items(), key=lambda kv: -kv[1]):
        print("   0x%02X %-14s %6d  %6.1f/с" % (t, TYPES.get(t, "?"), n, n / dur if dur else 0))
    if not datagrams:
        print("НИЧЕГО НЕ ПРИШЛО — проверьте, включено ли зеркало на .51")
    return 0


if __name__ == "__main__":
    sys.exit(main())
