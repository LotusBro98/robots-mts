#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LDROBOT LD19 / Waveshare D500 realtime reader (stream-safe).

Что делает:
- Потоковый парсер: кольцевой буфер, поиск сигнатуры 0x54 0x2C,
  извлечение ровно 47 байт, проверка официальным CRC8 (таблица Waveshare).
- Корректная интерполяция углов между start_angle и end_angle.
- Возвращает словарь {угол_в_градусах(float): дистанция_в_метрах(float)} за одну «революцию».
  Никакого маппинга к целым градусам (сохраняем реальные углы).
- Параметры: смещение угла, инверсия направления (clockwise), таймаут на сбор оборота.

Ссылки:
- Формат пакета и CRC8 (таблица) — Waveshare D200/LDx wiki (47 байт, 12 точек, CRC8).  См. D200 LiDAR Kit → Communication Protocol.  CRC8 — табличный, старт 0. :contentReference[oaicite:2]{index=2}
- LD19: UART 230400 8N1, поток мер сразу после старта; ось X вперёд, угол растёт по часовой. :contentReference[oaicite:3]{index=3}
"""

from __future__ import annotations

import time
import struct
from typing import Dict, Generator, Optional

import serial

# -------------------- Протокол --------------------
HEADER = 0x54
VERLEN = 0x2C
FRAME_LEN = 47           # 1+1+2+2 + 12*(2+1) + 2 + 2 + 1 = 47
POINTS_PER_PACK = 12
BYTES_PER_POINT = 3

# Официальная таблица CRC8 из Waveshare wiki (D200/LDx), начальное значение 0x00.
# См. раздел Communication Protocol → "The CRC checksum is calculated as follows" (CrcTable).
CRC8_TABLE = bytes([
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
    0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
    0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
    0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
    0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
    0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
    0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
    0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
    0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
    0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
    0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
    0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
    0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
    0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
    0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
    0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
])

def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC8_TABLE[(c ^ b) & 0xFF]
    return c

# -------------------- Потоковый парсер --------------------
def frames_stream(ser: serial.Serial, idle_sleep: float = 0.001) -> Generator[bytes, None, None]:
    """
    Непрерывно читает из UART и выдаёт валидированные 47-байтные кадры.
    Неблокирующий режим: читаем только ser.in_waiting байт; если 0 — короткий sleep.
    """
    buf = bytearray()
    MAX_BUF = 8192
    TRIM_TO = 4096

    # на всякий случай переведём порт в неблокирующий режим
    try:
        ser.timeout = 0  # non-blocking: read() вернёт мгновенно, даже если нет байтов
    except Exception:
        pass

    while True:
        # читаем только имеющееся
        try:
            n_wait = ser.in_waiting if hasattr(ser, "in_waiting") else 0
        except Exception:
            n_wait = 0

        if n_wait:
            try:
                chunk = ser.read(n_wait)
            except serial.SerialException:
                break  # порт закрылся
            if chunk:
                buf.extend(chunk)
        else:
            # ничего нет — не блокируемся, просто даём CPU отдохнуть
            time.sleep(idle_sleep)

        # поиск кадров внутри накопленного буфера
        i = 0
        end_search = len(buf) - 2
        while i <= end_search:
            if buf[i] == HEADER and buf[i+1] == VERLEN:
                avail = len(buf) - i
                if avail < FRAME_LEN:
                    break  # ждём догрузки хвоста
                frame = bytes(buf[i:i+FRAME_LEN])
                if crc8(frame[:-1]) == frame[-1]:
                    del buf[:i+FRAME_LEN]
                    yield frame
                    i = 0
                    end_search = len(buf) - 2
                    continue
                else:
                    i += 1
                    continue
            i += 1

        if len(buf) > MAX_BUF:
            del buf[:len(buf) - TRIM_TO]

# -------------------- Разбор кадра --------------------
def parse_frame(frame: bytes):
    """
    Возвращает dict:
      speed_dps (deg/s), start_deg, end_deg, timestamp_ms, points=[(ang_deg, dist_m, intensity), ...]
    """
    if len(frame) != FRAME_LEN or frame[0] != HEADER or frame[1] != VERLEN:
        return None

    # header(1) verlen(1) speed(2) start_angle(2)
    _, _, speed, start_angle = struct.unpack_from('<BBHH', frame, 0)
    # 12 точек по 3 байта
    pts = []
    off = 6
    for _ in range(POINTS_PER_PACK):
        dist_mm, inten = struct.unpack_from('<HB', frame, off)
        pts.append((dist_mm, inten))
        off += BYTES_PER_POINT
    end_angle, ts_ms = struct.unpack_from('<HH', frame, 42)

    start_deg = (start_angle % 36000) / 100.0
    end_deg   = (end_angle   % 36000) / 100.0

    # Линейная интерполяция по документации Waveshare
    # step = (end - start) / (N-1), с учётом wrap через 360
    diff = (end_deg - start_deg) % 360.0
    step = diff / (POINTS_PER_PACK - 1) if POINTS_PER_PACK > 1 else 0.0

    angles = [ (start_deg + i * step) % 360.0 for i in range(POINTS_PER_PACK) ]

    out_pts = []
    for ang_deg, (dist_mm, inten) in zip(angles, pts):
        # Фильтр «пустых» точек (0 или 0xFFFF) — по факту там мусор/нет данных
        if dist_mm in (0, 0xFFFF) or inten == 0:
            continue
        out_pts.append((ang_deg, dist_mm / 1000.0, inten))

    return {
        "speed_dps": float(speed),
        "start_deg": start_deg,
        "end_deg": end_deg,
        "timestamp_ms": int(ts_ms),
        "points": out_pts,
    }

# -------------------- Сбор одного оборота --------------------
def read_full_scan_from_serial(
    ser: serial.Serial,
    angle_offset: float = 0.0,
    clockwise: bool = True,
    max_revo_seconds: float = 2.0,
    min_points: int = 60,
) -> Dict[float, float]:
    def transform_angle(a_deg: float) -> float:
        a = a_deg if clockwise else (-a_deg)
        return (a + angle_offset) % 360.0

    def is_wrap(prev: float, curr: float) -> bool:
        return (prev - curr) > 180.0 if clockwise else (curr - prev) > 180.0

    # Фаза A: ждём первую границу оборота
    t0 = time.time()
    last = None
    while True:
        for frame in frames_stream(ser):
            parsed = parse_frame(frame)
            if not parsed:
                continue
            for raw_deg, _, _ in parsed["points"]:
                deg = transform_angle(raw_deg)
                if last is not None and is_wrap(last, deg):
                    last = deg  # стартовая точка после wrap
                    break  # выходим на сбор
                last = deg
            else:
                continue
            break  # вышли по wrap
        if last is not None:
            break
        if time.time() - t0 > max_revo_seconds:
            return {}

    # Фаза B: собираем до следующего wrap
    distances: Dict[float, float] = {}
    t1 = time.time()
    while True:
        for frame in frames_stream(ser):
            parsed = parse_frame(frame)
            if not parsed:
                continue
            for raw_deg, dist_m, _ in parsed["points"]:
                deg = transform_angle(raw_deg)
                if last is not None and is_wrap(last, deg):
                    # завершили РОВНО один круг
                    return distances if len(distances) >= min_points else {}
                distances[deg] = dist_m
                last = deg
        if time.time() - t1 > max_revo_seconds:
            return distances if len(distances) >= min_points else {}

# -------------------- Пример запуска --------------------
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", "-p", required=True)
    parser.add_argument("--baud", "-b", type=int, default=230400)
    parser.add_argument("--offset", type=float, default=0.0, help="angle offset, degrees")
    parser.add_argument("--ccw", action="store_true", help="interpret angles counter-clockwise")
    parser.add_argument("--timeout", type=float, default=2.0, help="max seconds to gather one revolution")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    try:
        scan = read_full_scan_from_serial(
            ser,
            angle_offset=args.offset,
            clockwise=not args.ccw,
            max_revo_seconds=args.timeout,
        )

        for i, (ang, dist) in enumerate(sorted(scan.items(), key=lambda kv: kv[0])):
            print(f"{ang:7.2f}°  {dist:6.3f} m")
        print(f"Total points: {len(scan)}")
    finally:
        ser.close()
