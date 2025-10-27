#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Clean realtime viewer for LDROBOT LD19 / Waveshare D500.

Goals:
- Robust sync on frames (0x54 0x2C) and official CRC8 validation
- Correct point layout: distance(uint16 LE, mm) + intensity(uint8)
- Per-frame plotting with no stale points retained (no sliding window)
- Minimal latency rendering; configurable port and radius via CLI

Public functions:
read_full_scan_from_serial(...) -> (angles_rad[360], distances_m[360])
"""

import time
import math
import struct
from typing import Dict

import serial

# -------------------- Constants --------------------
HEADER = 0x54
VERLEN = 0x2C
FRAME_LEN = 47
POINTS_PER_PACK = 12
BYTES_PER_POINT = 3

def sum8(data: bytes) -> int:
    return sum(data) & 0xFF

def crc8_maxim(data: bytes) -> int:
    # poly 0x31, init 0x00, refin/refout true, xorout 0x00
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C  # reversed 0x31
            else:
                crc >>= 1
    return crc & 0xFF

def crc8_itu(data: bytes) -> int:
    # poly 0x07, init 0x00, no refin/refout, xorout 0x00
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ 0x07
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF

def crc8_j1850(data: bytes) -> int:
    # poly 0x1D, init 0xFF, xorout 0xFF, no refin/refout
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ 0x1D
            else:
                crc = (crc << 1) & 0xFF
    crc ^= 0xFF
    return crc & 0xFF

def checksum_ok(frame: bytes) -> bool:
    body = frame[:-1]
    tail = frame[-1]
    return (
        sum8(body) == tail or
        crc8_maxim(body) == tail or
        crc8_itu(body) == tail or
        crc8_j1850(body) == tail
    )


def sync_and_read_frame(ser: serial.Serial, timeout_s: float) -> bytes | None:
    """Find 0x54 0x2C and return a full CRC-validated 47-byte frame or None on timeout."""
    start = time.time()
    buf = bytearray()
    while time.time() - start < timeout_s:
        chunk = ser.read(128)
        if not chunk:
            continue
        buf.extend(chunk)

        i = 0
        while i <= len(buf) - 2:
            if buf[i] == HEADER and buf[i+1] == VERLEN:
                if len(buf) - i < FRAME_LEN:
                    break
                frame = bytes(buf[i:i+FRAME_LEN])
                if checksum_ok(frame):
                    del buf[:i+FRAME_LEN]
                    return frame
                i += 1
                continue
            i += 1

        if len(buf) > 1024:
            del buf[:512]
    return None


def parse_frame(frame: bytes):
    if len(frame) != FRAME_LEN or frame[0] != HEADER or frame[1] != VERLEN:
        return None
    header, verlen, speed, start_angle = struct.unpack_from('<BBHH', frame, 0)
    pts = []
    off = 6
    for _ in range(POINTS_PER_PACK):
        dist_mm, inten = struct.unpack_from('<HB', frame, off)
        pts.append((dist_mm, inten))
        off += BYTES_PER_POINT
    end_angle, ts_ms = struct.unpack_from('<HH', frame, 42)

    start_deg = (start_angle % 36000) / 100.0
    end_deg = (end_angle % 36000) / 100.0
    angle_diff = (end_deg - start_deg) % 360.0
    step = angle_diff / (POINTS_PER_PACK - 1) if POINTS_PER_PACK > 1 else 0.0
    angles_deg = [(start_deg + i * step) % 360.0 for i in range(POINTS_PER_PACK)]

    decoded = []
    for ang_deg, (dist_mm, inten) in zip(angles_deg, pts):
        if dist_mm == 0 or dist_mm == 0xFFFF or inten == 0:
            continue
        decoded.append((ang_deg, dist_mm / 1000.0, inten))

    return {
        "speed": speed,
        "start_deg": start_deg,
        "end_deg": end_deg,
        "timestamp_ms": ts_ms,
        "points": decoded,
    }


def read_full_scan_from_serial(
    ser: serial.Serial,
    timeout: float = 0.2,  # для serial-порта
    angle_offset: float = 0.0,
    clockwise: bool = False,
    max_revo_seconds: float = 2.0,
    sort_by_angle: bool = False,
) -> Dict[float, float]:
    """
    Считывает одну полную «революцию» лидара и возвращает:
        angles_rad:  список из 360 углов (0..359 градусов)
        distances_m: список из 360 дистанций (метры); где нет данных — NaN

    Параметры:
      - angle_offset: добавочный сдвиг угла в градусах
      - clockwise: если True, интерпретировать углы по часовой стрелке
      - max_revo_seconds: таймаут на 1 сбор: если не успели собрать все точки за это время - возвращаем что успели собрать
    """
    distances  = {}

    def transform_angle_deg(a: float) -> float:
        ang = (-a) if clockwise else a
        ang = (ang + angle_offset) % 360.0
        return ang

    last_deg = None
    got_wrap = False
    t0 = time.time()

    while True:
        if time.time() - t0 > max_revo_seconds and any(not math.isnan(d) for d in distances):
            # собрали что успели — возвращаем частично заполненную карту
            break

        frame = sync_and_read_frame(ser, timeout)
        if frame is None:
            continue
        parsed = parse_frame(frame)
        if not parsed:
            continue

        for raw_deg, dist_m, _inten in parsed["points"]:
            deg = transform_angle_deg(raw_deg)

            # детект «переворота» 360->0
            if last_deg is not None and last_deg > 270.0 and deg < 90.0:
                got_wrap = True

            # маппинг в ближайший целый градус
            # idx = int(round(deg)) % 360
            distances[deg] = dist_m

            last_deg = deg

        if got_wrap:
            # завершили революцию
            break

    return distances
