#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Clean realtime viewer for LDROBOT LD19 / Waveshare D500.

Goals:
- Robust sync on frames (0x54 0x2C) and official CRC8 validation
- Correct point layout: distance(uint16 LE, mm) + intensity(uint8)
- Per-frame plotting with no stale points retained (no sliding window)
- Minimal latency rendering; configurable port and radius via CLI

Usage examples:
  python ld19_view_clean.py --port /dev/tty.usbserial-0001 --rmax 6
  python ld19_view_clean.py -p /dev/tty.usbserial-0001 -r 8 --baud 230400
"""

import sys
import time
import math
import struct
import argparse

import serial
import matplotlib.pyplot as plt
from matplotlib import cm, colors


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


def main():
    ap = argparse.ArgumentParser(description="LD19 realtime viewer (fresh frame only)")
    ap.add_argument("--port", "-p", default="/dev/tty.usbserial-0001", help="Serial port path")
    ap.add_argument("--baud", "-b", type=int, default=230400, help="Baud rate")
    ap.add_argument("--timeout", "-t", type=float, default=0.2, help="Serial read timeout seconds")
    ap.add_argument("--rmax", "-r", type=float, default=6.0, help="Polar plot radius in meters")
    ap.add_argument("--markersize", "-s", type=float, default=6.0, help="Scatter marker size")
    ap.add_argument("--angle-offset", type=float, default=0.0, help="Additive angle offset in degrees")
    ap.add_argument("--clockwise", action="store_true", help="Interpret lidar angles as clockwise")
    ap.add_argument("--accum", type=int, default=0, help="Keep last N frames (0 = off)")
    ap.add_argument("--revo", action="store_true", help="Show last full revolution (stable panorama)")
    ap.add_argument("--color-mode", choices=["intensity","time"], default="intensity", help="Color by intensity or time")
    ap.add_argument("--cmap", default="viridis", help="Matplotlib colormap name")
    ap.add_argument("--fade-tau", type=float, default=0.0, help="Time constant for alpha fade (seconds); 0 disables")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    print(f"[LD19] Opened {args.port} @ {args.baud} baud")

    plt.ion()
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_rmax(args.rmax)
    ax.set_title("LD19 live — fresh frame")

    angle_offset = args.angle_offset

    def transform_angle_deg(a: float) -> float:
        ang = (-a) if args.clockwise else a
        ang = (ang + angle_offset) % 360.0
        return ang

    def on_key(event):
        nonlocal angle_offset
        step_small = 1.0
        step_big = 5.0
        if event.key in ("left", "a"):
            angle_offset -= step_small
        elif event.key in ("right", "d"):
            angle_offset += step_small
        elif event.key == "[":
            angle_offset -= step_big
        elif event.key == "]":
            angle_offset += step_big
        else:
            return
        print(f"[angle_offset] {angle_offset:.2f} deg")

    fig.canvas.mpl_connect('key_press_event', on_key)

    # Buffers and state
    window = []  # for --accum
    last_revo_points = []  # list of (theta_rad, r, inten, ts)
    current_revo_points = []
    last_angle_deg = None  # for wrap detection

    # Stats
    frame_count = 0
    fps_ema = 0.0
    last_t = time.time()

    try:
        while True:
            frame = sync_and_read_frame(ser, args.timeout)
            if frame is None:
                continue
            parsed = parse_frame(frame)
            if not parsed:
                continue

            points = parsed["points"]

            now = time.time()
            # Transform angles and enrich with timestamp and intensity
            transformed_full = [
                (math.radians(transform_angle_deg(p[0])), p[1], p[2], now)
                for p in points
            ]

            plot_points = []  # each: (theta_rad, r, inten, ts)

            if args.revo:
                for th_rad, r_m, inten, ts in transformed_full:
                    deg = (math.degrees(th_rad) % 360.0)
                    if last_angle_deg is not None and last_angle_deg > 270.0 and deg < 90.0:
                        # wrap detected: finalize revolution
                        last_revo_points = current_revo_points
                        current_revo_points = []
                    current_revo_points.append((th_rad, r_m, inten, ts))
                    last_angle_deg = deg
                plot_points = last_revo_points if last_revo_points else current_revo_points
            else:
                # Optional accumulation window
                transformed_simple = [(th, r, inten, now) for th, r, inten, _ in transformed_full]
                if args.accum > 0:
                    window.extend(transformed_simple)
                    max_points = args.accum * POINTS_PER_PACK
                    if len(window) > max_points:
                        del window[:len(window) - max_points]
                    plot_points = window
                else:
                    plot_points = transformed_simple

            # Colors
            cvals = []
            cmap_obj = plt.get_cmap(args.cmap)
            if args.color_mode == "intensity":
                norm = colors.Normalize(vmin=0, vmax=255)
                cvals = [p[2] for p in plot_points]
                base_rgba = cmap_obj(norm(cvals)) if cvals else []
            else:  # time
                if plot_points:
                    t_min = min(p[3] for p in plot_points)
                    t_max = max(p[3] for p in plot_points)
                    span = (t_max - t_min) if (t_max > t_min) else 1.0
                    norm = colors.Normalize(vmin=0.0, vmax=1.0)
                    cvals = [ (p[3]-t_min)/span for p in plot_points ]
                    base_rgba = cmap_obj(norm(cvals))
                else:
                    base_rgba = []

            # Apply fade by age (alpha)
            if args.fade_tau and args.fade_tau > 0 and plot_points:
                aged_rgba = []
                for (rgba, pt) in zip(base_rgba, plot_points):
                    age = max(0.0, now - pt[3])
                    alpha = math.exp(-age / args.fade_tau)
                    aged_rgba.append((rgba[0], rgba[1], rgba[2], alpha))
                colors_rgba = aged_rgba
            else:
                colors_rgba = base_rgba

            ax.clear()
            ax.set_rmax(args.rmax)
            # title with live stats and offset
            dt = max(1e-6, now - last_t)
            fps_inst = 1.0 / dt
            fps_ema = 0.9 * fps_ema + 0.1 * fps_inst if fps_ema > 0 else fps_inst
            frame_count += 1
            last_t = now

            ax.set_title(f"LD19 — offset={args.angle_offset:.1f}°, fps={fps_ema:.1f}")
            if plot_points:
                th = [p[0] for p in plot_points]
                rr = [p[1] for p in plot_points]
                if colors_rgba is not None and len(colors_rgba) > 0:
                    ax.scatter(th, rr, s=args.markersize, c=colors_rgba)
                else:
                    ax.scatter(th, rr, s=args.markersize)
            ax.text(0.02, 0.98,
                    f"pts={len(plot_points)}\nmode={'revo' if args.revo else ('accum' if args.accum>0 else 'frame')}\ncolor={args.color_mode}",
                    transform=ax.transAxes, va='top')
            plt.pause(0.001)
    except KeyboardInterrupt:
        print("\n[LD19] Stopped")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
