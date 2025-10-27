#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""python3 record_video.py --width 640 --height 360"""
import cv2
import time
import argparse
from pathlib import Path

def find_video_device(max_index=5):
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Камера найдена: /dev/video{i}")
            return cap, i
        cap.release()
    print("Камера не найдена.")
    return None, None

def format_elapsed(t):
    # t (float сек) -> "MM:SS.mmm"
    m, s = divmod(t, 60)
    return f"{int(m):02d}:{int(s):02d}.{int((t-int(t))*1000):03d}"

def pick_fourcc(path_suffix):
    suf = path_suffix.lower()
    # mp4 — удобно, но не на всех сборках доступен mp4v
    if suf.endswith(".mp4"):
        return cv2.VideoWriter_fourcc(*"mp4v")
    # avi с MJPG — максимально совместим
    return cv2.VideoWriter_fourcc(*"MJPG")

def main():
    ap = argparse.ArgumentParser(description="Запись видео с таймером в углу")
    ap.add_argument("-o", "--output", default="capture.mp4",
                    help="Путь к выходному файлу (.mp4 или .avi)")
    ap.add_argument("--fps", type=float, default=10.0, help="Целевая частота кадров")
    ap.add_argument("--width", type=int, default=0, help="Ширина кадра (0 — оставить как есть)")
    ap.add_argument("--height", type=int, default=0, help="Высота кадра (0 — оставить как есть)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Длительность записи в секундах (0 — без ограничения, Ctrl+C для остановки)")
    args = ap.parse_args()

    # Открываем камеру
    cap, cam_index = find_video_device()
    if not cap:
        return

    # Устанавливаем желаемое разрешение (если задано)
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # Получаем фактические параметры
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps < 1 or fps > 240:
        fps = args.fps if args.fps > 0 else 30.0

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480

    # Готовим VideoWriter
    out_path = Path(args.output)
    fourcc = pick_fourcc(out_path.suffix)
    out = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))
    if not out.isOpened():
        # Пытаемся fallback на AVI + MJPG
        print("Не удалось открыть файл для записи с выбранным кодеком. Пытаюсь AVI/MJPG…")
        out_path = out_path.with_suffix(".avi")
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        out = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))
        if not out.isOpened():
            print("Сбой открытия видеозаписи. Прерывание.")
            cap.release()
            return

    print(f"Запись: {out_path} @ {fps:.1f} FPS, {w}x{h}, камера /dev/video{cam_index}")

    # Настройки таймера-оверлея
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    thickness = 2
    color = (255, 255, 255)
    shadow = (0, 0, 0)
    org = (10, 30)  # положение текста

    start = time.monotonic()
    frame_period = 1.0 / fps
    next_deadline = time.monotonic()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Кадр не получен, останавливаюсь.")
                break

            elapsed = time.monotonic() - start
            label = format_elapsed(elapsed)

            # Обводка/тень
            cv2.putText(frame, label, (org[0]+1, org[1]+1), font, font_scale, shadow, thickness+2, cv2.LINE_AA)
            # Текст
            cv2.putText(frame, label, org, font, font_scale, color, thickness, cv2.LINE_AA)

            out.write(frame)

            if args.duration > 0 and elapsed >= args.duration:
                print("Достигнута заданная длительность, стоп.")
                break

            # Пейсинг по FPS (мягкий; VideoWriter сам не «тормозит»)
            next_deadline += frame_period
            sleep_for = next_deadline - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # если отстаём, не спим и подхватываем следующее окно
                next_deadline = time.monotonic()

    except KeyboardInterrupt:
        print("Остановка по Ctrl+C.")
    finally:
        out.release()
        cap.release()
        print(f"Готово. Файл сохранён: {out_path}")

if __name__ == "__main__":
    main()
