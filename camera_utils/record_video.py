#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""python3 record_video.py --width 640 --height 360"""
import cv2
import time
import argparse
from pathlib import Path

SUPPORTED_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

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
    m, s = divmod(t, 60)
    return f"{int(m):02d}:{int(s):02d}.{int((t-int(t))*1000):03d}"

def pick_fourcc(path_suffix):
    suf = path_suffix.lower()
    if suf.endswith(".mp4"):
        return cv2.VideoWriter_fourcc(*"mp4v")
    return cv2.VideoWriter_fourcc(*"MJPG")

def snapshot_params_for(ext: str):
    ext = ext.lower()
    if ext in (".jpg", ".jpeg"):
        return [cv2.IMWRITE_JPEG_QUALITY, 90]
    if ext == ".png":
        return [cv2.IMWRITE_PNG_COMPRESSION, 3]
    # для остальных — без параметров
    return []

def main():
    ap = argparse.ArgumentParser(description="Запись видео или постоянный снимок с таймером в углу")
    ap.add_argument("-o", "--output", default="capture.mp4",
                    help="Путь к выходному видео (.mp4 или .avi) (игнорируется в режиме --snapshot)")
    ap.add_argument("--fps", type=float, default=30.0, help="Целевая частота кадров")
    ap.add_argument("--width", type=int, default=0, help="Ширина кадра (0 — оставить как есть)")
    ap.add_argument("--height", type=int, default=0, help="Высота кадра (0 — оставить как есть)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Длительность записи в секундах (0 — без ограничения, Ctrl+C для остановки)")

    # режим простой перезаписи одного файла
    ap.add_argument("--snapshot", action="store_true",
                    help="Вместо видео сохранять последний кадр в один и тот же файл")
    ap.add_argument("--snapshot-path", default="last_camera_image.jpg",
                    help="Куда сохранять кадр (.jpg/.png/...)")
    args = ap.parse_args()

    cap, cam_index = find_video_device()
    if not cap:
        return

    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps < 1 or fps > 240 or args.fps != fps:
        fps = args.fps if args.fps > 0 else 30.0

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480

    out = None
    out_path = None
    if not args.snapshot:
        out_path = Path(args.output)
        fourcc = pick_fourcc(out_path.suffix)
        out = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))
        if not out.isOpened():
            print("Не удалось открыть файл для записи с выбранным кодеком. Пытаюсь AVI/MJPG…")
            out_path = out_path.with_suffix(".avi")
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            out = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))
            if not out.isOpened():
                print("Сбой открытия видеозаписи. Прерывание.")
                cap.release()
                return
        print(f"Запись ВИДЕО: {out_path} @ {fps:.1f} FPS, {w}x{h}, камера /dev/video{cam_index}")
    else:
        snap_path = Path(args.snapshot_path)
        if snap_path.suffix.lower() not in SUPPORTED_EXTS:
            raise ValueError(f"Расширение {snap_path.suffix} не поддерживается. Используй одно из: {sorted(SUPPORTED_EXTS)}")
        snap_path.parent.mkdir(parents=True, exist_ok=True)
        snap_params = snapshot_params_for(snap_path.suffix)
        print(f"Режим SNAPSHOT: {snap_path} @ {fps:.1f} FPS, {w}x{h}, камера /dev/video{cam_index}")

    # таймер-оверлей
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    thickness = 2
    color = (255, 255, 255)
    shadow = (0, 0, 0)
    org = (10, 30)

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

            cv2.putText(frame, label, (org[0]+1, org[1]+1), font, font_scale, shadow, thickness+2, cv2.LINE_AA)
            cv2.putText(frame, label, org, font, font_scale, color, thickness, cv2.LINE_AA)

            if args.snapshot:
                # обычная запись поверх того же файла
                ok = cv2.imwrite(str(snap_path), frame, snap_params)
                if not ok:
                    # На Windows это может случиться, если файл открыт зрителем с блокировкой.
                    # Просто сообщим и продолжим (следующий кадр попробует снова).
                    print("Предупреждение: не удалось сохранить кадр (файл может быть залочен вьювером).")
            else:
                out.write(frame)

            if args.duration > 0 and elapsed >= args.duration:
                print("Достигнута заданная длительность, стоп.")
                break

            next_deadline += frame_period
            sleep_for = next_deadline - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_deadline = time.monotonic()

    except KeyboardInterrupt:
        print("Остановка по Ctrl+C.")
    finally:
        if out is not None:
            out.release()
        cap.release()
        if args.snapshot:
            print(f"Готово. Последний кадр доступен в: {snap_path}")
        else:
            print(f"Готово. Файл сохранён: {out_path}")

if __name__ == "__main__":
    main()
