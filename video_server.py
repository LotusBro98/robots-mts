#!/usr/bin/env python3
import socket
import struct
import cv2 as cv
import time

ROBOT_VIDEO_IP = "0.0.0.0"
ROBOT_VIDEO_PORT = 5000

JPEG_QUALITY = 70  # 0-100, выше = лучше/толще

def video_server():
    # Открываем камеру (подставь нужный индекс или URL)
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("[VID] Не удалось открыть камеру")
        return

    # Настраиваем TCP-сокет
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((ROBOT_VIDEO_IP, ROBOT_VIDEO_PORT))
    srv.listen(1)
    print(f"[VID] Listening on {ROBOT_VIDEO_IP}:{ROBOT_VIDEO_PORT}")

    try:
        while True:
            print("[VID] Ожидаю подключение клиента...")
            conn, addr = srv.accept()
            print(f"[VID] Клиент подключился: {addr}")

            try:
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        print("[VID] Не удалось прочитать кадр, жду...")
                        time.sleep(0.1)
                        continue

                    # Сжимаем в JPEG
                    encode_param = [int(cv.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                    ok, encoded = cv.imencode(".jpg", frame, encode_param)
                    if not ok:
                        print("[VID] Ошибка JPEG-кодирования")
                        continue

                    data = encoded.tobytes()
                    length = len(data)

                    # Отправляем длину (4 байта, big-endian) + сам кадр
                    header = struct.pack("!I", length)
                    conn.sendall(header + data)

                    # Можно ограничить FPS (например, 20)
                    time.sleep(0.05)

            except (ConnectionResetError, BrokenPipeError):
                print("[VID] Клиент отключился")
                conn.close()
            except Exception as e:
                print(f"[VID] Ошибка в стриме: {e}")
                conn.close()

    except KeyboardInterrupt:
        print("\n[VID] KeyboardInterrupt, выхожу...")
    finally:
        cap.release()
        srv.close()
        print("[VID] Сервер видео остановлен")


if __name__ == "__main__":
    video_server()
