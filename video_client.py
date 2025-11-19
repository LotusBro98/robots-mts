#!/usr/bin/env python3
import socket
import struct
import time

import cv2 as cv
import numpy as np

ROBOT_IP = "192.168.1.11"   # <-- IP робота
ROBOT_VIDEO_PORT = 5000

def recvall(sock, n):
    """Прочитать ровно n байт или вернуть None при обрыве."""
    data = b""
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

def main():
    while True:
        try:
            print(f"[VID] Подключаюсь к {ROBOT_IP}:{ROBOT_VIDEO_PORT}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ROBOT_IP, ROBOT_VIDEO_PORT))
            print("[VID] Подключение установлено.")

            while True:
                header = recvall(sock, 4)
                if header is None:
                    print("[VID] Сервер закрыл соединение.")
                    break

                (length,) = struct.unpack("!I", header)
                jpg_data = recvall(sock, length)
                if jpg_data is None:
                    print("[VID] Не удалось получить данные кадра.")
                    break

                np_data = np.frombuffer(jpg_data, dtype=np.uint8)
                frame = cv.imdecode(np_data, cv.IMREAD_COLOR)
                if frame is None:
                    print("[VID] Ошибка декодирования кадра.")
                    continue

                cv.imshow("Robot Camera", frame)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    print("[VID] Выход по 'q'")
                    sock.close()
                    cv.destroyAllWindows()
                    return

            sock.close()
            cv.destroyAllWindows()
            print("[VID] Повторное подключение через 2 секунды...")
            time.sleep(2)

        except ConnectionRefusedError:
            print("[VID] Сервер недоступен, жду 2 секунды...")
            time.sleep(2)
        except KeyboardInterrupt:
            print("\n[VID] KeyboardInterrupt, выхожу.")
            break
        except Exception as e:
            print(f"[VID] Ошибка: {e}")
            time.sleep(1)

    cv.destroyAllWindows()

if __name__ == "__main__":
    main()