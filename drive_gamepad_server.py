#!/usr/bin/env python3
import socket
import struct
import time

from chassis import RobotChassis


# ==== НАСТРОЙКИ СЕРВЕРА ====
UDP_PORT = 9999          # можно поменять при желании
IDLE_TIMEOUT = 0.5       # сек, через сколько секунд без команд остановить робота

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.1)

    print(f"[SRV] Listening on 0.0.0.0:{UDP_PORT}")
    last_cmd_time = time.time()

    # Формат пакета: 2 float32 (linear, angular)
    packet_format = "ff"
    packet_size = struct.calcsize(packet_format)

    robot = RobotChassis()
    robot.connect()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) < packet_size:
                    print(f"[SRV] Too short packet from {addr}")
                    continue

                linear, angular = struct.unpack(packet_format, data[:packet_size])
                # Вызываем управление
                robot.send_drive(linear, angular)
                last_cmd_time = time.time()

            except socket.timeout:
                # Если давно не было команд — стопим робота
                if time.time() - last_cmd_time > IDLE_TIMEOUT:
                    robot.send_drive(0.0, 0.0)
                    last_cmd_time = time.time()
            except Exception as e:
                print(f"[SRV] Error: {e}")

    except KeyboardInterrupt:
        print("\n[SRV] KeyboardInterrupt, stopping robot.")
        robot.send_drive(0.0, 0.0)

    finally:
        sock.close()
        print("[SRV] Server stopped")

if __name__ == "__main__":
    main()