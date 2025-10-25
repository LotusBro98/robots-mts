"""Обертка, реализующая интерфейс, такой же, как на реальном роботе. Должна использовать """
import os
import socket
import struct

import numpy as np

# Не "хардкодьте" адреса
CMD_HOST  = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = str(os.getenv("TEL_HOST", "0.0.0.0"))
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
PROTO     = str(os.getenv("PROTO", "tcp"))


class EmulatedRobot:
    def __init__(self):
        self.sock_cmd = None
        self.sock_tel = None

    def connect(self):
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if PROTO == "udp":
            self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_tel.bind((TEL_HOST, TEL_PORT))
        else:
            self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_tel.bind((TEL_HOST, TEL_PORT))
            self.sock_tel.listen(1)
            print(f"[client] waiting for telemetry TCP on {TEL_HOST}:{TEL_PORT}...")
            conn, _ = self.sock_tel.accept()
            self.sock_tel = conn
            print("[client] connected to udp_diff telemetry")
        
    def disconnect(self):
        self.sock_cmd.close()
        self.sock_tel.close()

    def send_command(self, v: float, w: float):
        packet = struct.pack("<2f", v, w)
        self.sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))

    def recv_tel(self, ref_angle=0):  # TODO: доделать общий интерфейс для реального робота и виртуального
        if PROTO == "udp":
            data, _ = self.sock_tel.recvfrom(65535)
        else:
            size_bytes = self.sock_tel.recv(4)
            if not size_bytes:
                raise RuntimeError("error receiving telemetry")
                # return None
            size = struct.unpack("<I", size_bytes)[0]
            data = self._recv_all(self.sock_tel, size)

        if not data.startswith(b"WBTG"):  # ВНИМАНИЕ! Было: WBT2
            print('WBT2 -> WBTG')
            raise RuntimeError("error receiving telemetry. WBT2 -> WBTG")
            # return None

        # теперь в пакете: 9 float после "WBTG" (36 байт)
        # "<6f" -> "<9f"
        header_size = 4 + 9 * 4
        odom_x, odom_y, odom_th, vx, vy, vth, wx, wy, wz = struct.unpack("<9f", data[4:header_size])
        n = struct.unpack("<I", data[header_size:header_size + 4])[0]
        ranges = []
        if n > 0:
            ranges = struct.unpack(f"<{n}f", data[header_size + 4:header_size + 4 + 4 * n])

        # return odom_x, odom_y, odom_th, (vx, vy, vth), (wx, wy, wz), ranges
        return (
            np.array([odom_x, odom_y]),
            odom_th - ref_angle,
            np.array([vx, vy]),
            vth,
            np.array([wx, wy, wz]),
            np.array(ranges),
        )

    def _recv_all(self, sock, size):
        buf = b""
        while len(buf) < size:
            chunk = sock.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf
