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


def connect_robot():
    global sock_cmd
    global sock_tel

    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if PROTO == "udp":
        sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_tel.bind((TEL_HOST, TEL_PORT))
    else:
        sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_tel.bind((TEL_HOST, TEL_PORT))
        sock_tel.listen(1)
        print(f"[client] waiting for telemetry TCP on {TEL_HOST}:{TEL_PORT}...")
        conn, _ = sock_tel.accept()
        sock_tel = conn
        print("[client] connected to udp_diff telemetry")


def disconnect_robot():
    sock_cmd.close()
    sock_tel.close()


def send_cmd(v: float, w: float):
    packet = struct.pack("<2f", v, w)
    sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))


def recv_all(sock, size):
    buf = b""
    while len(buf) < size:
        chunk = sock.recv(size - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def recv_tel(ref_angle=0):
    if PROTO == "udp":
        data, _ = sock_tel.recvfrom(65535)
    else:
        size_bytes = sock_tel.recv(4)
        if not size_bytes:
            raise RuntimeError("error receiving telemetry")
            # return None
        size = struct.unpack("<I", size_bytes)[0]
        data = recv_all(sock_tel, size)

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
    
