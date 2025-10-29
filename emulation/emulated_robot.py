"""Обертка, реализующая интерфейс, такой же, как на реальном роботе. Должна использовать """
import os
import socket
import struct
import threading
import traceback

import numpy as np

from robot_base import Robot

# Не "хардкодьте" адреса
CMD_HOST  = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = str(os.getenv("TEL_HOST", "0.0.0.0"))
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
PROTO     = str(os.getenv("PROTO", "tcp"))


class EmulatedRobot(Robot):
    GYRO_CORR_COEFF = 0.975
    MAX_ACCELERATION = 0.042
    MAX_SPEED_MpS = 0.42
    MAX_ROT_SPEED_RpS = 0.372
    MAX_ANG_ACCELERATION = 0.14883143

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
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

        self._start_capture()
        self.wait_until_initialized()

    def disconnect(self):
        self._stop_capture()
        self.sock_cmd.close()
        self.sock_tel.close()

    def _start_capture(self):
        self.do_capture_sensors = True
        self.sensors_thread = threading.Thread(target=self._capture_worker, daemon=True)
        self.sensors_thread.start()

    def _stop_capture(self):
        self.do_capture_sensors = False
        if self.sensors_thread is not None:
            self.sensors_thread.join()
            self.sensors_thread = None

    def _capture_worker(self):
        while self.do_capture_sensors:
            try:
                pos, th, vel, th_vel, gyro, ranges = self._recv_tel()
            except:
                traceback.print_exc()
                break
            gyro *= self.GYRO_CORR_COEFF

            self._update_odometry(pos, th, vel, th_vel)
            self._update_gyro(gyro)
            self._update_lidar(ranges)

    def send_drive(self, v: float, w: float):
        packet = struct.pack("<2f", v, w)
        self.sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))

    def _recv_tel(self, ref_angle=0):  # TODO: доделать общий интерфейс для реального робота и виртуального
        # TODO: вытаскивать отсюда единообразно odom_x, odom_y, odom_th, vx, vy, vth
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
            #         cur_front_wall_dist = ranges[len(ranges) // 2]
            # cur_left_wall_dist = ranges[-1] * np.sin(np.deg2rad(45))
            # cur_right_wall_dist = ranges[0] * np.sin(np.deg2rad(45))
        # Для проверки: ranges_by_angle = {"-45": ranges[0], "0": ranges[len(ranges) // 2], "45": ranges[-1]}
        ranges_by_angle = {round(angle, 3): rng for angle, rng in zip(np.arange(-45, 45, 0.25), ranges)}
        ranges_by_angle[45] = ranges_by_angle[44.75]
        # return odom_x, odom_y, odom_th, (vx, vy, vth), (wx, wy, wz), ranges
        return (
            np.array([odom_x, odom_y]),
            odom_th - ref_angle,
            np.array([vx, vy]),
            vth,
            np.array([wx, wy, wz]),
            ranges_by_angle,
        )

    def _recv_all(self, sock, size):
        buf = b""
        while len(buf) < size:
            chunk = sock.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf
