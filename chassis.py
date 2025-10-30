import json
import time
import traceback
from typing import Dict
import serial
import threading
import numpy as np

from navigator_utils import normalize, round_angle
from robot_base import Robot
from robot_lidar_alt import read_full_scan_from_serial


LIDAR_PORT="/dev/ttyUSB0"
LIDAR_BAUT = 230400
LIDAR_SERIAL_TIMEOUT = 0.2


class RobotChassis(Robot):
    MAX_ACCELERATION = 1.0
    MAX_SPEED_MpS = 0.53
    MAX_ROT_SPEED_RpS = 0.53
    MAX_ANG_ACCELERATION = 0.5

    WHEEL_DISTANCE = 15.15  # In cm
    LIDAR_BLIND_ZONES = [(180, 15)]

    def __init__(self, port: str = "/dev/ttyACM1", **kwargs):
        super().__init__(**kwargs)
        self.port = port
        self.sensors_thread = None

    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
        self.lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUT, timeout=LIDAR_SERIAL_TIMEOUT)
        self._start_capture()

    def disconnect(self):
        self._stop_capture()
        self.ser.close()
        self.lidar_ser.close()

    def send_drive(self, v: float, w: float):
        """
        v: float — линейная скорость вперёд (0..1)
        w: float — угловая скорость (поворот) влево/вправо (-1..1)
                >0 — влево, <0 — вправо

        Моторы управляются в диапазоне -1800..1800.
        """
        MAX_SPEED = 1800  # Единицы измерения - 0.1rpm

        v = max(-1.0, min(1.0, v))
        w = max(-1.0, min(1.0, w))

        # (v, w) → (left, right)
        # поворот влево: левый мотор медленнее, правый быстрее
        left_speed  = (v - w) * MAX_SPEED
        right_speed = (v + w) * MAX_SPEED

        left_speed  = int(max(-MAX_SPEED, min(MAX_SPEED, left_speed)))
        right_speed = int(max(-MAX_SPEED, min(MAX_SPEED, right_speed)))
        self.send_command(T=1, L=left_speed, R=right_speed)

    def send_command(self, **kwargs):
        cmd_json = json.dumps(kwargs, separators=(',', ':'))
        # print("Cmd to chassis:", cmd_json)
        self.ser.write((cmd_json + "\r\n").encode())

    def _start_capture(self):
        self.do_capture_sensors = True
        self.last_msg = None
        self.pos = np.array([0.0, 0.0], dtype=np.float32)
        self.angle = 0
        self.vel = np.array([0.0, 0.0], dtype=np.float32)
        self.vth = 0
        self.send_command(T=131, cmd=1)
        self.sensors_thread = threading.Thread(target=self._capture_wheel_sensors, daemon=True)
        self.sensors_thread.start()
        self.lidar_thread = threading.Thread(target=self._capture_lidar, daemon=True)
        self.lidar_thread.start()

    def _stop_capture(self):
        self.do_capture_sensors = False

        if self.sensors_thread is not None:
            self.sensors_thread.join()
            self.sensors_thread = None
            self.send_command(T=131, cmd=0)
        
        if self.lidar_thread is not None:
            self.lidar_thread.join()
            self.lidar_thread = None

    def remove_lidar_blind_zones(self, distances_by_angle: Dict[float, float]):
        angles = np.array(distances_by_angle.keys())
        ranges = np.array(distances_by_angle.values())

        blind_mask = np.zeros_like(angles, dtype=np.bool_)
        for a, da in self.LIDAR_BLIND_ZONES:
            blind_mask |= round_angle(angles - a, radians=False) < da
        filtered = dict(zip(angles[~blind_mask], ranges[~blind_mask]))
        return filtered

    def _capture_lidar(self):
        print("Started lidar capture")
        while self.do_capture_sensors:
            try:
                distances_by_angle: dict[float, float] = read_full_scan_from_serial(  # массив расстояний
                    self.lidar_ser,
                    # LIDAR_SERIAL_TIMEOUT,
                    angle_offset=0.0,
                    clockwise=False,
                    max_revo_seconds=2.0,
                    # sort_by_angle=True,
                )
                # self.lidar_distances_by_angle = distances_by_angle
                # self.lidar_distances_by_direction = {-45: distances_by_angle[314], 0: distances_by_angle[0], 45: distances_by_angle[45]}
                distances_by_angle = self.remove_lidar_blind_zones(distances_by_angle)
                self._update_lidar(distances_by_angle)
                # print(distances_by_angle)
            except:
                traceback.print_exc()
                break
            # try:
            #     self.sensors_callback(msg)
            # except:
            #     traceback.print_exc()
            #     continue

    def _capture_wheel_sensors(self):
        print("Started wheel capture")
        while self.do_capture_sensors:
            try:
                msg = self.ser.read_until(b"\r\n").strip(b"\r\n")
            except:
                traceback.print_exc()
                break

            try:
                msg = json.loads(msg.decode())
            except Exception as e:
                print("Error parsing message from chassis:", e, msg)
                continue

            try:
                self.sensors_callback(msg)
            except:
                traceback.print_exc()
                continue

    prev_odom_time = time.monotonic()
    def calc_odometry(self, msg, last_msg):  # TODO: отдавать отсюда odom_x, odom_y, odom_th, vx, vy, vth как из EmulatedRobot.recv_tel
        delta_left = msg["odl"] - last_msg["odl"]
        delta_right = msg["odr"] - last_msg["odr"]

        linear_delta = 0.5 * (delta_left + delta_right) * 0.01 # original unit is cm
        angular_delta = (delta_right - delta_left) / self.WHEEL_DISTANCE

        dir_before = np.array([np.cos(self.angle), np.sin(self.angle)])
        self.angle += angular_delta
        dir_after = np.array([np.cos(self.angle), np.sin(self.angle)])

        delta_pos = linear_delta * normalize(dir_before + dir_after)
        self.pos += delta_pos

        # Учесть, что робот в этом отрезке едет по дуге. 
        # Текущая формула последовательно едет прямо потом по углу. 
        # Усреднить от "проехал прямо затем повернул" и "повернул затем проехал прямо"

        time_now = time.monotonic()
        dt = time_now - self.prev_odom_time
        self.prev_odom_time = time_now
        
        self.vth = angular_delta / dt
        self.vel = np.array([linear_delta, 0], dtype=np.float32)
        
        # print("ODOM: ", delta_left, delta_right, linear_delta, delta_pos, self.pos)
        self._update_odometry(self.pos.copy(), self.angle, self.vel.copy(), self.vth)


    def sensors_callback(self, msg):  # TODO: взять это за основу?
        # print("Chassis sensors: ", msg) # {"T":1001,"M1":0,"M2":0,"M3":0,"M4":0,"odl":3247,"odr":8920,"v":963}
        if self.last_msg is None:
            self.last_msg = msg
            return
        
        self.calc_odometry(msg, self.last_msg)
        self.last_msg = msg


def main():
    robot = RobotChassis("/dev/ttyACM1")
    robot.connect()
    try:
        time.sleep(1000)
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()
