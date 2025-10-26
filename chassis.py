import json
import time
import traceback
import serial
import threading
import numpy as np

from robot_lidar import read_full_scan_from_serial


LIDAR_PORT="/dev/ttyUSB0"
LIDAR_BAUT = 230400
LIDAR_SERIAL_TIMEOUT = 0.2


class RobotChassis:
    def __init__(self, port: str = "/dev/ttyACM1"):
        self.port = port
        self.sensors_thread = None

    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
        self.lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUT, timeout=LIDAR_SERIAL_TIMEOUT)

    def disconnect(self):
        self.stop_sensors_capture()
        self.ser.close()

    def send_drive(self, v: float, w: float):
        """
        v: float — линейная скорость вперёд (0..1)
        w: float — угловая скорость (поворот) влево/вправо (-1..1)
                >0 — влево, <0 — вправо

        Моторы управляются в диапазоне -18000..18000 (единицы 0.1 rpm).
        """
        MAX_RPM = 1800
        SCALE = MAX_RPM  # перевод в 0.1rpm

        v = max(-1.0, min(1.0, v))
        w = max(-1.0, min(1.0, w))

        # (v, w) → (left, right)
        # поворот влево: левый мотор медленнее, правый быстрее
        left_speed  = (v - w) * SCALE
        right_speed = (v + w) * SCALE

        left_speed  = int(max(-MAX_RPM * 10, min(MAX_RPM * 10, left_speed)))
        right_speed = int(max(-MAX_RPM * 10, min(MAX_RPM * 10, right_speed)))
        self.send_command(T=1, L=left_speed, R=right_speed)

    def send_command(self, **kwargs):
        cmd_json = json.dumps(kwargs, separators=(',', ':'))
        print("Cmd to chassis:", cmd_json)
        self.ser.write((cmd_json + "\r\n").encode())

    def start_sensor_capture(self):
        self.do_capture_sensors = True
        self.last_msg = None
        self.pos = np.array([0.0, 0.0], dtype=np.float32)
        self.angle = 0
        self.send_command(T=131, cmd=1)
        self.sensors_thread = threading.Thread(target=self._capture_wheel_sensors, daemon=True)
        self.sensors_thread.start()
        self.lidar_thread = threading.Thread(target=self._capture_lidar, daemon=True)
        self.lidar_thread.start()

    def stop_sensors_capture(self):
        self.do_capture_sensors = False
        if self.sensors_thread is not None:
            self.sensors_thread.join()
            self.sensors_thread = None
            self.send_command(T=131, cmd=0)

    def recv_tel(self, ref_angle=0):
        odom_x = 0
        odom_y = 0
        odom_th = 0
        vx = 0
        vy = 0
        vth = 0
        return (
            np.array([odom_x, odom_y]),
            odom_th - ref_angle,
            np.array([vx, vy]),
            vth,
            np.array([0, 0, 0]),
            self.lidar_distances_by_direction,
        )

    def _capture_lidar(self):
        print("Started wheel capture")
        while self.do_capture_sensors:
            try:
                distances_by_angle: dict[float, float] = read_full_scan_from_serial(  # массив расстояний
                    self.lidar_ser,
                    LIDAR_SERIAL_TIMEOUT,
                    angle_offset=0.0,
                    clockwise=False,
                    max_revo_seconds=2.0,
                    sort_by_angle=True,
                )
                self.lidar_distances_by_angle = distances_by_angle
                self.lidar_distances_by_direction = {-45: distances_by_angle[314], 0: distances_by_angle[0], 45: distances_by_angle[45]}
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

    def calc_odometry(self, msg, last_msg):  # TODO: отдавать отсюда odom_x, odom_y, odom_th, vx, vy, vth как из EmulatedRobot.recv_tel
        delta_left = msg["odl"] - last_msg["odl"]
        delta_right = msg["odr"] - last_msg["odr"]

        linear_delta = 0.5 * (delta_left + delta_right) * 0.01 # original unit is cm
        angular_delta = (delta_right - delta_left) / self.WHEEL_DISTANCE

        # self.pos += np.array([
        #     np.cos(self.angle)
        # ])
        # Учесть, что робот в этом отрезке едет по дуге. 
        # Текущая формула последовательно едет прямо потом по углу. 
        # Усреднить от "проехал прямо затем повернул" и "повернул затем проехал прямо"


    def sensors_callback(self, msg):  # TODO: взять это за основу?
        print("Chassis sensors: ", msg) # {"T":1001,"M1":0,"M2":0,"M3":0,"M4":0,"odl":3247,"odr":8920,"v":963}
        if self.last_msg is None:
            self.last_msg = msg
            return
        
        self.calc_odometry(msg, self.last_msg)
        self.last_msg = msg

def main():
    robot = RobotChassis("/dev/ttyACM1")
    robot.connect()
    robot.start_sensor_capture()
    time.sleep(1000)
    robot.stop_sensors_capture()
    robot.disconnect()

if __name__ == "__main__":
    main()
