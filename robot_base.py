from dataclasses import dataclass
import threading
import time
from typing import Dict

import numpy as np

from navigator import Navigator
from navigator_utils import round_angle


class Latest:
    """Потокобезопасное хранилище последнего значения с меткой времени и замером FPS."""
    __slots__ = ("_data", "_ts", "_lock", "_count", "_start_time", "name")
    PRINT_FPS_EVERY_N_TIMES = 200
    SHOW_FPS = False

    def __init__(self, name: str):
        self._data = None
        self._ts = None
        self._lock = threading.Lock()
        self._count = 0
        self._start_time = time.monotonic()
        self.name = name

    def set(self, value):
        """Сохраняет новое значение и метку времени, периодически выводит FPS."""
        with self._lock:
            self._data = value
            self._ts = time.monotonic()
            self._count += 1

            if self.SHOW_FPS and self._count % self.PRINT_FPS_EVERY_N_TIMES == 0:
                now = time.monotonic()
                dt = now - self._start_time
                if dt > 0:
                    fps = self.PRINT_FPS_EVERY_N_TIMES / dt
                    print(f"\n[{self.name}] FPS: {fps:.1f}")
                self._start_time = now  # сбрасываем отсчёт

    def get(self):
        """Возвращает (value, timestamp) — последнюю запись и время её обновления."""
        with self._lock:
            return self._data, self._ts

    def age(self):
        """Сколько секунд прошло с последнего обновления."""
        with self._lock:
            if self._ts is None:
                return float("inf")
            return time.monotonic() - self._ts


@dataclass
class SensorData:
    pos: np.ndarray  # массив из 2 чисел - координаты
    vel: np.ndarray
    angle: np.ndarray
    angle_vel: np.ndarray
    lidar_ranges: Dict[float, float]

    def __post_init__(self):
        right_end_pt = -80
        right_pt = -30
        left_pt = 30
        left_end_pt = 80
        # self.cur_front_wall_dist = min((rng for angle, rng in self.lidar_ranges.items() if round_angle(angle, radians=False) < left_pt and round_angle(angle, radians=False) > right_pt), default=0)
        # self.cur_left_wall_dist = min((rng for angle, rng in self.lidar_ranges.items() if round_angle(angle, radians=False) < left_end_pt and round_angle(angle, radians=False) > left_pt), default=0) * np.sin(np.deg2rad(45))
        # self.cur_right_wall_dist = min((rng for angle, rng in self.lidar_ranges.items() if round_angle(angle, radians=False) > right_end_pt and round_angle(angle, radians=False) < right_pt), default=0) * np.sin(np.deg2rad(45))

        ranges_xy = {round_angle(np.deg2rad(angle)): rng for angle, rng in self.lidar_ranges.items()}
        ranges_xy = {angle: (rng * np.cos(angle), rng * np.sin(angle), rng) for angle, rng in ranges_xy.items()}

        ROBOT_WIDTH = 0.2
        OFFS = -0.0
        self.cur_front_wall_dist = min((x for angle, (x, y, rng) in ranges_xy.items() if abs(y) < ROBOT_WIDTH / 2 and x > ROBOT_WIDTH / 4), default=0)
        # right and left wall estimated in stripe, not in cone
        self.cur_right_wall_dist = min([rng for angle, (x, y, rng) in ranges_xy.items() if y < 0 and x + y > -ROBOT_WIDTH / 2 + OFFS and x + y < ROBOT_WIDTH / 2 + OFFS], default=1000) * np.sin(np.deg2rad(45))
        self.cur_left_wall_dist = min([rng for angle, (x, y, rng) in ranges_xy.items() if y > 0 and x - y > -ROBOT_WIDTH / 2 + OFFS and x - y < ROBOT_WIDTH / 2 + OFFS], default=1000) * np.sin(np.deg2rad(45))


class Robot:
    MAX_ACCELERATION: float
    MAX_SPEED_MpS: float
    MAX_ROT_SPEED_RpS: float
    MAX_ANG_ACCELERATION: float

    prev_angle_vel = 0

    def __init__(self, demo_render_navigator: bool = False, file_rendering_navigator: bool = True) -> None:
        if demo_render_navigator:
            self.navigator = Navigator(show_demo=True, render_mode="window", render_fps=15.0)
        elif file_rendering_navigator:
            self.navigator = Navigator(show_demo=True, render_mode="file", render_fps=2.0, render_out_dir="navigator_images")
        else:
            self.navigator = Navigator(show_demo=False)
        self._latest_odometry = Latest("odometry")
        self._latest_lidar = Latest("lidar")
        self._latest_gyro = Latest("gyro")
        self._latest_nav = Latest("nav")
        self.initialized = False

    def _update_odometry(self, odom_pos, odom_th, odom_vel, odom_th_vel):
        self._latest_odometry.set((odom_pos, odom_th, odom_vel, odom_th_vel))
        self._latest_nav.set(self.navigator.update_from_odometry(odom_pos, odom_th))

    def _update_gyro(self, gyro):
        self._latest_gyro.set(gyro)
        self._latest_nav.set(self.navigator.update_from_gyro(gyro))

    def _update_lidar(self, lidar_ranges):
        self._latest_lidar.set(lidar_ranges)
        self._latest_nav.set(self.navigator.update_from_lidar(lidar_ranges))
        self.initialized = len(self.navigator.points) > 0

    def recv_sensors(self) -> SensorData:
        self.wait_until_initialized()
        (nav_pos, nav_angle), ts = self._latest_nav.get()
        (odom_pos, odom_th, odom_vel, odom_th_vel), ts = self._latest_odometry.get()
        (lidar_ranges), ts = self._latest_lidar.get()

        alpha = 0.1
        self.prev_angle_vel = odom_th_vel * alpha + self.prev_angle_vel * (1 - alpha)
        data = SensorData(
            pos=nav_pos,
            angle=nav_angle,
            vel=odom_vel,
            angle_vel=self.prev_angle_vel,
            lidar_ranges=lidar_ranges
        )
        return data

    def wait_until_initialized(self):
        while not self.initialized:
            time.sleep(0.01)

    def send_drive(self, v: float, w: float): ...

    def connect(self): ...

    def disconnect(self): ...
