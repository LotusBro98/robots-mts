from dataclasses import dataclass
import threading
import time
from typing import Dict

import numpy as np

from navigator import Navigator


class Latest:
    """Потокобезопасное хранилище последнего значения с меткой времени."""
    __slots__ = ("_data", "_ts", "_lock")

    def __init__(self):
        self._data = None
        self._ts = None
        self._lock = threading.Lock()

    def set(self, value):
        """Сохраняет новое значение и метку времени."""
        with self._lock:
            self._data = value
            self._ts = time.monotonic()

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
    pos: np.ndarray
    vel: np.ndarray
    angle: np.ndarray
    angle_vel: np.ndarray
    lidar_ranges: Dict[float, float]


class Robot:
    def __init__(self, demo_render_navigator: bool = True, file_rendering_navigator: bool = False) -> None:
        if demo_render_navigator:
            self.navigator = Navigator(show_demo=True, render_mode="window", render_fps=15.0)
        elif file_rendering_navigator:
            self.navigator = Navigator(show_demo=True, render_mode="file", render_fps=1.0, render_out_dir="frames")
        else:
            self.navigator = Navigator(show_demo=False)
        self._latest_odometry = Latest()
        self._latest_lidar = Latest()
        self._latest_nav = Latest()
        self.initialized = False

    def _update_odometry(self, odom_pos, odom_th, odom_vel, odom_th_vel):
        self._latest_odometry.set((odom_pos, odom_th, odom_vel, odom_th_vel))
        nav_pos, nav_angle = self.navigator.update_from_odometry(odom_pos, odom_th)
        self._latest_nav.set((nav_pos, nav_angle))

    def _update_lidar(self, lidar_ranges):
        self._latest_lidar.set(lidar_ranges)
        nav_pos, nav_angle = self.navigator.update_from_lidar(lidar_ranges)
        self._latest_nav.set((nav_pos, nav_angle))
        self.initialized = True

    def recv_sensors(self) -> SensorData:
        (nav_pos, nav_angle), ts = self._latest_nav.get()
        (odom_pos, odom_th, odom_vel, odom_th_vel), ts = self._latest_odometry.get()
        (lidar_ranges), ts = self._latest_lidar.get()
        data = SensorData(
            pos=nav_pos,
            angle=nav_angle,
            vel=odom_vel,
            angle_vel=odom_th_vel,
            lidar_ranges=lidar_ranges
        )
        return data

    def wait_until_initialized(self):
        while not self.initialized:
            time.sleep(0.01)

    def send_drive(self, v: float, w: float): ...

    def connect(self): ...

    def disconnect(self): ...
