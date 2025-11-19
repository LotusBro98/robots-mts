import os
import threading
import time
from typing import Dict, List, Tuple

import matplotlib
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch
import matplotlib.patches as mpatches
import numpy as np

from cartographer import Cartographer
from navigator_utils import fit_line_polar_ransac, transform_points
from pathfinder import Pathfinder


class Navigator:
    def __init__(self, show_demo: bool = True,
                 render_mode: str = "window",  # "window" | "file" | "off"
                 render_fps: float = 10.0,
                 render_out_dir: str | None = "navigator_images"):
        self.show_demo = show_demo
        self.render_mode = render_mode if show_demo else "off"
        self.render_fps = max(0.1, float(render_fps))
        self.render_out_dir = render_out_dir

        self.pos = np.zeros((2,), dtype=np.float32)
        self.angle = 0
        self.points = np.zeros((0, 2), dtype=np.float32)
        self.relative_points = np.zeros((0, 2), dtype=np.float32)

        # Для безопасного вызова display() даже без демо
        self.cur_lidar_pts = np.zeros((0, 2), dtype=np.float32)
        self.cur_matched_pts = np.zeros((0, 2), dtype=np.float32)
        self.last_display_time = 0

        self.prev_odom_pos = np.zeros((2,), dtype=np.float32)
        self.prev_odom_angle = 0
        self.odom_pos_offset = np.zeros((2,), dtype=np.float32)
        self.odom_angle_offset = 0

        self.lock = threading.Lock()

        # Рендер в файл — без оконного backend
        if self.render_mode == "file":
            matplotlib.use("Agg")  # безопасно до создания Figure

        # Ленивая инициализация фигуры: создадим при первом кадре
        self.fig = None
        self.ax = None
        self.scat1 = self.scat2 = self.scat3 = self.scat4 = None
        self.bg = None
        self.arrow = None
        self.path = np.zeros((0, 2))
        self.goal = None

        # Поток рендера
        self._render_stop = threading.Event()
        self._render_thread = None
        if self.render_mode in ("window", "file"):
            self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
            self._render_thread.start()

        self.pathfinder = Pathfinder(
            input_cb=lambda: (self.points, self.pos, self.goal), 
            output_cb=lambda path: setattr(self, "path", path),
        )

        self.cartographer = Cartographer(
            input_cb=lambda: (self.points, self.relative_points, self.pos, self.angle),
            output_cb=self._cartographer_update_cb,
        )

    def _cartographer_update_cb(self, world_points, cur_matched_pts, points, dpos, dth):
        self.points = world_points
        self.cur_matched_pts = cur_matched_pts
        self.cur_lidar_pts = points
        with self.lock:
            self.pos = self.pos + dpos
            self.angle = self.angle + dth

    def stop(self):
        self.pathfinder.stop()
        self.cartographer.stop()
        self._render_stop.set()
        self._render_thread.join()

    def _init_plot(self):
        if self.fig is not None:  # уже создано
            return
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        if self.render_mode == "window":
            plt.show(block=False)
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_aspect("equal", adjustable="box")
        self.scat1 = self.ax.scatter([], [], s=4)
        self.scat2 = self.ax.scatter([], [], s=4)
        self.scat3 = self.ax.scatter([], [], s=1)
        self.scat4 = self.ax.scatter([], [], s=100, c='green')
        self.scat5 = self.ax.scatter([], [], s=2)
        self.arrow = FancyArrowPatch(
            (0, 0), (0, 0),
            arrowstyle='-|>',
            mutation_scale=16,   # размер наконечника
            lw=1.8,
            color='green',
            zorder=5
        )
        self.ax.add_patch(self.arrow)
        self.fig.canvas.draw()
        self.bg = self.fig.canvas.copy_from_bbox(self.ax.bbox)

    def _update_plot(self, old_points, points, pts_from, robot_pos, robot_angle):
        # вызывается только из потока рендера
        if self.fig is None:
            self._init_plot()

        self.scat1.set_offsets(old_points)
        self.scat2.set_offsets(pts_from)
        self.scat3.set_offsets(points)
        self.scat4.set_offsets(robot_pos[None, :])
        self.scat5.set_offsets(self.path)

        # Стрелка
        arrow_len = 0.4
        dx = arrow_len * np.cos(robot_angle)
        dy = arrow_len * np.sin(robot_angle)
        head_xy = (robot_pos[0] + dx, robot_pos[1] + dy)
        self.arrow.set_positions((robot_pos[0], robot_pos[1]), head_xy)

        # Блиттинг
        self.fig.canvas.restore_region(self.bg)
        self.ax.draw_artist(self.scat1)
        self.ax.draw_artist(self.scat2)
        self.ax.draw_artist(self.scat3)
        self.ax.draw_artist(self.scat4)
        self.ax.draw_artist(self.scat5)
        self.ax.draw_artist(self.arrow)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()
        self._draw_external_openings()

    def _draw_external_openings(self):
        """Вызывается из render thread. Рисует рамочки/точки по self._overlay_openings (OpeningResult)."""
        if self.fig is None or not hasattr(self, "_overlay_openings"):
            return

        # очистка прошлых артефактов
        if not hasattr(self, "_opening_artists"):
            self._opening_artists = []
        for a in self._opening_artists:
            try:
                a.remove()
            except Exception:
                pass
        self._opening_artists.clear()

        openings = self._overlay_openings
        if not openings:
            return

        def _rect_world_from_robot_frame(x_mid, y_mid, gap_len, inlier_tol, pose_pos, pose_ang):
            # прямоугольник в СК робота: длина = gap_len по оси X, высота = 2*inlier_tol по оси Y
            x0, x1 = x_mid - gap_len / 2.0, x_mid + gap_len / 2.0
            y0, y1 = y_mid - inlier_tol, y_mid + inlier_tol
            rect_rb = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)

            R = np.array([[np.cos(pose_ang), -np.sin(pose_ang)],
                        [np.sin(pose_ang),  np.cos(pose_ang)]], dtype=float)
            return (rect_rb @ R.T) + pose_pos

        # толщину "стены" (полувысота рамки) берём из set_external_openings(..., inlier_tol=...)
        inlier_tol = getattr(self, "_overlay_inlier_tol", 0.06)

        for side in ("left", "right"):
            res = openings.get(side) if openings else None
            # res — это OpeningResult или None
            if res is None or not getattr(res, "found", False) or (getattr(res, "x_mid", None) is None):
                continue

            # поза на момент детекта (если ты её где-то прикладываешь),
            # иначе текущая поза навигатора
            pose = getattr(res, "pose", None)  # допустимо отсутствует у OpeningResult
            pose_pos = np.asarray(pose.get("pos"), float) if isinstance(pose, dict) and "pos" in pose else self.pos
            pose_ang = float(pose.get("angle")) if isinstance(pose, dict) and "angle" in pose else self.angle

            # длина проёма: пробуем достать из debug, иначе дефолт
            gap_len = None
            dbg = getattr(res, "debug", None)
            if isinstance(dbg, dict):
                gap_len = dbg.get("gap_len", None)
            if gap_len is None:
                gap_len = 0.22  # аккуратный дефолт, если детектор не передал ширину

            rect_w = _rect_world_from_robot_frame(res.x_mid, res.y_mid, gap_len, inlier_tol, pose_pos, pose_ang)

            color = "tab:purple" if side == "left" else "tab:red"
            poly = mpatches.Polygon(rect_w, closed=True, fill=False, lw=2.0, ls="--", ec=color, zorder=0)
            self.ax.add_patch(poly)
            self._opening_artists.append(poly)

            # центр проёма на стене
            gap_world = getattr(res, "world_gap_center", None)
            if isinstance(gap_world, (tuple, list, np.ndarray)) and len(gap_world) == 2:
                gx, gy = float(gap_world[0]), float(gap_world[1])
                dot_gap = self.ax.scatter([gx], [gy], s=30, c=color, zorder=0)
                self._opening_artists.append(dot_gap)

            # проекция на ось движения (куда падает перпендикуляр из центра)
            proj_world = getattr(res, "world_proj_point", None)
            if isinstance(proj_world, (tuple, list, np.ndarray)) and len(proj_world) == 2:
                px, py = float(proj_world[0]), float(proj_world[1])
                mark = self.ax.scatter([px], [py], s=80, c="cyan", marker="o", edgecolors="gray", zorder=0)
                self._opening_artists.append(mark)

        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            pass

        def stop_rendering(self):
            if self._render_thread and self._render_thread.is_alive():
                self._render_stop.set()
                self._render_thread.join()
            # Закрываем окно только в главном потоке и если реально было окно
            if self.render_mode == "window" and threading.current_thread() is threading.main_thread():
                try:
                    import matplotlib.pyplot as plt
                    plt.close(self.fig)
                except Exception:
                    pass

        def _safe_atexit_close(self):
            # вызывается уже в момент сворачивания интерпретатора
            try:
                self.stop_rendering()
            except Exception:
                pass
    
    def get_relative_points(self, max_dist=None, angle_shift=0, max_angle=None, max_abs_y=None):
        dth = -self.angle - np.deg2rad(angle_shift)
        M = np.array([
            [np.cos(dth), np.sin(dth)],
            [-np.sin(dth), np.cos(dth)],
        ])
        points = (self.points - self.pos) @ M
        
        if max_dist is not None:
            dist = np.linalg.norm(points, axis=-1)
            points = points[dist < max_dist]
        
        if max_angle is not None:
            angles = np.arctan2(points[..., 1], points[..., 0])
            points = points[abs(angles) < np.deg2rad(max_angle)]

        if max_abs_y is not None:
            points = points[abs(points[..., 1]) < max_abs_y]

        return points
    
    def unproject_to_world(self, points):
        points = np.array(points, float)
        dth = self.angle
        M = np.array([
            [np.cos(dth), np.sin(dth)],
            [-np.sin(dth), np.cos(dth)],
        ])
        points = points @ M + self.pos
        return points
    
    def project_to_robot(self, points, pos=None, angle=None):
        if pos is None:
            pos = self.pos
        if angle is None:
            angle = self.angle

        dth = -angle
        M = np.array([
            [np.cos(dth), np.sin(dth)],
            [-np.sin(dth), np.cos(dth)],
        ])
        points = (points - pos) @ M
        return points

    def update_from_odometry(self, odom_pos, odom_angle):
        with self.lock:
            angle = self.angle

        odom_delta = odom_pos - self.prev_odom_pos
        self.prev_odom_pos = odom_pos.copy()

        odom_angle_delta = odom_angle - self.prev_odom_angle
        self.prev_odom_angle = odom_angle

        odom_delta = transform_points(odom_delta, 0, angle - odom_angle)
        
        with self.lock:
            # self.angle = self.angle + odom_angle_delta
            self.pos = self.pos + odom_delta
            return self.pos, self.angle
        
    prev_gyro_time = time.monotonic()
    prev_gyro_w = 0
    def update_from_gyro(self, gyro):
        _, w, _ = gyro
        
        time_now = time.monotonic()
        dt = time_now - self.prev_gyro_time
        self.prev_gyro_time = time_now

        w_med = 0.5 * (self.prev_gyro_w + w)
        self.prev_gyro_w = w

        dth = w_med * dt

        with self.lock:
            self.angle = self.angle + dth
            return self.pos, self.angle

    def update_from_lidar(self, lidar_data: Dict[float, float]):
        ranges = np.array(list(lidar_data.values()))
        angles = np.deg2rad(np.array(list(lidar_data.keys())))

        self.relative_points = np.stack([
            ranges * np.cos(angles),
            ranges * np.sin(angles)
        ], axis=-1)

        self.cartographer.notify()

        with self.lock:
            return self.pos, self.angle

    def get_wall_dist(self, center_angle=-45, max_angle=20, max_dist=2):
        points = self.get_relative_points(max_dist=max_dist, angle_shift=center_angle, max_angle=max_angle)
        if len(points) == 0:
            return 0
        
        angles = np.arctan2(points[..., 1], points[..., 0])
        dists = np.linalg.norm(points, axis=-1)
        wall_dists = dists * abs(np.sin(angles + np.deg2rad(center_angle)))

        min_dist = np.min(wall_dists)
        return min_dist
    
    def get_wall_dist_and_angle(self, center_angle=-45, max_angle=20, max_dist=2):
        points = self.get_relative_points(max_dist=max_dist, angle_shift=center_angle, max_angle=max_angle)
        if len(points) == 0:
            return None, None

        wall_dist, wall_angle, _ = fit_line_polar_ransac(points, min_inliers=5, max_wall_angle=30)
        
        return wall_dist, wall_angle

    def _render_loop(self):
        """Запускается только в render thread"""
        period = 1.0 / self.render_fps
        frame_idx = 0

        # для вывода в файл — подготовить директорию
        if self.render_mode == "file":
            out_dir = self.render_out_dir or "navigator_images"
            os.makedirs(out_dir, exist_ok=True)

        t_next = time.perf_counter()
        while not self._render_stop.is_set():
            t0 = time.perf_counter()

            # Снимок данных под лок
            with self.lock:
                old_points = self.points.copy()
                pts_cur = getattr(self, "cur_lidar_pts", np.zeros((0, 2)))
                pts_from = getattr(self, "cur_matched_pts", np.zeros((0, 2)))
                robot_pos = self.pos.copy()
                robot_angle = self.angle

            # Рисуем
            if self.render_mode == "window":
                self._update_plot(old_points, pts_cur, pts_from, robot_pos, robot_angle)
            elif self.render_mode == "file":
                self._update_plot(old_points, pts_cur, pts_from, robot_pos, robot_angle)
                # сохранить кадр PNG
                out_path = os.path.join(self.render_out_dir or "navigator_images", f"last_frame.png")
                self.fig.savefig(out_path, dpi=100, bbox_inches="tight")
                frame_idx += 1

            # Дождаться следующего слота по FPS
            t_next += period
            sleep_time = t_next - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # если не успеваем — не накапливаем лаг
                t_next = time.perf_counter()

    def set_external_openings(self, openings: dict | None, inlier_tol: float = 0.06):
        """
        Передаёт в навигатор результаты внешнего детектора.
        Ожидается формат как у find_openings_left_right():
            {
              "left":  { ... как из find_side_opening ... }  | None,
              "right": { ... } | None,
              "nearest": { ... } | None
            }
        Обязательные поля у каждого непустого dict:
            - x_mid (м, вдоль движения, в СК робота на момент детекта)
            - gap_len (м)
            - gap_world: (x,y) мировых координат центра проёма
            - proj_world: (x,y) — проекция на ось движения (куда падает перпендикуляр)
        Необязательно, но полезно:
            - pose: {"pos": (x,y), "angle": float}  # поза робота в момент детекта
              Если нет, используется текущая поза (возможен небольшой параллакс).
        """
        with self.lock:
            self._overlay_openings = openings
            self._overlay_inlier_tol = float(inlier_tol)
