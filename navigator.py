import os
import threading
import time
from typing import Dict

import matplotlib
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch
import numpy as np
from scipy.spatial import cKDTree

def _cell_key(pt, cell_size):
    # ключ ячейки в окрестности размером cell_size
    return (int(np.floor(pt[0] / cell_size)), int(np.floor(pt[1] / cell_size)))

class Candidate:
    __slots__ = ("pos", "hits", "sum_sq", "last_frame")
    def __init__(self, pos, frame_id):
        self.pos = pos.astype(float)
        self.hits = 1
        self.sum_sq = 0.0   # сумма квадратов смещений относительно текущего центра (для std)
        self.last_frame = frame_id

    def update(self, p, ema_alpha=None):
        # экспоненциальное/гармоническое усреднение
        if ema_alpha is None:
            ema_alpha = 1.0 / (self.hits + 1)  # «честное» среднее

        delta = p - self.pos
        self.pos += ema_alpha * delta
        # обновим суммарный разброс (Welford light)
        self.sum_sq += float((p - self.pos) @ (p - self.pos))
        self.hits += 1

    @property
    def std(self):
        if self.hits <= 1:
            return np.inf
        return np.sqrt(self.sum_sq / (self.hits - 1))


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
        self._grid = {}                               # воксельная сетка для быстрой проверки (cell -> index в points)
        self.candidates = {}                          # cell -> Candidate
        self._frame_id = 0

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

        # Поток рендера
        self._render_stop = threading.Event()
        self._render_thread = None
        if self.render_mode in ("window", "file"):
            self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
            self._render_thread.start()

    def _init_plot(self):
        if self.fig is not None:  # уже создано
            return
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        if self.render_mode == "window":
            plt.show(block=False)
        self.ax.set_xlim(-1, 8)
        self.ax.set_ylim(-1, 8)
        self.ax.set_aspect("equal", adjustable="box")
        self.scat1 = self.ax.scatter([], [], s=4)
        self.scat2 = self.ax.scatter([], [], s=4)
        self.scat3 = self.ax.scatter([], [], s=1)
        self.scat4 = self.ax.scatter([], [], s=100, c='green')
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
        self.ax.draw_artist(self.arrow)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

    def _get_nearest_neighbors(self, points, min_dist=0.025):
        if len(self.points) == 0:
            return np.zeros([0, 2]), np.zeros([0, 2]), points

        diff = points[:, None, :] - self.points[None, :, :]
        dist = np.linalg.norm(diff, axis=-1)
        idx = np.argmin(dist, axis=-1)
        mask = np.take_along_axis(dist, idx[:, None], axis=1)[:, 0] < min_dist
        idx = idx[mask]
        # return self.points[idx], np.take_along_axis(diff[mask], idx[:, None, None], axis=1)[:, 0, :]
        return self.points[idx], points[mask], points[~mask]

    def match_nearest(self,
        small: np.ndarray,
        big: np.ndarray,
        max_dist: float,
        unique: bool = True,
    ):
        """
        Сопоставляет точки из small к ближайшим соседям в big.

        Параметры
        ---------
        small : (Ns, D) np.ndarray
            Малое облако точек.
        big : (Nb, D) np.ndarray
            Большое облако точек (по нему ищем соседей).
        max_dist : float | None
            Максимальная допустимая дистанция до соседа. Если None — принимаем любого ближайшего.
        unique : bool
            Если True — делаем соответствие один-ко-одному: одна точка big используется не более одного раза
            (берём пару с минимальной дистанцией).

        Возвращает
        ----------
        big_matched : (M, D) np.ndarray
            Точки из большого облака, для которых нашли пары (в порядке, согласованном со small_matched).
        small_matched : (M, D) np.ndarray
            Точки из малого облака, которым соответствуют big_matched.
        small_unmatched : (K, D) np.ndarray
            Точки из малого облака, которым пары не нашлось (или они отсеяны по max_dist).
        """
        small = np.asarray(small, dtype=float)
        big = np.asarray(big, dtype=float)

        if small.size == 0 or big.size == 0:
            return (np.empty((0, big.shape[1])), np.empty((0, small.shape[1])))

        # --- Пытаемся использовать быстрый KD-дерево (scipy), иначе — fallback на NumPy ---
        tree = cKDTree(big)
        dists, idx = tree.query(small, k=1, workers=-1)  # dists: (Ns,), idx: (Ns,)

        # Маска «проходит порог»
        ok = dists <= max_dist

        # Если требуется уникальность — оставляем для каждого индекса big только ближайшую пару
        if unique:
            # Берём только кандидатов, которые прошли порог
            cand_ids = np.nonzero(ok)[0]          # индексы в small
            cand_big = idx[cand_ids]              # кандидаты в big
            cand_d = dists[cand_ids]

            # Сортируем по расстоянию, чтобы первым закреплять самые близкие пары
            order = np.argsort(cand_d, kind="stable")
            cand_ids = cand_ids[order]
            cand_big = cand_big[order]

            used_big = np.zeros(big.shape[0], dtype=bool)
            keep_small_mask = np.zeros(small.shape[0], dtype=bool)

            for s_i, b_i in zip(cand_ids, cand_big):
                if not used_big[b_i]:
                    used_big[b_i] = True
                    keep_small_mask[s_i] = True

            matched_small_idx = np.nonzero(keep_small_mask)[0]
        else:
            matched_small_idx = np.nonzero(ok)[0]

        # Формируем выходы
        big_matched = big[idx[matched_small_idx]]
        small_matched = small[matched_small_idx]

        return big_matched, small_matched

    
    def _transform_points(self, points, dpos, dth, pos=(0,0)):
        M = np.array([
            [np.cos(dth), np.sin(dth)],
            [-np.sin(dth), np.cos(dth)],
        ])
        points = (points - pos) @ M + pos + dpos
        return points
    
    def _get_relative_points(self, max_dist=None, angle_shift=0, max_angle=None):
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

        return points
    
    def _estimate_transform(self, pts_from, pts_to, center):
        if len(pts_from) == 0:
            return np.zeros((2,), dtype=np.float32), 0
        
        # M, inliers = cv.estimateAffinePartial2D(pts_from, pts_to)
        # if M is None or np.isnan(M).any():
        #     return np.zeros((2,), dtype=np.float32), 0
        # dpos = M[:, 2]
        # print()
        # print(M)
        # dth = np.arctan2(M[0, 1], M[0, 0])
        # return dpos, dth

        pts_from = pts_from - center
        pts_to = pts_to - center
        normals = self._transform_points(pts_from + pts_to, 0, np.pi/2)
        normals /= np.linalg.norm(normals, axis=-1, keepdims=True)
        dth = np.sum((pts_from - pts_to) * normals, axis=-1).mean()

        pts_aligned = self._transform_points(pts_from, 0, dth)
        dpos = (pts_aligned - pts_to)
        # self.scat2.set_offsets(dpos * 20)
        dpos = dpos.mean(axis=0)

        return dpos, dth
    
    def _add_new_points(self, points, min_dist, inside_dist=0.1, ignore_outliers=True):
        if len(self.points) == 0:
            self.points = points[:1]
            print(points[:1])
            for pt in points:
                self._add_new_points(pt[None], min_dist, inside_dist, ignore_outliers=False)
            return

        tree_big = cKDTree(self.points)
        dists, idx = tree_big.query(points, k=1, workers=-1)

        if ignore_outliers:
            tree_small = cKDTree(points)
            dists_small, idx = tree_small.query(points, k=2, workers=-1)
            dists_small = dists_small[:, 1]
            outliers_mask = dists_small < inside_dist
        else:
            outliers_mask = np.ones_like(dists, dtype=np.bool_)
        
        new_points = points[(dists > min_dist) & outliers_mask]
        self.points = np.append(self.points, new_points, axis=0)

    def _rebuild_grid(self, cell_size):
        self._grid.clear()
        for i, pt in enumerate(self.points):
            self._grid[_cell_key(pt, cell_size)] = i

    def _is_far_from_map(self, p, min_dist, tree=None):
        # быстрая проверка по вокселю + при желании тонкая по KDTree
        if self.points.shape[0] == 0:
            return True
        base_cell = _cell_key(p, min_dist)
        # проверяем текущую и 8 соседних ячеек
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                idx = self._grid.get((base_cell[0]+dx, base_cell[1]+dy))
                if idx is None: 
                    continue
                if np.linalg.norm(self.points[idx] - p) <= min_dist:
                    return False
        # опционально можно добить точной проверкой по KDTree, если нужно
        return True

    def add_scan_with_buffer(self,
                             points_world: np.ndarray,
                             min_dist: float,
                             promote_hits: int = 20,
                             promote_std: float = 0.05,
                             candidate_cell_scale: float = 0.9,
                             max_candidate_age: int = 30,
                             ema_alpha: float | None = None):
        """
        points_world      : (N,2) текущий снимок в мировой системе (уже скорректирован по позе)
        min_dist          : минимальная дистанция между «узлами» карты
        inside_dist       : точка считается «реальной» (не шум), если рядом есть другие точки скана (< inside_dist)
        promote_hits      : сколько раз точка должна стабильно «попасть», чтобы стать частью карты
        promote_std       : максимальный разброс кандидата (м) для повышения
        candidate_cell_scale : размер сетки кандидатов = min_dist * scale (чуть меньше, чтобы быстрее схлопывались)
        max_candidate_age : через сколько кадров без наблюдений кандидат удаляется
        ema_alpha         : None -> 1/(hits+1); иначе явная константа EMA (например, 0.3)
        """
        self._frame_id += 1
        points = np.asarray(points_world, dtype=float)
        if points.size == 0:
            return

        # 1) быстрая проверка «точка не одиночка» в самом скане
        #    (игнорируем выбросы, у которых ближайший сосед в текущем кадре далеко)
        from scipy.spatial import cKDTree
        tree_small = cKDTree(points)
        nn_d, nn_i = tree_small.query(points, k=2, workers=-1)   # вторая колонка — ближайший сосед кроме самой точки

        # 2) подготовим сетку карты (если пустая — ускорим старт)
        if len(self._grid) == 0 and self.points.shape[0] > 0:
            self._rebuild_grid(min_dist)

        # 3) отсекаем точки, которые слишком близко к существующей карте (соблюдаем разрежение min_dist)
        #    («далеко от карты» — кандидаты на добавление/обновление)
        far_mask = np.empty(points.shape[0], dtype=bool)
        for i, p in enumerate(points):
            far_mask[i] = self._is_far_from_map(p, min_dist)
        candidate_pts = points[far_mask]

        # 4) обновляем/создаём кандидатов в сетке кандидатов
        cand_cell = min_dist * candidate_cell_scale
        seen_cells = set()
        for p in candidate_pts:
            key = _cell_key(p, cand_cell)
            seen_cells.add(key)
            c = self.candidates.get(key)
            if c is None:
                self.candidates[key] = Candidate(p, self._frame_id)
            else:
                c.update(p, ema_alpha)
                c.last_frame = self._frame_id

        # 5) понижаем «возраст» для не увиденных кандидатов и удаляем старые
        dead = []
        for key, c in self.candidates.items():
            if key not in seen_cells and (self._frame_id - c.last_frame) > max_candidate_age:
                dead.append(key)
        for key in dead:
            self.candidates.pop(key, None)

        # 6) проверяем условия повышения кандидатов в карту
        to_promote = []
        for key, c in self.candidates.items():
            if c.hits >= promote_hits and c.std <= promote_std and self._is_far_from_map(c.pos, min_dist):
                to_promote.append(key)

        if to_promote:
            new_pts = np.array([self.candidates[k].pos for k in to_promote], dtype=float)
            # добавляем в карту
            if self.points.size == 0:
                self.points = new_pts
            else:
                self.points = np.vstack([self.points, new_pts])
            # обновляем сетку карты локально
            for p in new_pts:
                self._grid[_cell_key(p, min_dist)] = len(self.points) - 1  # индексы тут примерные; сетка всё равно только для окрестности
            # убираем повышенных из кандидатов
            for k in to_promote:
                self.candidates.pop(k, None)

    def update_from_odometry(self, odom_pos, odom_angle):
        with self.lock:
            angle_offset = self.odom_angle_offset

        odom_delta = self._transform_points(odom_pos, -self.prev_odom_pos, angle_offset, pos=self.prev_odom_pos)
        self.prev_odom_pos = odom_pos
        odom_angle_delta = odom_angle - self.prev_odom_angle
        self.prev_odom_angle = odom_angle

        with self.lock:
            # self.angle += odom_angle_delta
            self.pos += odom_delta
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
            self.angle += dth
            return self.pos, self.angle

    def update_from_lidar(self, lidar_data: Dict[float, float]):
        ranges = np.array(list(lidar_data.values()))
        angles = np.deg2rad(np.array(list(lidar_data.keys())))

        relative_points = np.stack([
            ranges * np.cos(angles),
            ranges * np.sin(angles)
        ], axis=-1)

        with self.lock:
            angle = self.angle
            pos = self.pos

        points = self._transform_points(relative_points, pos, angle, (0, 0))

        pts_from, pts_to = self.match_nearest(points, self.points, max_dist=0.1, unique=False)
        dpos, dth = self._estimate_transform(pts_from, pts_to, pos)
        points = self._transform_points(points, dpos, dth, pos)
        if len(self.points) == 0:
            self._add_new_points(points, min_dist=0.05)
        else:
            self.add_scan_with_buffer(points, min_dist=0.05, promote_hits=10, max_candidate_age=3, candidate_cell_scale=0.1)
        
        dpos *= 0.1
        MAX_DPOS = 0.01
        dpos = np.clip(dpos, -MAX_DPOS, MAX_DPOS)
        
        dth *= 0.1
        MAX_DTH = np.deg2rad(0.1)
        dth = np.clip(dth, -MAX_DTH, MAX_DTH)
    
        self.cur_lidar_pts = points
        self.cur_matched_pts = pts_from

        with self.lock:
            self.pos += dpos
            self.angle += dth
            self.odom_angle_offset += dth
            return self.pos, self.angle

    last_display_time = 0
    def display(self):
        return
        # cur_time = time.time()
        # if cur_time - self.last_display_time > 1:
        #     self._update_plot(self.points, self.cur_lidar_pts, self.cur_matched_pts)
        #     self.last_display_time = cur_time

    def get_wall_dist(self, center_angle=-45, max_angle=20, max_dist=2):
        points = self._get_relative_points(max_dist=max_dist, angle_shift=center_angle, max_angle=max_angle)
        if len(points) == 0:
            return 0
        
        angles = np.arctan2(points[..., 1], points[..., 0])
        dists = np.linalg.norm(points, axis=-1)
        wall_dists = dists * abs(np.sin(angles + np.deg2rad(center_angle)))

        min_dist = np.min(wall_dists)
        return min_dist

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

    def stop_rendering(self):
        if self._render_thread and self._render_thread.is_alive():
            self._render_stop.set()
            self._render_thread.join(timeout=1.0)
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
