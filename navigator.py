import threading
import time
from typing import Dict, List
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from scipy.spatial import cKDTree
from collections import defaultdict

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
    def __init__(self):
        self.pos = np.zeros((2,), dtype=np.float32)
        self.angle = 0
        self.points = np.zeros((0, 2), dtype=np.float32)
        self._grid = {}                               # воксельная сетка для быстрой проверки (cell -> index в points)
        self.candidates = {}                          # cell -> Candidate
        self._frame_id = 0

        self.prev_odom_pos = np.zeros((2,), dtype=np.float32)
        self.prev_odom_angle = 0
        self.odom_pos_offset = np.zeros((2,), dtype=np.float32)
        self.odom_angle_offset = 0

        self.lock = threading.Lock()

        self._init_plot()

    def _init_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(15,7))
        self.ax.set_xlim(-1, 14)
        self.ax.set_ylim(-1, 6)
        self.ax.set_aspect("equal", adjustable="box")
        self.scat1 = self.ax.scatter([], [], s=4)
        self.scat2 = self.ax.scatter([], [], s=4)
        self.scat3 = self.ax.scatter([], [], s=1)
        self.fig.canvas.draw()
        self.bg = self.fig.canvas.copy_from_bbox(self.ax.bbox)

    def _update_plot(self, old_points, points, pts_from):
        self.scat1.set_offsets(old_points)
        self.scat2.set_offsets(pts_from)
        self.scat3.set_offsets(points)

        # Блиттинг: восстанавливаем фон, рисуем артиш и блитим только область осей
        self.fig.canvas.restore_region(self.bg)
        self.ax.draw_artist(self.scat1)
        self.ax.draw_artist(self.scat2)
        self.ax.draw_artist(self.scat3)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()
        # маленькая пауза даёт GUI-циклу обработать события
        try:
            plt.pause(0.0001)
        except:
            pass
        
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
            self.angle += odom_angle_delta
            self.pos += odom_delta
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
    
        self.cur_lidar_pts = points
        self.cur_matched_pts = pts_from

        with self.lock:
            self.pos += dpos
            self.angle += dth
            self.odom_angle_offset += dth
            return self.pos, self.angle

    last_time = 0
    def display(self):
        cur_time = time.time()
        if cur_time - self.last_time > 1:
            self._update_plot(self.points, self.cur_lidar_pts, self.cur_matched_pts)
            self.last_time = cur_time

        