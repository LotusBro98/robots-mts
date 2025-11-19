import multiprocessing as mp
import os
from queue import Empty, Full
import signal
import threading
import time
from typing import Callable, Dict, Tuple
import numpy as np
from navigator_utils import estimate_update_point_to_line_robust, filter_visible_2d, keep_closer, remove_far_outliers, transform_points, voxel_downsample
from scipy.spatial import cKDTree


def _cell_key(pt: np.ndarray, cell_size):
    # ключ ячейки в окрестности размером cell_size
    if len(pt.shape) == 1:
        return (int(np.round(pt[0] / cell_size)), int(np.round(pt[1] / cell_size)))
    else:
        return [tuple(np.int32(np.round(p / cell_size)).tolist()) for p in pt]

class Candidate:
    __slots__ = ("pos", "hits", "sum_sq", "last_frame", "_promoted")
    def __init__(self, pos, frame_id):
        self.pos = pos.astype(float)
        self.hits = 1
        self.sum_sq = 0.0   # сумма квадратов смещений относительно текущего центра (для std)
        self.last_frame = frame_id
        self._promoted = False

    def update(self, p, ema_alpha, all_cands, cand_cell):
        old_key = _cell_key(self.pos, cand_cell)
        
        # экспоненциальное/гармоническое усреднение
        if ema_alpha is None:
            ema_alpha = 1.0 / (self.hits + 1)  # «честное» среднее

        delta = p - self.pos
        self.pos = self.pos + ema_alpha * delta
        # обновим суммарный разброс (Welford light)
        self.sum_sq += float((p - self.pos) @ (p - self.pos))
        self.hits += 1

        new_key = _cell_key(self.pos, cand_cell)
        if new_key != old_key:
            all_cands.pop(old_key)
            if new_key in all_cands:
                all_cands[new_key].update(self.pos, ema_alpha, all_cands, cand_cell)
            else:
                all_cands[new_key] = self

    def promote(self):
        self._promoted = True

    @property
    def std(self):
        if self.hits <= 1:
            return np.inf
        return np.sqrt(self.sum_sq / (self.hits - 1))


def _estimate_transform(pts_from, pts_to, center):
    if len(pts_from) == 0:
        return np.zeros((2,), dtype=np.float32), 0

    dpos, dth, quality, info = estimate_update_point_to_line_robust(
        pts_from, pts_to, center=center,
    )

    return -dpos, -dth

def match_nearest(
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

class CartohrapherNested:
    def __init__(self):
        self.candidates: Dict[Tuple[int,int], Candidate] = {}  
        self._frame_id = 0
        self.max_dist_robot=5.0

    def add_scan_with_buffer(self,
                                points_world: np.ndarray,
                                min_dist: float,
                                promote_hits: int = 3,
                                world_enter_hits: int = 2,
                                max_candidate_age: int = 15,
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
        points = voxel_downsample(points, min_dist)
        points = remove_far_outliers(points, min_dist * 2)
        if points.size == 0:
            return points

        # 4) обновляем/создаём кандидатов в сетке кандидатов
        seen_cells = set()
        for p in points:
            key = _cell_key(p, min_dist)
            seen_cells.add(key)
            c: Candidate = self.candidates.get(key)
            if c is None:
                self.candidates[key] = Candidate(p, self._frame_id)
            else:
                c.update(p, ema_alpha, self.candidates, min_dist)
                c.last_frame = self._frame_id

        # 5) понижаем «возраст» для не увиденных кандидатов и удаляем старые
        dead = []
        for key, c in self.candidates.items():
            if key not in seen_cells and (self._frame_id - c.last_frame) > max_candidate_age and not c._promoted:
                dead.append(key)
        for key in dead:
            self.candidates.pop(key, None)

        # 6) проверяем условия повышения кандидатов в карту
        for key, c in self.candidates.items():
            if c.hits >= promote_hits:
                c.promote()

        new_pts = [c.pos for c in self.candidates.values() if c.hits >= world_enter_hits]
        points = np.stack(new_pts) if len(new_pts) > 0 else np.zeros((0, 2))
        return points
    
    def update_map(self, world_points, relative_points, pos, angle):
        relative_points = keep_closer(relative_points, 0, self.max_dist_robot)
        relative_points = voxel_downsample(relative_points, grid_size=0.01)
        # relative_points = resample_lidar_by_distance(relative_points, step=0.05, max_gap=0.2)
        # relative_points = remove_far_outliers(relative_points, dist_thresh=0.1)
        
        world_points = keep_closer(world_points, pos, self.max_dist_robot)
        world_points = filter_visible_2d(world_points, pos)
        cur_matched_pts = world_points
        for i in range(10):
            points = transform_points(relative_points, pos, angle, (0, 0))

            pts_from, pts_to = match_nearest(points, world_points, max_dist=0.2, unique=True)
            dpos, dth = _estimate_transform(pts_from, pts_to, pos)
            pos = pos + dpos
            angle = angle + dth
            
            if np.linalg.norm(dpos) < 1e-3 and abs(dth) < 1e-3:
                break
            
        points = transform_points(relative_points, pos, angle, (0, 0))
        world_points = self.add_scan_with_buffer(points, min_dist=0.025, promote_hits=10, world_enter_hits=3, max_candidate_age=3, ema_alpha=0.1)
        
        return world_points, cur_matched_pts, points, pos, angle
    

def _cartographer_process(req_q: mp.Queue, res_q: mp.Queue):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    os.nice(18)

    nest = CartohrapherNested()
    while True:
        world_points, relative_points, pos, angle, stop = req_q.get()
        if stop:
            break
        orig_pos = pos.copy()
        orig_angle = angle
        world_points, cur_matched_pts, points, pos, angle = nest.update_map(world_points, relative_points, pos, angle)
        res_q.put((world_points, cur_matched_pts, points, pos - orig_pos, angle - orig_angle))


class Cartographer:
    def __init__(self, input_cb: Callable[[], Tuple[np.ndarray, np.ndarray, np.ndarray, float]], output_cb: Callable[[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float], None]):
        self.input_cb = input_cb
        self.output_cb = output_cb
        
        self._req_q = mp.Queue(maxsize=1)
        self._res_q = mp.Queue(maxsize=1)

        self._proc = mp.Process(
            target=_cartographer_process,
            daemon=True,
            args=[self._req_q, self._res_q]
        )
        self._proc.start()

        self._stop = False
        self._thr = threading.Thread(
            target=self._cartographer_thread,
            daemon=True,
        )
        self._thr.start()

    def stop(self):
        self._stop = True
        self._thr.join()

    def _cartographer_thread(self):
        while not self._stop:
            try:
                world_points, cur_matched_pts, points, dpos, dth = self._res_q.get(timeout=1)
                self.output_cb(world_points, cur_matched_pts, points, dpos, dth)
            except Empty:
                continue
        self._req_q.put((None, None, None, None, True))
        self._proc.join()
        self._req_q.close()
        self._req_q.join_thread()
        self._res_q.close()
        self._res_q.join_thread()

    def notify(self):
        if self._stop:
            return
        
        world_points, relative_points, pos, angle = self.input_cb()
        try:
            self._req_q.put_nowait((world_points, relative_points, pos, angle, False))
        except Full:
            pass
