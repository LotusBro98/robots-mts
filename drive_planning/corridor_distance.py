import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, Literal, Tuple

Side = Literal["left", "right"]

@dataclass
class OpeningResult:
    side: Side
    found: bool
    distance: Optional[float]                 # расстояние вперёд (по направлению робота) до проекции центра проёма
    world_proj_point: Optional[Tuple[float,float]]  # точка на оси движения (мировые координаты), куда падает перпендикуляр из центра проёма
    world_gap_center: Optional[Tuple[float,float]]  # сам центр проёма на линии стены (мировые координаты)
    x_mid: Optional[float]                    # то же, но в СК робота
    y_mid: Optional[float]
    debug: dict                               # любые отладочные поля

def _rotmat(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s],[s, c]])

def find_side_opening(
    points_xy_world: np.ndarray,          # shape (N,2)
    robot_xy: Tuple[float,float],         # (x,y) мира
    robot_heading_rad: float,             # куда «смотрим»
    side: Side,
    Smax: float = 4.0,                    # дальность вперёд для анализа
    y_band: Tuple[float,float] = (0.08, 1.2),   # минимальная/максимальная |y| (м) для кандидатов стены
    ransac_iters: int = 200,
    inlier_tol: float = 0.06,             # полуширина «полосы стены» (м)
    max_tilt_deg: float = 20.0,           # допустимый наклон стены к оси x (шум коридоров)
    dx: float = 0.05,                     # размер бина по x (м)
    min_open: float = 0.20,               # минимальная длина разрыва (м) — «дверной проём»
    need_wall_before_after_bins: int = 2  # требуем по ≥N бинов «есть стена» до и после проёма
) -> OpeningResult:
    """
    Возвращает первый (ближайший по x) валидный проём на выбранном борту.
    """
    assert points_xy_world.ndim == 2 and points_xy_world.shape[1] == 2

    # 1) в СК робота
    R = _rotmat(robot_heading_rad).T               # мир -> робот
    p_rel = (points_xy_world - np.asarray(robot_xy)) @ R.T
    x, y = p_rel[:,0], p_rel[:,1]

    # фильтр «впереди» и по борту
    if side == "right":
        mask_side = (x >= 0) & (x <= Smax) & (y <= -y_band[0]) & (y >= -y_band[1])
    else:  # left
        mask_side = (x >= 0) & (x <= Smax) & (y >=  y_band[0]) & (y <=  y_band[1])

    X = x[mask_side]
    Y = y[mask_side]
    pts = np.stack([X, Y], axis=1)
    if pts.shape[0] < 8:
        return OpeningResult(side, False, None, None, None, None, None, {"reason":"not_enough_points"})

    # 2) RANSAC по прямой y = a*x + b
    best_inliers = None
    best_ab = (0.0, np.median(Y))  # запасной вариант — горизонтальная на медиане
    max_inliers = 0
    max_tilt = math.tan(math.radians(max_tilt_deg))

    rng = np.random.default_rng(12345)
    for _ in range(ransac_iters):
        i, j = rng.integers(0, pts.shape[0], size=2)
        if i == j:
            continue
        (x1, y1), (x2, y2) = pts[i], pts[j]
        if abs(x2 - x1) < 1e-6:
            continue
        a = (y2 - y1) / (x2 - x1)
        if abs(a) > max_tilt:
            continue
        b = y1 - a * x1
        # инлаеры — точки в полосе inlier_tol вокруг линии
        dist = np.abs(pts[:,1] - (a*pts[:,0] + b))
        inliers = dist <= inlier_tol
        n = int(inliers.sum())
        if n > max_inliers:
            max_inliers = n
            best_inliers = inliers
            best_ab = (a, b)

    a, b = best_ab
    if best_inliers is not None and best_inliers.sum() >= 8:
        # уточним a,b по МНК на инлаерах
        Xi = pts[best_inliers, 0]
        Yi = pts[best_inliers, 1]
        A = np.vstack([Xi, np.ones_like(Xi)]).T
        a, b = np.linalg.lstsq(A, Yi, rcond=None)[0]

    # 3) дискретизация по x и карта наличия стены
    bins = np.arange(0.0, Smax + dx, dx)
    bin_ids = np.clip((X / dx).astype(int), 0, len(bins)-2)
    # «есть стена» если есть инлаеры в этом бине
    y_on_line = a*X + b
    has_wall = np.zeros(len(bins)-1, dtype=bool)
    near = np.abs(Y - y_on_line) <= inlier_tol
    for k in np.unique(bin_ids[near]):
        has_wall[k] = True

    # 4) поиск первого разрыва достаточной длины с «стеной» до и после
    run_start = None
    for k in range(len(has_wall)):
        if not has_wall[k] and run_start is None:
            # начался разрыв
            run_start = k
        if (has_wall[k] and run_start is not None) or (k == len(has_wall)-1 and run_start is not None):
            run_end = k if has_wall[k] else k+1  # полуинтервалы
            gap_len = (run_end - run_start) * dx
            # проверим окружение
            before_ok = has_wall[max(0, run_start-need_wall_before_after_bins):run_start].all() if run_start>0 else False
            after_ok  = has_wall[run_end:min(len(has_wall), run_end+need_wall_before_after_bins)].all() if run_end < len(has_wall) else False
            if gap_len >= min_open and before_ok and after_ok:
                # центр проёма по x
                x_mid = (run_start*dx + run_end*dx)/2.0
                y_mid = a*x_mid + b
                # 5) обратно в мир: точка проекции на ось движения (y=0) и сам центр проёма
                Rw = _rotmat(robot_heading_rad)          # робот -> мир
                proj_world = np.array([x_mid, 0.0]) @ Rw.T + np.asarray(robot_xy)
                gap_world  = np.array([x_mid, y_mid]) @ Rw.T + np.asarray(robot_xy)
                return OpeningResult(
                    side=side, found=True, distance=float(x_mid),
                    world_proj_point=(float(proj_world[0]), float(proj_world[1])),
                    world_gap_center=(float(gap_world[0]), float(gap_world[1])),
                    x_mid=float(x_mid), y_mid=float(y_mid),
                    debug={
                        "a": float(a), "b": float(b),
                        "gap_len": float(gap_len),
                        "run_bins": (int(run_start), int(run_end)),
                        "inlier_tol": inlier_tol, "dx": dx,
                        "max_inliers": int(max_inliers)
                    }
                )
            run_start = None

    return OpeningResult(side, False, None, None, None, None, None,
                         {"reason":"no_valid_gap", "a":float(a), "b":float(b)})

def find_openings_left_right(points_xy_world, robot_xy, robot_heading_rad, **kw) -> dict[str, OpeningResult]:
    """Ищет проёмы и слева, и справа; возвращает ближайший по расстоянию вперёд."""
    left  = find_side_opening(points_xy_world, robot_xy, robot_heading_rad, side="left", **kw)
    right = find_side_opening(points_xy_world, robot_xy, robot_heading_rad, side="right", **kw)

    best = None
    for res in (left, right):
        if res.found:
            if best is None or res.distance < best.distance:
                best = res
    return {"left": left, "right": right, "nearest": best}
