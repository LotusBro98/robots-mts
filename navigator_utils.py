from matplotlib import pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN


def _estimate_normals_pca(pts, k=8, center=None):
    tree = cKDTree(pts)
    _, idx = tree.query(pts, k=min(k, len(pts)))
    normals = np.zeros_like(pts, dtype=float)
    for i, nbr in enumerate(idx):
        P = pts[nbr]
        mu = P.mean(axis=0)
        C = (P - mu).T @ (P - mu) / max(1, P.shape[0] - 1)
        eigvals, eigvecs = np.linalg.eigh(C)
        tangent = eigvecs[:, np.argmax(eigvals)]
        n = np.array([-tangent[1], tangent[0]])
        n /= np.linalg.norm(n) + 1e-12
        normals[i] = n
    if center is not None:
        v = pts - center
        flip = np.sum(normals * v, axis=1) < 0
        normals[flip] *= -1.0
    return normals

def _huber_weights(r, delta=0.1):
    a = np.abs(r)
    w = np.ones_like(a)
    m = a > delta
    if np.any(m):
        w[m] = delta / (a[m] + 1e-12)
    return w

def largest_value_cluster(arr: np.ndarray, eps):
    arr = arr.reshape(-1, 1)
    labels = DBSCAN(eps=eps, min_samples=1).fit(arr).labels_
    unique, counts = np.unique(labels, return_counts=True)
    best_label = unique[np.argmax(counts)]
    cluster_mean = np.median(arr[labels == best_label])
    return cluster_mean

def estimate_update_point_to_line_robust(
    pts_from, pts_to, center=None, k_normals=6,
    huber_delta=0.10, reg_tangential=1e-3,
    # робаст-пороги:
    min_pts=20,                   # минимальное число пар
    soft_clip_pos=0.05,           # мягкий лимит нормы dpos (м) за шаг
    soft_clip_th=np.deg2rad(3.0), # мягкий лимит |dθ| за шаг
):
    """
    Возвращает:
      dpos(2,), dth, quality (0..1), info dict (диагностика)
    Если качество низкое — апдейт плавно приглушается к нулю.
    """
    info = {}
    if len(pts_from) == 0 or len(pts_to) == 0:
        return np.zeros(2), 0.0, 0.0, info

    c = np.zeros(2) if center is None else np.asarray(center, float)
    p = np.asarray(pts_from, float) - c
    q = np.asarray(pts_to,   float) - c
    N = min(len(p), len(q))
    p, q = p[:N], q[:N]

    # метрика: сколько пар реально есть
    if N < min_pts:
        info.update(N=N, reason="too_few_points")
        return np.zeros(2), 0.0, 0.0, info

    n = _estimate_normals_pca(q, k=k_normals, center=c)  # (N,2)

    # A x ≈ b, x = [tx, ty, dθ]
    Jp = np.column_stack((-p[:,1], p[:,0]))
    A = np.column_stack([n, np.sum(n * Jp, axis=1, keepdims=True)])
    b = np.sum(n * (q - p), axis=1)

    # робаст-веса
    w = _huber_weights(b, delta=huber_delta)
    Aw = A * w[:, None]
    bw = b * w

    # решение с небольшой регуляризацией по всем трём параметрам
    lam = reg_tangential
    Rreg = np.sqrt(lam) * np.eye(3)
    x = np.linalg.lstsq(np.vstack([Aw, Rreg]), np.concatenate([bw, np.zeros(3)]), rcond=None)[0]
    tx, ty, dth = x

    # мягкий клип шагов (чтобы одиночный сбой не ронял карту)
    step = np.hypot(tx, ty)
    if step > soft_clip_pos:
        scale = soft_clip_pos / (step + 1e-12)
        tx *= scale; ty *= scale
    if abs(dth) > soft_clip_th:
        dth = np.sign(dth) * soft_clip_th

    # Используем знание о том что стены под 90 градусов
    angles = np.arctan2(n[:, 1], n[:, 0])
    angles = round_angle(angles * 4) / 4
    # ang_offs = np.median(angles)
    ang_offs = largest_value_cluster(angles, eps=0.05)
    dth = ang_offs

    return np.array([tx, ty]), float(dth), 0, None


def normalize(vec: np.ndarray) -> np.ndarray:
    return vec / np.linalg.norm(vec, axis=-1, keepdims=True)

def vec_angle(vec: np.ndarray) -> np.ndarray:
    return np.arctan2(vec[..., 1], vec[..., 0])

def project_scalar(base: np.ndarray, vec: np.ndarray) -> float:
    return (normalize(base) * vec).sum(-1)

def round_angle(angle, radians=True):
    if not radians:
        angle = np.deg2rad(angle)
    angle = (angle + np.pi) % (2 * np.pi) - np.pi
    if not radians:
        angle = np.rad2deg(angle)
    return angle

def direction_vec(angle, radians=True):
    if not radians:
        angle = np.deg2rad(angle)
    return np.array([
        np.cos(angle), np.sin(angle)
    ], dtype=np.float32)

def transform_points(points, dpos, dth, pos=(0,0)):
        M = np.array([
            [np.cos(dth), np.sin(dth)],
            [-np.sin(dth), np.cos(dth)],
        ])
        points = (points - pos) @ M + pos + dpos
        return points

def keep_closer(points, center, max_dist):
    return points[np.linalg.norm(points - center, axis=-1) < max_dist]

def fit_line_polar_ransac(points: np.ndarray,
                          dist_thresh: float = 0.05,
                          min_inliers: int = 30,
                          max_iters: int = 300,
                          max_wall_angle: float = 30,
                          seed: int | None = None):
    """
    Оценивает прямую x*cos(theta) + y*sin(theta) = rho (rho>=0) по точкам с выбросами.
    RANSAC -> затем уточнение по инлаерам (total least squares).

    Параметры:
      points      : (N,2) массив точек.
      dist_thresh : порог расстояния до прямой (м) для инлаеров.
      min_inliers : минимум инлаеров, чтобы считать модель валидной.
      max_iters   : число итераций RANSAC.
      seed        : опционально для воспроизводимости.

    Возвращает:
      rho, theta, inliers_mask
      (если модель не найдена: rho=None, theta=None, inliers_mask=None)
    """
    P = np.asarray(points, float)
    if P.ndim != 2 or P.shape[1] != 2:
        raise ValueError("points must be of shape (N,2)")
    P = P[np.isfinite(P).all(axis=1)]
    N = len(P)
    if N < 2:
        return None, None, None

    rng = np.random.default_rng(seed)

    best_inliers = None
    best_count = 0

    def model_from_two(a, b):
        # нормаль к отрезку: n = R90*(b-a)
        t = b - a
        if np.allclose(t, 0):
            return None, None  # вырождение
        n = np.array([-t[1], t[0]], dtype=float)
        n /= (np.linalg.norm(n) + 1e-12)
        rho = float(n @ a)
        # нормализуем знак так, чтобы rho >= 0
        if rho < 0:
            n = -n
            rho = -rho
        return n, rho

    # --- RANSAC ---
    for _ in range(max_iters):
        i, j = rng.choice(N, size=2, replace=False)
        n, rho = model_from_two(P[i], P[j])
        if n is None:
            continue
        # расстояния до прямой: |n·p - rho|
        d = np.abs(P @ n - rho)
        inliers = d <= dist_thresh
        cnt = int(inliers.sum())
        if cnt > best_count:
            best_count = cnt
            best_inliers = inliers
            # быстрый выход, если уже отличный консенсус
            if best_count >= max(min_inliers, int(0.9 * N)):
                break

    if best_inliers is None or best_count < min_inliers:
        return None, None, None

    # --- Уточнение по инлаерам (total least squares / PCA) ---
    Q = P[best_inliers]
    # главная компонента = касательная к стене
    mu = Q.mean(axis=0)
    C = (Q - mu).T @ (Q - mu) / max(1, len(Q) - 1)
    eigvals, eigvecs = np.linalg.eigh(C)
    tangent = eigvecs[:, np.argmax(eigvals)]
    normal = np.array([-tangent[1], tangent[0]])
    normal /= (np.linalg.norm(normal) + 1e-12)

    # направление нормали — «от робота к стене» (т.е. чтобы rho >= 0)
    # для линии справедливо: n·p ≈ const -> возьмём среднее по инлаерам
    rho = float(np.mean(Q @ normal))
    if rho < 0:
        normal = -normal
        rho = -rho

    theta = float(np.arctan2(normal[1], normal[0]))

    if theta is not None and abs(theta) > np.deg2rad(max_wall_angle):
        return None, None, None

    return rho, theta, best_inliers

def remove_far_outliers(points: np.ndarray, dist_thresh: float):
    if len(points) < 2:
        return points

    tree = cKDTree(points)
    # расстояние до второго ближайшего соседа (k=2, k=1 — сама точка)
    d, _ = tree.query(points, k=2)
    min_dist = d[:, 1]

    return points[min_dist <= dist_thresh]

def filter_visible_2d(points, 
                      origin, 
                      radius=0.05, 
                      dist_tol=0.2):
    pts = np.asarray(points, float)
    o = np.asarray(origin, float)
    if pts.size == 0:
        return pts

    rel = pts - o
    dist = np.linalg.norm(rel, axis=1)
    ang = np.arctan2(rel[:, 1], rel[:, 0])

    safe_d = np.maximum(dist, 1e-12)
    alpha = np.arcsin(np.clip(radius / safe_d, -1.0, 1.0))

    N = len(pts)
    visible = np.ones(N, dtype=bool)

    # Обрабатываем точки от ближних к дальним
    order = np.argsort(dist)

    for i in order:
        if not visible[i]:
            continue
        d_i = dist[i]
        a_i = ang[i]
        alpha_i = alpha[i]

        dtheta = ang - a_i
        dtheta = (dtheta + np.pi) % (2.0 * np.pi) - np.pi

        farther = dist > d_i + dist_tol
        in_sector = np.abs(dtheta) <= alpha_i
        mask = farther & in_sector

        visible[mask] = False

    return pts[visible]


def voxel_downsample(points: np.ndarray, grid_size: float):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return points.copy()

    voxel_idx = np.floor(points / grid_size).astype(int)
    uniq, inv = np.unique(voxel_idx, axis=0, return_inverse=True)
    M = uniq.shape[0]  # число финальных вокселей

    # Считаем количество точек в каждом вокселе
    counts = np.bincount(inv, minlength=M)  # shape (M,)

    # Суммы координат по вокселям:
    # Для этого используем bincount по каждому измерению
    D = points.shape[1]
    sums = np.zeros((M, D), dtype=float)
    for d in range(D):
        sums[:, d] = np.bincount(inv, weights=points[:, d], minlength=M)

    # Центроид = сумма / количество
    out = sums / counts[:, None]
    return out


def mean_nearest_distance(a: np.ndarray, b: np.ndarray, median=False) -> float:
    if len(a) == 0 or len(b) == 0:
        return 0
    ta = cKDTree(b)
    tb = cKDTree(a)
    da = np.square(ta.query(a, k=1)[0])
    db = np.square(tb.query(b, k=1)[0])
    if median:
        return 0.5 * (np.median(da) + np.median(db))
    else:
        return np.sqrt((da.mean() + db.mean()) * 0.5)

def resample_lidar_by_distance(
    points: np.ndarray,
    step: float,
    max_gap: float | None = None,
) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points.copy()

    # длины отрезков между соседями
    diffs = points[1:] - points[:-1]
    seg_len = np.linalg.norm(diffs, axis=1)  # (N-1,)

    # маска "разрывов" — где расстояние слишком большое
    if max_gap is not None:
        breaks = seg_len > max_gap
    else:
        breaks = np.zeros_like(seg_len, dtype=bool)

    # индексы, где скан рвётся: сегменты [start_i, end_i]
    # разрыв между i и i+1 => сегмент заканчивается на i
    break_idx = np.where(breaks)[0]
    starts = np.concatenate([[0], break_idx + 1])
    ends   = np.concatenate([break_idx, [len(points) - 1]])

    out = []

    for s, e in zip(starts, ends):
        if e <= s:
            continue

        seg_pts = points[s:e+1]          # (K,2)
        seg_diffs = seg_pts[1:] - seg_pts[:-1]
        seg_seglen = np.linalg.norm(seg_diffs, axis=1)

        # кумулятивная длина вдоль сегмента
        cumlen = np.concatenate([[0.0], np.cumsum(seg_seglen)])  # (K,)

        total_len = cumlen[-1]
        if total_len <= 0:
            continue

        # новые "целевые" длины вдоль сегмента: 0, step, 2*step, ...
        num_new = int(np.floor(total_len / step)) + 1
        target_s = np.linspace(0.0, total_len, num_new)

        # интерполяция по длине дуги для x и y отдельно
        x = np.interp(target_s, cumlen, seg_pts[:, 0])
        y = np.interp(target_s, cumlen, seg_pts[:, 1])

        seg_resampled = np.stack([x, y], axis=1)
        out.append(seg_resampled)

    if not out:
        return points[:1].copy()

    return np.vstack(out)

def _smoothed_direction(path_pts, start_idx, max_points_ahead=5, decay=0.7):
    """
    Усреднённое направление по нескольким отрезкам вперёд.
    
    path_pts: (N, 2) — точки траектории
    start_idx: int — индекс отрезка, с которого начинаем (отрезок [i, i+1])
    max_points_ahead: сколько отрезков максимум смотреть вперёд
    decay: коэффициент затухания (0<decay<=1), чем меньше — тем локальнее усреднение.
    """
    N = path_pts.shape[0]
    # последний индекс отрезка, который можем взять (отрезок j — это [j, j+1], j <= N-2)
    last_seg_idx = min(N - 2, start_idx + max_points_ahead - 1)

    dir_vec = np.zeros(2, dtype=float)
    any_seg = False

    for j in range(start_idx, last_seg_idx + 1):
        seg = path_pts[j + 1] - path_pts[j]
        if np.allclose(seg, 0):
            continue
        w = decay ** (j - start_idx)  # 1, decay, decay^2, ...
        dir_vec += w * seg
        any_seg = True

    if not any_seg or np.allclose(dir_vec, 0):
        # запасной вариант — просто первый отрезок
        j = min(start_idx, N - 2)
        dir_vec = path_pts[j + 1] - path_pts[j]

    return dir_vec

def heading_and_crosstrack_error(
        path_pts,
        robot_pos,
        robot_heading,
        max_points_ahead=10,
        decay=0.8
    ):
    """
    То же самое, что раньше, но направление траектории берём как
    усреднённое по нескольким отрезкам вперёд.
    
    path_pts: np.ndarray (N, 2) — упорядоченный набор точек траектории
    robot_pos: np.ndarray (2,) — [x_r, y_r]
    robot_heading: float — угол робота в радианах
    max_points_ahead: int — сколько отрезков смотреть вперёд для усреднения
    decay: float — коэффициент затухания весов (0<decay<=1)
    
    return:
        angle_error: float — отклонение угла робота (рад, [-pi, pi))
        cross_track_error: float — поперечное отклонение (м) со знаком
    """
    path_pts = np.asarray(path_pts, dtype=float)
    robot_pos = np.asarray(robot_pos, dtype=float)

    if path_pts.shape[0] < 2:
        raise ValueError("Нужно минимум 2 точки траектории")

    best_dist2 = float("inf")
    best_seg_idx = None
    best_seg_vec = None
    best_Q = None

    for i in range(path_pts.shape[0] - 1):
        A = path_pts[i]
        B = path_pts[i + 1]
        v = B - A          # вектор отрезка
        w = robot_pos - A  # от A до робота

        vv = np.dot(v, v)
        if vv == 0:
            continue

        t = np.dot(w, v) / vv
        t_clamped = np.clip(t, 0.0, 1.0)

        Q = A + t_clamped * v
        diff = Q - robot_pos
        dist2 = np.dot(diff, diff)

        if dist2 < best_dist2:
            best_dist2 = dist2
            best_seg_idx = i
            best_seg_vec = v
            best_Q = Q

    if best_seg_idx is None or best_Q is None:
        raise RuntimeError("Не удалось найти корректный отрезок траектории")

    # Можно слегка "сдвинуть" стартовый индекс вперёд, если робот ближе к концу отрезка:
    # например, если проекция ближе к P_{i+1}, считаем, что мы уже "на" следующем отрезке.
    A = path_pts[best_seg_idx]
    B = path_pts[best_seg_idx + 1]
    v = B - A
    vv = np.dot(v, v)
    if vv > 0:
        t_along = np.dot(best_Q - A, v) / vv  # в [0,1]
        if t_along > 0.5 and best_seg_idx < path_pts.shape[0] - 2:
            start_idx_for_dir = best_seg_idx + 1
        else:
            start_idx_for_dir = best_seg_idx
    else:
        start_idx_for_dir = best_seg_idx

    # 1) Сглаженное направление пути
    smooth_vec = _smoothed_direction(
        path_pts,
        start_idx=start_idx_for_dir,
        max_points_ahead=max_points_ahead,
        decay=decay
    )
    path_angle = np.arctan2(smooth_vec[1], smooth_vec[0])
    angle_error = round_angle(path_angle - robot_heading)

    # 2) Поперечная ошибка (как раньше — относительно ближайшей точки best_Q)
    n = robot_pos - best_Q
    cross_track_dist = np.linalg.norm(n)

    if cross_track_dist == 0.0:
        cross_track_error = 0.0
    else:
        # знак через псевдо-crossproduct между локальным направлением best_seg_vec и n
        z = best_seg_vec[0] * n[1] - best_seg_vec[1] * n[0]
        sign = np.sign(z) if z != 0 else 0.0
        cross_track_error = sign * cross_track_dist

    return angle_error, cross_track_error
