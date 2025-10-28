import numpy as np
from scipy.spatial import cKDTree

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

def estimate_update_point_to_line_robust(
    pts_from, pts_to, center=None, k_normals=8,
    huber_delta=0.10, reg_tangential=1e-3,
    # робаст-пороги:
    min_pts=20,                   # минимальное число пар
    min_cond=1e-3,                # минимум отношения s_min/s_max для (A^T W A)
    min_obs_rot=1e-3,             # минимум наблюдаемости угла (сумма |n^T J p|)
    min_obs_trans=1e-2,           # минимум наблюдаемости трансляции (сумма ||n||; в 2D ~число точек)
    soft_clip_pos=0.2,            # мягкий лимит нормы dpos (м) за шаг
    soft_clip_th=np.deg2rad(5.0), # мягкий лимит |dθ| за шаг
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

    # наблюдаемость: трансляция и угол
    obs_rot = np.sum(np.abs(Aw[:, 2]))          # n^T Jp вклад по углу
    obs_trans = np.linalg.norm(Aw[:, :2], 'fro')  # суммарная «сила» по tx,ty

    # условность
    H = Aw.T @ Aw  # нормальные уравнения
    s = np.linalg.svd(H, compute_uv=False)
    cond = s[-1] / (s[0] + 1e-12) if s[0] > 0 else 0.0

    info.update(N=N, obs_rot=obs_rot, obs_trans=obs_trans, cond=cond)

    # жёсткие отказы (совсем плохо) — нулевой апдейт
    if (obs_rot < min_obs_rot and obs_trans < min_obs_trans) or cond < min_cond:
        info["reason"] = "unobservable_or_ill_conditioned"
        return np.zeros(2), 0.0, 0.0, info

    # мягкий «gain» по трём факторам качества: количество, условность, наблюдаемость
    qN   = np.clip((N - min_pts) / (3*min_pts), 0.0, 1.0)        # от 0 к 1 по мере роста N
    qC   = np.clip((cond - min_cond) / (1.0 - min_cond), 0.0, 1.0)  # лучше при большем cond
    qObs = np.clip(min(obs_rot/(10*min_obs_rot), 1.0) * min(obs_trans/(10*min_obs_trans), 1.0), 0.0, 1.0)
    quality = float((qN * qC * qObs) ** (1/2))   # можно настроить степень

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

    # применяем адаптивный gain (0..1)
    tx *= quality
    ty *= quality
    dth *= quality

    return np.array([tx, ty]), float(dth), quality, info


def normalize(vec: np.ndarray) -> np.ndarray:
    return vec / np.linalg.norm(vec, axis=-1, keepdims=True)


def project_scalar(base: np.ndarray, vec: np.ndarray) -> float:
    return (normalize(base) * vec).sum(-1)

