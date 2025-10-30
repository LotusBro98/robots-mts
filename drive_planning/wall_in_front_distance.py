import numpy as np
import math

def front_clearance_simple(navigator,
                           Smax: float = 5.0,
                           front_half_width: float = 0.22,
                           min_pts: int = 6,
                           q: float = 0.10) -> tuple[float, dict]:
    """
    Оценивает расстояние до «стены спереди» очень просто и устойчиво:
    берём точки в коридоре |y|<=front_half_width, x>0 (СК робота) до Smax,
    и возвращаем 10-й перцентиль по x (а не минимум), чтобы не реагировать на выбросы.
    """
    # точки карты/скана в СК робота
    pts = navigator.get_relative_points(max_dist=Smax)
    if pts.size == 0:
        return math.inf, {"reason": "no_points"}

    x = pts[:, 0]
    y = pts[:, 1]
    mask = (x > 0.0) & (np.abs(y) <= front_half_width)
    if not np.any(mask):
        return math.inf, {"reason": "no_front_points"}

    xs = x[mask]
    if xs.size < min_pts:
        # мало точек — используем минимум, но пометим это
        return float(np.min(xs)), {"n": int(xs.size), "est": "min"}
    # робастно: нижний перцентиль по x
    d = float(np.quantile(xs, q))
    return d, {"n": int(xs.size), "est": f"q{int(q*100)}"}
