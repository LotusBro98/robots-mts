#!/usr/bin/env python3
"""
Узнать индекс камеры глубины:
v4l2-ctl --list-devices

Измени DEPTH_CAMERA_INDEX
"""
import cv2
import numpy as np

# --- НАСТРОЙКИ ---

DEPTH_CAMERA_INDEX = 0  # номер камеры в системе (попробуй 0,1,2...)
OUTPUT_PATH = "cliff_line.jpg"

# какую часть кадра считать "полом" (берём нижнюю половину)
ROI_TOP_FRAC = 0.5      # верхняя граница ROI как доля высоты (0..1)
FLOOR_TOLERANCE = 1.2   # во сколько раз глубина может отличаться от средней по полу
CLIFF_FACTOR = 1.3      # глубина, превышающая floor_z * CLIFF_FACTOR, считается обрывом


def grab_depth_frame(cap):
    """Считывает один кадр глубины."""
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Не удалось считать кадр с камеры глубины")

    # Часто глубина приходит как 16UC1 (uint16). Если кадр 2-канальный/3-канальный,
    # придётся адаптировать под свой формат.
    if frame.dtype != np.uint16 and frame.dtype != np.uint8:
        print("Предупреждение: неожиданный тип кадра:", frame.dtype)

    return frame


def find_cliff_edges(depth):
    """
    Находит левый и правый края пола (обрыв) по одному кадру глубины.
    Возвращает координаты (x_left, x_right, y_top, y_bottom) в пикселях.
    """

    h, w = depth.shape[:2]

    # Берём только нижнюю часть кадра как область пола
    roi_top = int(h * ROI_TOP_FRAC)
    roi = depth[roi_top:h, :]

    # Игнорируем нулевые/максимальные значения как "нет данных"
    roi_valid = roi.astype(np.float32)
    roi_valid[roi_valid <= 0] = np.nan

    # По каждому столбцу берём медиану глубины (устойчиво к шуму)
    col_median = np.nanmedian(roi_valid, axis=0)  # shape: (w,)

    # Оценим среднюю глубину пола по всем валидным точкам
    floor_z = np.nanmedian(col_median)
    if np.isnan(floor_z):
        raise RuntimeError("Не удалось оценить плоскость пола (нет валидной глубины)")

    # Для наглядности: столбец считается «полом», если глубина близка к floor_z
    is_floor = (col_median > 0) & (col_median < floor_z * FLOOR_TOLERANCE)

    # Столбец считается «обрывом», если глубина резко больше средней (или NaN)
    is_cliff = np.isnan(col_median) | (col_median > floor_z * CLIFF_FACTOR)

    # Ищем слева/справа ближайший к центру столбец, где ПОЛ перестаёт быть полом
    cx = w // 2

    # Ищем левый край пола: идём от центра влево, пока пол; первый не-пол → край
    left = cx
    while left >= 0 and is_floor[left]:
        left -= 1
    x_left = max(left, 0)

    # Ищем правый край пола: от центра вправо
    right = cx
    while right < w and is_floor[right]:
        right += 1
    x_right = min(right, w - 1)

    # Для красоты рисовать будем вертикальные линии по всей ROI
    y_top = roi_top
    y_bottom = h - 1

    return x_left, x_right, y_top, y_bottom, is_floor, is_cliff


def main():
    cap = cv2.VideoCapture(DEPTH_CAMERA_INDEX, cv2.CAP_V4L2)

    if not cap.isOpened():
        raise RuntimeError(f"Не удалось открыть камеру с индексом {DEPTH_CAMERA_INDEX}")

    # Можно выставить нужное разрешение, если поддерживается:
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv2.CAP_PROP_FPS, 30)

    print("Считываю кадр глубины...")
    depth = grab_depth_frame(cap)
    cap.release()

    # Если кадр многоканальный – берём один канал
    if depth.ndim == 3:
        depth = depth[:, :, 0]

    # Находим края обрыва
    x_left, x_right, y_top, y_bottom, is_floor, is_cliff = find_cliff_edges(depth)

    # --- ВИЗУАЛИЗАЦИЯ И СОХРАНЕНИЕ ---

    # Нормализуем глубину в 8-бит, чтобы сохранить как картинку
    depth_float = depth.astype(np.float32)
    valid_mask = depth_float > 0
    if np.any(valid_mask):
        # нормализуем только по валидным значениям
        min_val = np.nanmin(depth_float[valid_mask])
        max_val = np.nanmax(depth_float[valid_mask])
    else:
        min_val, max_val = 0, 1

    depth_norm = (depth_float - min_val) / max(1e-6, (max_val - min_val))
    depth_norm[~valid_mask] = 0
    depth_8u = np.clip(depth_norm * 255, 0, 255).astype(np.uint8)

    # Переводим в цвет для более понятной картинки
    depth_color = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)

    h, w = depth.shape[:2]

    # Рисуем линию обрыва (левый и правый края)
    cv2.line(depth_color, (x_left, y_top), (x_left, y_bottom), (0, 0, 255), 2)
    cv2.line(depth_color, (x_right, y_top), (x_right, y_bottom), (0, 0, 255), 2)
    cv2.line(depth_color, (x_left, (y_top + y_bottom) // 2),
             (x_right, (y_top + y_bottom) // 2), (0, 255, 0), 2)

    # Дополнительно можно подсветить «обрывные» столбцы внизу кадра
    for x in range(w):
        if is_cliff[x]:
            depth_color[h - 5:h, x] = (255, 255, 255)

    cv2.imwrite(OUTPUT_PATH, depth_color)
    print(f"Картинка с линией обрыва сохранена в {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
