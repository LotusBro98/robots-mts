import numpy as np
import cv2 as cv
import os
import time

# === НАСТРОЙКИ ===
CAMERA_INDEX = 0                  # индекс камеры (обычно 0)
OUTPUT_DIR = "calib_frames"       # папка для сохранения кадров
PATTERN_SIZE = (6, 6)             # (кол-во внутренних углов по колонкам, по строкам)
SQUARE_SIZE = 1.0                 # условный размер клетки (можно потом поменять на метры)
TARGET_FRAMES = 30                # сколько УСПЕШНЫХ кадров (с найденной доской) нужно сохранить
TARGET_FPS = 2                    # "логический" FPS, чтобы не снимать слишком часто

# termination criteria для cornerSubPix
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 3D-точки шахматной доски в её системе координат
objp = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3D точки (для калибровки)
imgpoints = []  # 2D точки (для калибровки)

os.makedirs(OUTPUT_DIR, exist_ok=True)

cap = cv.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError(f"Не удалось открыть камеру с индексом {CAMERA_INDEX}")

# Попробуем запросить низкий FPS (может игнорироваться драйвером)
cap.set(cv.CAP_PROP_FPS, TARGET_FPS)

saved_idx = 0
frame_idx = 0
last_time = time.time()

print(f"Нужно собрать {TARGET_FRAMES} успешных кадров с шахматной доской.")
print("Начинаю съёмку... (Ctrl+C для принудительной остановки)")

try:
    while saved_idx < TARGET_FRAMES:
        # ограничиваем частоту захвата кадров
        now = time.time()
        elapsed = now - last_time
        min_dt = 1.0 / TARGET_FPS
        if elapsed < min_dt:
            time.sleep(min_dt - elapsed)
        last_time = time.time()

        ret, img = cap.read()
        if not ret:
            print("Камера перестала отдавать кадры. Останавливаюсь.")
            break

        frame_idx += 1

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # поиск шахматной доски
        ret_cb, corners = cv.findChessboardCorners(gray, PATTERN_SIZE, None)

        if ret_cb:
            # уточняем координаты углов
            corners2 = cv.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria
            )

            objpoints.append(objp)
            imgpoints.append(corners2)

            # рисуем углы на копии кадра
            display_img = img.copy()
            cv.drawChessboardCorners(display_img, PATTERN_SIZE, corners2, ret_cb)

            save_path = os.path.join(OUTPUT_DIR, f"calib_{saved_idx:03d}.png")
            cv.imwrite(save_path, display_img)
            saved_idx += 1

            remaining = TARGET_FRAMES - saved_idx
            print(f"[{saved_idx}/{TARGET_FRAMES}] Сохранил кадр: {save_path}. Осталось: {remaining}")
        else:
            # Можно иногда писать, что доска не найдена, но чтобы не спамить — пропустим
            pass

finally:
    cap.release()

print(f"\nСъёмка завершена. Успешных кадров: {saved_idx}")

# Калибровка, если набралось достаточно кадров
if saved_idx >= 5:
    # предполагаем, что последний gray ещё существует
    img_h, img_w = gray.shape[:2]
    print("Запускаю калибровку камеры...")

    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, (img_w, img_h), None, None
    )

    print("\nКалибровка выполнена.")
    print("Средняя ошибка реконструкции (reprojection error):", ret)
    print("Матрица камеры (K):\n", cameraMatrix)
    print("Коэффициенты дисторсии:", distCoeffs.ravel())

    fs = cv.FileStorage("camera_calib.yaml", cv.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", cameraMatrix)
    fs.write("dist_coeffs", distCoeffs)
    fs.release()
    print("Параметры камеры сохранены в файл camera_calib.yaml")
else:
    print("Слишком мало успешных кадров для калибровки. Собери хотя бы 5–10.")
