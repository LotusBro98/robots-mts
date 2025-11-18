import multiprocessing as mp
import signal
import threading
from typing import Callable, Tuple
import numpy as np
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder
from scipy.spatial import cKDTree
# import cv2 as cv

from pathfinding.core.grid import Grid

def points_to_grid(pts, robot_size=0.05, grid_size=0.05, additional_pts=(np.zeros((0, 2)))):
    pts = np.asarray(pts, float)
    additional_pts = np.stack(additional_pts, axis=0)

    minmax_pts = np.concatenate([pts, additional_pts], axis=0)
    xmin, ymin = np.min(minmax_pts, axis=0) - 2 * grid_size
    xmax, ymax = np.max(minmax_pts, axis=0) + 2 * grid_size

    nx = int((xmax - xmin)/grid_size) + 1
    ny = int((ymax - ymin)/grid_size) + 1

    grid = np.zeros((ny, nx), dtype=np.uint8)

    pts_grid = np.stack(np.meshgrid(
        np.linspace(xmin, xmax, nx), 
        np.linspace(ymin, ymax, ny),
    ), axis=-1)
    tree = cKDTree(pts)

    dists, idx = tree.query(pts_grid, k=1) 
    
    weights = (1 / ((dists - robot_size) / robot_size))

    # cv.imshow("grid", weights)
    # cv.waitKey(1)

    grid = Grid(matrix=weights)

    return grid, grid_size, xmin, ymin


def find_shortest_path(points, pos, goal):
    # Это почти твоя логика, только вынесенная в отдельную функцию
    grid, grid_size, xmin, ymin = points_to_grid(points, additional_pts=[pos, goal])

    pt_from = np.float32(pos)
    pt_to   = np.float32(goal)

    pt_from = np.int32(np.float32(pt_from - (xmin, ymin)) / grid_size)
    pt_to   = np.int32(np.float32(pt_to   - (xmin, ymin)) / grid_size)

    start = grid.node(*pt_from)
    end   = grid.node(*pt_to)

    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)

    if not path:
        return np.zeros((0, 2), dtype=np.float32)

    path = np.asarray([(n.x, n.y) for n in path], dtype=np.float32)
    path = path * grid_size + (xmin, ymin)
    return path


def _pathfinder_process(req_q: mp.Queue, res_q: mp.Queue):
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    # этот код крутится в отдельном ПРОЦЕССЕ → на другом ядре
    while True:
        points, pos, goal, stop = req_q.get(timeout=2)
        if stop:
            break
        path = find_shortest_path(points, pos, goal)
        res_q.put(path, timeout=2)


class Pathfinder:
    def __init__(self, input_cb: Callable[[], Tuple[np.ndarray, np.ndarray, np.ndarray]], output_cb: Callable[[np.ndarray], None]):
        self.input_cb = input_cb
        self.output_cb = output_cb

        self._req_q = mp.Queue()
        self._res_q = mp.Queue()

        self._proc = mp.Process(
            target=_pathfinder_process,
            daemon=True,
            args=[self._req_q, self._res_q]
        )
        self._proc.start()

        self._stop = False
        self._thr = threading.Thread(
            target=self._pathfinder_thread,
            daemon=True,
        )
        self._thr.start()

    def stop(self):
        self._stop = True
        self._thr.join(timeout=1)

    def _pathfinder_thread(self):
        while not self._stop:
            self._update_request(*self.input_cb(), False)
            self.output_cb(self._poll_result())
        self._update_request(None, None, None, True)
        self._proc.join(timeout=1)

    def _update_request(self, points, pos, goal, stop):
        # вызывать из основного процесса, когда появились новые points/pos/goal
        self._req_q.put((points, pos, goal, stop), timeout=2)

    def _poll_result(self) -> np.ndarray:
        # не блокируемся, просто забираем путь, если уже посчитали
        path = self._res_q.get(timeout=2)
        return path