import multiprocessing as mp
import numpy as np
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder

from navigator_utils import points_to_grid


def find_shortest_path_worker(points, pos, goal):
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


class Pathfinder:
    def __init__(self):
        self.points = None
        self.pos    = (0.0, 0.0)
        self.goal   = (2.0, 2.0)
        self.path   = np.zeros((0, 2), dtype=np.float32)

        self._req_q = mp.Queue()
        self._res_q = mp.Queue()

        self._proc = mp.Process(
            target=self._pathfinder_process,
            daemon=True
        )
        self._proc.start()

    def _pathfinder_process(self):
        # этот код крутится в отдельном ПРОЦЕССЕ → на другом ядре
        while True:
            points, pos, goal = self._req_q.get()
            path = find_shortest_path_worker(points, pos, goal)
            self._res_q.put(path)

    def update_request(self, points, pos, goal):
        # вызывать из основного процесса, когда появились новые points/pos/goal
        self._req_q.put((points, pos, goal))

    def poll_result(self) -> np.ndarray:
        # не блокируемся, просто забираем путь, если уже посчитали
        path = self._res_q.get()
        return path