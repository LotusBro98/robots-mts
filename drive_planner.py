import numpy as np
from driver import Driver


class RobotDrivePlanner:
    def __init__(self, driver: Driver):
        self.driver = driver
        self.navigator = driver.robot.navigator
        self.robot = driver.robot

        self.DRIVES = [
            (self.driver.maze_forward, (0.25,)),
            (self.driver.maze_turn, (90,)),
            (self.driver.maze_turn, (-90,)),
            (self.driver.maze_turn, (90,)),
            (self.driver.maze_turn, (-90,)),
            (self.driver.maze_forward, (0.5,)),
            (self.driver.maze_turn, (-90,)),
            (self.driver.maze_forward, (0.5,)),
            (self.driver.maze_turn, (90,)),
            (self.driver.maze_forward, (3,)),
            (self.driver.maze_turnaround, ()),
            (self.driver.maze_forward, (3,)),
        ]
        

    """
    использует Driver для вызова манёвров, Robot.recv_sensors для получения данных датчиков, Robot.navigator.
    Главная логика:
    - если точно тупик, разворачиваемся
    - если сейчас есть поворот направо -> направо
    - если впереди коридор, едем до поворота или на макс расстояние пока не станет видно дальше
    - если сейчас есть поворот налево -> налево
    """
    cnt = 0
    def calculate_next_drive(self):
        """
        Рассчитывает следующий манёвр для текущего положения робота.
        """
        ret = self.DRIVES[self.cnt] + ({},)
        self.cnt += 1
        return ret
    
    def main_loop(self):
        while True:
            next_drive, args, kwargs = self.calculate_next_drive()
            next_drive(*args, **kwargs)
            if self.cnt >= len(self.DRIVES):
                break

    @staticmethod
    def _calculate_distance_to_next_turn_in_corridor(points: np.ndarray) -> tuple[str, float]:
        """Возвращает расстояние в метрах до следующего поворота, и само направление - направо/налево"""
        distance = 0
        return distance


