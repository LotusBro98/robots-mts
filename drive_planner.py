import numpy as np
from driver import Driver


class RobotDrivePlanner:
    def __init__(self, driver: Driver):
        self.driver = driver
        self.navigator = driver.robot.navigator
        self.robot = driver.robot
    """
    использует Driver для вызова манёвров, Robot.recv_sensors для получения данных датчиков, Robot.navigator.
    Главная логика:
    - если точно тупик, разворачиваемся
    - если сейчас есть поворот направо -> направо
    - если впереди коридор, едем до поворота или на макс расстояние пока не станет видно дальше
    - если сейчас есть поворот налево -> налево
    """
    def calculate_next_drive(self):
        """
        Рассчитывает следующий манёвр для текущего положения робота.
        Наверное другой метод будет в цикле вызывать этот, и отправлять его на моторы.
        """
        args = ()
        return (self.driver.maze_forward, (args))

    @staticmethod
    def _calculate_distance_to_next_turn_in_corridor(points: np.ndarray) -> tuple[str, float]:
        """Возвращает расстояние в метрах до следующего поворота, и само направление - направо/налево"""
        distance = 0
        return distance


