import time
import numpy as np

from chassis import RobotChassis
from driver import Driver

robot = RobotChassis()
robot.connect()
driver = Driver(robot)
driver.MAX_SPEED = 0.2
driver.MAX_ROT_SPEED = 0.02
driver.MAX_SPEED_ON_TURN = 0.05


try:
    # повернуться на определённый градус
    # print("Rotate 90")
    # driver.rotate(10)
    # driver.stop()
    # print("done")

    # # проехать заданное расстояние
    # print("Drive 0.5 m")
    # driver.drive(max_speed=0.1, max_dist=0.1)
    # driver.stop()
    # print("Done")

    # подъехать к стенке (не ближе заданного расстояния)
    # print("Drive until 0.5 m to wall")
    # driver.drive(max_speed=0.1, front_wall_dist=0.5)
    # driver.freeze()
    # print("Done")

    # driver.maze_forward((1, 0), max_speed=0.5)
    driver.maze_turn(-90)

    # # поездить вдоль стенки
    # print("Drive right wall 2 m")
    # driver.drive(right_wall_dist=0.5, max_dist=2, max_speed=0.5)
    # driver.freeze()
    # print("Done")


    # print("connect_robot")
    # robot.connect()
    # robot.start_sensor_capture()
    # print("robot connected")
    # time.sleep(5)
    # robot.send_drive(v=1, w=0)
    # print("Оба мотора едут прямо")
    # time.sleep(10)
    # robot.send_drive(v=0, w=0)
    # print("Оба остановлены")
    # time.sleep(5)
    # robot.send_drive(v=0.5, w=1)
    # print("Широкий поворот налево")
    # time.sleep(10)
    # robot.send_drive(v=0, w=0)
    # print("Оба остановлены")
    # time.sleep(5)
    # robot.disconnect()
    # print("disconnect_robot")
finally:
    driver.stop()
