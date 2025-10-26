import numpy as np
from driver import Driver
from emulation.emulated_robot import EmulatedRobot

robot = EmulatedRobot()
robot.connect()
driver = Driver(robot)
driver.update_ref_angle()

try:
    while True:
        driver.drive_maze(
            max_speed=0.5,
            front_wall_dist=0.25,
            right_wall_dist=0.25, 
            front_wall_smooth_stop_dist=1.2,
            side_wall_smooth_stop_dist=0.4
        )
        driver.freeze()
finally:
    robot.disconnect()
