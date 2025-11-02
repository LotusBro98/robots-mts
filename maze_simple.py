from chassis import RobotChassis
from driver import Driver
from emulation.emulated_robot import EmulatedRobot

# robot = EmulatedRobot(demo_render_navigator=False, file_rendering_navigator=True)  # Flags: demo_render_navigator, file_rendering_navigator
# robot.connect()
# driver = Driver(robot)

robot = RobotChassis()
robot.connect()
driver = Driver(robot)
driver.MAX_SPEED = 0.8
driver.MAX_ROT_SPEED = 0.8

try:
    while True:
        driver.drive_maze(
            front_wall_dist=0.15,
            right_wall_dist=0.20, 
            front_wall_smooth_stop_dist=0.5,
            side_wall_smooth_stop_dist=0.1
        )
finally:
    driver.stop()
    robot.disconnect()
    # robot.navigator.stop_rendering()
