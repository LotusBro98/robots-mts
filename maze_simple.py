from chassis import RobotChassis
from driver import Driver
from emulation.emulated_robot import EmulatedRobot

robot = EmulatedRobot(demo_render_navigator=False, file_rendering_navigator=True)  # Flags: demo_render_navigator, file_rendering_navigator
robot.connect()
driver = Driver(robot)

# robot = RobotChassis()
# robot.connect()
# driver = Driver(robot)
# driver.MAX_SPEED = 1.0
# driver.MAX_ROT_SPEED = 1.0

try:
    while True:
        driver.drive_maze(
            front_wall_dist=0.20,
            right_wall_dist=0.275, 
            front_wall_smooth_stop_dist=1.0,
            side_wall_smooth_stop_dist=0.3
        )
        driver.freeze()
finally:
    driver.stop()
    robot.disconnect()
    robot.navigator.stop_rendering()
