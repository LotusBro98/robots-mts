from drive_planner import RobotDrivePlanner
from driver import Driver
from emulation.emulated_robot import EmulatedRobot

robot = EmulatedRobot(demo_render_navigator=False, file_rendering_navigator=True)  # Flags: demo_render_navigator, file_rendering_navigator
robot.connect()
driver = Driver(robot)
planner = RobotDrivePlanner(driver)

try:
    planner.main_loop()
finally:
    driver.stop()
    robot.disconnect()

# driver.maze_forward(0.25)
# driver.maze_turn(90)
# driver.maze_turn(-90)
# driver.maze_turn(90)
# driver.maze_turn(-90)
# driver.maze_forward(0.5)
# driver.maze_turn(-90)
# driver.maze_forward(0.5)
# driver.maze_turn(90)
# driver.maze_forward(3)
# driver.maze_turnaround()
# driver.maze_forward(3)