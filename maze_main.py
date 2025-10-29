from driver import Driver
from emulation.emulated_robot import EmulatedRobot

robot = EmulatedRobot(demo_render_navigator=False, file_rendering_navigator=True)  # Flags: demo_render_navigator, file_rendering_navigator
robot.connect()
driver = Driver(robot)

driver.maze_forward((0.25, 0))
driver.maze_turn(90)
driver.maze_turn(-90)
driver.maze_turn(90)
driver.maze_turn(-90)
driver.maze_forward((0.5, 0))
driver.maze_turn(-90)
driver.maze_forward((0.5, 0))
driver.maze_turn(90)
driver.maze_forward((3.0, 0))