# IMPORT FROM LIBRARIES
# * HAL (Hardware Abstraction Library) library class
# Contains the functions that sends and receives information to and from the hardware (Gazebo).
# * GUI (Graphical User Interface Class) library class
# Contains the functions used to view the debugging information, like image widgets.
# * OpenCV library class
# Contains functions that allow you to read/display images and apply different filters to them.
# * time module
# Provides functions related to time measurement and manipulation.
# * base module
# Import the base module from the OMPL (Open Motion Planning Library) library.
# * geometric module
# Import the geometric module from the OMPL (Open Motion Planning Library) library.
# * math module
# Contains standard math functions such as math.sqrt or math.pi.
# * sqrt function
# Import the sqrt (square root) function from the math module.
# * NumPy library class
# Contains functions that perform numerical calculations and array manipulations.
import HAL
import GUI
import cv2
import time
from ompl import base as ob
from ompl import geometric as og
import math
from math import sqrt
import numpy as np

warehouse = None
robot_size = 4

image = cv2.imread("/resources/exercises/amazon_warehouse/images/map.png", cv2.IMREAD_GRAYSCALE)
map_height, map_width = image.shape
warehouse = {
    'image': image,
    'map_height': map_height,
    'map_width': map_width,
    'real_width': 20.62,
    'real_height': 13.6,
    'x_scale_factor': 20.62 / map_width,
    'y_scale_factor': 13.6 / map_height
}

# init_planner() function
# Initialize the trajectory planner, set the workspace boundaries, defines the robot's state 
# space and state validity checks.
def init_planner(size):
    global space, si, pdef
    robot_size = size
    dimensions = [0, 0, warehouse['map_width'], warehouse['map_width']]
    space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, dimensions[0])
    bounds.setLow(1, dimensions[1])
    bounds.setHigh(0, dimensions[2])
    bounds.setHigh(1, dimensions[3])
    space.setBounds(bounds)
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    pdef = ob.ProblemDefinition(si)

# isStateValid() function
# Check if a specific position is free of obstacles.
def isStateValid(state):
    x = min(int(state.getX()), warehouse['map_width'] - 1)
    y = min(int(state.getY()), warehouse['map_height'] - 1)
    if warehouse['image'][y][x] <= 250:
        return False
    circle_radius = int(robot_size / 2)
    for px in range(max(0, x - circle_radius), min(warehouse['map_width'], x + circle_radius)):
        for py in range(max(0, y - circle_radius), min(warehouse['map_height'], y + circle_radius)):
            if (px - x) ** 2 + (py - y) ** 2 <= circle_radius ** 2:
                if warehouse['image'][py][px] == 0:
                    return False
    return True

# plan() function
# Plan a trajectory from the robot's current position to the target position by using a 
# route planning algorithm.
def plan():
    start = ob.State(space)
    start().setX(robot_position[0])
    start().setY(robot_position[1])
    start().setYaw(robot_position[2])
    goal = ob.State(space)
    goal().setX(target_position[0])
    goal().setY(target_position[1])
    goal().setYaw(target_position[2])
    pdef.setStartAndGoalStates(start, goal)
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
    planner = og.SORRTstar(si)
    planner.setRange(15)
    planner.setProblemDefinition(pdef)
    planner.setup()
    solved = planner.solve(1.0)
    if solved:
        sol_path = pdef.getSolutionPath()
        path = create_numpy_path(sol_path.printAsMatrix())
        return path
    return None

# create_numpy_path() function
# Converts the calculated trajectory into a coordinate array handled by numpy.
def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))
    for i in range(length):
        array[i][0] = round(float(lines[i].split(" ")[0]))
        array[i][1] = round(float(lines[i].split(" ")[1]))
    return array

# coordinates2pixels() function
# Converts coordinates in the global system to their equivalent in pixels.
def coordinates2pixels(coordinates, scale):
    pixel_coordinates = []
    for i in coordinates:
        x_pixel = round(int(-i[1] / scale[0]) + 209)
        y_pixel = round(int(-i[0] / scale[1]) + 140)
        pixel_coordinates.append((x_pixel, y_pixel))
    return pixel_coordinates

# pixels2coordinates() function
# Converts pixel coordinates to their equivalent in the global system.
def pixels2coordinates(pixels, scale):
    global_coordinates = []
    for i in pixels:
        x_global = round(-(i[1] - 140) * scale[1], 4)
        y_global = round(-(i[0] - 209) * scale[0], 4)
        global_coordinates.append((x_global, y_global))
    return global_coordinates

# absolute2relative() function
# Convert absolute coordinates to relative coordinates, taking as reference the position 
# and orientation of the robot.
def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    dx = x_abs - robotx
    dy = y_abs - roboty
    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)
    return x_rel, y_rel

# STATES
TRAJECTORY_DEFINITION = 1
TRAJECTORY_EXECUTION = 2
SHELF_MANIPULATION = 3

shelves = coordinates2pixels([(3.728,  0.579)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))         # SHELF 1
# shelves = coordinates2pixels([(3.728, -1.242)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))       # SHELF 2
# shelves = coordinates2pixels([(3.728, -3.039)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))       # SHELF 3
# shelves = coordinates2pixels([(3.728, -4.827)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))       # SHELF 4
# shelves = coordinates2pixels([(3.728, -6.781)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))       # SHELF 5
# shelves = coordinates2pixels([(3.728, -8.665)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))       # SHELF 6

final_position = coordinates2pixels([(0,  0.579)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))      # SHELF 1
# final_position = coordinates2pixels([(0, -1.242)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))    # SHELF 2
# final_position = coordinates2pixels([(0, -3.039)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))    # SHELF 3
# final_position = coordinates2pixels([(0, -4.827)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))    # SHELF 4
# final_position = coordinates2pixels([(0, -6.781)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))    # SHELF 5
# final_position = coordinates2pixels([(0, -8.665)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))    # SHELF 6

state = TRAJECTORY_DEFINITION
carrying_shelf, achieved_local_target, achieved_global_target, robot_aligned = False, False, False, False
targets_counter, paths_counter = 0, 2

while True:

    # CURRENT POSITION (X,Y) AND ORIENTATION OF THE ROBOT
    robot_x, robot_y, robot_yaw = HAL.getPose3d().x, HAL.getPose3d().y, HAL.getPose3d().yaw

    # TRAJECTORY_DEFINITION STATE
    # In this state, the robot generates a trajectory from its current position to a target 
    # position, which can be the position of a shelf, if not is lifting none, or the unloading
    # position, in case the robot is carrying a shelf at that time.
    if state == TRAJECTORY_DEFINITION:
        init_planner(4)
        robot_pixel_coordinates = coordinates2pixels([(robot_x, robot_y)], (warehouse['x_scale_factor'], warehouse['y_scale_factor']))
        robot_position = (robot_pixel_coordinates[0][0], robot_pixel_coordinates[0][1], robot_yaw)
        if not carrying_shelf:
            target_position = (shelves[targets_counter][0], shelves[targets_counter][1], np.pi)
            targets_counter += 1
        else:
            target_position = (final_position[0][0], final_position[0][1], np.pi)
        path = plan()
        GUI.showPath(path)
        path = pixels2coordinates(path, (warehouse['x_scale_factor'], warehouse['y_scale_factor']))
        time.sleep(5)
        state = TRAJECTORY_EXECUTION

    # TRAJECTORY_EXECUTION STATE
    # In this state, the robot follows the planned trajectory in the previous state. If it has been 
    # reached the local target, the robot advances to the next point on the route. On the other hand,
    # if the global goal has been reached, the robot stops and changes state. And finally, if the 
    # robot is aligned and without a shelf, it goes to the next state.
    elif state == TRAJECTORY_EXECUTION:
        print("FOLLOWING TRAJECTORY ...")
        if achieved_local_target:
            achieved_local_target = not achieved_local_target
            paths_counter += 1
        if paths_counter >= len(path):
            paths_counter = 2
            achieved_global_target = not achieved_global_target
            HAL.setV(0)
            HAL.setW(0)
            print("TARGET REACHED")
            time.sleep(5)
        if not achieved_global_target:
            local_target_coordinates = absolute2relative(path[paths_counter][0], path[paths_counter][1], robot_x, robot_y, robot_yaw)
            if math.sqrt(local_target_coordinates[0] ** 2 + local_target_coordinates[1] ** 2) <= 0.075:
                achieved_local_target = not achieved_local_target
            HAL.setV(0.06)
            HAL.setW(math.atan2(local_target_coordinates[1], local_target_coordinates[0]) * 0.35)
        else:
            if not carrying_shelf:
                if robot_aligned:
                    HAL.setW(0)
                    robot_aligned = not robot_aligned
                    achieved_global_target = not achieved_global_target
                    state = SHELF_MANIPULATION
                else:
                    if abs((np.pi - robot_yaw) * 0.1) < 0.025:
                        HAL.setW(0)
                        robot_aligned = not robot_aligned
                HAL.setV(0)
                HAL.setW((np.pi - robot_yaw) * 0.1)
            else:
                state = SHELF_MANIPULATION

    # SHELF_MANIPULATION STATE
    # In this state, the robot manipulates the shelf depending on the position it is in. If the 
    # robot has a raised shelf, it releases it and waits 5 minutes, if this is the case. On 
    # the contrary, he lifts it up. Finally, once this action is carried out, it returns to the 
    # first state to plan the next route.
    elif state == SHELF_MANIPULATION:
        if carrying_shelf:
            print("PUTTING DOWN SHELF ...")
            HAL.putdown()
            time.sleep(300)
        else:
            print("LIFTING SHELF ...")
            HAL.lift()
        time.sleep(5)
        carrying_shelf = not carrying_shelf
        state = TRAJECTORY_DEFINITION