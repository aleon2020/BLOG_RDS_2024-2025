# IMPORT FROM LIBRARIES
# * HAL (Hardware Abstraction Library) library class
# Contains the functions that sends and receives information to and from the hardware (Gazebo).
# * GUI (Graphical User Interface Class) library class
# Contains the functions used to view the debugging information, like image widgets.
# * NumPy library class
# Contains functions that perform numerical calculations and array manipulations.
# * math module
# Contains standard math functions such as math.sqrt or math.pi.
# * queue module
# Contains functions for adding/removing items and synchronization between threads.
import HAL
import GUI
import numpy as np
import math 
from queue import Queue

# INITIALIZATION OF VARIABLES
grid_cells = []
obstacle_cells = []
return_cells = []
critic_cells = []

# world_to_pixel_transformation() function
# Converts real world coordinates (x,y) to pixel coordinates on the map.
def world_to_pixel_transformation(x_world_coordinate, y_world_coordinate):
  transformed_pixel_coordinates = np.dot(np.array([[-4.77221256e-02,  1.03843339e+02,  4.12187318e+02], [-1.00920862e+02, -6.30253600e-01,  5.73125438e+02]]), np.array([[x_world_coordinate], [y_world_coordinate], [1]]))
  return transformed_pixel_coordinates[0], transformed_pixel_coordinates[1]

# pixel_to_world_transformation() function
# Converts pixel coordinates on the map to real-world coordinates (x,y).
def pixel_to_world_transformation(x_pixel_coordinate, y_pixel_coordinate):
  transformed_world_coordinates = np.dot(np.array([[-6.01387637e-05, -9.90872563e-03,  5.70373116e+00], [ 9.62986306e-03, -4.55364259e-06, -3.96669762e+00]]), np.array([[x_pixel_coordinate], [y_pixel_coordinate], [1]]))
  return transformed_world_coordinates[0], transformed_world_coordinates[1]

# is_obstacle_cell() function
# Check whether a cell is an obstacle or not.
def is_obstacle_cell(target_cell):
  if target_cell in obstacle_cells:
    return False
  else:
    return True

# find_current_cell_position() function
# Gets the current position in pixels where the robot is located.
def find_current_cell_position(current_cell):
  x_rounded_coordinate = np.around(current_cell[0], 0)
  y_rounded_coordinate = np.around(current_cell[1], 0)
  for cell in grid_cells:
    for (x, y) in cell:
      if x == x_rounded_coordinate and y == y_rounded_coordinate:
        return cell
  return None

# get_current_cell_neighbors() function
# Returns the neighboring cells of the current cell.
def get_current_cell_neighbors(current_cell):
  neighbors_list = []
  neighbors_list.append(find_current_cell_position((current_cell[0][0], current_cell[0][1] - 33)))
  neighbors_list.append(find_current_cell_position((current_cell[0][0] - 33, current_cell[0][1])))
  neighbors_list.append(find_current_cell_position((current_cell[0][0] + 33, current_cell[0][1])))
  neighbors_list.append(find_current_cell_position((current_cell[0][0], current_cell[0][1] + 33)))
  return neighbors_list

# compute_cleaning_path() function
# Calculates the robot's cleaning route by searching for free cells and avoiding obstacles.
def compute_cleaning_path(actual_cell):
  cleaning_path = []
  previous_direction = None
  while actual_cell is not None:
    for x, y in actual_cell:
      if actual_cell in critic_cells:
        navigation_gridmap[x][y] = 130
      else:
        navigation_gridmap[x][y] = 132
    GUI.showNumpy(navigation_gridmap)
    obstacle_cells.append(actual_cell)
    neighbors = get_current_cell_neighbors(actual_cell)
    for neighbor in neighbors:
      if neighbor != actual_cell or neighbor not in return_cells or is_obstacle_cell(neighbor):
        return_cells.append(neighbor)
    target_cell = None
    target_direction = None
    if is_obstacle_cell(neighbors[0]):
      target_cell = neighbors[0]
      target_direction = "left"
    elif is_obstacle_cell(neighbors[1]):
      target_cell = neighbors[1]
      target_direction = "up"
    elif is_obstacle_cell(neighbors[2]):
      target_cell = neighbors[2]
      target_direction = "down"
    elif is_obstacle_cell(neighbors[3]):
      target_cell = neighbors[3]
      target_direction = "right"
    else:
      nearest_cell = None
      minimum_distance = 1000
      for cell in return_cells:
        if cell == actual_cell or is_obstacle_cell(cell) == False:
          continue
        distance = math.sqrt((cell[(len(cell) - 1) // 2][0] - actual_cell[(len(actual_cell) - 1) // 2][0]) ** 2 + (cell[(len(cell) - 1) // 2][1] - actual_cell[(len(actual_cell) - 1) // 2][1]) ** 2)
        if distance < minimum_distance:
          nearest_cell = cell
          minimum_distance = distance
      target_cell = nearest_cell
      if target_cell not in cleaning_path and target_cell not in critic_cells:
        cleaning_path.append(actual_cell)
        cleaning_path.append(target_cell)
        critic_cells.append(target_cell)
    if target_direction != previous_direction:
      if actual_cell not in cleaning_path:
        cleaning_path.append(actual_cell)
      previous_direction = target_direction
    actual_cell = target_cell
  return cleaning_path

# execute_motion_control() function
# Controls the robot's movement toward a target position by adjusting speeds and turns.
def execute_motion_control(destination):
  heading_error = math.atan2(pixel_to_world_transformation(destination[0], destination[1])[1] - HAL.getPose3d().y, pixel_to_world_transformation(destination[0], destination[1])[0] - HAL.getPose3d().x) - HAL.getPose3d().yaw
  while abs(heading_error) > 0.018:
    if heading_error > 0:
      HAL.setW(0.2)
    else:
      HAL.setW(-0.2)
    heading_error = math.atan2(pixel_to_world_transformation(destination[0], destination[1])[1] - HAL.getPose3d().y, pixel_to_world_transformation(destination[0], destination[1])[0] - HAL.getPose3d().x) - HAL.getPose3d().yaw
  HAL.setW(0)
  previous_heading_error = 0
  distance = math.sqrt((pixel_to_world_transformation(destination[0], destination[1])[0] - HAL.getPose3d().x) ** 2 + (pixel_to_world_transformation(destination[0], destination[1])[1] - HAL.getPose3d().y) ** 2)
  while distance > 0.1:
    distance = math.sqrt((pixel_to_world_transformation(destination[0], destination[1])[0] - HAL.getPose3d().x) ** 2 + (pixel_to_world_transformation(destination[0], destination[1])[1] - HAL.getPose3d().y) ** 2)
    error = math.atan2(pixel_to_world_transformation(destination[0], destination[1])[1] - HAL.getPose3d().y, pixel_to_world_transformation(destination[0], destination[1])[0] - HAL.getPose3d().x) - HAL.getPose3d().yaw
    derivative_error = (error - previous_heading_error) / 0.1
    previous_heading_error = error
    angular_velocity = 0.6 * error + 0.5 * derivative_error
    if angular_velocity < 0.3 and angular_velocity > -0.3:
      HAL.setW(angular_velocity)
    else:
      HAL.setW(0)
    HAL.setV(0.2)
  HAL.setV(0)
  HAL.setW(0)
  return

# compute_cell_path() function
# Implements the BFS algorithm to find the best path between two cells.
def compute_cell_path(start, finish):
  search_queue = Queue(maxsize = 0) 
  visited_cells = [] 
  search_queue.put((start, []))
  while not search_queue.empty():
    node, current_path = search_queue.get()
    if node[0] == finish[0] and node[1] == finish[1]:
      return current_path
    if not any((x[0] == node[0] and x[1] == node[1]) for x in visited_cells): 
      visited_cells.append(node)
      for child in get_current_cell_neighbors(node):
        if child is None:
          continue
        if not any((x[0] == child[0] and x[1] == child[1]) for x in visited_cells):
          search_queue.put((child, current_path + [child] ))
  return current_path

array = GUI.getMap('/resources/exercises/vacuum_cleaner_loc/images/mapgrannyannie.png')
GUI.showNumpy(np.zeros((array.shape[0], array.shape[1])))
navigation_gridmap = np.zeros((array.shape[0], array.shape[1]))

for x in range(0, array.shape[0], 33):
  for y in range(0, array.shape[1], 33):
    cell = []
    for i in range(x, min(x + 33, array.shape[0])):
      for j in range(y, min(y + 33, array.shape[1])):
        cell.append((i, j))
    grid_cells.append(cell)

for cell in grid_cells:
  for x, y in cell:
    if (array[x][y] == 0).any():
      if cell in obstacle_cells:
        continue
      else:
        obstacle_cells.append(cell)
    else:
      navigation_gridmap[x][y] = 127

for cell in obstacle_cells:
  for x, y in cell:
    navigation_gridmap[x][y] = 128

actual_cell = find_current_cell_position(world_to_pixel_transformation(HAL.getPose3d().x, HAL.getPose3d().y))

for x, y in actual_cell:
  navigation_gridmap[x][y] = 134

GUI.showNumpy(navigation_gridmap)

for cell in compute_cleaning_path(actual_cell):
  if cell in critic_cells:
    critic_cells.remove(cell)
    for path_cell in compute_cell_path(find_current_cell_position(world_to_pixel_transformation(HAL.getPose3d().x, HAL.getPose3d().y)), cell):
      execute_motion_control(path_cell[(len(path_cell) - 1) // 2])
  else:
    execute_motion_control(cell[(len(cell) - 1) // 2])
      
while True:
    print("FULL HOUSE CLEANED SUCCESSFULLY!")
