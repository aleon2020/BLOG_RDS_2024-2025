# IMPORT FROM LIBRARIES
# * HAL (Hardware Abstraction Library) library class
# Contains the functions that sends and receives information to and from the hardware (Gazebo).
# * GUI (Graphical User Interface Class) library class
# Contains the functions used to view the debugging information, like image widgets.
# * NumPy library class
# Contains functions that perform numerical calculations and array manipulations.
# * math module
# Contains standard math functions such as math.sqrt or math.pi.
# * OpenCV library class
# Contains functions that allow you to read/display images and apply different filters to them.
import HAL
import GUI
import numpy as np
import math
import cv2

# ROBOT ORIENTATIONS
WEST_ORIENTATION = 0
NORTH_ORIENTATION = - math.pi / 2
EAST_ORIENTATION = -math.pi
EAST_LEFT_ORIENTATION = math.pi
SOUTH_ORIENTATION = math.pi / 2

# Cell Class
class Cell:

  # __init__() function
  # Defines a map cell, as well as its coordinates, state, and neighboring cells.
  def __init__(self, x_cell_center, y_cell_center, cell_row, cell_column):
    self.row = cell_row
    self.column = cell_column
    self.center_x = x_cell_center
    self.center_y = y_cell_center
    self.obstacle = False
    self.visited = False
    self.north_neighbor = 0
    self.east_neighbor = 0
    self.south_neighbor = 0
    self.west_neighbor = 0

  # visited_cell() function
  # Mark a cell as 'already visited'.
  def visited_cell(self):
    self.visited = True
  
  # obstacle_cell() function
  # Mark a cell as an obstacle.
  def obstacle_cell(self):
    self.obstacle = True

# image_conversion() function
# Converts the map image to a numerical format so that it can be processed.
# To do this, the original map is resized and a new empty one is initialized,
# marking red (132) those pixels that are obstacles and white (127) those that are not.
def image_conversion(old_map):
  raw_map = cv2.resize(old_map, (512, 512))
  new_map = np.zeros((raw_map.shape[0], raw_map.shape[1]))
  for row in range(new_map.shape[0]):
    for column in range(new_map.shape[1]):
      if np.any(raw_map[row, column, :3]) == 1.0:
        new_map[row, column] = 127
      else:
        new_map[row, column] = 132
  return new_map

# generate_cell_matrix() function
# Generate a cell array based on the map, in addition to creating a cell
# for each initial center position in both X and Y.
def generate_cell_matrix(raw_map):
  row, column = raw_map.shape
  cells_array = [[None] * (column // 17) for _ in range(row // 17)]
  x_cell_center = 8
  y_cell_center = 8
  for i in range(row // 17):
    x_cell_center = 8
    for j in range(column // 17):
      cells_array[i][j] = Cell(x_cell_center, y_cell_center, i, j)
      x_cell_center += 17
    y_cell_center += 17
  return cells_array

# set_obstacle_cell() function
# Mark the cell as an obstacle if it is red (132).
def set_obstacle_cell(cell):
  corner_x = cell.center_x - 8
  corner_y = cell.center_y - 8
  for i in range(17):
    for j in range(17):
      if array[i + corner_y, j + corner_x] == 132:
        cell.obstacle_cell()
        break
    if array[i + corner_y, j + corner_x] == 132:
      break

# mark_visited_cell() function
# Mark the cell blue (128) if it has already been visited, so
# can no longer pass through there and is considered an obstacle.
def mark_visited_cell(cell):
  corner_x = cell.center_x - 8
  corner_y = cell.center_y - 8
  for i in range(17):
    for j in range(17):
      array[i + corner_y, j + corner_x] = 128

# mark_obstacle_cell() function
# Mark the red cell (132), so it is considered an obstacle
# and the robot can no longer pass through there.
def mark_obstacle_cell(cell):
  corner_x = cell.center_x - 8
  corner_y = cell.center_y - 8
  for i in range(17):
    for j in range(17):
      array[i + corner_y, j + corner_x] = 132

# mark_critic_cell() function
# Mark the cell yellow (130) if it is in the one called critical zone, so it redirects its path.
def mark_critic_cell(cell):
  corner_x = cell.center_x - 8
  corner_y = cell.center_y - 8
  for i in range(17):
    for j in range(17):
      array[i + corner_y, j + corner_x] = 130

# set_cell_neighbors() function
# Sets the neighbors of each cell based on its position in an array of cells, 
# omitting those that are obstacles.
def set_cell_neighbors(cells):
  for row in range(len(cells)):
    for column in range(len(cells)):
      if (cells[row][column].obstacle == True):
        continue
      else:
        cells[row][column].north_neighbor = cells[row - 1][column]
        cells[row][column].south_neighbor = cells[row + 1][column]
        cells[row][column].east_neighbor = cells[row][column + 1]
        cells[row][column].west_neighbor = cells[row][column - 1]

# map_to_cell_conversion() function
# Converts the map coordinates to their equivalent in the cell.
def map_to_cell_conversion(map_x, map_y):
  cell_x = round(-2.9555517292899 * map_x + 16.6514905522259)
  cell_y = round(3.0048943492781 * map_y + 11.6877761275171)
  return cells[cell_y][cell_x]

# cell_to_map_conversion() function
# Converts the cell coordinates to their equivalent on the map.
def cell_to_map_conversion(cell_x, cell_y):
  map_x = (cell_x - 16.6514905522259) / (-2.9555517292899)
  map_y = (cell_y - 11.6877761275171) / (3.0048943492781)
  return map_x, map_y

# update_neighboring_cells() function
# Save the neighbors of the cells already visited and the unvisited cells in the neighbor list.
def update_neighboring_cells(cell):
  neighbors_list = [cell.north_neighbor, cell.east_neighbor, cell.south_neighbor, cell.west_neighbor]
  for neighbor in neighbors_list:
    if neighbor in free_cells and not neighbor.visited and neighbor not in visited_neighbors: 
      visited_neighbors.append(neighbor)
  if cell in visited_neighbors:
    visited_neighbors.remove(cell)

# euclidian_points_distance() function
# Calculate the Euclidean distance between two points.
def euclidian_points_distance(a, b):
  distance = math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
  return distance

# euclidian_cells_distance() function
# Calculates the Euclidean distance between two cells.
def euclidian_cells_distance(origin_distance, destination_distance):
  destination_x, destination_y = cell_to_map_conversion(destination_distance.column, destination_distance.row) 
  origin_x, origin_y = cell_to_map_conversion(origin_distance.column, origin_distance.row)
  distance = euclidian_points_distance([destination_x, destination_y], [origin_x, origin_y])
  return distance

# find_nearest_free_cell() function
# Find the nearest cell that has not been visited.
def find_nearest_free_cell(reference_cell, cells_list):
  distance = 1000
  destiny = None
  for cell in cells_list:
    if cell in free_cells and euclidian_cells_distance(cell, reference_cell) < distance:
      distance = euclidian_cells_distance(reference_cell, cell)
      destiny = cell
  if destiny is not None:
    mark_critic_cell(destiny)
  return destiny, distance

# calculate_next_step() function
# Find the shortest path from the cell's neighbors.
def calculate_next_step(current_cell, destiny):
  neighbors_list = [current_cell.north_neighbor, current_cell.east_neighbor, current_cell.south_neighbor, current_cell.west_neighbor]
  next_cell, _ = find_nearest_free_cell(destiny, neighbors_list)
  if next_cell is None:
    return None, None
  current_cell = map_to_cell_conversion(HAL.getPose3d().x, HAL.getPose3d().y)
  if current_cell.row > next_cell.row:
    next_orientation = NORTH_ORIENTATION
  elif current_cell.row < next_cell.row:
    next_orientation = SOUTH_ORIENTATION
  elif current_cell.column > next_cell.column:
    next_orientation = WEST_ORIENTATION
  elif current_cell.column < next_cell.column:
    next_orientation = EAST_ORIENTATION
  return next_cell, next_orientation

# set_state() function
# Sets the orientation the robot is in.
def set_state(orientation):
  if orientation == NORTH_ORIENTATION:
    state = "NORTH"
  elif orientation == EAST_ORIENTATION:
    state = "EAST"
  elif orientation == SOUTH_ORIENTATION:
    state = "SOUTH"
  else:
    state = "WEST"
  return state

# Returns a numpy array with the image data in a 3 dimensional array (R, G, B, A).
array = image_conversion(GUI.getMap('/resources/exercises/vacuum_cleaner_loc/images/mapgrannyannie.png'))

# Initialization of map cells.
cells = generate_cell_matrix(array)
free_cells = []
visited_neighbors = []

# Mapping the environment to know which cells are obstacles
# and which ones have not yet been visited, which are added to a list.
for row in range(len(cells)):
  for column in range(len(cells)):
    set_obstacle_cell(cells[row][column])
    if (cells[row][column].obstacle == True):
      mark_visited_cell(cells[row][column])
    else:
      free_cells.append(cells[row][column])

set_cell_neighbors(cells)

# Create a grid on the map.
width, height = array.shape
for j in range(0, width, 17):
  cv2.line(array, (j, 0), (j, height), (0, 0, 0), 1)
for i in range(0, width, 17):
  cv2.line(array, (0, i), (width, i), (0, 0, 0), 1)

# Shows the map with grid and obstacles.
GUI.showNumpy(array)

# Initialization of control variables for the execution of the algorithm.
iterations = 0
state = "WEST"
next_orientation = WEST_ORIENTATION
turn = False
return_point = False
path = False
east_left_turn = False

while True:
    
    # Update map display every 10 iterations.
    if iterations == 10:
      GUI.showNumpy(array)
      iterations = 0
    current_cell = map_to_cell_conversion(HAL.getPose3d().x, HAL.getPose3d().y)

    # Marks the current cell as visited if it was not already visited.
    if not current_cell.visited:
      current_cell.visited_cell()
      mark_obstacle_cell(current_cell)

    # Management of the robot's rotation to change orientation. 
    if turn:
      destiny_orientation = next_orientation
      if destiny_orientation == EAST_ORIENTATION and not east_left_turn:
        rotation_difference = abs(HAL.getPose3d().yaw - destiny_orientation)
        if rotation_difference <= 0.15:
          HAL.setW(0)
          turn = False
      else:
        if east_left_turn:
          destiny_orientation = EAST_LEFT_ORIENTATION
        rotation_difference = abs(HAL.getPose3d().yaw - destiny_orientation)
        if rotation_difference < 0.1:
          HAL.setW(0)
          turn = False

    # Saves neighbors of cells that have already been visited.
    else:
      update_neighboring_cells(current_cell)

      # Moment at which the robot must go to a critical point.
      if return_point:
        if not path:
          current_orientation = next_orientation
          path, next_orientation = calculate_next_step(current_cell, return_point)
          if path is None:
            continue
          turn = True
          HAL.setV(0)
          HAL.setW(0.3)
        else:
          if current_cell == return_point:
            return_point = False
            state = set_state(next_orientation)
            HAL.setV(0.6)
          elif current_cell == path:
            HAL.setV(0)
            path = False
          else:
            HAL.setV(0.6)

      # Check if the robot is at a critical point.
      else:
        critical_point = True
        neighbors_list = [current_cell.north_neighbor, current_cell.east_neighbor, current_cell.south_neighbor, current_cell.west_neighbor]
        for neighbor in neighbors_list:
          if neighbor in free_cells and not neighbor.visited:
            critical_point = False
            break
        if critical_point:
          return_point, _ = find_nearest_free_cell(current_cell, visited_neighbors)
          path = False
          state = None

      # State machine based on the BSA algorithm (Backtracking Spiral Algorithm).
      if (state == "NORTH"):
        if (current_cell.north_neighbor.obstacle == False) and (current_cell.north_neighbor.visited == False): # x
          HAL.setV(0.5)
        else:
          y_visited_cell = cell_to_map_conversion(current_cell.north_neighbor.column, current_cell.north_neighbor.row)[1]
          if abs(HAL.getPose3d().y - y_visited_cell) < 0.8:
            next_orientation = EAST_ORIENTATION
            turn = True
            HAL.setV(0)
            HAL.setW(0.3)
            state = "EAST"
      elif (state == "EAST"):
        if (current_cell.east_neighbor.obstacle == False) and (current_cell.east_neighbor.visited == False):
          HAL.setV(0.5)
        else:
          x_visited_cell = cell_to_map_conversion(current_cell.east_neighbor.column, current_cell.east_neighbor.row)[0]
          if abs(HAL.getPose3d().x - x_visited_cell) < 0.8:
            next_orientation = SOUTH_ORIENTATION
            turn = True
            HAL.setV(0)
            HAL.setW(0.3)
            state = "SOUTH"
      elif (state == "SOUTH"):
        if (current_cell.south_neighbor.obstacle == False) and (current_cell.south_neighbor.visited == False):
          HAL.setV(0.5)
        else:
          y_visited_cell = cell_to_map_conversion(current_cell.south_neighbor.column, current_cell.south_neighbor.row)[1]
          if abs(HAL.getPose3d().y - y_visited_cell) < 0.8:
            next_orientation = WEST_ORIENTATION
            turn = True
            HAL.setV(0)
            HAL.setW(0.3)
            state = "WEST"
      elif (state == "WEST"):
        if (current_cell.west_neighbor.obstacle == False) and (current_cell.west_neighbor.visited == False):
          HAL.setV(0.5)
        else:
          x_visited_cell = cell_to_map_conversion(current_cell.west_neighbor.column, current_cell.west_neighbor.row)[0]
          if abs(HAL.getPose3d().x - x_visited_cell) < 0.8:
            next_orientation = NORTH_ORIENTATION
            turn = True
            HAL.setV(0)
            HAL.setW(0.3)
            state = "NORTH"
    iterations += 1