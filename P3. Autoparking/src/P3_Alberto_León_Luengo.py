# IMPORT FROM LIBRARIES
# * HAL (Hardware Abstraction Library) library class
# Contains the functions that sends and receives information to and from the hardware (Gazebo).
# * GUI (Graphical User Interface Class) library class
# Contains the functions used to view the debugging information, like image widgets.
# * NumPy library class
# Contains functions that perform numerical calculations and array manipulations.
# * math module
# Contains standard math functions such as math.sqrt or math.pi.
import HAL
import GUI
import numpy as np
import math

# parse_laser_data() function
# Takes the values ​​collected by the laser, adjusts them to make sure they are within range
# and returns a list of value-angle pairs.
def parse_laser_data(laser_data):
  laser = []
  for i in range(180):
    dist = laser_data.values[i]
    angle = math.radians(i)
    if math.isfinite(dist):
      if dist > laser_data.maxRange:
        dist = laser_data.maxRange
    else:
      if math.isinf(dist):
        dist = laser_data.maxRange
      elif math.isnan(dist):
        dist = laser_data.minRange
    laser.append((dist, angle))
  return laser

# CAR STATES
SEARCHING_PARKING_SPACE = 1
SEARCHING_REFERENCE_CARS = 2
PARKING_PROCESS = 3
TURNING_CAR = 4
REVERSING_CAR = 5
GET_INTO_PARKING_SPACE = 6
MOVE_CAR_FORWARD = 7
MOVE_CAR_BACKWARD = 8
PARKED_CAR = 9

# CAR CURRENT STATES
CURRENT_CAR_STATE = SEARCHING_PARKING_SPACE
CURRENT_PARKING_PROCESS_STATE = TURNING_CAR

# CAR DETECTED VARIABLES
front_car_detected, back_car_detected, no_car_detected = False, False, False

# COUNTERS
no_sidecar_detected, no_sidecar_found = 0, 0
free_front_space, free_back_space, free_parking_space = 0, 0, 0
front_car_distance, back_car_distance = 0, 0
back_laser_values_reversing, front_laser_values_reversing, front_laser_values_forwarding = 0, 0, 0
current_angle = 0
street_direction_respect_alignment, aligned_values, aligned_degrees = [], [], []

while True:

    # INITIAL LINEAR AND ANGULAR SPEED OF THE CAR
    V, W = 0.6, 0

    # CAR ALIGNMENT AND STEERING
    # If the car is not parking and if the street direction has not been calculated yet,
    # the value range [45, 135] of the laser is traversed. On the other hand, if no other
    # car from the side, the obtained value-angle pairs are saved. And finally, it fits
    # alignment using a linear regression and the angular velocity is calculated from the
    # alignment value obtained.
    if CURRENT_CAR_STATE != PARKING_PROCESS:
      if len(street_direction_respect_alignment) == 0:
        aligned_values_list, aligned_degrees_list = [], []
        for i in range(45, 135):
          if parse_laser_data(HAL.getRightLaserData())[i][0] != HAL.getBackLaserData().maxRange:
            aligned_values_list.append(parse_laser_data(HAL.getRightLaserData())[i][0])
            aligned_degrees_list.append(math.degrees(parse_laser_data(HAL.getRightLaserData())[i][1]))
          else:
            no_sidecar_detected += 1
            if 120 > i >= 100:
              no_sidecar_found += 1
        if no_sidecar_found >= 18:
          aligned_values_list = aligned_values
          aligned_degrees_list = aligned_degrees
        no_sidecar_found = 0
        if no_sidecar_detected >= 75:
          no_sidecar_detected = 0
          aligned_values = aligned_values_list
          aligned_degrees = aligned_degrees_list
        no_sidecar_detected = 0
      else:
        aligned_values_list = street_direction_respect_alignment[0]
        aligned_degrees_list = street_direction_respect_alignment[1]
      matrix = np.vstack([aligned_degrees_list, np.ones(len(aligned_degrees_list))]).T
      slope, intercept = np.linalg.lstsq(matrix, aligned_values_list, rcond=None)[0]
      if 0.001 > np.degrees(np.arctan(slope)) > -0.001:
        W = 0
      else:
        W = (-np.arctan(slope)) * 10.5

    # SEARCH FOR A PARKING SPACE
    # Check if there is space both in front and behind the car, and if both distances
    # are the same, begin to analyze the space as a possible parking space and if 
    # it ends up adjusting to the searched values, it begins to search for reference cars.
    if CURRENT_CAR_STATE == SEARCHING_PARKING_SPACE:
      print("SEARCHING FOR A PARKING SPACE ...")
      for i in range(3):
        if parse_laser_data(HAL.getFrontLaserData())[i][0] >= 7:
          free_front_space += 1
        if parse_laser_data(HAL.getBackLaserData())[-i-1][0] >= 7:
          free_back_space += 1
      if free_front_space - free_back_space == 0:
        V = 0.45
        for i in parse_laser_data(HAL.getRightLaserData()):
          if 104 > math.degrees(i[1]) > 76:
            distance = 6.4 / math.sin(i[1])
          else:
            distance = 1.65 / math.cos(i[1])
            if distance <= 0:  
              distance = 6.4 + distance
          if i[0] >= distance:
            free_parking_space += 1
        if free_parking_space >= 175:
          print ("PARKING SPACE FOUND")
          street_direction_respect_alignment = [aligned_values_list, aligned_degrees_list]
          current_angle = HAL.getPose3d().yaw
          CURRENT_CAR_STATE = SEARCHING_REFERENCE_CARS
        free_parking_space = 0
      free_front_space, free_back_space = 0, 0

    # SEARCH FOR REFERENCE CARS
    # Start looking for a reference car, both in front and behind, and if both
    # found near the car, the parking process starts.
    elif CURRENT_CAR_STATE == SEARCHING_REFERENCE_CARS:
      print("SEARCHING FOR REFERENCE CARS ...")
      if not front_car_detected and not back_car_detected and not no_car_detected:
        front_reference_car, back_reference_car = 0, 0
        for i in range(15, 66):
          if parse_laser_data(HAL.getFrontLaserData())[i][0] < 6:
            front_reference_car += 1
          if parse_laser_data(HAL.getBackLaserData())[-i][0] < 6:
            back_reference_car += 1
        front_car_detected = front_reference_car > 10
        back_car_detected = back_reference_car > 10
        no_car_detected = not front_car_detected and not back_car_detected
      if front_car_detected:
        for i in range(23):
          if parse_laser_data(HAL.getBackLaserData())[-i-1][0] <= 5:
            front_car_distance += 1
        for i in range(75, 106):
          if parse_laser_data(HAL.getRightLaserData())[i][0] <= 5:
            front_car_distance += 1
      if not front_car_detected and back_car_detected:
        for i in range(30, 61):
          if HAL.getBackLaserData().maxRange > parse_laser_data(HAL.getBackLaserData())[-i][0] > HAL.getBackLaserData().maxRange - 2:
            back_car_distance += 1    
      if front_car_distance >= 53 or back_car_distance >= 5:
        CURRENT_CAR_STATE = PARKING_PROCESS
      front_car_distance, back_car_distance = 0, 0

    # PARKING PROCESS
    elif CURRENT_CAR_STATE == PARKING_PROCESS:
      V, W = 0, 0

      # CAR TURN
      if CURRENT_PARKING_PROCESS_STATE == TURNING_CAR:
        print("TURNING CAR ...")
        desired_angle = math.pi / 4 + current_angle
        if not (HAL.getPose3d().yaw >= desired_angle - 0.01 and HAL.getPose3d().yaw <= desired_angle + 0.01):
          V, W = -0.35, (desired_angle - HAL.getPose3d().yaw) * 25.5
        else:
          CURRENT_PARKING_PROCESS_STATE = REVERSING_CAR

      # CAR REVERSE
      elif CURRENT_PARKING_PROCESS_STATE == REVERSING_CAR:
        print("REVERSING CAR ...")
        V = -0.35
        for i in range(40):
          if back_car_detected:
            if HAL.getBackLaserData().maxRange > parse_laser_data(HAL.getBackLaserData())[i][0]:
              back_laser_values_reversing += 1
          if not back_car_detected and front_car_detected:
            if 9 < parse_laser_data(HAL.getFrontLaserData())[i][0] and parse_laser_data(HAL.getFrontLaserData())[i][0] != HAL.getFrontLaserData().maxRange:
              front_laser_values_reversing += 1
        if front_laser_values_reversing > 8 or back_laser_values_reversing > 0:
          V = 0
          CURRENT_PARKING_PROCESS_STATE = GET_INTO_PARKING_SPACE

      # PUT THE CAR INSIDE THE PARKING SPACE
      elif CURRENT_PARKING_PROCESS_STATE == GET_INTO_PARKING_SPACE:
        print("PUTTING THE CAR INSIDE THE PARKING SPACE ...")
        V = -0.23
        back_laser_values_reversing = 0
        for i in range(20):
          if parse_laser_data(HAL.getBackLaserData())[i][0] < 1:
            back_laser_values_reversing += 1
        if back_laser_values_reversing == 0:
          for i in range(87, 93):
            if parse_laser_data(HAL.getBackLaserData())[i][0] < 0.8:
              back_laser_values_reversing += 1
        desired_angle = current_angle
        if not (HAL.getPose3d().yaw >= current_angle - 0.03 and HAL.getPose3d().yaw <= current_angle + 0.03):
          W = -100
        else:
          V, W = 0, 0
          CURRENT_PARKING_PROCESS_STATE = PARKED_CAR
        if back_laser_values_reversing >= 2:
          CURRENT_PARKING_PROCESS_STATE = MOVE_CAR_FORWARD
          V = 0.23

      # FORWARD MOVEMENT OF THE CAR
      elif CURRENT_PARKING_PROCESS_STATE == MOVE_CAR_FORWARD:
        print("CAR MOVING FORWARD ...")
        V, W = 0.35, -100
        for i in range(88, 92):
          if parse_laser_data(HAL.getFrontLaserData())[i][0] < 0.5:
            front_laser_values_forwarding += 1
          elif front_laser_values_forwarding != 0 and parse_laser_data(HAL.getFrontLaserData())[i][0] >= 0.4:
            front_laser_values_forwarding = -1
            break
        if front_laser_values_forwarding != 0:
          V, W = -0.35, 100
          if front_laser_values_forwarding == -1:
            CURRENT_PARKING_PROCESS_STATE = MOVE_CAR_BACKWARD

      # BACKWARD MOVEMENT OF THE CAR
      elif CURRENT_PARKING_PROCESS_STATE == MOVE_CAR_BACKWARD:
        print("CAR MOVING BACKWARD ...")
        V, W = -0.35, -100
        back_laser_values_reversing = 0
        for i in range(88, 92): 
          if parse_laser_data(HAL.getBackLaserData())[i][0] < 0.75:
            back_laser_values_reversing += 1
            break
        if back_laser_values_reversing != 0:
          CURRENT_PARKING_PROCESS_STATE = PARKED_CAR

      # PARKED CAR
      elif CURRENT_PARKING_PROCESS_STATE == PARKED_CAR:
        print("CAR PARKED SUCCESSFULLY")
        V, W = 0, 0

    # Sending linear and angular velocity to the car depending on the state it is in.
    HAL.setV(V)
    HAL.setW(W)