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
# * time module
# Provides functions related to time measurement and manipulation.
import HAL
import GUI
import numpy as np
import math
import cv2
import time

survivors_found_list = []
face_cascade_classifier = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# detect_and_rotate_face() function
# The drone searches for a face through its camera. If you can't find it, rotate the camera several 
# angles to improve detection. If it detects a face, it draws a rectangle on the image around the 
# face detected.
def detect_and_rotate_face(camera_image):
  gray_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)
  face_detector = face_cascade_classifier.detectMultiScale(gray_image, 1.1, 5)
  if len(face_detector) == 0:
    for angle in [45, 90, 135, 180, 225, 270, 315, 360]: 
      (height, width) = gray_image.shape[:2]
      rotated_image = cv2.warpAffine(gray_image, cv2.getRotationMatrix2D((width // 2, height // 2), angle, 1.0), (width, height))
      face_detector = face_cascade_classifier.detectMultiScale(rotated_image, 1.1, 5)
      if len(face_detector) > 0:
        gray_image = rotated_image
        break
  if len(face_detector) > 0:
    x_coordinate, y_coordinate, width, height = face_detector[0]
    cv2.rectangle(gray_image, (x_coordinate, y_coordinate), (x_coordinate + width, y_coordinate + height), (255, 0, 0), 2)
    GUI.showLeftImage(gray_image)
    return True
  return False

# navigate_and_search_survivors() function
# The drone moves towards specific coordinates while searching for faces through its camera. If it
# detects a face at a new location, prints its coordinates in UTM format.
def navigate_and_search_survivors(x_target_coordinate, y_target_coordinate, patrolling):
  distance = 1
  while distance > 0.1:
    GUI.showImage(HAL.get_ventral_image())
    distance = math.sqrt((x_target_coordinate - HAL.get_position()[0]) ** 2 + (y_target_coordinate - HAL.get_position()[1]) ** 2)
    HAL.set_cmd_pos(x_target_coordinate, y_target_coordinate, 1, 0)
    if patrolling == True:
      if detect_and_rotate_face(HAL.get_ventral_image()) == True:
        new_survivor = True
        for survivor in survivors_found_list:
          if math.sqrt((HAL.get_position()[0] - survivor[0]) ** 2 + (HAL.get_position()[1] - survivor[1]) ** 2) < 3:
            new_survivor = False
            break 
        if new_survivor == True:
          print('NEW SURVIVOR FOUND AT %d N %d E (UTM COORDINATES)' % (4459132 - HAL.get_position()[1], 430532 - HAL.get_position()[0]))
          survivors_found_list.append(HAL.get_position())
          if len(survivors_found_list) == 6:
            print("ALL SURVIVORS FOUND")
  return

# STATES
NAVIGATE_TO_SEARCH_ZONE = 1
PATROLLING = 2
RETURN_TO_BASE = 3

x_starting_coordinate, y_starting_coordinate = HAL.get_position()[0], HAL.get_position()[1]
state = NAVIGATE_TO_SEARCH_ZONE
starting_time = time.time()

while True:

    # NAVIGATE_TO_SEARCH_ZONE STATE
    # This state initializes the drone takeoff and moves it to the designated coordinates for 
    # start the search mission. Makes sure the drone is in the air before proceeding to the next
    # patrolling phase.
    if state == NAVIGATE_TO_SEARCH_ZONE:
      HAL.takeoff(1)
      print("STARTING RESCUE OPERATION ...")
      navigate_and_search_survivors(40, -30, False)
      navigate_and_search_survivors(55, -45, False)
      state = PATROLLING

    # PATROLLING STATE
    # In this state, the drone patrols a defined area for a maximum of 500 seconds. Perform movements
    # in a certain pattern (left, up, right, above) while searching for survivors, updating their 
    # position with each step. At the end patrolling time, the drone changes to completion status.
    elif state == PATROLLING:
      x_current_coordinate, y_current_coordinate = HAL.get_position()[0], HAL.get_position()[1]
      while time.time() - starting_time < 500:
        navigate_and_search_survivors(x_current_coordinate - 30, y_current_coordinate, True)
        x_current_coordinate, y_current_coordinate = HAL.get_position()[0], HAL.get_position()[1]
        navigate_and_search_survivors(x_current_coordinate, y_current_coordinate + 1.5, True)
        x_current_coordinate, y_current_coordinate = HAL.get_position()[0], HAL.get_position()[1]
        navigate_and_search_survivors(x_current_coordinate + 30, y_current_coordinate, True)
        x_current_coordinate, y_current_coordinate = HAL.get_position()[0], HAL.get_position()[1]
        navigate_and_search_survivors(x_current_coordinate, y_current_coordinate + 1.5, True)
        x_current_coordinate, y_current_coordinate = HAL.get_position()[0], HAL.get_position()[1]
      state = RETURN_TO_BASE  

    # RETURN_TO_BASE STATE
    # In this state, the drone prints a warning message about low battery and proceeds to 
    # return to its initial position, confirming that the mission has been completed.
    elif state == RETURN_TO_BASE:
      print("LOW BATTERY LEVEL, RETURNING TO BASE ...")
      navigate_and_search_survivors(x_starting_coordinate, y_starting_coordinate, False)
      HAL.land()