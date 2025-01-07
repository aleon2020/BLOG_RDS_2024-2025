# IMPORT FROM LIBRARIES
# * yaml library class
# Contains functions to parse and emit YAML.
# * pathlib library class
# Contains classes to handle filesystem paths in an object-oriented manner.
# * GUI (Graphical User Interface Class) library class
# Contains functions used to view the debugging information, like image widgets.
# * HAL (Hardware Abstraction Library) library class
# Contains functions that sends and receives information to and from the hardware (Gazebo).
# * pyapriltags library class
# Contains functions to detect AprilTag markers in images.
# * OpenCV library class
# Contains functions that allow you to read/display images and apply different filters to them.
# * NumPy library class
# Contains functions that perform numerical calculations and array manipulations.
# * time module
# Contains functions related to time measurement and manipulation.
# * random module
# Contains functions that generate random numbers and perform random selections.
import yaml
from pathlib import Path
import GUI
import HAL
import pyapriltags
import cv2
import numpy as np
import time

# INITIALIZATION OF VARIABLES
# Initialize the robot's initial coordinates, marker settings,
# the coordinates of the corners of the markers and the AprilTags detector.
x, y, yaw = 0, 0, 0
conf = yaml.safe_load(
    Path("/resources/exercises/marker_visual_loc/apriltags_poses.yaml").read_text()
)
tags = conf["tags"]
apriltag_black_corners = np.array([[-0.12, 0.12, 0], [0.12, 0.12, 0], [0.12, -0.12, 0], [-0.12, -0.12, 0]], dtype=np.float32)
detector = pyapriltags.Detector(searchpath=["apriltags"], families="tag36h11")

# INITIALIZATION OF STATES AND ELAPSED TIME
time_start = time.time()
FORWARD = 1
TURN = 2
state = TURN

while True:

    # MOTION CONTROL
    # Go forward for 5 seconds and turn left for 10 seconds, permanently.
    HAL.setV(2)
    if state == FORWARD:
        HAL.setV(1)
        HAL.setW(0)
        if time.time() - time_start >= 5:
            state = TURN
            time_start = time.time()
    elif state == TURN:
        HAL.setV(1)
        HAL.setW(10)
        if time.time() - time_start >= 10:
            state = FORWARD
            time_start = time.time()

    # IMAGE CAPTURE AND PROCESSING
    # Takes the image from the camera and converts it to grayscale.
    print("[INFO] loading image...")
    image = HAL.getImage()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # GENERATION OF THE CAMERA MATRIXES
    # Creates the camera's intrinsic matrix and its distortion coefficients.
    size = image.shape
    focal_length = size[1]
    center = (size[1] / 2, size[0] / 2)
    camera_matrix = np.array(
        [[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]],
        dtype="double",
    )
    dist_coeffs = np.zeros((4, 1))

    # APRILTAGS DETECTION
    # Detect AprilTags by the camera by selecting the closest one,
    # in case more than one has been detected at the same time.
    # And if no AprilTag is detected, prints the robot odometry coordinates.
    print("[INFO] detecting AprilTags...")
    results = detector.detect(gray)
    print("[INFO] {} total AprilTags detected".format(len(results)))
    if len(results) == 0:
        x, y, yaw = HAL.getOdom().x, HAL.getOdom().y, HAL.getOdom().yaw
        GUI.showEstimatedPose((x, y, yaw))
        print(f"[INFO] odom x: {x}, y: {y}, yaw: {np.degrees(yaw)}")
        continue
    r = None
    minimum_distance = float('inf')
    for apriltag in results:
        apriltag_corners = np.array(apriltag.corners, dtype=np.float32)
        _, _, tvec = cv2.solvePnP(apriltag_black_corners, apriltag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        distance = np.linalg.norm(tvec)
        if distance < minimum_distance:
            minimum_distance = distance
            r = apriltag
    if r is None:
        continue

    # CALCULATION OF THE ROBOT POSITION USING MATRICES
    # Get information about the detected AprilTag.
    tag_id = f"tag_{r.tag_id}"
    x_tag, y_tag, yaw_tag = tags[tag_id]["position"]
    apriltag_corners = np.array(r.corners, dtype=np.float32)
    success, rvec, tvec = cv2.solvePnP(apriltag_black_corners, apriltag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

    if success:

        # TRANSFORMATION BETWEEN REFERENCE FRAMES
        # - camera2robot_matrix: Transformation from camera to robot frame.
        # - camera2marker_matrix: Camera to marker frame transformation.
        # - marker2camera_matrix: Inverse of the previous transformation.
        # - x_axis_rotation_matrix: -90º rotation matrix of the X axis.
        # - z_axis_rotation_matrix: 90º rotation matrix of the Z axis.
        # - world2marker_matrix: Transformation from world to marker frame.
        # - world2camera_matrix: Resulting matrix that calculates the position (x, y, yaw) of the robot.
        camera2robot_matrix = np.array([[1, 0, 0, 0.069], [0, 1, 0, -0.047], [0, 0, 1, 0.107], [0, 0, 0, 1]])
        camera2marker_matrix = np.zeros((4, 4))
        camera2marker_matrix[3, 3] = 1
        camera2marker_matrix[:3, :3] = cv2.Rodrigues(rvec)[0]
        camera2marker_matrix[:3, 3] = tvec.ravel()
        marker2camera_matrix = np.linalg.inv(camera2marker_matrix)
        x_axis_rotation_matrix = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        z_axis_rotation_matrix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        marker2camera_matrix = np.dot(z_axis_rotation_matrix @ x_axis_rotation_matrix, marker2camera_matrix)
        world2marker_matrix = np.array([[np.cos(yaw_tag), -np.sin(yaw_tag), 0, x_tag], [np.sin(yaw_tag),  np.cos(yaw_tag), 0, y_tag], [0, 0, 1, 0.8], [0, 0, 0, 1]])
        world2camera_matrix = np.matmul(world2marker_matrix, marker2camera_matrix, camera2robot_matrix)[:3, :3]
        pitch = np.arctan2(-world2camera_matrix[2, 0], np.sqrt(world2camera_matrix[0, 0]**2 + world2camera_matrix[1, 0]**2))
        x = float(np.matmul(world2marker_matrix, marker2camera_matrix, camera2robot_matrix)[0, 3])
        y = float(np.matmul(world2marker_matrix, marker2camera_matrix, camera2robot_matrix)[1, 3])
        yaw = float(np.matmul(world2marker_matrix, marker2camera_matrix, camera2robot_matrix)[2, 3])
        
        # VIEWING THE RESULTS
        # Show the image with marker information and the estimated position of the robot.
        GUI.showImage(image)
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(
            image,
            tagFamily,
            (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )
        print("[INFO] tag family: {}".format(tagFamily))
        GUI.showEstimatedPose((x, y, yaw))
        print(f"[INFO] robot x: {x}, y: {y}, yaw: {np.degrees(yaw)}")