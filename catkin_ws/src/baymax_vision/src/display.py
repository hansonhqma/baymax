#!/usr/bin/python3

import rospy
import sensor_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import cv2 as cv
import numpy as np
import cvmanip as cvm
from cv_bridge import CvBridge


# node info
NODE_NAME = "baymax_vision"

# subscribed messages
IMAGE_TOPIC = "/usb_cam/image_raw"
CURRENT_TASK_TOPIC = "/state/current_task"
TARGET_ID_TOPIC = "/state/target_id"
BASE_CAM_TF_TOPIC = "/tf/base_to_cam"

# published messages
TARGET_POS_TF = "/vision/targetpos"

# node state
CURRENT_TASK = None
TARGET_ID = None
BASE_CAM_TF = None

# load image matrices
calibration_matrix = np.load("/home/hanson/baymax/catkin_ws/src/baymax_vision/src/calibration_matrix.npy")
distortion_coeffs = np.load("/home/hanson/baymax/catkin_ws/src/baymax_vision/src/distortion_coefficients.npy")

# specify marker detection properties
marker_type = cv.aruco.DICT_6X6_250
aruco_dict = cv.aruco.Dictionary_get(marker_type)

aruco_params = cv.aruco.DetectorParameters_create()

targetid_to_arucoid = {
    "red":0,
    "green":1,
    "blue":2,
    "yellow":3
}

rospy.init_node(NODE_NAME)

bridge = CvBridge()

def image_handler( msg ):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

    if not CURRENT_TASK == 'identify_target' or TARGET_ID == None or BASE_CAM_TF == None:
        # don't need to do anything
        cvm.quickshow(frame)
        return
    
    grayscale_image = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    #aruco_id = targetid_to_arucoid[TARGET_ID]

    corners, ids, _ = cv.aruco.detectMarkers(grayscale_image, aruco_dict, parameters=aruco_params)

    for i in range(len(corners)):
        # estimate pose
        rot_camtotarget, tf_camtotarget, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], 0.02,
        calibration_matrix, distortion_coeffs)

        # calculate base to target tf
        

        cv.aruco.drawDetectedMarkers(frame, corners)
        cv.drawFrameAxes(frame, calibration_matrix, distortion_coeffs, rot_camtotarget, tf_camtotarget, 0.01)


    cvm.quickshow(frame)

    # look for specific aruco marker
    # publish tf

def current_task_handler ( msg ):
    # update current task
    global CURRENT_TASK
    CURRENT_TASK = msg.data

def target_id_handler( msg ):
    # update target id
    global TARGET_ID
    TARGET_ID = msg.data

def base_cam_tf_handler( msg ):
    # update base tool transformation
    global BASE_CAM_TF
    BASE_CAM_TF = msg


rospy.Subscriber(IMAGE_TOPIC, sensor_msgs.msg.Image, image_handler)
rospy.Subscriber(CURRENT_TASK_TOPIC, std_msgs.msg.String, current_task_handler)
rospy.Subscriber(TARGET_ID_TOPIC, std_msgs.msg.String, target_id_handler)
rospy.Subscriber(BASE_CAM_TF_TOPIC, geometry_msgs.msg.Pose, base_cam_tf_handler)
rospy.spin()

