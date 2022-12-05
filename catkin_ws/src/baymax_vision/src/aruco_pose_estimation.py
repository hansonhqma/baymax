#!/usr/bin/python3

import os

import rospy
import std_msgs.msg
import geometry_msgs.msg

import cv2 as cv
import cvmanip as cvm
import numpy as np


from scipy.spatial.transform import Rotation


# node info
NODE_NAME = "baymax_vision"

# subscribed messages
CURRENT_TASK_TOPIC = "/state/current_task"
TARGET_ID_TOPIC = "/state/target_id"
BASE_CAM_TF_TOPIC = "/tf/base_to_cam"
VISION_START_TOPIC = "/vision/start"

# published messages
TARGET_POS_TF = "/vision/targetpos"

# node state
CURRENT_TASK = None
TARGET_ID = None
BASE_CAM_TF = None

# load image matrices
LOGNAME = os.environ.get("USER")
calibration_matrix = np.load("/home/"+LOGNAME+"/baymax/catkin_ws/src/baymax_vision/src/calibration_matrix.npy")
distortion_coeffs = np.load("/home/"+LOGNAME+"/baymax/catkin_ws/src/baymax_vision/src/distortion_coefficients.npy")

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

CAPTURE = cv.VideoCapture(0)

w, h = 640, 480

newcameramtx, roi = cv.getOptimalNewCameraMatrix(calibration_matrix, distortion_coeffs, (w,h), 1, (w,h))

def start_vision( _ ):
    while True:
        ret, frame = CAPTURE.read()

        if not ret:
            continue

        frame = cv.undistort(frame, calibration_matrix, distortion_coeffs, None, newcameramtx)

        cleaned_frame = cv.bitwise_not(cv.inRange(frame, (0, 0, 0), (180, 255, 190)))
            
        if not CURRENT_TASK == 'identify_target' or TARGET_ID == None or BASE_CAM_TF == None:
            # don't need to do anything
            cvm.quickshow(frame)
            cvm.quickshow(cleaned_frame, "cleaned_frame")
            continue

        print("looking for {} marker".format(TARGET_ID))
        corners, ids, _ = cv.aruco.detectMarkers(cleaned_frame, aruco_dict, parameters=aruco_params)

        arucoid = targetid_to_arucoid.get(TARGET_ID)

        # get current camera pose
        pos_basetocam = np.array([[BASE_CAM_TF.position.x], [BASE_CAM_TF.position.y], [BASE_CAM_TF.position.z]])

        quat_basetocam = np.array([BASE_CAM_TF.orientation.x, BASE_CAM_TF.orientation.y, BASE_CAM_TF.orientation.z, BASE_CAM_TF.orientation.w])

        rot_basetocam = Rotation.from_quat(quat_basetocam).as_matrix()

        for i in range(len(corners)):
            frame = cv.aruco.drawDetectedMarkers(frame, corners)
            if not arucoid == ids[i][0]:
                continue
            # estimate pose for current target id
            
            rot_camtotarget, pos_camtotarget, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], 0.045,
            calibration_matrix, distortion_coeffs)

            # calculate base to target tf
            pos_camtotarget = pos_camtotarget.reshape((3, 1))
            pos_basetotarget = pos_basetocam + rot_basetocam @ pos_camtotarget

            print("cam to target:", pos_camtotarget)
            print("base to target:", pos_basetotarget)

            tf_msg = geometry_msgs.msg.Pose()
            tf_msg.position.x = pos_basetotarget[0][0]
            tf_msg.position.y = pos_basetotarget[1][0]
            tf_msg.position.z = pos_basetotarget[2][0]

            target_tf_publisher.publish(tf_msg)
            
            frame = cv.drawFrameAxes(frame, calibration_matrix, distortion_coeffs, rot_camtotarget, pos_camtotarget, 0.01)


        cvm.quickshow(frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

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


rospy.Subscriber(CURRENT_TASK_TOPIC, std_msgs.msg.String, current_task_handler)
rospy.Subscriber(TARGET_ID_TOPIC, std_msgs.msg.String, target_id_handler)
rospy.Subscriber(BASE_CAM_TF_TOPIC, geometry_msgs.msg.Pose, base_cam_tf_handler)
rospy.Subscriber(VISION_START_TOPIC, std_msgs.msg.String, start_vision)
target_tf_publisher = rospy.Publisher(TARGET_POS_TF, geometry_msgs.msg.Pose, queue_size=1)
rospy.spin()

