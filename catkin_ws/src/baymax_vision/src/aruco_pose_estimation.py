#!/usr/bin/python3

import os

import rospy
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from collections import deque

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

CAM_TO_TARGET_MARKER = "/marker/cam_to_target"
BASE_TO_TARGET_MARKER = "/marker/base_to_target"

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

arucoid_to_targetid = {
    0:"red",
    1:"green",
    2:"blue",
    3:"yellow"
}

arucoid_to_rviz_color = {
    0:(255, 0, 0),
    1:(0, 255, 0),
    2:(0, 0, 255),
    3:(255, 255, 0)
}

ARUCO_TAG_SIZE = 0.0215 # m

rospy.init_node(NODE_NAME)

CAPTURE = cv.VideoCapture(4)

w, h = 640, 480

newcameramtx, roi = cv.getOptimalNewCameraMatrix(calibration_matrix, distortion_coeffs, (w,h), 1, (w,h))

def start_vision( _ ):
    while True:
        ret, frame = CAPTURE.read()

        if not ret:
            continue

        frame = cv.undistort(frame, calibration_matrix, distortion_coeffs, None, newcameramtx)

        cleaned_frame = cv.bitwise_not(cv.inRange(frame, (0, 0, 0), (180, 255, 80)))
            
        if CURRENT_TASK == 'zero' or TARGET_ID == None or BASE_CAM_TF == None:
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
            id = ids[i][0]
            #if not arucoid == id:
                #continue
            
            # estimate pose for current target id
            
            rot_camtotarget, pos_camtotarget, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], ARUCO_TAG_SIZE,
            calibration_matrix, distortion_coeffs)
            print(rot_camtotarget)

            # calculate base to target tf
            pos_camtotarget = pos_camtotarget.reshape((3, 1))
            pos_basetotarget = pos_basetocam + rot_basetocam @ pos_camtotarget

            print("cam to target:", pos_camtotarget)
            print("base to target:", pos_basetotarget)

            # take care of actual tf before marker tf
            tf_msg = geometry_msgs.msg.PoseStamped()
            tf_msg.header.frame_id = arucoid_to_targetid.get(id)
            tf_msg.pose.position.x = pos_basetotarget[0][0]
            tf_msg.pose.position.y = pos_basetotarget[1][0]
            tf_msg.pose.position.z = pos_basetotarget[2][0]

            target_tf_publisher.publish(tf_msg)

            btt_marker_msg = visualization_msgs.msg.Marker()
            ctt_marker_msg = visualization_msgs.msg.Marker()
            btt_marker_msg.id = id
            ctt_marker_msg.id = id*10
            btt_marker_msg.type = 1
            ctt_marker_msg.type = 2

            btt_marker_msg.header.frame_id = 'base_link'
            ctt_marker_msg.header.frame_id = 'link_cam'

            for msg in [btt_marker_msg, ctt_marker_msg]:
                msg.scale.x = 0.025
                msg.scale.y = 0.025
                msg.scale.z = 0.025
                tag_color = arucoid_to_rviz_color.get(id)
                if arucoid == id:
                    msg.color.a = 1
                else:
                    msg.color.a = 0.5
                msg.color.r = tag_color[0]
                msg.color.g = tag_color[1]
                msg.color.b = tag_color[2]

            btt_marker_msg.pose.position.x = pos_basetotarget[0][0]
            btt_marker_msg.pose.position.y = pos_basetotarget[1][0]
            btt_marker_msg.pose.position.z = pos_basetotarget[2][0]

            ctt_marker_msg.pose.position.x = pos_camtotarget[0][0]
            ctt_marker_msg.pose.position.y = pos_camtotarget[1][0]
            ctt_marker_msg.pose.position.z = pos_camtotarget[2][0]

            btt_marker_publisher.publish(btt_marker_msg)
            ctt_marker_publisher.publish(ctt_marker_msg)

            if arucoid == id:
                frame = cv.drawFrameAxes(frame, calibration_matrix, distortion_coeffs, rot_camtotarget, pos_camtotarget, 0.01)
            #cleaned_frame = cv.cvtColor(cleaned_frame, cv.COLOR_GRAY2RGB)
            #cleaned_frame = cv.drawFrameAxes(cleaned_frame, calibration_matrix, distortion_coeffs, rot_camtotarget, pos_camtotarget, 0.01)


        cvm.quickshow(frame)
        cvm.quickshow(cleaned_frame, "cleaned_frame")
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
target_tf_publisher = rospy.Publisher(TARGET_POS_TF, geometry_msgs.msg.PoseStamped, queue_size=1)
ctt_marker_publisher = rospy.Publisher(CAM_TO_TARGET_MARKER, visualization_msgs.msg.Marker, queue_size = 10)
btt_marker_publisher = rospy.Publisher(BASE_TO_TARGET_MARKER, visualization_msgs.msg.Marker, queue_size = 10)
rospy.spin()

