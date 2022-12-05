#!/usr/bin/python3
import math
import numpy as np
import time
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
import threading

from pathgen import squared_velocity_path

import naive_fk
import fk_ik_jq as kin

# ROSpy node/msg names
NODE_NAME = "smbus_sim_node"
READ_JOINT_MSG = "/base/read_joints"
SET_JOINT_MSG = "/base/set_joints"
SET_JOINT_PATH_MSG = "/base/set_joints_path"
SET_XYZ_MSG = "/base/set_xyz"
SET_JOINT_STEP = "/base/set_joints_step"

def write_angles():
    while not terminate:
        msg = Float32MultiArray()
        msg.data = tuple( joints )
        pub.publish( msg )
        time.sleep( 0.05 )

def handle_write_angles( msg ):
    global joints
    joints = [x%(2*math.pi) for x in msg.data]

def handle_write_step( msg ):
    global joints

    new_joints = [(msg.data[i] + joints[i])%(2*math.pi) for i in range(6)]
    joints = new_joints

def handle_path( msg ):
    global joints
    lamda_max = 100 # maybe change

    joint_deltas = [msg.data[i] - joints[i] for i in range(6)]

    paths = [[squared_velocity_path(0, lamda_max, joint_deltas[i]) for i in range(6)]]
    for i in range(1, lamda_max):
        paths.append([paths[-1][j]+squared_velocity_path(i, lamda_max, joint_deltas[j]) for j in range(6)])
    for i in range(lamda_max):
        for j in range(6):
            paths[i][j] += joints[j]
    
    print("writing paths")
    np.save("/home/hanson/baymax/paths.npy", np.array(paths))
    for jpos in paths:
        joints = [x for x in jpos]
        rate.sleep()
        
def handle_xyz( msg ):
    global joints
    lamda_max = 100
    # get current xyz
    initial_xyz = naive_fk.tf_base_to_tool(*(joints[:5]))
    # calculate delta xyz
    delta_xyz = [msg.data[i] - initial_xyz[i] for i in range(3)]

    # generate xyz motion profiles
    xyz_paths = [[squared_velocity_path(0, lamda_max, delta_xyz[i]) for i in range(3)]]
    for i in range(1, lamda_max):
        xyz_paths.append([xyz_paths[-1][dim]+squared_velocity_path(i, lamda_max, delta_xyz[dim]) for dim in range(3)])
    for i in range(lamda_max):
        for dim in range(3):
            xyz_paths[i][dim] += initial_xyz[dim]

    # ik on motion profiles to get joint motion profiles
    xyz_paths = np.array(xyz_paths).reshape((lamda_max, 3))
    np.save("/home/hanson/baymax/xyz_paths.npy", xyz_paths)

    joint_paths = []
    recent_config = [x for x in joints[:5]]
    for i in range(len(xyz_paths)):
        angles = kin.DOFBOT.ik(xyz_paths[i], np.array(recent_config))
        recent_config = [x for x in angles]
        joint_paths.append(list(angles)+[joints[5]])

    # move
    print("writing paths")
    np.save("/home/hanson/baymax/joint_paths.npy", np.array(joint_paths))
    for jpos in joint_paths:
        joints = [x for x in jpos]
        rate.sleep()
        

rospy.init_node( NODE_NAME )
rate = rospy.Rate(50)
pub = rospy.Publisher( READ_JOINT_MSG, Float32MultiArray, queue_size=10 )
rospy.Subscriber( SET_JOINT_MSG, Float32MultiArray, handle_write_angles )
rospy.Subscriber( SET_JOINT_PATH_MSG, Float32MultiArray, handle_path )
rospy.Subscriber( SET_JOINT_STEP, Float32MultiArray, handle_write_step )
rospy.Subscriber( SET_XYZ_MSG, Float32MultiArray, handle_xyz )

terminate = False
joints = [ 0.0 ] * 6

t = threading.Thread( target = write_angles )
t.start()

try:
    rospy.spin()
except KeyboardInterrupt:
    pass
finally:
    terminate = True
    t.join()
