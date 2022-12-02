#!/usr/bin/python3
import driver
import math
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

import tinyik

from pathgen import squared_velocity_path

# naive ik approach

chain = [[0, 0, 0.066], 'z', [0, 0, 0.0415], 'x', [0, 0, 0.0828], 'x', [0, 0, 0.0828], 'x', [0, 0, 0.0739]]
dofbot = tinyik.Actuator(chain)

# ROSpy node/msg names
NODE_NAME = "smbus_node"

READ_JOINT_MSG = "/base/read_joints"

SET_JOINT_MSG = "/base/set_joints"
SET_JOINT_PATH_MSG = "/base/set_joints_path"
SET_XYZ_MSG = "/base/set_xyz"
MOVE_JOINT_MSG = "/base/move_joints"

SET_TORQUE_MSG = "/base/set_torque"

CURRENT_ANGLES = None

def handle_read_angles( angles ):
    global CURRENT_ANGLES
    msg = Float32MultiArray()
    msg.data = tuple( angles )
    pub.publish( msg )
    CURRENT_ANGLES = angles

def handle_write_angles( msg ):
    if( len( msg.data ) == 6 ):
        for angle in msg.data:
            if( type( angle ) != float ):
                print( "Error, angle type invalid" )
                return
        arm.set_joints( list( msg.data ) )
    else:
        print( "Error, angle message invalid" )

def handle_move_angles( msg ):
    global CURRENT_ANGLES

    if( len( msg.data ) == 6 ):
        for angle in msg.data:
            if( type( angle ) != float ):
                print( "Error, angle type invalid" )
                return
        new_angles = [msg.data[i] + CURRENT_ANGLES[i] for i in range(6)]
        arm.set_joints(new_angles)
    else:
        print( "Error, angle message invalid" )

def handle_path( msg ):
    global CURRENT_ANGLES
    lamda_max = 100

    joint_deltas = [msg.data[i] - CURRENT_ANGLES[i] for i in range(6)]

    paths = [[squared_velocity_path(0, lamda_max, joint_deltas[i]) for i in range(6)]]
    for i in range(1, lamda_max):
        paths.append([paths[-1][j]+squared_velocity_path(i, lamda_max, joint_deltas[j]) for j in range(6)])
    for i in range(lamda_max):
        for j in range(6):
            paths[i][j] += CURRENT_ANGLES[j]
    
    for jpos in paths:
        arm.set_joints(jpos)
        rate.sleep()

def handle_xyz( msg ):
    global CURRENT_ANGLES
    lamda_max = 100
    # get current xyz
    dofbot.angles = [x for x in CURRENT_ANGLES]
    initial_xyz = dofbot.ee
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
    joint_paths = []
    for i in range(lamda_max):
        dofbot.ee = [x for x in xyz_paths[i]]
        joint_paths.append(dofbot.angles)

    # move
    for jpos in joint_paths:
        arm.set_joints(jpos)
        rate.sleep()
        

def handle_setting_torque( msg ):
    arm.set_torque( msg.data )


rospy.init_node( NODE_NAME )
rate = rospy.Rate(50)
pub = rospy.Publisher( READ_JOINT_MSG, Float32MultiArray, queue_size=10 )
rospy.Subscriber( SET_JOINT_MSG, Float32MultiArray, handle_write_angles )
rospy.Subscriber( SET_JOINT_PATH_MSG, Float32MultiArray, handle_path )
rospy.Subscriber( MOVE_JOINT_MSG, Float32MultiArray, handle_move_angles )
rospy.Subscriber( SET_TORQUE_MSG, Bool, handle_setting_torque )
rospy.Subscriber( SET_XYZ_MSG, Float32MultiArray, handle_xyz )

arm = driver.Arm_Dev()

try:
    arm.start( handle_read_angles )
    rospy.spin()

except KeyboardInterrupt:
    pass
finally:
    arm.stop()
