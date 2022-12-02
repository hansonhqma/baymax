#!/usr/bin/python3
import driver
import math
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

from pathgen import squared_velocity_path

# ROSpy node/msg names
NODE_NAME = "smbus_node"
READ_JOINT_MSG = "/base/read_joints"
SET_JOINT_MSG = "/base/set_joints"
MOVE_JOINT_MSG = "/base/move_joints"
SET_TORQUE_MSG = "/base/set_torque"
SET_JOINT_PATH_MSG = "/base/set_joints_path"

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
    

        

def handle_setting_torque( msg ):
    arm.set_torque( msg.data )


rospy.init_node( NODE_NAME )
rate = rospy.Rate(50)
pub = rospy.Publisher( READ_JOINT_MSG, Float32MultiArray, queue_size=10 )
rospy.Subscriber( SET_JOINT_MSG, Float32MultiArray, handle_write_angles )
rospy.Subscriber( SET_JOINT_PATH_MSG, Float32MultiArray, handle_path )
rospy.Subscriber( MOVE_JOINT_MSG, Float32MultiArray, handle_move_angles )
rospy.Subscriber( SET_TORQUE_MSG, Bool, handle_setting_torque )

arm = driver.Arm_Dev()

try:
    arm.start( handle_read_angles )
    rospy.spin()

except KeyboardInterrupt:
    pass
finally:
    arm.stop()
