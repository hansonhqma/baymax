#!/usr/bin/python3
import driver
import math
import time
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

# ROSpy node/msg names
NODE_NAME = "smbus_node"
READ_JOINT_MSG = "/base/read_joints"
SET_JOINT_MSG = "/base/set_joints"
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

def handle_setting_torque( msg ):
    arm.set_torque( msg.data )

rospy.init_node( NODE_NAME )
pub = rospy.Publisher( READ_JOINT_MSG, Float32MultiArray, queue_size=10 )
rospy.Subscriber( SET_JOINT_MSG, Float32MultiArray, handle_write_angles )
rospy.Subscriber( SET_TORQUE_MSG, Bool, handle_setting_torque )

arm = driver.Arm_Dev()

try:
    arm.start( handle_read_angles )
    rospy.spin()

except KeyboardInterrupt:
    pass
finally:
    arm.stop()

