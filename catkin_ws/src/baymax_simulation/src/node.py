#!/usr/bin/python3
import math
import time
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
import threading

# ROSpy node/msg names
NODE_NAME = "smbus_sim_node"
READ_JOINT_MSG = "/base/read_joints"
SET_JOINT_MSG = "/base/set_joints"
SET_JOINT_VEL = "/base/set_joints_vel"

VEL_SIGNATURE = 0

def write_angles():
    while not terminate:
        msg = Float32MultiArray()
        msg.data = tuple( joints )
        pub.publish( msg )
        time.sleep( 0.05 )

def handle_write_angles( msg ):
    global joints
    joints = msg.data
    print("rx")

def handle_write_vel( msg ):
    global joints
    global VEL_SIGNATURE
    global rate

    VEL_SIGNATURE += 1
    mysignature = VEL_SIGNATURE
    
    print("new velocity signature:", VEL_SIGNATURE)

    if list(msg.data)== [0.0] * 6: # stop
        return

    while mysignature == VEL_SIGNATURE:
        new_joints = [(msg.data[i] + joints[i])%(2*math.pi) for i in range(6)]
        joints = new_joints
        time.sleep(0.02)

rospy.init_node( NODE_NAME )
pub = rospy.Publisher( READ_JOINT_MSG, Float32MultiArray, queue_size=10 )
rospy.Subscriber( SET_JOINT_MSG, Float32MultiArray, handle_write_angles )
rospy.Subscriber( SET_JOINT_VEL, Float32MultiArray, handle_write_vel )

terminate = False
joints = [ 0.0 ] * 6

t = threading.Thread( target = write_angles )
t.start()

try:
    rospy.spin()
    rate = rospy.Rate(50)
except KeyboardInterrupt:
    pass
finally:
    terminate = True
    t.join()
