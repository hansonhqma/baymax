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

rospy.init_node( NODE_NAME )
pub = rospy.Publisher( READ_JOINT_MSG, Float32MultiArray, queue_size=10 )
rospy.Subscriber( SET_JOINT_MSG, Float32MultiArray, handle_write_angles )

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
