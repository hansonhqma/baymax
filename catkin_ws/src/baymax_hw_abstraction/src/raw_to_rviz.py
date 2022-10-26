#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

def handler( angles ):
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    msg.position = angles.data[:-1]
    pub.publish( msg )

rospy.init_node( "echo_node" )
pub = rospy.Publisher( '/joint_states', JointState, queue_size=10 )
rospy.Subscriber( "/base/read_joints", Float32MultiArray, handler )

rospy.spin()
