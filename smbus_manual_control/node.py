import driver
import math
import time
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

def handle_read_angles( angles ):
    msg = Float32MultiArray()
    msg.data = tuple( angles )
    pub.publish( msg )

def handle_write_angles( msg ):
    print( "received" )
    if( len( msg.data ) == 6 ):
        for angle in msg.data:
            if( type( angle ) != float ):
                return
        arm.set_joints( list( msg.data ) )

def handle_setting_torque( msg ):
    arm.set_torque( msg.data )

rospy.init_node( "smbus_node" )
pub = rospy.Publisher( '/read_joints', Float32MultiArray, queue_size=10 )
rospy.Subscriber( "/set_joints", Float32MultiArray, handle_write_angles )
rospy.Subscriber( "/set_torque", Bool, handle_setting_torque )

arm = driver.Arm_Dev()

try:
    arm.start( handle_read_angles )
    rospy.spin()

except KeyboardInterrupt:
    pass
finally:
    arm.stop()

