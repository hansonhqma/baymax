import rospy
from std_msgs.msg import Int32MultiArray

import Arm_Lib

dofbot = Arm_Lib.Arm_Lib.Arm_Device()

dofbot.Arm_serial_set_torque(1)

def move_arm(joint_data):
    joint_angles = joint_data.data
    for i in range(len(joint_angles)):
        joint_id = i+1
        dofbot.Arm_serial_servo_write(joint_id, joint_angles[i], 1)

def listener():
    rospy.init_node('child')
    rospy.Subscriber('/cmdr/joint_angles', Int32MultiArray, move_arm)

    rospy.spin()

listener()
