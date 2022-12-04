#!/usr/bin/python3
import driver
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import geometry_msgs.msg

import numpy as np
import tinyik
import naive_fk
import fk_ik_jq as kin

from pathgen import squared_velocity_path

# naive ik approach

base_to_tool = [[0, 0, 0.066], 'z', [0, 0, 0.0415], 'x', [0, 0, 0.0828], 'x', [0, 0, 0.0828], 'x', [0, 0, 0.0739], 'z', [0, 0, 0.08]]
dofbot_base_to_tool = tinyik.Actuator(base_to_tool)
dofbot_base_to_tool_broadcaster = tinyik.Actuator(base_to_tool)

# ROSpy node/msg names
NODE_NAME = "hw_layer_node"

READ_JOINT_MSG = "/base/read_joints"
BASE_TO_TOOL_TF_MSG = "/tf/base_to_tool"
BASE_TO_CAM_TF_MSG = "/tf/base_to_cam"

SET_JOINT_MSG = "/base/set_joints"
SET_JOINT_PATH_MSG = "/base/set_joints_path"
SET_XYZ_MSG = "/base/set_xyz"
MOVE_JOINT_MSG = "/base/move_joints"

SET_TORQUE_MSG = "/base/set_torque"
TASKID_TOPIC = "/base/taskid"

CURRENT_ANGLES = None

def handle_read_angles( angles ):
    # publish joint angles
    global CURRENT_ANGLES
    msg = Float32MultiArray()
    msg.data = tuple( angles )
    joint_angle_publisher.publish( msg )
    CURRENT_ANGLES = angles

    # base to tool tf
    tf_basetotool = geometry_msgs.msg.Pose()
    pos_tool = naive_fk.tf_base_to_tool(*(angles[:5])).reshape((3))
    rot_tool = naive_fk.rotmatrix_to_quaternion(naive_fk.rot_base_to_tool(*(angles[:5])))
    tf_basetotool.position.x = pos_tool[0]
    tf_basetotool.position.y = pos_tool[1]
    tf_basetotool.position.z = pos_tool[2]

    # base to cam tf
    tf_basetocam = geometry_msgs.msg.Pose()
    pos_cam = naive_fk.tf_base_to_cam(*(angles[:4])).reshape((3))
    rot_cam = naive_fk.rotmatrix_to_quaternion(naive_fk.rot_base_to_cam(*(angles[:4])))

    tf_basetocam.orientation.x = rot_cam[0]
    tf_basetocam.orientation.y = rot_cam[1]
    tf_basetocam.orientation.z = rot_cam[2]
    tf_basetocam.orientation.w = rot_cam[3]
    
    tf_basetocam.position.x = pos_cam[0]
    tf_basetocam.position.y = pos_cam[1]
    tf_basetocam.position.z = pos_cam[2]

    base_to_tool_tf_publisher.publish(tf_basetotool)
    base_to_cam_tf_publisher.publish(tf_basetocam)
    
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
    for i in range(len(paths)):
        for j in range(6):
            paths[i][j] += CURRENT_ANGLES[j]
    
    for jpos in paths:
        arm.set_joints(jpos)
        rate.sleep()

    # publish completed task id
    completed_task_id_msg = Int32()
    completed_task_id_msg.data = msg.layout.data_offset
    taskid_publisher.publish(completed_task_id_msg)

def handle_xyz( msg:Float32MultiArray ):
    global CURRENT_ANGLES
    lamda_max = 100
    # get current xyz
    initial_xyz = naive_fk.tf_base_to_tool(*(CURRENT_ANGLES[:5]))
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
    joint_paths.append(list(kin.DOFBOT.ik(2.5, np.array(xyz_paths[0]), np.array(CURRENT_ANGLES[:5])))+[0])
    for i in range(1, lamda_max):
        joint_paths.append(list(kin.DOFBOT.ik(2.5, np.array(xyz_paths[i]), np.array(joint_paths[-1][:5])))+[0])

    # move
    for jpos in joint_paths:
        arm.set_joints(jpos)
        rate.sleep()
    
    # publish completed task id
    completed_task_id_msg = Int32()
    completed_task_id_msg.data = msg.layout.data_offset
    taskid_publisher.publish(completed_task_id_msg)
        

def handle_setting_torque( msg ):
    arm.set_torque( msg.data )

rospy.init_node( NODE_NAME )
rate = rospy.Rate(50)
joint_angle_publisher = rospy.Publisher( READ_JOINT_MSG, Float32MultiArray, queue_size=10 )
base_to_tool_tf_publisher = rospy.Publisher( BASE_TO_TOOL_TF_MSG, geometry_msgs.msg.Pose, queue_size=10 )
base_to_cam_tf_publisher = rospy.Publisher( BASE_TO_CAM_TF_MSG, geometry_msgs.msg.Pose, queue_size=10 )
taskid_publisher = rospy.Publisher( TASKID_TOPIC, Int32, queue_size=1)
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
