#!/usr/bin/python3
import rospy
import std_msgs.msg
import geometry_msgs.msg

import random

# ros info
NODE_NAME = "motion_controller"

rospy.init_node(NODE_NAME)
rate = rospy.Rate(50)

# subscribed topics
CURRENT_TASK_TOPIC = "/state/current_task"
TARGET_POS_TF = "/vision/targetpos"
BASE_READ_JOINTS_TOPICS = "/base/read_joints"
TASKID_TOPIC = "/base/taskid"
TARGETID_TOPIC = "/state/target_id"

# published topics
BASE_SET_JOINTS_RAW_TOPIC = "/base/set_joints"
BASE_SET_JOINTS_TOPIC = "/base/set_joints_path"
BASE_SET_XYZ_TOPIC = "/base/set_xyz"
BASE_SET_XYZ_OBSTACLE_TOPIC = "/base/set_xyz_obstacle"
BASE_ENABLE_TORQUE = "/base/set_torque"

# preset positions
STANDBY_JOINT_CONFIG = [0, 0.3, -0.8, -2.3, 0, 0]
ZERO_CONFIG = [0] * 6
DEPOSIT_JOINT_CONFIG = [-2.1, 1, -1.5, -1, 0, 0]

STANDBY_LEFT_CONFIG = [1, 0.3, -0.8, -2.3, 0, 0]
STANDBY_RIGHT_CONFIG = [-1, 0.3, -0.8, -2.3, 0, 0]

dance1 = [2, -0.5, 0.8, 1.7, 0, 0]
dance2 = [-1*x for x in dance1]

# node state info
CURRENT_TASK = None
OPEN_TASKS_IDS = set()
TFS_BASE_TO_TARGET = {
    "red":None,
    "blue":None,
    "green":None,
    "yellow":None
}

TARGET_ID = None # color

CURRENT_JOINTS = None

def current_task_handler ( msg ):
    global CURRENT_TASK
    # update current task
    if msg.data == CURRENT_TASK and not msg.data == "grasp" or msg.data == "identify_target":
        # redundant except for grasp
        return

    CURRENT_TASK = msg.data

    state_msg = std_msgs.msg.Float32MultiArray()
    if CURRENT_TASK == "standby":
        state_msg.data = STANDBY_JOINT_CONFIG
        state_msg.data[5] = CURRENT_JOINTS[5]
        print("Standby")

    elif CURRENT_TASK == "zero":
        state_msg.data = ZERO_CONFIG
        state_msg.data[5] = CURRENT_JOINTS[5]
        print("Zero")
    
    elif CURRENT_TASK == "deposit":
        state_msg.data = DEPOSIT_JOINT_CONFIG
        state_msg.data[5] = CURRENT_JOINTS[5]
        print("Deposit")

    elif CURRENT_TASK == "reach":
        # needs target pos msg
        if TFS_BASE_TO_TARGET[TARGET_ID] == None:
            print("motion controller not yet received target pose")
            return
        
        posemsg = TFS_BASE_TO_TARGET.get(TARGET_ID)
        
        state_msg.data = [
            posemsg.pose.position.x,
            posemsg.pose.position.y,
            posemsg.pose.position.z,
        ]

        set_xyz_publisher.publish(state_msg)
        return
    
    elif CURRENT_TASK == "grasp":
        state_msg.data = [x for x in CURRENT_JOINTS]
        if state_msg.data[5] <= 0:
            state_msg.data[5] = 1.01 # close
        else:
            state_msg.data[5] = -1.5708

        set_joints_raw_publisher.publish(state_msg)
        return
    
    elif CURRENT_TASK == "dance":

        for pose in (STANDBY_JOINT_CONFIG, dance1, dance2, dance1, STANDBY_JOINT_CONFIG):
            current_task_id = get_new_taskid()
            OPEN_TASKS_IDS.add(current_task_id)
            state_msg.data = pose

            # add taskid to msg and publish
            state_msg.layout.data_offset = current_task_id
            set_joints_publisher.publish(state_msg)
            # while task is still open, sleep
            while current_task_id in OPEN_TASKS_IDS:
                rate.sleep()
        
        return
    
    elif CURRENT_TASK == 'sweep':
        while CURRENT_TASK == 'sweep':
            print("identifying...")
            current_task_id = get_new_taskid()
            OPEN_TASKS_IDS.add(current_task_id)
            state_msg.data = STANDBY_LEFT_CONFIG

            # add taskid to msg and publish
            state_msg.layout.data_offset = current_task_id
            state_msg.layout.dim = [std_msgs.msg.MultiArrayDimension()]
            state_msg.layout.dim[0].size = 300
            print("running taskid {}".format(current_task_id))
            set_joints_publisher.publish(state_msg)
            # while task is still open, sleep
            while current_task_id in OPEN_TASKS_IDS:
                rate.sleep()
            print("finished taskid {}".format(current_task_id))
            
            # done, do an intermediate check
            if not CURRENT_TASK == 'sweep':
                break

            current_task_id = get_new_taskid()
            OPEN_TASKS_IDS.add(current_task_id)
            state_msg.data = STANDBY_RIGHT_CONFIG

            # add taskid to msg and publish
            state_msg.layout.data_offset = current_task_id
            state_msg.layout.dim = [std_msgs.msg.MultiArrayDimension()]
            state_msg.layout.dim[0].size = 300
            print("running taskid {}".format(current_task_id))
            set_joints_publisher.publish(state_msg)
            # while task is still open, sleep
            while current_task_id in OPEN_TASKS_IDS:
                rate.sleep()
            print("finished taskid {}".format(current_task_id))

        return

    set_joints_publisher.publish(state_msg)

def complete_task_id_handler( msg ):
    # removes completed task id from open tasks
    global OPEN_TASKS_IDS
    if msg.data in OPEN_TASKS_IDS:
        OPEN_TASKS_IDS.remove(msg.data)

def tf_base_to_target_handler( msg:geometry_msgs.msg.PoseStamped ):
    global TFS_BASE_TO_TARGET
    arucoid = msg.header.frame_id
    TFS_BASE_TO_TARGET[arucoid] = msg

def read_joints_handler( msg ):
    global CURRENT_JOINTS
    CURRENT_JOINTS = msg.data

def targetid_handler( msg ):
    global TARGET_ID
    TARGET_ID = msg.data
    print("motion controller recevied new target id: {}".format(TARGET_ID))

def get_new_taskid():
    global OPEN_TASKS_IDS
    for i in range(1000):
        gen = random.randint(0, 255)
        if gen not in OPEN_TASKS_IDS: # valid task id
            return gen

    return None

    
    

rospy.Subscriber(CURRENT_TASK_TOPIC, std_msgs.msg.String, current_task_handler)
rospy.Subscriber(TASKID_TOPIC, std_msgs.msg.Int32, complete_task_id_handler)
rospy.Subscriber(TARGETID_TOPIC, std_msgs.msg.String, targetid_handler )
rospy.Subscriber(TARGET_POS_TF, geometry_msgs.msg.PoseStamped, tf_base_to_target_handler)
rospy.Subscriber(BASE_READ_JOINTS_TOPICS, std_msgs.msg.Float32MultiArray, read_joints_handler)
set_joints_publisher = rospy.Publisher(BASE_SET_JOINTS_TOPIC, std_msgs.msg.Float32MultiArray, queue_size=1)
set_joints_raw_publisher = rospy.Publisher(BASE_SET_JOINTS_RAW_TOPIC, std_msgs.msg.Float32MultiArray, queue_size=1)
enable_torque_publisher = rospy.Publisher(BASE_ENABLE_TORQUE, std_msgs.msg.Bool, queue_size=1)
set_xyz_publisher = rospy.Publisher(BASE_SET_XYZ_TOPIC, std_msgs.msg.Float32MultiArray, queue_size=1)
set_xyz_obstacle_publisher = rospy.Publisher(BASE_SET_XYZ_OBSTACLE_TOPIC, std_msgs.msg.Float32MultiArray, queue_size=1)

torque_on = std_msgs.msg.Bool()
torque_on.data = "true"
enable_torque_publisher.publish(torque_on)

rospy.spin()