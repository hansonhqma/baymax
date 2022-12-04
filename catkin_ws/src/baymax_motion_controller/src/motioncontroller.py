#!/usr/bin/python3
import rospy
import std_msgs.msg

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

# published topics
BASE_SET_JOINTS_RAW_TOPIC = "/base/set_joints"
BASE_SET_JOINTS_TOPIC = "/base/set_joints_path"
BASE_SET_XYZ_TOPIC = "/base/set_xyz"

# preset positions
STANDBY_JOINT_CONFIG = [0, 0.8, -0.8, -2.3, 0, 0]
ZERO_CONFIG = [0] * 6

STANDBY_LEFT_CONFIG = [2, 0.8, -0.8, -2.3, 0, 0]
STANDBY_RIGHT_CONFIG = [-2, 0.8, -0.8, -2.3, 0, 0]

# node state info
CURRENT_TASK = None
OPEN_TASKS_IDS = set()

def current_task_handler ( msg ):
    global CURRENT_TASK
    # update current task
    if msg.data == CURRENT_TASK:
        # redundant
        return

    CURRENT_TASK = msg.data

    state_msg = std_msgs.msg.Float32MultiArray()
    if CURRENT_TASK == "standby":
        state_msg.data = STANDBY_JOINT_CONFIG
        print("Standby")

    elif CURRENT_TASK == "zero":
        state_msg.data = ZERO_CONFIG
        print("Zero")
    
    elif CURRENT_TASK == 'identify_target':
        print("i was here identifying")
        while CURRENT_TASK == 'identify_target':
            print("identifying...")
            current_task_id = get_new_taskid()
            OPEN_TASKS_IDS.add(current_task_id)
            state_msg.data = STANDBY_LEFT_CONFIG

            # add taskid to msg and publish
            state_msg.layout.data_offset = current_task_id
            print("running taskid {}".format(current_task_id))
            set_joints_publisher.publish(state_msg.data)
            # while task is still open, sleep
            while current_task_id in OPEN_TASKS_IDS:
                rate.sleep()
            print("finished taskid {}".format(current_task_id))
            
            # done, do an intermediate check
            if not CURRENT_TASK == 'identify_target':
                break

            current_task_id = get_new_taskid()
            OPEN_TASKS_IDS.add(current_task_id)
            state_msg.data = STANDBY_RIGHT_CONFIG

            # add taskid to msg and publish
            state_msg.layout.data_offset = current_task_id
            print("running taskid {}".format(current_task_id))
            set_joints_publisher.publish(state_msg.data)
            # while task is still open, sleep
            while current_task_id in OPEN_TASKS_IDS:
                rate.sleep()
            print("finished taskid {}".format(current_task_id))

        return

    set_joints_publisher.publish(state_msg)

def complete_task_id_handler( msg ):
    # removes completed task id from open tasks
    global OPEN_TASKS_IDS
    OPEN_TASKS_IDS.remove(msg.data)

def get_new_taskid():
    global OPEN_TASKS
    for i in range(1000):
        gen = random.randint(0, 255)
        if gen not in OPEN_TASKS: # valid task id
            return gen

    return None
    
    

rospy.Subscriber(CURRENT_TASK_TOPIC, std_msgs.msg.String, current_task_handler)
rospy.Subscriber(TASKID_TOPIC, std_msgs.msg.Int32, complete_task_id_handler)
set_joints_publisher = rospy.Publisher(BASE_SET_JOINTS_TOPIC, std_msgs.msg.Float32MultiArray, queue_size=1)

rospy.spin()