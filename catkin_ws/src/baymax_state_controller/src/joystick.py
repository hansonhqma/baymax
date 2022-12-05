#!/usr/bin/python3
import rospy
import sensor_msgs.msg
import std_msgs.msg

NODE_NAME = "joystick_state_controller"

CONTROLLER_TOPIC = "/joy"

CURRENT_TASK_TOPIC = "/state/current_task"
TARGET_ID_TOPIC = "/state/target_id"

rospy.init_node(NODE_NAME)

def handle_input( msg:sensor_msgs.msg.Joy ):
    # buttons set targetid
    # dpad:
    #   down: zero
    #   left: standby
    #   right: identify_target

    # target id
    button_to_targetid_map = {
        0:"green",
        1:"red",
        2:"blue",
        3:"yellow"
    }
    for i in range(4):
        if msg.buttons[i] == 1:
            print("State controller publishing new target id: {}".format(button_to_targetid_map.get(i)))
            tgtmsg = std_msgs.msg.String()
            tgtmsg.data = button_to_targetid_map.get(i)
            target_id_pub.publish(tgtmsg)
            return
    
    # current task
    task_msg = std_msgs.msg.String()
    valid = False
    if msg.axes[0] == 1: # standby
        task_msg.data = "standby"
        valid = True
    
    elif msg.axes[0] == -1: # identify_target
        task_msg.data = "identify_target"
        valid = True
    
    elif msg.axes[1] == -1: # zero
        task_msg.data = "zero"
        valid = True
    
    if valid:
        print("new state: {}".format(task_msg.data))
        task_pub.publish(task_msg)

rospy.Subscriber(CONTROLLER_TOPIC, sensor_msgs.msg.Joy, handle_input)
target_id_pub = rospy.Publisher(TARGET_ID_TOPIC, std_msgs.msg.String, queue_size=1)
task_pub = rospy.Publisher(CURRENT_TASK_TOPIC, std_msgs.msg.String, queue_size=1)

rospy.spin()