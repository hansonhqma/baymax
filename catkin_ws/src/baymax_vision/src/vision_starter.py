#!/usr/bin/python3
import rospy
import std_msgs.msg
import time

NODE_NAME = "vision starter"

VISION_START_TOPIC = "/vision/start"

rospy.init_node(NODE_NAME)

start = rospy.Publisher(VISION_START_TOPIC, std_msgs.msg.String, queue_size=1)

msg = std_msgs.msg.String()
msg.data = "start"

time.sleep(2)

start.publish(msg)