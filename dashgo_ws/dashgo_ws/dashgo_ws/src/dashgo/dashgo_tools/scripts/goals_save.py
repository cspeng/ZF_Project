#!/usr/bin/env python
#coding=utf-8

import rospy
import math
from geometry_msgs.msg import PointStamped,PoseStamped
from move_base_msgs.msg import *
import json_file

#单点目标点连续点击（自定义方向）
def click_callback(msg):
    #保存目标点
    goal={"position": {"x":msg.point.x,"y":msg.point.y,"z":msg.point.z}, "quaternion": {"x":0.0,"y":0.0,"z":0.0,"w":1.0}} 
    file_name="goals";
    json_file.json_append(goal , file_name)
    print 'save a path goal point'
    

rospy.init_node('path_point_save')
click_sub = rospy.Subscriber('/clicked_point',PointStamped,click_callback)
file_name="goals";
json_file.json_clear(file_name)
rospy.spin()

