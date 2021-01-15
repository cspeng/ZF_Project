#!/usr/bin/env python
#coding=utf-8

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import json_file

#单点目标点连续点击（自定义方向）
def goals_callback(msg):
    #保存目标点
    markers = msg.markers
    for marker in markers:
        goal={"position": {"x":marker.pose.position.x,"y":marker.pose.position.y,"z":marker.pose.position.z}, "quaternion": {"x":marker.pose.orientation.x,"y":marker.pose.orientation.y,"z":marker.pose.orientation.z,"w":marker.pose.orientation.w}} 
        file_name="goals_app";
        json_file.json_append(goal , file_name)
        print 'save a path goal point'
    
markerArray = MarkerArray()
rospy.init_node('path_point_save_app')
mark_sub = rospy.Subscriber('/path_point',MarkerArray,goals_callback)
file_name="goals_app";
json_file.json_clear(file_name)
rospy.spin()

