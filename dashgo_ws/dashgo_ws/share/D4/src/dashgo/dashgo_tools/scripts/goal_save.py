#!/usr/bin/env python
#coding=utf-8
import rospy
import math
from geometry_msgs.msg import PointStamped,PoseStamped
from move_base_msgs.msg import *
import json_file

#单点目标点连续点击（自定义方向）
def goal_callback(msg):
    move_pose=msg.goal.target_pose.pose
    #保存目标点
    goal={"position": {"x":move_pose.position.x,"y":move_pose.position.y,"z":move_pose.position.z}, "quaternion": {"x":move_pose.orientation.x,"y":move_pose.orientation.y,"z":move_pose.orientation.z,"w":move_pose.orientation.w}} 
    file_name="goal";
    json_file.json_append(goal , file_name)
    print 'save a path goal point'
    

rospy.init_node('path_point_save')
goal_sub = rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,goal_callback)
file_name="goal";
json_file.json_clear(file_name)
rospy.spin()

