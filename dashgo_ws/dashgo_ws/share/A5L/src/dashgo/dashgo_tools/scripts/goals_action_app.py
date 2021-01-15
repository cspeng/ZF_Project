#!/usr/bin/env python
#coding=utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped,PoseStamped
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
import json_file
import time

#多点循环导航
    

#导航目标点状态处理
def status_callback(msg):
    global goal_pub, index,markerArray
    global try_again

    if(msg.status.status == 3):
        try_again = 1
        print 'Goal reached'
        if index == count:
            index = 0

        pose = PoseStamped()
        pose.header.frame_id = "/map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = markerArray.markers[index].pose.position.x
        pose.pose.position.y = markerArray.markers[index].pose.position.y
        pose.pose.position.z = markerArray.markers[index].pose.position.z
        pose.pose.orientation.x = markerArray.markers[index].pose.orientation.x
        pose.pose.orientation.y = markerArray.markers[index].pose.orientation.y
        pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
        pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
        goal = MoveBaseActionGoal()
        goal.header = pose.header
        goal.goal_id.id = "move_base/move_base_client_loop"+str(rospy.Time.now())
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal.target_pose = pose
        action_goal_pub.publish(goal)

        index += 1
    else:
        
        if try_again == 1:
            print 'Goal cannot reached has some error :',msg.status.status," try again!!!!"
            index = index-1;
            try_again = 0
        else:
            print 'Goal cannot reached has some error :',msg.status.status," again , now go to next goal!!!!"
            if index == len(markerArray.markers):
                index=0

        pose = PoseStamped()
        pose.header.frame_id = "/map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = markerArray.markers[index].pose.position.x
        pose.pose.position.y = markerArray.markers[index].pose.position.y
        pose.pose.position.z = markerArray.markers[index].pose.position.z
        pose.pose.orientation.x = markerArray.markers[index].pose.orientation.x
        pose.pose.orientation.y = markerArray.markers[index].pose.orientation.y
        pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
        pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
        goal = MoveBaseActionGoal()
        goal.header = pose.header
        goal.goal_id.id = "move_base/move_base_client_loop"+str(rospy.Time.now())
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal.target_pose = pose
        action_goal_pub.publish(goal)

        index += 1

markerArray = MarkerArray()
count = 0       #total goal num
index = 0       #current goal point index
try_again = 1  # try the fail goal once again

rospy.init_node('path_point_loop')

mark_pub = rospy.Publisher('/path_point', MarkerArray,queue_size=100)
action_goal_pub = rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=1)
goal_status_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,status_callback)

file_name="goals_app";
goals=list()
goals=json_file.json_read_lines(goals , file_name)
for goal in goals:
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = goal[u"position"][u"x"]
    marker.pose.position.y = goal[u"position"][u"y"]
    marker.pose.position.z = goal[u"position"][u"z"]
    marker.pose.orientation.x = goal[u"quaternion"][u"x"]
    marker.pose.orientation.y = goal[u"quaternion"][u"y"]
    marker.pose.orientation.z = goal[u"quaternion"][u"z"]
    marker.pose.orientation.w = goal[u"quaternion"][u"w"]
    count=count+1
    marker.text = str(count)
    marker.id = len(markerArray.markers);
    markerArray.markers.append(marker)

time.sleep(1)
#mark_pub.publish(markerArray)

pose = PoseStamped()
pose.header.frame_id = "/map"
pose.header.stamp = rospy.Time.now()
pose.pose.position.x = markerArray.markers[index].pose.position.x
pose.pose.position.y = markerArray.markers[index].pose.position.y
pose.pose.position.z = markerArray.markers[index].pose.position.z
pose.pose.orientation.x = markerArray.markers[index].pose.orientation.x
pose.pose.orientation.y = markerArray.markers[index].pose.orientation.y
pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
goal = MoveBaseActionGoal()
goal.header = pose.header
goal.goal_id.id = "move_base/move_base_client_loop"+str(rospy.Time.now())
goal.goal_id.stamp = rospy.Time.now()
goal.goal.target_pose = pose
action_goal_pub.publish(goal)
index += 1

rospy.spin()

