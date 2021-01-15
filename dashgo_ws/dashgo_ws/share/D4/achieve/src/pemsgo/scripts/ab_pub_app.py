#!/usr/bin/env python
#coding=utf-8

import v_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import rospy
#import json_file
import time

rospy.init_node('path_ab')


def callback_vmsg_marker(vm):
    marker = Marker()
    marker.id = vm.id
    marker.header = vm.header

    marker.header.frame_id = "/odom_combined"
    marker.ns = "odom_strip"
    marker.action = vm.action
    marker.type = vm.type
    marker.scale = vm.scale
    marker.pose = vm.pose;
    marker.color = vm.color
    marker.points = vm.points
    mark_pub.publish(marker)


# init msgs
rospy.Subscriber("/strip_ab", v_msgs.msg.Marker, callback_vmsg_marker)
mark_pub = rospy.Publisher('/path_ab', Marker,queue_size=10)

file_name="goals_app";
goals=list()
#goals=json_file.json_read_lines(goals , file_name)

rospy.loginfo("publish marker path_ab now.")
rospy.spin()

