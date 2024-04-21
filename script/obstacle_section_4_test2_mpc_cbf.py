#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin, atan2
import math
from tf.transformations import quaternion_from_euler

def generate_points(time, radius, center):
    
    for i in range(1):
        vel = 0.5
        theta = time + i * math.pi / 2
        x = center[i][0] + radius * math.cos(theta)
        y = center[i][1] + radius * math.sin(theta)
        points.append((x, y))
    return points

def gen_points_1(time, center):
    vel = 0.5
    # x = center[0]
    # y = center[1] - amplitude * math.sin(2 * math.pi * time / period)  # 使用正弦函数生成上下运动
    x = center[0] + time * vel 
    y = center[1]
    print("x:", x, " y:", y)
    return x, y

# def gen_points_2(time, center):
#     vel = 0.5
#     amplitude = 2.0  # 垂直方向上的振幅
#     period = 15.0  # 周期
#     x = center[0]
#     # y = center[1] + amplitude * math.sin(2 * math.pi * time / period)  # 使用正弦函数生成上下运动
#     y = center[1] - time * vel
#     return x, y

# def gen_points_3(time, center):
#     vel = 0.5
#     amplitude = 2.0  # 垂直方向上的振幅
#     period = 15.0  # 周期
#     x = center[0]
#     # y = center[1] - amplitude * math.sin(2 * math.pi * time / period)  # 使用正弦函数生成上下运动
#     y = center[1] + time * vel
#     return x, y

# def gen_points_4(time, center):
#     vel = 1.0
#     # x = center[0]
#     # y = center[1] - time * vel
#     x = center[0] - time * vel
#     y = center[1]
#     return x, y

def points_to_marker_array(points):
    marker_array = MarkerArray()
    for i, point in enumerate(points):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.5
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.id = i
        marker.ns = str(i)
        marker.pose.orientation.w = 1.0
        marker_array.markers.append(marker)
    return marker_array



rospy.init_node('dynamic_points_publisher', anonymous=True)
pub = rospy.Publisher('/obstacles', MarkerArray, queue_size=10)

# radius = 1.0
# center = [[0.0, 1.0], [-3.0, 4.0], [-3.0, -2.0], [-6.0, 1.0]]
center = [[0.0, -1.6], [10.0, 5.0], [15.0, -7.3], [30.0, 0.5]]

rate = rospy.Rate(10)
time = 0.0

while not rospy.is_shutdown():
    points = []
    points.append(gen_points_1(time, center[0]))
    # points.append(gen_points_2(time, center[1]))
    # points.append(gen_points_3(time, center[2]))
    # points.append(gen_points_4(time, center[3]))
    marker_array = points_to_marker_array(points)
    pub.publish(marker_array)
    time += 0.1
    rate.sleep()