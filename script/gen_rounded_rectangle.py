#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

def calculate_orientation(prev_point, curr_point):
    dx = curr_point[0] - prev_point[0]
    dy = curr_point[1] - prev_point[1]
    yaw = math.atan2(dy, dx)
    return yaw

def generate_rounded_rectangle(center_x, center_y, width, height, corner_radius, num_points):
    # 计算圆角矩形的参数
    half_width = width / 2
    half_height = height / 2

    # 计算圆角矩形的四个角点
    top_left = (center_x - half_width, center_y + half_height)
    top_right = (center_x + half_width, center_y + half_height)
    bottom_left = (center_x - half_width, center_y - half_height)
    bottom_right = (center_x + half_width, center_y - half_height)

    # 定义圆角矩形路径点
    path_points = []

    # 左上角圆弧段
    arc_center = (top_left[0] + corner_radius, top_left[1] - corner_radius)
    path_points.extend(generate_arc(arc_center, corner_radius, np.pi / 2, np.pi, num_points))

    # 左侧直线段
    path_points.extend(generate_line_segment(top_left[0], top_left[1] - corner_radius, bottom_left[0], bottom_left[1] + corner_radius, num_points))
    
    # 左下角圆弧段
    arc_center = (bottom_left[0] + corner_radius, bottom_left[1] + corner_radius)
    path_points.extend(generate_arc(arc_center, corner_radius, np.pi, np.pi * 3 / 2, num_points))

    # 底部直线段
    path_points.extend(generate_line_segment(bottom_left[0] + corner_radius, bottom_left[1], bottom_right[0] - corner_radius, bottom_right[1], num_points))

    # 右下角圆弧段
    arc_center = (bottom_right[0] - corner_radius, bottom_right[1] + corner_radius)
    path_points.extend(generate_arc(arc_center, corner_radius, np.pi * 3 / 2, 2 * np.pi, num_points))

    # 右侧直线段
    path_points.extend(generate_line_segment(bottom_right[0], bottom_right[1] + corner_radius, top_right[0], top_right[1] - corner_radius, num_points))
    
    # 右上角圆弧段
    arc_center = (top_right[0] - corner_radius, top_right[1] - corner_radius)
    path_points.extend(generate_arc(arc_center, corner_radius, 0, np.pi / 2, num_points))

    # 上侧直线段
    path_points.extend(generate_line_segment(top_right[0] - corner_radius, top_right[1], top_left[0] + corner_radius, top_left[1], num_points))

    # 转换为numpy数组
    path_points = np.array(path_points)

    return path_points

def generate_line_segment(start_x, start_y, end_x, end_y, num_points):
    x = np.linspace(start_x, end_x, num_points)
    y = np.linspace(start_y, end_y, num_points)
    return np.column_stack((x, y))

def generate_arc(center, radius, start_angle, end_angle, num_points):
    angles = np.linspace(start_angle, end_angle, num_points * 2 / np.pi)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return np.column_stack((x, y))

def publish_path_points(path_points):
    rospy.init_node('path_publisher', anonymous=True)
    pub = rospy.Publisher('/target_point/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 发布频率为10Hz
    prev_point = None

    for point in path_points:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = point[0]
        pose_msg.pose.position.y = point[1]

        if prev_point is not None:
            orientation = calculate_orientation(prev_point, point)
            pose_msg.pose.orientation.z = math.sin(orientation / 2.0)
            pose_msg.pose.orientation.w = math.cos(orientation / 2.0)
        else:
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
        prev_point = point

        pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    # 圆角矩形参数
    center_x = 0.0
    center_y = 0.0
    width = 10.0
    height = 10.0
    corner_radius = 2
    num_points = 100

    # 生成圆角矩形的路径点序列
    path_points = generate_rounded_rectangle(center_x, center_y, width, height, corner_radius, num_points)
    print(path_points)

    publish_path_points(path_points)