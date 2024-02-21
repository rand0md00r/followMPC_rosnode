#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from math import cos, sin, atan2
import math
from tf.transformations import quaternion_from_euler
import numpy as np

def generate_trajectory(trajectory_type):

    if trajectory_type == "circle":
        radius = 3.0
        center_x = -3.0
        center_y = 1.0
        angular_speed = 0.16

        def calculate_pose(time):
            theta = angular_speed * time
            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "odom"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0

            # Calculate the tangent angle
            tangent_angle = math.atan2(radius * math.cos(theta), -radius * math.sin(theta))

            # Convert the tangent angle to a quaternion
            quaternion = quaternion_from_euler(0.0, 0.0, tangent_angle)

            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            return pose_msg
        return calculate_pose

    if trajectory_type == "infinity":
        angular_speed = 0.05
        center_x = -6
        center_y = 2

        def calculate_pose(time):
            theta = angular_speed * time
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "odom"
            pose_msg.pose.position.x = 5 * cos(theta) / (sin(theta) ** 2 + 1) + center_x
            pose_msg.pose.position.y = 5 * sin(theta) * cos(theta) / (sin(theta) ** 2 + 1) + center_y
            dx = 5 * cos(theta + angular_speed * 0.1) / (sin(theta + angular_speed * 0.1) ** 2 + 1) + center_x - pose_msg.pose.position.x + 1e-5
            dy = 5 * sin(theta + angular_speed * 0.1) * cos(theta + angular_speed * 0.1) / (sin(theta + angular_speed * 0.1) ** 2 + 1) + center_y - pose_msg.pose.position.y
            grad = np.arctan2(dy, dx)

            quaternion = quaternion_from_euler(0.0, 0.0, grad)

            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            return pose_msg
        
        return calculate_pose
    
    if trajectory_type == "epitrochoid":
        R = 5
        r = 1
        d = 3
        angular_speed = 0.03
        scale_factor = 0.7
        def calculate_pose(time):            
            theta = angular_speed * time
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "odom"
            pose_msg.pose.position.x = scale_factor * ((R + r) * cos(theta) - d * cos(((R + r) / r) * theta))
            pose_msg.pose.position.y = scale_factor * ((R + r) * sin(theta) - d * sin(((R + r) / r) * theta))
            next_theta = theta + angular_speed * 0.1
            # dx = (5 * cos((theta + 0.1)) / (sin((theta + 0.1)) ** 2 + 1) - pose_msg.pose.position.x + 1e-5)
            # dy = (5 * sin((theta + 0.1)) * cos((theta + 0.1)) / (sin((theta + 0.1)) ** 2 + 1)- pose_msg.pose.position.y)
            dx =(scale_factor * ((R + r) * cos(next_theta) - d * cos(((R + r) / r) * next_theta))) - pose_msg.pose.position.x + 1e-5
            dy =(scale_factor * ((R + r) * sin(next_theta) - d * sin(((R + r) / r) * next_theta))) - pose_msg.pose.position.y
            grad =  atan2( dy, dx)

            quaternion = quaternion_from_euler(0.0, 0.0, grad)

            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            return pose_msg
        
        return calculate_pose

    if trajectory_type == "line":
        start_x = -3.0
        start_y = 1.0
        velocity = 0.5
        distance = 6.0

        def calculate_pose(time):
            x = start_x + velocity * time
            y = start_y
            if x > start_x + distance:
                x = start_x + distance
            theta = 0.0  # 设置朝向为0
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "odom"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = sin(theta / 2.0)
            pose_msg.pose.orientation.w = cos(theta / 2.0)
            return pose_msg

    else:
        raise ValueError("Unsupported trajectory type")

    return calculate_pose


def publish_dynamic_point():
    rospy.init_node('dynamic_point_publisher', anonymous=True)
    trajectory_type = rospy.get_param('~trajectory_type')
    # trajectory_type = "infinity"
    print(trajectory_type)
    rate = rospy.Rate(10)  # 发布频率为10Hz

    # 创建发布器
    # 动目标点
    target_pub = rospy.Publisher('/target_point/pose', PoseStamped, queue_size=10)
    target_path_pub = rospy.Publisher('/target_point/path', PoseArray, queue_size=10)

    time = 0.0  # 初始时间
    frame = "odom"

    target_path_msg = PoseArray()
    target_path_msg.header.frame_id = frame

    calculate_pose = generate_trajectory(trajectory_type)

    while not rospy.is_shutdown():
        # 创建并填充动点的消息
        point_msg = calculate_pose(time)

        # 发布动点的消息
        target_pub.publish(point_msg)

        # 将动点的位置信息添加到历史路径中
        target_path_msg.poses.append(point_msg.pose)
        target_path_pub.publish(target_path_msg)

        time += 0.1  # 每次增加0.1秒，用于控制点的运动速度

        rate.sleep()



if __name__ == '__main__':
    
    try:
        publish_dynamic_point()
        # pass
    except rospy.ROSInterruptException:
        pass