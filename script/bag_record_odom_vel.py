#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger_node', anonymous=True)
        self.odom_sub = rospy.Subscriber("/odom_gazebo", Odometry, self.odom_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.desire_pose_sub = rospy.Subscriber("/desire_pose", PoseStamped, self.desire_pose_callback)
        self.target_pose_sub = rospy.Subscriber("/target_point/pose", PoseStamped, self.target_pose_callback)
        self.pose_error_sub = rospy.Subscriber("/target_point/pose", PoseStamped, self.pose_error_callback)
        self.odom_data = None
        self.cmd_vel_data = None
        self.desire_pose_data = None
        self.target_pose_data = None
        self.pose_error = None

    def odom_callback(self, msg):
        self.odom_data = msg.pose.pose.position
        # self.save_to_file("/home/plot_graph/avoid_obstacle/TEB/odom_data.txt", "Odometry Position", msg.pose.pose.position)
        # print("write to file: odom_data.txt")

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = [msg.linear.x, msg.angular.z]
        # self.save_to_file("/home/plot_graph/avoid_obstacle/TEB/cmd_vel_data.txt", "Cmd Velocity (linear, angular)", self.cmd_vel_data)
        # print("write to file: cmd_vel_data.txt")

    def desire_pose_callback(self, msg):
        self.desire_pose_data = msg.pose.position
        # self.save_to_file("/home/plot_graph/avoid_obstacle/TEB/desire_pose_data.txt", "Desired Pose Position", self.desire_pose_data)
        # print("write to file: desire_pose_data.txt")
        
        if self.odom_data is None:
            return
        x_error = msg.pose.position.x - self.odom_data.x
        y_error = msg.pose.position.y - self.odom_data.y
        self.pose_error = math.sqrt(x_error**2 + y_error**2)
        self.save_error_to_file("/home/plot_graph/avoid_obstacle/pureMPC/pose_error_data.txt", self.pose_error - 0.5, self.odom_data.x)
        pass

    def target_pose_callback(self, msg):
        self.target_pose_data = msg.pose.position
        # self.save_to_file("/home/plot_graph/avoid_obstacle/TEB/target_pose_data.txt", "Target Pose Position", self.target_pose_data)
        # print("write to file: target_pose_data.txt")
        
    def pose_error_callback(self, msg):
        # if self.odom_data is None:
        #     return
        # x_error = msg.pose.position.x - self.odom_data.x
        # y_error = msg.pose.position.y - self.odom_data.y
        # self.pose_error = math.sqrt(x_error**2 + y_error**2)
        # self.save_error_to_file("/home/plot_graph/avoid_obstacle/TEB_collision/pose_error_data.txt", self.pose_error - 1.5 , self.odom_data.x)
        
        pass

    def save_error_to_file(self, filename, error_data, pose_data):
        if not os.path.exists(filename):
            with open(filename, 'w'):
                pass
        with open(filename, 'a') as f:
            f.write("{}, {}\n".format(pose_data, error_data))


    # def save_to_file(self, filename, label, data):
    #     if not os.path.exists(filename):
    #         with open(filename, 'w'):
    #             pass
    #     with open(filename, 'a') as f:
    #         f.write("{}\n".format(data))

if __name__ == '__main__':
    data_logger = DataLogger()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()
