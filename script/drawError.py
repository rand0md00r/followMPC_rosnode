#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import os

desired_pose = None
odom_pose = None
cmd_vel = None

global position_error
global time
global start_time
global linear_velocity
global angular_velocity
global dx_array
global dy_array
global odom_x
global odom_y
global desire_x
global desire_y
global target_x
global target_y

def desired_pose_callback(msg):
    global desired_pose
    desired_pose = msg.pose

def odom_callback(msg):
    global odom_pose
    odom_pose = msg.pose.pose

def cmd_vel_callback(msg):
    global cmd_vel
    cmd_vel = msg

def target_callback(msg):
    global target_pose
    target_pose = msg.pose

def calculate_rmse(desire_x, desire_y, target_x, target_y):
    # 确保输入的列表长度相等
    assert len(desire_x) == len(desire_y) == len(target_x) == len(target_y), "输入列表长度不一致"

    # 计算平方误差的累加和
    sum_squared_error = 0.0
    for dx, dy, tx, ty in zip(desire_x, desire_y, target_x, target_y):
        squared_error = (dx - tx) ** 2 + (dy - ty) ** 2
        sum_squared_error += squared_error

    # 计算均方根误差（RMSE）
    rmse = math.sqrt(sum_squared_error / len(desire_x))
    return rmse

def plot_position_error():
    # rospy.init_node('position_error_plot')

    # Create empty lists to store position error, time, linear velocity, and angular velocity
    position_error = []
    time = []
    start_time = rospy.Time.now().to_sec()
    linear_velocity = []
    angular_velocity = []
    dx_array = []
    dy_array = []
    odom_x = []
    odom_y = []
    desire_x = []
    desire_y = []
    target_x = []
    target_y = []
    dx_list = []
    dy_list = []

    while not rospy.is_shutdown():
        if desired_pose is not None and odom_pose is not None and cmd_vel is not None and target_pose is not None:
            # Calculate position error
            dx = desired_pose.position.x - odom_pose.position.x
            dy = desired_pose.position.y - odom_pose.position.y
            dx_array.append(dx)
            dy_array.append(dy)
            error = np.sqrt(dx**2 + dy**2)


            # Append position error, time, linear velocity, and angular velocity to the lists
            position_error.append(error)
            time.append(rospy.Time.now().to_sec() - start_time)
            linear_velocity.append(cmd_vel.linear.x)
            angular_velocity.append(cmd_vel.angular.z)
            odom_x.append(odom_pose.position.x)
            odom_y.append(odom_pose.position.y)
            desire_x.append(desired_pose.position.x)
            desire_y.append(desired_pose.position.y)
            target_x.append(target_pose.position.x)
            target_y.append(target_pose.position.y)
            dx_list.append(abs(dx))
            dy_list.append(abs(dy))

            # Clear the current plot
            plt.clf()

            # plt.plot(time, position_error, 'r-',label='Position Error')
            # plt.xlabel('Time')
            # plt.ylabel('Position Error')
            # plt.title('Position Error Plot')
            # # plt.ylim(0, 0.1)
            # plt.grid(True)
            # plt.legend()
            # image_name = "circle_diff_start_point_error.svg"

            # plt.plot(odom_x, odom_y, color='#66AAEE',label='Proposed')
            # plt.plot(desire_x, desire_y, color='#EE66AA',label='Desired Path')
            # # plt.plot(target_x, target_y, color='#66DD22',label='Target Path')
            # plt.xlabel('Position x')
            # plt.ylabel('Position y')
            # plt.axis('equal')  # 设置纵横比
            # plt.title('Position Plot')
            # plt.grid(True)
            # plt.legend()
            # image_name = "circle_diff_start_point.svg"

            plt.plot(time, dy_list, 'b-',label='Lateral Error')
            plt.xlabel('Time')
            plt.ylabel('Lateral Error')
            plt.title('Position Error Plot')
            # plt.ylim(0, 0.1)
            plt.grid(True)
            plt.legend()
            image_name = "Lateral_error.svg"


            # plt.plot(time, linear_velocity, 'b-',label='Target Path') 
            # plt.xlabel('Time')
            # plt.ylabel('Linear Velocity')
            # plt.title('Linear Velocity')
            # # plt.ylim(0, 1)
            # plt.grid(True)
            # plt.legend()
            # image_name = "line_vel.svg"

            # plt.plot(time, angular_velocity, 'g-',label='Target Path')
            # plt.xlabel('Time')
            # plt.ylabel('Linear Velocity')
            # plt.title('Linear Velocity')
            # # plt.ylim(-1, 1)
            # plt.grid(True)
            # plt.legend()
            # image_name = "ang_vel.svg"

            # Adjust plot layout
            plt.tight_layout()

            # Save the current plot as a vector graphics file (SVG format) with fixed width and height
            save_path_1 = os.path.join(os.getcwd(), "/home/bag/compare_test/" + image_name)
            plt.savefig(save_path_1, format='svg', dpi=300, bbox_inches='tight')

            # Update the plot
            plt.pause(0.1)

        rospy.sleep(0.1)

    rmse = calculate_rmse(desire_x, desire_y, odom_x, odom_y)
    print("\n\n\n")
    print("RMSE:", rmse)
    print("\n\n\n")

    # # return odom_x, odom_y, desire_x, desire_y
    # file_path = "/home/bag/compare_test/errortheta__bad.txt"
    # data = np.column_stack((time, odom_x, odom_y, desire_x, desire_y))
    # np.savetxt(file_path, data, delimiter=',')
    


def save_data_to_file(file_path):
    data = np.column_stack((position_error, time, linear_velocity, angular_velocity, odom_x, odom_y, desire_x, desire_y, target_x, target_y))
    np.savetxt(file_path, data, delimiter=',')

if __name__ == '__main__':
    rospy.init_node('draw_error_node')
    _desire_pose_topic = rospy.get_param('~desirePose_topic', '/desire_pose')
    _odom_topic = rospy.get_param('~odom_topic', '/odom_gazebo')
    _cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
    _target_topic = rospy.get_param('~target_topic', '/target_point/pose')
    print(_target_topic)
    
    rospy.Subscriber(_desire_pose_topic, PoseStamped, desired_pose_callback)
    rospy.Subscriber(_odom_topic, Odometry, odom_callback)
    rospy.Subscriber(_cmd_topic, Twist, cmd_vel_callback)
    rospy.Subscriber(_target_topic, PoseStamped, target_callback)

    # Set the figure size and position
    fig = plt.figure(figsize=(8, 8))
    fig.canvas.set_window_title('Position Error Plot')
    # fig.subplots_adjust(left=0.4, bottom=0.0, right=1.0, top=0.9, wspace=0.4, hspace=0.4)

    # Plot position error, linear velocity, and angular velocity
    plt.ion()
    plot_position_error()
    
    plt.ioff()

    plt.show()
    save_data_to_file("/home/bag/compare_test/theta_weight_bad.csv")