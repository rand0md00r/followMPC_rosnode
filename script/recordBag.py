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
import csv


def read_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        # headers = lines[0].strip().split(',')
        for line in lines[0:]:
            values = line.strip().split(',')
            data.append(values)
    return data

data_bad_pose = read_file('/home/bag/compare_test/errortheta__bad.txt')
# headers_good_pose, data_good_pose = read_file('/home/bag/compare_test/theta_weight_good_pose.txt')
# headers_real_pose, data_real_pose = read_file('/home/bag/compare_test/theta_weight_desire_pose.txt')

print(data_bad_pose[0][0],data_bad_pose[0][1],data_bad_pose[0][2])

bad_x = []
bad_y = []
good_x = []
good_y = []
real_x = []
real_y = []


ct_bad = 0
for entry in data_bad_pose:
    bad_x.append(entry[0])
    bad_y.append(entry[1])
    # if (ct_bad == 1):
    #     bad_x.append(entry[0])
    #     bad_y.append(entry[1])
    #     ct_bad = 0
    # else:
    #     ct_bad += 1

# ct_good = 0
# for g in data_good_pose:
#     if (ct_good == 20):
#         good_x.append(g[5])
#         good_y.append(g[6])
#         ct_good = 0
#     else:
#         ct_good += 1

# for r in data_real_pose:
#     real_x.append(r[4])
#     real_y.append(r[5])

print(bad_x)
plt.clf()
plt.plot(bad_x, bad_y, color='#66AAEE',label='No Error Angle Constraint')
# plt.plot(good_x, good_y, color='#EE66AA',label='Angle Constraint')
# plt.plot(real_x, real_y, color='#66DD22',label='Desired Pose')
image_name = 'angle_constraint.svg'

plt.xlabel('Position x')
plt.ylabel('Position y')
plt.axis('equal')  # 设置纵横比
plt.title('Add Error Angle Constraint')
# plt.grid(True)
plt.legend()
plt.tight_layout()

save_path_1 = os.path.join(os.getcwd(), "/home/bag/compare_test/" + image_name)
plt.savefig(save_path_1, format='svg', dpi=300, bbox_inches='tight')
plt.show()
print("Draw over!")