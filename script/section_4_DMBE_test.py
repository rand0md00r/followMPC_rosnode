# -*- coding: utf-8 -*-


import math
import matplotlib.pyplot as plt  
import numpy as np  
from matplotlib.patches import Ellipse  

DMBE_MAX_LENGTH_A = 2.0;
DMBE_MAX_LENGTH_B = 1.0;

def calculate_speed_and_heading(data):  
    # 结果列表，存储每个点的速度和航向角  
  
    # 时间间隔  
    time_interval = 0.4  
  
    # 遍历二维轨迹中的所有点，除了最后一个  
    for i in range(len(data) - 1):  
        x1, y1 = data[i]      # 当前点坐标  
        x2, y2 = data[i + 1]  # 下一个点坐标  
  
        # 计算速度，使用欧几里得距离除以时间  
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  
        speed = distance / time_interval  
  
        # 计算航向角，使用atan2函数并转换为度  
        heading_rad = math.atan2(y2 - y1, x2 - x1)  
        
        data[i].append(round(speed, 4))
        data[i].append(round(heading_rad, 4))
    
    return data 

def get_dmbe(data):
    dmbe = []
    print("data:", data)
    alpha = 0.1;
    for i in range(len(data) - 1):
        # a = DMBE_MAX_LENGTH_A / (1 + math.exp(-alpha * data[i][2] ))
        a = DMBE_MAX_LENGTH_A
        b = DMBE_MAX_LENGTH_B
        phi = data[i][3]
        h = data[i][0] - math.cos(phi) * math.sqrt(a**2 - b**2)
        k = data[i][1] - math.sin(phi) * math.sqrt(a**2 - b**2)
        # h = data[i][0]
        # k = data[i][1]
        
        dmbe.append([round(h, 4), round(k, 4), round(a, 4), round(b, 4), round(phi, 4)])
     
    return dmbe

def visualize_ellipses(ellipses):  
    fig, ax = plt.subplots()
      
    for ellipse_params in ellipses:  
        x, y, a, b, phi = ellipse_params  
        # Create an ellipse patch  
        ellipse = Ellipse(xy=(x, y), width=2*a, height=2*b, angle=np.degrees(phi), edgecolor='r', facecolor='none')  
          
        # Add the ellipse to the axes  
        ax.add_patch(ellipse)  
      
    # Set axes limits and aspect ratio to equal for a proper visualization  
    ax.set_xlim(min([e[0] - e[2] for e in ellipses]), max([e[0] + e[2] for e in ellipses]))  
    ax.set_ylim(min([e[1] - e[3] for e in ellipses]), max([e[1] + e[3] for e in ellipses]))  
    ax.set_aspect('equal', adjustable='datalim')  
      
    # Show the plot  
    
def visualize_coordinates(data):  
    # 分别提取x和y坐标  
    x_coords, y_coords = zip(*data)  # 使用解包操作来分开x和y坐标  
  
    # 创建一个新的图形  
    plt.figure()  
  
    # 绘制点  
    plt.scatter(x_coords, y_coords, marker='o')  
  
    # 设置轴标签  
    plt.xlabel('X Coordinate')  
    plt.ylabel('Y Coordinate')  
  
    # 设置标题  
    plt.title('Visualization of Coordinates')  
  
    # 显示图形  
    # plt.show()  
  
input_traj = [
[1.03384, 0.906871],
[1.08892, 1.01183],
[1.15023, 1.10782],
[1.21434, 1.19563],
[1.2793, 1.27741],
[1.34447, 1.3542],
[1.40979, 1.42642],
[1.47552, 1.49423],

]
#     [1.60, 1.0],  
#     [1.49, 0.9],  
#     # [1.45, 0.8],  
#     # [1.36, 0.57],  
#     # [1.29, -0.06],  
#     # [1.22, -0.71],  
#     # [1.13, -1.34],  
#     # [1.10, -2.00],  
#     # [1.05, -2.65],  
# ]

# calculate_speed_and_heading(input_traj)
# print(input_traj)
# dmbe = get_dmbe(input_traj)
dmbe = [
[1.83869, 2.44056, 2, 1, -2.05407],
[2.0213, 2.47151, 2, 1, -2.13922],
[2.17152, 2.50673, 2, 1, -2.20141],
[2.29168, 2.55185, 2, 1, -2.2421],
[2.39998, 2.59805, 2, 1, -2.27447],
[2.50639, 2.6387, 2, 1, -2.30614],
[2.61528, 2.67012, 2, 1, -2.34059],
]
# print(dmbe)
# exit()
# vis DMBE
fig, ax = plt.subplots()
for ellipse in dmbe:  
    x, y, a, b, theta = ellipse  
    
    theta_rad = np.rad2deg(theta)
    # 创建椭圆对象  
    ell = Ellipse(xy=(x, y), width=2*a, height=2*b, angle=theta_rad, edgecolor='b', facecolor='none')  
    ax.add_patch(ell)  
    
    # 计算焦点位置  
    c = np.sqrt(a**2 - b**2)  
    # 计算焦点在旋转坐标系中的位置  
    focus1_rotated = (c, 0)  
    focus2_rotated = (-c, 0)  
      
    # 应用旋转矩阵来得到焦点在原始坐标系中的位置  
    R = np.array([  
        [np.cos(theta), -np.sin(theta)],  
        [np.sin(theta), np.cos(theta)]  
    ])  
    focus1_original = np.dot(R, focus1_rotated) + (x, y)  
    focus2_original = np.dot(R, focus2_rotated) + (x, y)  
      
    # 绘制焦点  
    ax.plot(focus1_original[0], focus1_original[1], 'bo')  
    ax.plot(focus2_original[0], focus2_original[1], 'bo')  
    
    # 绘制原始轨迹
    x_coords, y_coords = zip(*input_traj)  # 使用解包操作来分开x和y坐标
    ax.plot(x_coords, y_coords, 'r-')
    ax.plot(x_coords, y_coords, 'r+')
    
    # 绘制椭圆中心
    ax.plot(x, y, 'b+')
    

# ax.set_xlim(-3, 3)  
# ax.set_ylim(-3, 3)  
ax.set_aspect('equal') 
# plt.show()


dx = 1.0 - 0.0
dy = 1.0 - 0.0

theta = math.degrees(math.atan2(dy, dx))
print("theta:", theta)
phi = 0.707

print(phi - theta)