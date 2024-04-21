# -*- coding: utf-8 -*-

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class VelPlotter:
    def __init__(self):
        rospy.init_node('velocity_plotter', anonymous=True)
        self.velocities = {'time': [], 'linear': [], 'angular': []}
        rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)

    def vel_callback(self, msg):
        current_time = rospy.get_time()
        self.velocities['time'].append(current_time)
        self.velocities['linear'].append(msg.linear.x)
        self.velocities['angular'].append(msg.angular.z)

    def plot_velocities(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.velocities['time'], self.velocities['linear'], self.velocities['angular'])
        ax.set_xlabel('Time')
        ax.set_ylabel('Linear Velocity')
        ax.set_zlabel('Angular Velocity')
        plt.show()

    def run(self):
        rospy.on_shutdown(self.plot_velocities)
        rospy.spin()

if __name__ == '__main__':
    vel_plotter = VelPlotter()
    vel_plotter.run()