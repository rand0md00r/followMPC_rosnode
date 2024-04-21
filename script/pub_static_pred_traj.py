#! /home/sgan/env/bin/python
# -*- coding: utf-8 -*-

import sys
sys.path.append('/home/work_space/src/traj_pred/')

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

class TrajectoryPredictionNode:
    def __init__(self):
        rospy.init_node('trajectory_Prediction_Node', anonymous=True, log_level=rospy.INFO)
        self.pub_esti = rospy.Publisher('traj_pred_node/estimations', MarkerArray, queue_size=8)

    def publish_marker_array(self):
        marker_array = MarkerArray()
        c1 = [
            (5, 1.14503), (5, 0.967427), (5, 0.789829), (5, 0.61223), (5, 0.434632), (5, 0.257033), (5, 0.0794349), (5, 0.0)
            
            
            
        ]
        c2 = [
            (10, -1.14503), (10, -0.967427), (10, -0.789829), (10, -0.61223), (10, -0.434632), (10, -0.257033), (10, -0.0794349), (10, 0.0)
        ]
        c3 = [
            (15, 1.14503), (15, 0.967427), (15, 0.789829), (15, 0.61223), (15, 0.434632), (15, 0.257033), (15, 0.0794349), (15, 0.0)
        ]
        c4 = [
            (20, -1.14503), (20, -0.967427), (20, -0.789829), (20, -0.61223), (20, -0.434632), (20, -0.257033), (20, -0.0794349), (20, 0.0)
        ]
        coordinates = []
        coordinates.append(c1)
        coordinates.append(c2)
        coordinates.append(c3)
        coordinates.append(c4)
        nums = len(coordinates)
        
        # 每个行人一个marker
        i = 0
        for ped in coordinates:
            i = i + 1
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.ns = "static_pred_traj"
            marker.type = marker.CUBE_LIST
            marker.action = marker.ADD
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            
            j = 0
            for x, y in ped:
                j = j + 1
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.5
                marker.points.append(point)
                marker.id = i * 10 + j
            
            marker_array.markers.append(marker)
        
        self.pub_esti.publish(marker_array)

    def run(self):
        rate = rospy.Rate(10)  # 设置发布频率为 10Hz
        while not rospy.is_shutdown():
            self.publish_marker_array()
            rate.sleep()

if __name__ == '__main__':
    node = TrajectoryPredictionNode()
    node.run()
