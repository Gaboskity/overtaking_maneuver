#!/usr/bin/env python3

import rospy
import math
import numpy as np
import tf.transformations as tf
from scipy.interpolate import CubicSpline

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker



class pub_path:

    def __init__(self):

        odom_topic_name = rospy.get_param('~odom', '/odom')
        self.pub_path = rospy.Publisher('/path', Path, queue_size=1)
        self.sub_odom = rospy.Subscriber(odom_topic_name, Odometry, self.odom_callback, queue_size=1)
        self.sub_obsatackle = rospy.Subscriber('/obstackle',Marker, self.obstackle_callback, queue_size=1)
        self.sub_valid_points = rospy.Subscriber('/valid', Bool, self.validity_callback, queue_size=1)

        self.target_positions = []
        self.start_pose = [0.0, 0.0]
        self.curr_pose_x = 0.0
        self.curr_pose_y = 0.0
        self.valid_points = 0

        self.target_positions = [(0.5, 0), (1, 0), (1.5, 0), (2, 0), (2.5, 0), (3, 0), (3.5, 0), (4, 0), (4.5, 0), (5, 0),
                                 (5.5, 0), (6, 0), (6.5, 0), (7, 0), (7.5, 0), (8, 0), (8.5, 0), (9, 0), (9.5, 0), (10, 0),
                                 (10.5, 0), (11, 0), (11.5, 0), (12, 0), (12.5, 0), (13, 0), (13.5, 0), (14, 0), (14.5, 0), (15, 0),
                                 (15.5, 0), (16, 0), (16.5, 0), (17, 0), (17.5, 0), (18, 0), (18.5, 0), (19, 0), (19.5, 0), (20, 0)]

    def validity_callback(self, msg:Bool):
        self.valid_points = msg.data
    

    def odom_callback(self, msg:Odometry):
        self.curr_pose_x = msg.pose.pose.position.x
        self.curr_pose_y = msg.pose.pose.position.y
        self.publish_path()

    def obstackle_callback(self, msg:Marker):
        if (self.valid_points == False):
            self.target_positions = [(0.5, 0), (1, 0), (1.5, 0), (2, 0), (2.5, 0), (3, 0), (3.5, 0), (4, 0), (4.5, 0), (5, 0),
                                 (5.5, 0), (6, 0), (6.5, 0), (7, 0), (7.5, 0), (8, 0), (8.5, 0), (9, 0), (9.5, 0), (10, 0),
                                 (10.5, 0), (11, 0), (11.5, 0), (12, 0), (12.5, 0), (13, 0), (13.5, 0), (14, 0), (14.5, 0), (15, 0),
                                 (15.5, 0), (16, 0), (16.5, 0), (17, 0), (17.5, 0), (18, 0), (18.5, 0), (19, 0), (19.5, 0), (20, 0)]
        else:
            start_point = msg.pose.position.x - msg.scale.x - 0.3
            y_dist = msg.pose.position.y + msg.scale.y + 0.2
            return_pose = msg.pose.position.x + msg.scale.x + 0.3

            for i in range(0, len(self.target_positions)):
                if (i*0.5 >= int(start_point)):
                    self.target_positions[i] = ((i*0.5)+1, y_dist)
                if (i*0.5 >= int(return_pose)):
                    self.target_positions[i] = ((i*0.5)+1, 0)
                


    def point_distances(self, point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    

    #calculating the number of samples based on the distances

    def distance_sample(self, pose1, pose2):
        distance = self.point_distances(pose1, pose2)
        sample_number = int(distance/0.01)
        return sample_number

    #generating a cubic spline path

    def calculate_yaw(self, dx, dy):
        return np.arctan2(dy, dx)

    def generate_path(self, pose_x, pose_y, num_samples):
        if len(self.target_positions) < 2:
            rospy.logwarn("Not enough target points to generate a spline path")
            return Path()
        else:
            x_points = [pose_x] + [pos[0] for pos in self.target_positions]
            y_points = [pose_y] + [pos[1] for pos in self.target_positions]
        
    
            spline = CubicSpline(x_points, y_points)
            x_new = np.linspace(min(x_points), max(x_points), num_samples)
            y_new = spline(x_new)
        
        
            dx = np.gradient(x_new)
            dy = np.gradient(y_new)
        
        
            path_msg = Path()
            path_msg.header.frame_id = "odom"
        
        
            for i in range(num_samples):
                pose = PoseStamped()
                pose.header.frame_id = "odom"
                pose.pose.position.x = x_new[i]
                pose.pose.position.y = y_new[i]
                pose.pose.position.z = 0  

            
                yaw = self.calculate_yaw(dx[i], dy[i])
            
        
                quaternion = tf.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
            
                path_msg.poses.append(pose)
        
            return path_msg
        
    def publish_path(self):
        num_samples = self.distance_sample(self.target_positions[0], self.target_positions[1])
        self.path = self.generate_path(self.start_pose[0], self.start_pose[1], num_samples)
        if self.path:
            self.pub_path.publish(self.path)
    
    

if __name__ == '__main__':
    rospy.init_node('path_creator')
    path_publisher = pub_path()
    rospy.spin()