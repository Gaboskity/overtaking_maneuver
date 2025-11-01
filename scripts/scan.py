#!/usr/bin/env python3

import math
import rospy

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class Scan:
    def __init__(self):
        scan_topic_name = rospy.get_param('~scan', '/scan')
        odom_topic_name = rospy.get_param('~odom', '/odom')

        self.sub_odom = rospy.Subscriber(odom_topic_name, Odometry, self.odom_callback, queue_size=1)
        self.sub_scan = rospy.Subscriber(scan_topic_name, LaserScan, self.scan_callback, queue_size=1)
        self.pub_marker = rospy.Publisher('/obstackle', Marker, queue_size=1)
        self.pub_detected = rospy.Publisher('/valid', Bool, queue_size= 1)

        self.curr_posx = 0.0
        self.curr_posy = 0.0
        self.obstackle_exist = Bool()
        self.obstackle_exist.data = 0

    def odom_callback(self, msg: Odometry):
        self.curr_posx = msg.pose.pose.position.x
        self.curr_posy = msg.pose.pose.position.y

    def scan_callback(self, msg: LaserScan):
        sumposex = 0.0
        sumposey = 0.0
        valid_points = 0

        x_max = float('-inf')
        x_min = float('inf')
        y_max = float('-inf')
        y_min = float('inf')

        # max_angle_offset = math.radians(20)
        # min_angle_offset = math.radians(270) 

        for i in range(len(msg.ranges)):
            range_val = msg.ranges[i]

            if not math.isfinite(range_val) or range_val <= msg.range_min or range_val > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            # if (angle >= min_angle_offset  or angle <= max_angle_offset):
            pose_x = range_val * math.cos(angle) + self.curr_posx
            pose_y = range_val * math.sin(angle) + self.curr_posy

            sumposex += pose_x
            sumposey += pose_y
            valid_points += 1

            x_max = max(x_max, pose_x)
            x_min = min(x_min, pose_x)
            y_max = max(y_max, pose_y)
            y_min = min(y_min, pose_y)

            if valid_points == 0:
                self.obstackle_exist.data = 0
                self.pub_detected.publish(self.obstackle_exist)
                rospy.logwarn("No valid LiDAR points detected!")

                obstacle = Marker()
                obstacle.header.frame_id = 'odom'
                obstacle.action = Marker.DELETE
                self.pub_marker.publish(obstacle)

                return
            else:
                self.obstackle_exist.data = 1
                self.pub_detected.publish(self.obstackle_exist)
                rospy.loginfo("LiDAR points detected")


                avg_pose_x = sumposex / valid_points
                avg_pose_y = sumposey / valid_points
                x_dist = x_max - x_min
                y_dist = y_max - y_min

                obstacle = Marker()
                obstacle.header.frame_id = 'odom'
                obstacle.header.stamp = rospy.Time.now()
                obstacle.ns = 'cylinder'
                obstacle.type = Marker.CYLINDER
                obstacle.action = Marker.ADD

                obstacle.pose.position.x = avg_pose_x
                obstacle.pose.position.y = avg_pose_y
                obstacle.pose.position.z = 0.0
                obstacle.pose.orientation.w = 1.0

                obstacle.scale.x = x_dist
                obstacle.scale.y = y_dist
                obstacle.scale.z = 0.5

                obstacle.color.a = 0.8
                obstacle.color.r = 1.0
                obstacle.color.g = 0.0
                obstacle.color.b = 0.0

                self.pub_marker.publish(obstacle)
            



if __name__ == '__main__':
    rospy.init_node("scan")
    scan_publisher = Scan()
    rospy.spin()

