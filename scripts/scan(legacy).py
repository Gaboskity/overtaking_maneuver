#!/usr/bin/env python3
import math
import numpy as np


import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Float32MultiArray


class scan:
    def __init__(self):
        scan_topic_name = rospy.get_param('~scan', '/scan')

        self.sub = rospy.Subscriber(scan_topic_name, LaserScan, self.scan_callback, queue_size=1)
        self.pub_point_cloud = rospy.Publisher('/PointCloud', PointCloud2, queue_size=1)
        self.pub_avoid=rospy.Publisher('/avoid', Float32MultiArray,queue_size=1)
    def scan_callback(self, msg: LaserScan): 
        object_coords = []
        forward_distances = []

        newPoint = []
        max_angle_offset = math.radians(15)
        min_angle_offset = math.radians(270)  

        for i in range(len(msg.ranges)):
            range_val = msg.ranges[i]
            angle = msg.angle_min + i * msg.angle_increment

            # Távolságok szűrése
            if (angle >= min_angle_offset  or angle <= max_angle_offset):
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                z = 0.0

                newPoint.append([x,y,z])

                if (-max_angle_offset <= angle or angle <= max_angle_offset):
                    forward_distances.append(range_val)
                    object_coords.append((x, y))

        PointCloud = pcl2.create_cloud_xyz32(msg.header, newPoint)
        self.pub_point_cloud.publish(PointCloud)
                    
        if not object_coords:
            rospy.loginfo("No object detected")
            return

       
        object_coords = np.array(object_coords)
        obj_x, obj_y = np.mean(object_coords, axis=0)
        rospy.loginfo(f"Object detected at: ({obj_x}, {obj_y})")

        
        forward_safe_distance = 0.2  
        left_safe_distance = 0     
        forward_avg = np.mean(forward_distances) if forward_distances else float('inf')

        overtaking_possible = (obj_y > left_safe_distance) and (forward_avg > forward_safe_distance)

        
        avoid_msg = Float32MultiArray()
        avoid_msg.data = [obj_x, obj_y, float(overtaking_possible)]  
        self.pub_avoid.publish(avoid_msg)

        
        if overtaking_possible:
            rospy.loginfo("maneuvre is possible, forward distance: {forward_avg:.2}, left clearance: {obj_y:.2}")
        else:
            rospy.logwarn("maneuvre is not possible, forward distance: {forward_avg:.2}, left clearance: {obj_y:.2}")

if __name__ == '__main__':
  rospy.init_node("scan")

  scan_publisher = scan()

  rospy.spin()
