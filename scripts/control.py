#!/usr/bin/env python3

import rospy
import math

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray


 #finding the closest point

class control:
    def __init__(self):
        self.err = 1
        self.quaternion = [0, 0, 0, 1]
        self.path_points = []
        self.current_position = Point()
        
        path_topic = rospy.get_param('~path', '/path')
        odom_topic_name = rospy.get_param('~odom', '/odom')
        cmd_topic_name = rospy.get_param('~cmd', '/cmd_vel')

        rospy.sleep(5)

        self.sub_path = rospy.Subscriber(path_topic, Path, self.callback_msg_path, queue_size=20)
        self.sub_curr_pose = rospy.Subscriber(odom_topic_name, Odometry, self.callback_msg_pose, queue_size=5)

        self.pub_closest_point = rospy.Publisher('closest_point', PoseStamped, queue_size=1)
        self.pub_lookahead_point = rospy.Publisher('Lookahead_point', PoseStamped, queue_size=1)
        self.pub_control = rospy.Publisher(cmd_topic_name, Twist, queue_size=1)

    def callback_msg_pose(self, msg: Odometry):
        if not self.path_points:
            rospy.logwarn("No path points received yet.")
            return
        

        self.current_position = msg.pose.pose.position
        curr_orientation = msg.pose.pose.orientation
        self.quaternion = [curr_orientation.x, curr_orientation.y, curr_orientation.z, curr_orientation.w]


    def callback_msg_path(self, msg):
        self.path_points.clear()
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            
            self.path_points.append([x, y])

        self.closest_point = self.find_closest_point(self.current_position, self.path_points)

        if self.closest_point is None:
            rospy.logwarn("No closest point found.")
            return

        closest_point_msg = PoseStamped()
        closest_point_msg.header.frame_id = "odom"
        closest_point_msg.pose.position.x = self.closest_point[0]
        closest_point_msg.pose.position.y = self.closest_point[1]
        closest_point_msg.pose.position.z = 0

        rospy.loginfo(closest_point_msg)
        self.pub_closest_point.publish(closest_point_msg)

        lookahead_point = self.calculate_lookahead_point(self.closest_point)
        if lookahead_point is None:
            rospy.logwarn("No lookahead point found.")
            return
        
        if lookahead_point:
            lookahead_point_msg = PoseStamped()
            lookahead_point_msg.header.frame_id = "odom"
            lookahead_point_msg.pose.position.x = lookahead_point[0]
            lookahead_point_msg.pose.position.y = lookahead_point[1]
            lookahead_point_msg.pose.position.z = 0
            self.pub_lookahead_point.publish(lookahead_point_msg)

        
        roll, pitch, yaw = euler_from_quaternion(self.quaternion)
        self.control = self.pid_controller(yaw, self.current_position.x, lookahead_point[0], self.current_position.y, lookahead_point[1])    
        
        control_msg = Twist()

      
        

        dist = math.sqrt((self.current_position.x - lookahead_point[0])**2 + (self.current_position.y - lookahead_point[1])**2)
        if (dist > 0.1):
            control_msg.linear.x = 0.3
            control_msg.linear.y = 0
            control_msg.linear.z = 0

            control_msg.angular.x = 0
            control_msg.angular.y = 0
            control_msg.angular.z = self.control
        else:
            
            control_msg.linear.x = 0.0
            control_msg.linear.y = 0
            control_msg.linear.z = 0

            control_msg.angular.x = 0
            control_msg.angular.y = 0
            control_msg.angular.z = 0.0

        self.pub_control.publish(control_msg)



    def point_distances(self, point1, point2):
        return math.sqrt((point1.x - point2[0])**2 + (point1.y - point2[1])**2)
    
    def find_closest_point(self, current_location, target_points):
        if not target_points:
            return None
        
        closest_point = None
        min_distance = float('inf') 

        for point in target_points:
            distance = self.point_distances(current_location, point)
            if distance < min_distance:
                min_distance = distance
                closest_point = point
    
        return closest_point
    
    def calculate_lookahead_point(self, closest_point):
        lookahead_distance = 0.5 
        closest_index = self.path_points.index(closest_point)
        cumulative_distance = 0.0

        for i in range(closest_index, len(self.path_points) - 1):
            current_point = self.path_points[i]
            next_point = self.path_points[i + 1]

            segment_distance = math.sqrt((next_point[0] - current_point[0]) ** 2 + (next_point[1] - current_point[1]) ** 2)
            cumulative_distance += segment_distance

            if cumulative_distance >= lookahead_distance:
                overshoot_distance = cumulative_distance - lookahead_distance

                direction_x = next_point[0] - current_point[0]
                direction_y = next_point[1] - current_point[1]
                direction_length = math.sqrt(direction_x ** 2 + direction_y ** 2)

                direction_x /= direction_length
                direction_y /= direction_length

                lookahead_x = next_point[0] - direction_x * overshoot_distance
                lookahead_y = next_point[1] - direction_y * overshoot_distance

                return (lookahead_x, lookahead_y)

        return self.path_points[-1]
    
    def pid_controller(self, theta, x, x_t, y,y_t):
        kp = 2
        theta_d = math.atan2(y_t-y, x_t-x)
        e = theta_d - theta
        e_d = math.atan2(math.sin(e), math.cos(e))
        omega = kp*e_d

        return (omega)

    


if __name__ == '__main__':
    rospy.init_node('control')

    point_publisher = control()

    rospy.spin()