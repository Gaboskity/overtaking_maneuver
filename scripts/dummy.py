#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist 

class speed:
    def __init__(self):
        cmd_topic = rospy.get_param('~cmd', '/agent2/cmd_vel')

        self.pub_cmd2 = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        rospy.sleep(5)

        control_msg = Twist()
        control_msg.linear.x = 0.1
        control_msg.linear.y = 0
        control_msg.linear.z = 0

        control_msg.angular.x = 0
        control_msg.angular.y = 0
        control_msg.angular.z = 0

        self.pub_cmd2.publish(control_msg)

if __name__ == '__main__':
    rospy.init_node('dummy')

    point_publisher = speed()

    rospy.spin()