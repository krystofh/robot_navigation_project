#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import sys

# Description:
# Script stops the robot in the simulation
# Input arguments: none
# Author: Krystof Hes
# Created: 6.2.2020 Sevilla


def main():
    # Init new node - the velocity Stopper
    try:
        rospy.init_node("velocityStopper")   
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    
    # Create a new velocity message
    velocity_message = Twist()
    velocity_message.linear.x = 0.0
    velocity_message.linear.y = 0.0
    velocity_message.linear.z = 0.0
    velocity_message.angular.x = 0.0
    velocity_message.angular.y = 0.0
    velocity_message.angular.z = 0.0

    # Create a new velocity publisher and publish the message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    pub.publish(velocity_message)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(velocity_message)
        rate.sleep()

if __name__ == '__main__':
    main()
    