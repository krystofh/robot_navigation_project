#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import sys

# Description:
# Script sets a foward speed of the robot in the simulation
# Input arguments: speed as float (argv[1] because first is dir)
# Author: Krystof Hes
# Created: 6.2.2020 Sevilla

x=0
y=0
yaw=0

def odometryCallback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def main(argv):
    # Init new node - the velocity controller
    try:
        rospy.init_node("velocityController")
        sub = rospy.Subscriber("/odom", Odometry, odometryCallback)     
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    
    # Parse the forward speed
    numArgs = len(sys.argv)
    print("I have %d arguments\n" % numArgs)
    print("These are:\n %s \n" % str(sys.argv))
    if (numArgs == 2):   # dir, speed, correct
        try:
            speed = float(sys.argv[1])
        except:
            print("Correct number of parameters but something went wrong with parsing")    
    else:
        print("Error - Wrong number of parameters!")
    
    # Create a new velocity message
    velocity_message = Twist()
    velocity_message.linear.x = speed
    velocity_message.linear.y = 0.0
    velocity_message.linear.z = 0.0
    velocity_message.angular.x = 0.0
    velocity_message.angular.y = 0.0
    velocity_message.angular.z = 0.0

    # Create a new velocity publisher and publish the message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(velocity_message)
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])
    