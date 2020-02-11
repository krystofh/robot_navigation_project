#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import sys

# Description:
# Script to observe and print the robot pose at an acceptable rate
# Used for debugging purposes
# Input arguments: rate as int (Hz)
# Author: Krystof Hes
# Created: 6.2.2020 Sevilla

x=0
y=0
yaw=0

def odometryCallback(msg):
    global x
    global y
    global yaw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def main(argv):
    # Init new node - the velocity controller
    try:
        rospy.init_node("poseObserver")
        sub = rospy.Subscriber("/odom", Odometry, odometryCallback)     
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    
    # Parse the rate
    numArgs = len(sys.argv)
    print("I have %d arguments\n" % numArgs)
    print("These are:\n %s \n" % str(sys.argv))
    if (numArgs == 2):   # dir, speed, correct
        try:
            rate = int(sys.argv[1])  # input rate in Hz
        except:
            print("Correct number of parameters but something went wrong with parsing")    
    else:
        print("Error - Wrong number of parameters!")
    

    # Create the printing loop
    loop_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        rospy.loginfo("\nPose:\n   [x] = %f\n   [y] = %f\n   [yaw] = %f" % (x,y,yaw))
        loop_rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])
    