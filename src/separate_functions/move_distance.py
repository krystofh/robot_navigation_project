#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import sys

# Description:
# Script moves the robot forward a given distance at a given speed
# For reverse direction set forward False
#            Input arguments: 
# argv[1] - distance - float
# argv[2] - speed - float
# argv[3] - forward - boolean
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
    global x, y, yaw
    # Init new node - the velocity controller
    try:
        rospy.init_node("velocityController")
        sub = rospy.Subscriber("/odom", Odometry, odometryCallback)     
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    
    # Parse the arguments
    numArgs = len(sys.argv)
    print("I have %d arguments\n" % numArgs)
    print("These are:\n %s \n" % str(sys.argv))
    if (numArgs == 4):   # dir, distance, speed
        try:
            distance = float(sys.argv[1])
            speed = float(sys.argv[2])
            forward = bool(sys.argv[3])
        except:
            print("Correct number of parameters but something went wrong with parsing")    
    else:
        print("Error - Wrong number of parameters!")
    
    # Create a new velocity message
    velocity_message = Twist()
    if (forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)
    velocity_message.linear.y = 0.0
    velocity_message.linear.z = 0.0
    velocity_message.angular.x = 0.0
    velocity_message.angular.y = 0.0
    velocity_message.angular.z = 0.0

    # Create a new velocity publisher and publish the message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(100)
    distance_covered = 0.0
    t0 = rospy.Time.now().to_sec()  # movement start time

    while not rospy.is_shutdown(): 
        pub.publish(velocity_message)
        rate.sleep()

        t1 = rospy.Time.now().to_sec()       
        distance_covered = (t1-t0) * speed
        rospy.loginfo(distance_covered)      
        if(distance_covered > distance):
            rospy.loginfo("Distance reached. Stopping.")
            break
    velocity_message.linear.x = 0.0
    pub.publish(velocity_message)

if __name__ == '__main__':
    main(sys.argv[1:])
    