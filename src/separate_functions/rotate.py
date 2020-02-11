#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import sys

# Description:
# Script sets a rotation speed of the robot in the simulation to achieve a given yaw angle
#     Input arguments:   speed as float (argv[1] because first is dir)
# argv[1] - angular_speed_degree - float
# argv[2] - relative_angle_degree - float
# argv[3] - clockwise - boolean
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
        rospy.init_node("velocityController")
        sub = rospy.Subscriber("/odom", Odometry, odometryCallback)     
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    
    # Parse the arguments
    numArgs = len(sys.argv)
    print("I have %d arguments\n" % numArgs)
    print("These are:\n %s \n" % str(sys.argv))
    if (numArgs == 4):   # dir, angular_speed_degree, relative_angle_degree, clockwise
        try:
            angular_speed_degree = float(sys.argv[1])
            relative_angle_degree = float(sys.argv[2])
            clockwise = bool(sys.argv[3])
        except:
            print("Correct number of parameters but something went wrong with parsing")    
    else:
        print("Error - Wrong number of parameters!")
    
    # Create a new velocity message
    velocity_message = Twist()
    velocity_message.linear.x = 0.0
    velocity_message.linear.y = 0.0
    velocity_message.linear.z = 0.0
    velocity_message.angular.x = 0.0
    velocity_message.angular.y = 0.0
    velocity_message.angular.z = 0.0
    
    angular_speed = math.radians(abs(angular_speed_degree))
    if(clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    # Create a new velocity publisher and publish the message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(100)
    t0 = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        t1 = rospy.Time.now().to_sec()
        angle_rotated_deg = (t1-t0) * angular_speed_degree
        print(angle_rotated_deg)

        if (angle_rotated_deg < relative_angle_degree):
            pub.publish(velocity_message)
            rate.sleep()
        else:
            print("Angle reached. Rotation stops.")
            rospy.loginfo("Angle reached. Rotation stops.")
            break

    # Stop the rotation        
    velocity_message.angular.z = 0.0
    pub.publish(velocity_message)
    
if __name__ == '__main__':
    main(sys.argv[1:])
    