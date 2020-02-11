#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time
import sys

# Description:
# Script moves the robot to a given point
# For reverse direction set forward False
#            Input arguments: 
# argv[1] - x - float
# argv[2] - y - float
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
    if (numArgs == 3):   # x and y points of the goal
        try:
            x_goal = float(sys.argv[1])
            y_goal = float(sys.argv[2])
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

    # Create a new velocity publisher and publish the message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(100)
    t0 = rospy.Time.now().to_sec()  # movement start time

    # PID Params
    K_linear = 0.5           # P-control param for orientation
    K_angular = 0.3          # P-control param for rotation
    max_speed = 0.1          # max linear speed allowed, in m/s
    max_angular_speed = 0.6  # max angular speed allowed, in rad/s

    # Control loop
    # Consists of two PID (currently only P) controllers - angle and linear
    # first the robot orients itself and then it goes foward
    while not rospy.is_shutdown(): 
        goal_angle = math.atan2((y_goal-y),(x_goal-x))
        angle_error = goal_angle-yaw
        angular_speed = K_angular * angle_error
        if (angular_speed > max_angular_speed):  # saturate the controller output
            angular_speed = max_angular_speed
        velocity_message.angular.z = angular_speed

        distance_error = abs(math.sqrt((x_goal - x)**2 + (y_goal - y)**2))
        linear_speed = K_linear * distance_error
        if(linear_speed > max_speed):   # saturate the controller output at max speed
            linear_speed = max_speed
        if(angle_error < math.radians(10)):    # first rotate to 10 deg precision and then start linear motion
            velocity_message.linear.x = linear_speed

        

        pub.publish(velocity_message)
        rate.sleep()

        t1 = rospy.Time.now().to_sec()           
        if(distance_error < 0.01):
            rospy.loginfo("Distance reached in %f s. Stopping." % (t1-t0))
            break
    # Stop the robot
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    pub.publish(velocity_message)

if __name__ == '__main__':
    main(sys.argv[1:])
    