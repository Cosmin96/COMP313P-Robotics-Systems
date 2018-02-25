#!/usr/bin/env python


import rospy
from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from PyKDL import Rotation
import sys, getopt
import math


class stdr_controller():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('stdr_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_callback)
        self.current_pose = Odometry()
        self.distance_tolerance = 0.01
 
    def current_callback(self, data):
        self.current_pose = data

    def robomov(self, lin_velocity, rot_velocity, duration, vel_msg):
	pose = self.current_pose.pose.pose

	# Get the position vector. ROS uses nested types for generality, but it gets to be a bit
	# cumbersome
	position = pose.position
	 
	# The pose returns the orientation as a quaternion, which is a 4D representation of 3D
	# rotations. We just want the heading angle, so some conversion is required
	# 
	orientation = pose.orientation

	theta = 2 * atan2(orientation.z, orientation.w)	

	vel_msg.linear.x = lin_velocity
	vel_msg.angular.z = rot_velocity
	self.velocity_publisher.publish(vel_msg)
	rospy.sleep(duration)
		
	# Show the output
	rospy.loginfo('Current position, x: {}, y:{}, theta:{}'.format(position.x, position.y, theta))
	vel_msg.linear.x = 0.0
	vel_msg.angular.z = 0.0
	self.velocity_publisher.publish(vel_msg)

    def run(self):
	waypoints = open(sys.argv[1])
	x_list = []
	y_list = []
	theta_list = []
	lines = waypoints.readlines()
	for line in lines:	
		values = line.split(' ')
		print(line)
		x_list.append(float(values[0]))
		y_list.append(float(values[1]))
		theta_list.append(float(values[2]) * pi / 180)

        # Sleep for 1s before starting. This gives time for all the parts of stdr to start up
        rospy.sleep(1.0)
	for i in range(0, len(x_list)):
		x_val = x_list[i]
		y_val = y_list[i]
		theta_val = theta_list[i]
		
		vel_msg = Twist()
	    
		pose = self.current_pose.pose.pose

		# Get the position vector. ROS uses nested types for generality, but it gets to be a bit
		# cumbersome
		position = pose.position
	 
		# The pose returns the orientation as a quaternion, which is a 4D representation of 3D
		# rotations. We just want the heading angle, so some conversion is required
		# 
		orientation = pose.orientation

		theta = 2 * atan2(orientation.z, orientation.w)
		initial_x = position.x
		initial_y = position.y
		
		angle_of_rotation = math.atan2(y_val - initial_y, x_val - initial_x)
		 
		self.robomov(0, (angle_of_rotation - theta), 1, vel_msg)
		self.robomov(math.sqrt((x_val - initial_x) ** 2 + (y_val - initial_y) ** 2), 0, 1, vel_msg)
		pose = self.current_pose.pose.pose
		position = pose.position
		orientation = pose.orientation
		theta = 2 * atan2(orientation.z, orientation.w)
		self.robomov(0, (theta_val - theta), 1, vel_msg)
		
   
if __name__ == '__main__':
    try:
        #Testing our function
        x = stdr_controller()
        x.run()

    except rospy.ROSInterruptException: pass
