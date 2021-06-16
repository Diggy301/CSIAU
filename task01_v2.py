#!/usr/bin/env python
#_*_coding:utf-8_*_

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, radians, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global robotX, robotY, robotTheta
robotX = 0.0
robotY = 0.0
robotTheta = 0.0

def callback(msg):
	global robotX, robotY, robotTheta
	robotX = msg.pose.pose.position.x
	robotY = msg.pose.pose.position.y
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	robotTheta = yaw


def getDistance(Xn, Yn):
	return sqrt((Xn - robotX)**2 + (Yn - robotY)**2)

def linear_speed(Xn, Yn, Kp = 5):
	return Kp * getDistance(Xn, Yn)

def robot_angle(Xn, Yn):
	ang = atan2(Yn - robotY, Xn - robotX)
	if ang > pi:
		return ang-2*pi
	if ang < -pi:
		return ang+2*pi
	return ang

def angular_speed(Xn, Yn, Kp = .5):
	return Kp * (robot_angle(Xn, Yn) - robotTheta)


def robotControl():
	rospy.init_node('check_odometry', anonymous = True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
	odom_sub = rospy.Subscriber('/odom', Odometry, callback)
	vel_msg = Twist()
	rate = rospy.Rate(10)
	
	tolerance = 0.1
	
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	
	n = int(input("how many waypoints?: "))
	for i in range(n):
		print("Enter the cordinates of point "+chr(ord('A')+i))
		Xn = float(input("Desired position x: "))
		Yn = float(input("Desired position y: "))
		
		while getDistance(Xn, Yn) > tolerance:
			vel_msg.linear.x = linear_speed(Xn, Yn)
			vel_msg.angular.z = angular_speed(Xn, Yn)
			pub.publish(vel_msg)
			rate.sleep()
	
	print("Robot has reached the final waypoint")
	rospy.spin()
	
if __name__ == '__main__':
	try:
		robotControl()
	except rospy.ROSInterruptException:
		pass