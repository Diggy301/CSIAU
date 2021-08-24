#!/usr/bin/env python
#_*_coding:utf-8_*_



import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow, atan2, sqrt, radians, pi
import numpy as np

#map limits: (-25,-25) to (25, 25)

class Robot:
	def __init__(self):
		rospy.init_node('task', anonymous = False)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.callback_laser)
		self.gas_sub = rospy.Subscriber('/Gas', String, self.callback_gas)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)
		self.vel_msg = Twist()
		self.rate = rospy.Rate(10)

		self.gas = 0
		self.robot_x = 0
		self.robot_y = 0
		self.robot_theta = 0
		self.laser = []
		self.waypoint_x, self.waypoint_y = (0,0)
		self.get_waypoint()


	def callback_laser(self, msg):
		self.laser = msg.ranges

	def callback_gas(self, msg):
		self.gas = float(msg.data)

	def callback_odom(self, msg):
		self.robot_x = msg.pose.pose.position.x
		self.robot_y = msg.pose.pose.position.y

		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.robot_theta = yaw

	def move(self):
		if self.check_waypoint():
			self.get_waypoint()
		else:
			self.move()

	def check_waypoint(self):
		d = sqrt((self.robot_x - self.waypoint_x)**2 + (self.robot_y - self.waypoint_y)**2)
		if d < 0.5:
			return True
		return False
	
	def  get_waypoint(self):
		self.waypoint_x, self.waypoint_y = np.random.uniform(-25,25,[1,2])[0]

	def move(self):
		# check if direction to wyapoint is blocked
		# if no move
		# if yes turn to "best side"
			# return to "line" - move
		print('aa')
		pass
		


if __name__ == '__main__':
	try:
		robot = Robot()
		while not rospy.is_shutdown():
			robot.move()
	except rospy.ROSInterruptException:
		pass
