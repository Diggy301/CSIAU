#!/usr/bin/env python
#_*_coding:utf-8_*_

import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class robot:
	def __init__(self):
		rospy.init_node('check_odometry', anonymous = True)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.callback_laser)
		self.vel_msg = Twist()
		self.rate = rospy.Rate(10)
		self.laser = []
		
		self.vel_msg.linear.x = 0.0
		self.vel_msg.linear.y = 0.0
		self.vel_msg.linear.z = 0.0
		self.vel_msg.angular.x = 0.0
		self.vel_msg.angular.y = 0.0
		self.vel_msg.angular.z = 0.0
		self.run = True
		self.Lspeed = 10.0
		self.Aspeed = 2.0
		
		
	def callback_laser(self, laser):
		self.laser = laser.ranges
	
	def getLaserValue(self, ang):
		self.rate.sleep()
		return self.laser[ang]
	
	def robotForward(self):
		self.vel_msg.linear.x = self.Lspeed
		self.vel_msg.angular.z = 0.0
	
	def robotSTOP(self):
		self.vel_msg.linear.x = 0.0
		self.vel_msg.angular.z = 0.0
		self.run = False
	
	def move(self):
		while self.run:
			if self.getLaserValue(127) > 2:
				self.robotForward()
			else:
				self.robotturn()
				
			self.robotMove()
			
	def robotturn(self):
		self.vel_msg.linear.x = 0.0
		self.vel_msg.angular.z = self.Aspeed
		
	def robotMove(self):
		self.pub.publish(self.vel_msg)
		self.rate.sleep()

			
if __name__ == '__main__':
	try:
		r = robot()
		while not rospy.is_shutdown():
			r.move()
	except rospy.ROSInterruptException:
		pass
