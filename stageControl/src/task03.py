#!/usr/bin/env python
#_*_coding:utf-8_*_

import math
import rospy
import random
from std_msgs.msg import String
from nav_msgs.msg import Odometry

global roboX, roboY

def callback(msg):
	global roboX, roboY
	roboX = msg.pose.pose.position.x
	roboY = msg.pose.pose.position.y

def gasData():
	global roboX, roboY
	roboX = 0.0
	roboY = 0.0
	
	rospy.init_node('gasData', anonymous = True)
	rospy.Subscriber('/odom', Odometry, callback)
	pub = rospy.Publisher('/Gas', String, queue_size=15, latch=False)
	rate = rospy.Rate(10) #10Hz
	
	alfa = 100
	gasPosX = -15
	gasPosY = 10
	
	mu = 0
	sigma = 0.5
	
	while not rospy.is_shutdown():
		distance = math.sqrt(pow(roboX-gasPosX,2) + pow(roboY-gasPosY,2))
		aux = math.pow(distance,2)/100
		gasValue = alfa*math.exp(-aux)
		if distance < 23:
			gasNoise = gasValue + random.gauss(mu, sigma)
			pub.publish(str(round(gasNoise, 2)))
		else:
			pub.publish(str(round(gasValue, 2)))
		rate.sleep()

if __name__ == '__main__':
	try:
		gasData()
	except rospy.ROSInterruptException:
		pass
