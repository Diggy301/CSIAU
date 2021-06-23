#!/usr/bin/env python
#_*_coding:utf-8_*_

import rospy
import random
import os

def launchWorld():
	number = random.randint(1, 5)
	world_file = "rosrun stage_ros stageros stageDisruptive/disruptive"+str(number)+".world"
	os.system(world_file)

if __name__ == '__main__':
	try:
		launchWorld()
	except rospy.ROSInterruptException:
		pass