#!/usr/bin/env python
#_*_coding:utf-8_*_



import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow, atan2, sqrt, radians, pi, atan
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
		
		self.GOALSEEK = 1
		self.WALLFOLLOW = 2
		self.CIRCLEMOVE = 3

		self.gas = 0
		self.robot_x = 0
		self.robot_y = 0
		self.robot_theta = 0
		self.laser = []
		self.waypoint_x, self.waypoint_y = 0, 0
		self.cont = 0
		self.state = self.GOALSEEK
		self.last_angular_vel = 0
	
		
		#self.get_waypoint()


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
		# while gas == 0
			# move to random point
		# circle move
		# move to best direction whlie avoiding walls
		
		# after a while do circle move again and repeat untili gas 100 +- 1		
	
		if len(self.laser): # wait for first laser read
			if (self.state == self.GOALSEEK) or (self.state == self.WALLFOLLOW):
				self.move_to_waypoint()
			if self.state == self.CIRCLEMOVE:
				self.circle_move()

		self.pub.publish(self.vel_msg)
		self.last_angular_vel = self.vel_msg.angular.z


	def check_waypoint(self):
		d = sqrt((self.robot_x - self.waypoint_x)**2 + (self.robot_y - self.waypoint_y)**2)
		if d < 0.2:
			return True
		return False
	
	def  get_waypoint(self):
		self.waypoint_x, self.waypoint_y = np.random.uniform(-25,25,[1,2])[0]

	def move_to_waypoint(self):
		if not self.check_waypoint():
			
			waypoint_direction_d = self.get_angle_to_waypoint()
			#print(waypoint_direction_d)
			
			# if direction < 0, turn right
			self.get_linear_vel()

			if self.state == self.GOALSEEK:
				print('GOALSEEK')
				self.get_goalseek_rot(waypoint_direction_d)
		
				if self.check_path_blocked(0):
					self.state = self.WALLFOLLOW
			
			if self.state == self.WALLFOLLOW:
				print('WALLFOLLOW')	
				self.get_wallfollow_rot(waypoint_direction_d)	

				if not self.check_path_blocked(waypoint_direction_d):
					self.state = self.GOALSEEK				
		else:
			self.state = self.CIRCLEMOVE
			self.circlemove_init()
			print('arrived')

	
	def get_linear_vel(self):
		Kp = 0.5
		K = 1

		dy = self.robot_y-self.waypoint_y
		dx = self.robot_x-self.waypoint_x
		d = sqrt(dx**2 + dy**2)
		
		self.vel_msg.linear.x = Kp*d + K
	
	def get_goalseek_rot(self, ang):

		Kp = 2
		if abs(ang) < 15:
			self.vel_msg.angular.z = 0
		else:
			self.vel_msg.angular.z = 0.6*self.last_angular_vel + 0.4*(Kp*ang*pi/180)




	def get_wallfollow_rot(self, ang):

		best_ang = self.get_closest_opening(135+ang)

		if abs(best_ang-135) < 10:
			self.vel_msg.angular.z = 0

		else:
			Kp = 1
			w = Kp*(best_ang-135)*pi/180

			self.vel_msg.angular.z = 0.6*self.last_angular_vel + 0.4*w


		
	def get_closest_opening(self, angle):
		binary_map = [0 if value < 4.5 else 1 for value in self.laser]

		directions = []
		islandsize = 0
		for ind, value in enumerate(binary_map):
		    if value:
			islandsize += 1
		    else:
			if islandsize > 10:
			    directions.append(ind-int(islandsize/2)-1)
			islandsize = 0

		if islandsize > 10:
		    directions.append(ind-int(islandsize/2))	
		
		return min(directions, key = lambda x: abs(x-angle))

	def check_path_blocked(self, direction):
		ang = 135 + int(direction)
		print(ang)
		laser_values = self.laser[ang-20:ang+20]
		if any(l < 4 for l in laser_values):
			# path is blocked			
			return True		
		
		return False		
						
		
	def get_angle_to_waypoint(self):
		ang = atan2(self.waypoint_y-self.robot_y, self.waypoint_x-self.robot_x)
		
		ang = (ang - self.robot_theta) * 180 / pi
		
		if ang > 180:
			return ang-360
		if ang < -180:
			return ang+360
		return ang
	
	def circlemove_init(self):
		self.circlemove_init_cx = self.robot_x
		self.circlemove_init_cy = self.robot_y


	def circle_move(self):		

		# DO CIRCLE
		# WHILE DOING CIRCLE CHECK FOR COLISIONS
		# AFTER CIRCLE (REACHING INITIAL POTITION) GET NEW WAYPOINT		
		pass


if __name__ == '__main__':
	try:
		robot = Robot()
		while not rospy.is_shutdown():
			robot.move()
	except rospy.ROSInterruptException:
		pass


#TODO hitting topright corner of second yelllow block
# CIRCLE MOVE
