#!/usr/bin/env python
#_*_coding:utf-8_*_

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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

		self.waypoint_x = 0
		self.waypoint_y = 0

		self.cont = 0
		self.state = self.GOALSEEK
		self.last_angular_vel = 0
		
		self.circlemove_values = []
		self.circlemove_vcont = 0
		
		self.flag = 1
	
		


	def callback_laser(self, msg):
		self.laser = msg.ranges

	def callback_gas(self, msg):
		self.gas = float(msg.data)

	def callback_odom(self, msg):
		"""
		ROS callback function to read odometry values and calculate
		yaw (rotation angle in world base) 
		"""
		self.robot_x = msg.pose.pose.position.x
		self.robot_y = msg.pose.pose.position.y

		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.robot_theta = yaw

	def move(self):
		if self.flag:
			self.start()

		if self.gas > 99: # can be higher 
			print('GAS SOURCE')
			exit()
	
		if len(self.laser): # wait for first laser read

			if (self.state == self.GOALSEEK) or (self.state == self.WALLFOLLOW):
				self.move_to_waypoint()
			if self.state == self.CIRCLEMOVE:
				self.circle_move()

		self.pub.publish(self.vel_msg)
		self.last_angular_vel = self.vel_msg.angular.z

	
	def start(self):
		if self.robot_x and self.robot_y:
			self.waypoint_x = self.robot_x + 100*np.cos(self.robot_theta)
			self.waypoint_y = self.robot_y
		if self.gas != 0.0:
			self.flag = 0
			self.state = self.CIRCLEMOVE
			self.circlemove_init()
			print('FOUND SOME GAS')

	def check_waypoint(self, tolerance):
		dx = self.robot_x - self.waypoint_x
		dy = self.robot_y - self.waypoint_y
		d = np.sqrt(dx**2 + dy**2)

		if d < tolerance:
			return True
		return False
	

	def move_to_waypoint(self):
		if not self.check_waypoint(0.2):
			waypoint_direction_d = self.get_angle_to_waypoint()

			self.get_linear_vel()

			if self.state == self.GOALSEEK:
				#print('GOALSEEK')
				self.get_goalseek_rot(waypoint_direction_d)
		
				if self.check_path_blocked(0):
					self.state = self.WALLFOLLOW
			
			if self.state == self.WALLFOLLOW:
				#print('WALLFOLLOW')	
				self.get_wallfollow_rot(waypoint_direction_d)	

				if not self.check_path_blocked(waypoint_direction_d):
					self.state = self.GOALSEEK				
		else:
			self.state = self.CIRCLEMOVE
			self.circlemove_init()

	
	def get_linear_vel(self):
		"""
		Give linear velocity to robot. Values chosen randomly.
		Changing these values might also require changing rotation params
		"""
		Kp = 0.5
		K = 1

		dy = self.robot_y-self.waypoint_y
		dx = self.robot_x-self.waypoint_x
		d = np.sqrt(dx**2 + dy**2)
		
		self.vel_msg.linear.x = Kp*d + K
	
	def get_goalseek_rot(self, ang):
		"""
		Give angular velocity to robot in goalseek mode.
		Angular velocity is a function proportional to the angle to
		to the waypoint in radians with a randomly chosen weight.
		In order to smooth rotations the angular velocity is also
		weighted with the angular velocity of the previous iteration.
		"""
		Kp = 2
		if abs(ang) < 5:#15
			self.vel_msg.angular.z = 0
		else:
			w = Kp*ang*np.pi/180
			self.vel_msg.angular.z = 0.6*self.last_angular_vel + 0.4*w




	def get_wallfollow_rot(self, ang):
		"""
		Give angular velocity to robot in wallfollow mode.
		Angular velocity is a function proportional to the angle to
		to the waypoint in radians with a randomly chosen weight.
		In order to smooth rotations the angular velocity is also
		weighted with the angular velocity of the previous iteration.
		"""
		
		# Get best angle according to surroundings and desired direction
		best_ang = self.get_closest_opening(135+ang)


		if abs(best_ang-135) < 5:
			self.vel_msg.angular.z = 0

		else:
			Kp = 1
			w = Kp*(best_ang-135)*np.pi/180
			
			if abs(w-self.last_angular_vel) > 3:
				# if change in angular vel is too big, ignore
				self.vel_msg.angular.z = self.last_angular_vel
			else:
				self.vel_msg.angular.z = 0.6*self.last_angular_vel + 0.4*w

		
	def get_closest_opening(self, angle):
		"""
		Function to return optimal angle. Optimal angle is the
		closest to the direction to the waypoint that is not blocked
		"""
		
		# 0's for blocked and 1's for open
		binary_map = [0 if value < 3.5 else 1 for value in self.laser]

		# if straight path (+- 15deg) is open to straigth
		mid = 135
		if all(l == 1  for l in binary_map[mid-15:mid+16]):
        		return mid

		# get midpoint of islands (groups of 1's) bigger than 20
		directions = []
		islandsize = 0
		min_island_size = 20
		for ind, value in enumerate(binary_map):
		    if value:
			islandsize += 1
		    else:
			if islandsize > min_island_size:
			    directions.append(ind-islandsize//2-1)

			islandsize = 0
		if islandsize > min_island_size:
		    directions.append(ind-islandsize//2)

		# return midpoint of island closest to angle
		return min(directions, key = lambda x: abs(x-angle))

	

	def check_path_blocked(self, direction):
		"""
		Function to return True if path is blocked in specified
		direction (+- 15deg). Otherwise returns False
		"""
		
		freedom = 15
		threshold = 4
		ang = 135 + int(direction)		
		laser_values = self.laser[ang-freedom:ang+freedom]
		
		# if any value in range < threshold: path is blocked
		# range is from direction-freedom to direction+freedom
		if any(l < threshold for l in laser_values):
			# path is blocked			
			return True		
		
		return False		
						
		
	def get_angle_to_waypoint(self):
		"""
		Get relative angle from robot direction to waypoint (ex: -30
		means 30 degrees to the right)
		"""
		ang = np.arctan2(self.waypoint_y-self.robot_y, self.waypoint_x-self.robot_x)
		
		ang = (ang - self.robot_theta) * 180 / np.pi
		
		if ang > 180:
			return ang-360
		if ang < -180:
			return ang+360
		return ang
	
	def circlemove_init(self, turn=None):
		self.waypoint_x = self.robot_x 
		self.waypoint_y = self.robot_y
		self.circlemove_values = []
		self.circlemove_vcont = 0
		
		self.vel_msg.linear.x = 1
		if  all(l > 4 for l in self.laser[:135]):
			self.vel_msg.angular.z = -0.5
			print('CIRCLE MOVE START (R)')
		elif all(l > 4 for l in self.laser[135:]):
			self.vel_msg.angular.z = 0.5
			print('CIRCLE MOVE START (L)')
		else:
			self.waypoint_x = self.robot_x + 7*np.cos(self.robot_theta)
			self.waypoint_y = self.robot_y + 7*np.sin(self.robot_theta)
			self.state = self.GOALSEEK
			print('NO SPACE FOR CIRCLEMOVE')
			
		

	def circle_move(self):				
		if self.circlemove_vcont % 500 == 0:
			self.circlemove_values.append((self.robot_x, self.robot_y, self.gas))

		
		if self.circlemove_vcont > 100000 and self.check_waypoint(0.35):

			print('circle move OVER')
			# define new waypoint
			self.get_waypoint_from_circlemove()

			self.state = self.GOALSEEK

		self.circlemove_vcont += 1

	def get_waypoint_from_circlemove(self):
		# http://paulbourke.net/geometry/circlesphere/
		nvalues = len(self.circlemove_values)
		x1,y1,_ = self.circlemove_values[0]
		x2,y2,_ = self.circlemove_values[nvalues//3]
		x3,y3,_ = self.circlemove_values[2*nvalues//3]
		
		ma = (y2-y1)/(x2-x1)
		mb = (y3-y2)/(x3-x2)

		cx = (ma*mb*(y1-y3) + mb*(x1+x2) - ma*(x2+x3)) / (2*(mb-ma))
		cy = -(1/ma)*(cx-(x1+x2)/2)+(y1+y2)/2
		
		px, py, _ = max(self.circlemove_values, key=lambda val: val[2])
		

		# standard vector equation of line
		k = 10 # chosen randomly
		self.waypoint_x = cx + k*(px-cx)
		self.waypoint_y = cy + k*(py-cy)



if __name__ == '__main__':
	try:
		robot = Robot()
		while not rospy.is_shutdown():
			robot.move()
	except rospy.ROSInterruptException:
		pass





# TODO: dont need to do full circlemove
# gasvalue increases and then starts decreasing, we dont need more
# but we can only do this if we have enough points
