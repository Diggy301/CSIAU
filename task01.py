#!/usr/bin/env python
#_*_coding:utf-8_*_

# Libraries
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, radians, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Global variables
global vel_msg, robotX, robotY, robotTheta, waypoints, current_waypoint, Kp_angular, Kp_linear, linearmove, angularmove
robotX = 0.0
robotY = 0.0
robotTheta = 0.0
linearmove = 0.0
angularmove = 0.0
waypoints = {'tolerance':0.1, 'points':[(2,8),(8,-7),(-5,-5)]} 
current_waypoint = 0
Kp_angular = .5
Kp_linear  = 10


# Callback function, return the robot position in x and y and the robot angle
def callback(msg):
	global robotX, robotY, robotTheta, waypoints, current_waypoint, linearmove, angularmove
	robotX = msg.pose.pose.position.x # Get the position of the robot in X
	robotY = msg.pose.pose.position.y # Get the position of the robot in Y
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	robotTheta = yaw  # Get the robot angle
	
	distance, err_ang = getData(robotX, robotY, waypoints['points'][current_waypoint][0], waypoints['points'][current_waypoint][1], robotTheta)
	
	if checkWaypoint(distance, waypoints['tolerance']):
		if current_waypoint < len(waypoints['points'])-1:
			current_waypoint += 1
		else:
			linearmove = 0
			angularmove = 0
	else:
		linearmove = distance*Kp_linear
		angularmove = err_ang*Kp_angular
		
	vel_msg.linear.x = linearmove
	vel_msg.angular.z = angularmove
	
	
			 
def getData(x0, y0, x1, y1, theta):
	d = sqrt((x1 - x0)**2 + (y1 - y0)**2)
	ang = atan2(y1 - y0, x1 - x0) - theta 
	
	if ang > pi:
		return d, ang-2*pi
	if ang < -pi:
		return d, ang+2*pi
	return d, ang
	
def checkWaypoint(d, tol):
	if d <= tol:
		return True
	return False
	

def robotControl():
	global pub, vel_msg
	rospy.init_node('check_odometry', anonymous = True) # Initializes a ROS node
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10) # Create a publisher to the velocity topic
	odom_sub = rospy.Subscriber('/odom', Odometry, callback)  # Create a subscriber to the odometry topic
	vel_msg = Twist() # Create a variable receiving the message type Twist to speed topic publications
	rate = rospy.Rate(10) # Operating rate
	
	# Speed initialization
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

	while not rospy.is_shutdown():
		print("Robot position in X:", robotX)
		print("Robot position in Y:", robotY)
		pub.publish(vel_msg) # Publish the speed
		rate.sleep()
  
	rospy.spin() # Stop the ROS node pressing Ctrl+C

# Main function
if __name__ == '__main__':
	try:
		robotControl()
	except rospy.ROSInterruptException:
		pass