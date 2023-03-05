#!/usr/bin/env python3
import rospy 
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt

class TurtleBot:
	def __init__(self, beta_, alpha_ ):
		rospy.init_node("turtle_control")
		self.velocity_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)
		self.pose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.update_pose)
		
		self.beta = beta_
		self.alpha = alpha_
		self.pose = Pose()
		self.rate = rospy.Rate(10)
		
	def update_pose(self,data):
		""" a callback function which is called each time a new message of type Pose is recieved """
		self.pose = data
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)
		
	def euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y),2))
	
	def x_distance(self, goal_pose):
		return - (goal_pose.x - self.pose.x)
	
	def y_distance(self, goal_pose):
		return - (goal_pose.y - self.pose.y)
		
	def x_linear_vel(self, goal_pose):
		return sqrt(sqrt(self.beta)) * self.x_distance(goal_pose)
	
	def y_linear_vel(self, goal_pose):
		return sqrt(sqrt(self.beta)) * self.y_distance(goal_pose)
	
	# I am not going to use this method
	def linear_vel(self, goal_pose):
        	return self.beta * self.euclidean_distance(goal_pose)
  
	def steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
	
	def angular_vel(self, goal_pose):
		return self.alpha * (self.steering_angle(goal_pose) - self.pose.theta)
	
	def move2goal(self):
		goal_pose = Pose()
		
		goal_pose.x = 1 #this guy is an input
		goal_pose.y = 1 #this guy is an unput
		
		distance_tolerance = 0.02
		
		vel_message = Twist()
		
		
		while self.euclidean_distance(goal_pose) >= distance_tolerance :
			
			vel_message.linear.x = self.x_linear_vel(goal_pose)
			vel_message.linear.y = self.y_linear_vel(goal_pose)
			vel_message.linear.y = 0
			
			vel_message.angular.x = 0
			vel_message.angular.y = 0
			vel_message.angular.z = self.angular_vel(goal_pose)
			
			self.velocity_pub.publish(vel_message)
			self.rate.sleep()
		
		# stop the robot
		vel_message.linear.x = 0
		vel_message.linear.y = 0
		
		vel_message.angular.z = 0
		
		rospy.spin()
		
alpha = rospy.get_param("alpha",0)
beta = rospy.get_param("beta",0)
x = TurtleBot(beta, alpha)
x.move2goal()

		

			
		
		
	


