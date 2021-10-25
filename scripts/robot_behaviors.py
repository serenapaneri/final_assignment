#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_srvs.srv import *

#to check if the target is reached
target_reached_ = None

#ROS publisher for a new goal
pub_new_goal_ = None

#ROS publisher to stop robot's velocity
pub_stop_vel_ = None

def set_goal():
	"""
	This function is created with the aim for setting a 
	new goal.
	When the variable goal is set, it'll be published
	through the publisher pub_new_goal_.
	"""
	global pub_new_goal_
	
	goal = MoveBaseActionGoal()

	goal.goal.target_pose.header.frame_id = "map"
	goal.goal.target_pose.pose.orientation.w = 1
	goal.goal.target_pose.pose.position.x = rospy.get_param('des_pos_x')
	goal.goal.target_pose.pose.position.y = rospy.get_param('des_pos_y')

	pub_new_goal_.publish(goal)

def stop_robot():
	"""
	This function is created with the aim to stop the 
	velocity of the robot.
	The velocity is published through the publisher 
	pub_stop_vel_.
	"""
	global pub_stop_vel_
	
	stop_vel= Twist()
	stop_vel.linear.x = 0
	stop_vel.linear.y = 0
	stop_vel.angular.z = 0

	pub_stop_vel_.publish(stop_vel)


def clbk_target(res):
	"""
	This CallBack function is used when the status of
	the robot has been changed.
	The status checked is the one "SUCCEEDED" which correspond to
	the one in which the robot reaches the target.
	It is useful to understand when a new behavior can be sent.
	"""
	global target_reached_
	
	status = res.status.status
	
	if status == res.status.SUCCEEDED:
		target_reached_ = True
		rospy.set_param("/bug0_reached", target_reached_)
		print "Target reached"

def behavior():
	"""
	Main function that has to manages the possible 
	behaviours of the robot.
	"""
	global pub_new_goal_, target_reached_, pub_stop_vel_
	
	time.sleep(10)
	
	#target set as reached
	target_reached_ = True
	
	#init the node robot_behaviors
	rospy.init_node('robot_behaviors')

	#getting the deafult input
	input_ = rospy.get_param("/input")

	#ROS subscriber that uses clbk_target
	rospy.Subscriber("/move_base/result", MoveBaseActionResult, clbk_target)

	#ROS publisher for the new goal
	pub_new_goal_ = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
	
	#ROS publisher to stop robot's velocity
	pub_stop_vel_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
	#ROS service that reads the input sent by the user
	srv_user_interface = rospy.ServiceProxy('/user_interface_service', SetBool)
	srv_user_interface(True)

	#ROS service that provides a random target
	srv_first_behavior = rospy.ServiceProxy('/first_behavior_service', Empty)

	#ROS service that set new target revived by the user
	srv_user_first_behavior = rospy.ServiceProxy('/user_first_behavior_service', Empty)

	#ROS client of wall following
	srv_wall_follower = rospy.ServiceProxy('wall_follower_service', SetBool)
	srv_wall_follower(False)

	#ROS client using bug0 algorithm
	srv_client_bug0 = rospy.ServiceProxy('bug0_service', SetBool)
	
	#the program starts with the move_base algorithm
	#so bug0 is set as false
	bug0 = False
	srv_client_bug0(bug0)

	#variables used to avoid repeating several times the behaviors
	wall_ok = False
	stop_ok = False
	bug0_ok = False
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		#for each cycle it reads the input set one the param
		input_ = rospy.get_param("/input")
		
		#if the bug0 is active, reads the reached value from the param
		if bug0:
			target_reached_ = rospy.get_param("/bug0_reached")

		#if the behavior 1 is set and the target is reached
		if input_ == 1 and target_reached_:
			target_reached_ = False
			rospy.set_param("/bug0_reached", target_reached_)
			
			#reset values 
			wall_ok = False
			stop_ok = False
			bug0_ok = False
			
			#stop wall follower
			srv_wall_follower(False)
			#stop the robot in order to reach the new target
			stop_robot()
			
			#next random target position
			srv_first_behavior()
			print ("Random target position [" + str(rospy.get_param('des_pos_x')) + ", " + str(rospy.get_param('des_pos_y')) + "]\n")
			
			#if the bug0 algorithm is not activated then use move_base
			#to reach the goal
			if not bug0:
				set_goal()
			#else activate bug0
			elif bug0:
				srv_client_bug0(bug0)
			
			#allow the user to choose a new behavior
			srv_user_interface(True)

		#if the behavior 2 (user target) is set and the target is reached
		elif input_ == 2 and target_reached_:
			target_reached_ = False
			rospy.set_param("/bug0_reached", target_reached_)
			
			#reset values
			wall_ok = False
			stop_ok = False
			bug0_ok = False

			#stop the user input in order to make him
			#to choose a new target position
			srv_user_interface(False)
			#stop wall follower
			srv_wall_follower(False)
			#stop the robot to reach the new target positio
			stop_robot()
			
			#read the user new target position through the service
			srv_user_first_behavior()
			
			#if the bug0 algorithm is not active use move_base
			#to reach the goal
			if not bug0:
				set_goal()	
			#else activate bug0
			elif bug0:
				srv_client_bug0(bug0)

		#if the behavior 3 and the target is reached
		elif input_ == 3 and target_reached_:
			#if the behavior isn't already set
			if not wall_ok:
				print "\nWall follower is active"
				
				#if bug0 is activated, disable it 
				if bug0:
					srv_client_bug0(False)
				
				#reset value
				stop_ok = False
				bug0_ok = False
				
				#set true the wall follower 
				wall_ok = True
				
				#wall follower service
				srv_wall_follower(True)
				
				#reset target position
				rospy.set_param("des_pos_x", 0)
				rospy.set_param("des_pos_y", 0)
				
				#allow the user to select a new behavior
				srv_user_interface(True)

		#if the behavior is 4 and the target is reached
		elif input_ == 4 and target_reached_:
			#if the behavior is not set yet
			if not stop_ok:
				print "\nStop the robot"
				
				#reset values
				wall_ok = False
				bug0_ok = False
				
				#set true the stop_robot variable
				stop_ok = True
				
				#stop wall follower
				srv_wall_follower(False)
				stop_robot() #stop the robot
				#allow the user to select a new behavior
				srv_user_interface(True)

		#if the behavior 5 and the target is reached
		elif input_ == 5 and target_reached_: 
			#if the behavior is not set yet
			if not bug0_ok:
				
				#reset values
				wall_ok = False
				stop_ok = False
				
				#bug0 
				bug0_ok = True
				
				#change the algorithm 
				bug0 = not bug0
				srv_client_bug0(bug0)
				
				#stop wall follower
				srv_wall_follower(False)
				stop_robot() 
				#allow the user to select a new behavior
				srv_user_interface(True)
				
				if bug0:
					print "\nSwtiching to Bug0 algorithm"
				else:
					print "\nSwitching to Move Base algorithm"

		rate.sleep()

if __name__ == '__main__':
    try:
        behavior()
    except rospy.ROSInterruptException:
        pass
