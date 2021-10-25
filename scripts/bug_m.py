#! /usr/bin/env python

import rospy
import time

# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations

# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist

import math


pub = None

#client service for 'go to point' 
srv_client_go_to_point_ = None

#client service for 'wall follower'
srv_client_wall_follower_ = None

#yaw angle set equal to zero
yaw_ = 0

#maximum error allowed for yaw angle
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees

#position of the robot
position_ = Point()

#desired position 
desired_position_ = Point()

#previous value of the coordinate x
prev_x=0

#previous value of the coordinate y
prev_y=0

#coordinate x,y and z to set the desired position 
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0

#regions of the laser
regions_ = None

#possible states that implement the behavior of bug0
state_desc_ = ['Go to point', 'wall following', 'target reached']
state_ = 2
# 0 - go to point
# 1 - wall following
# 2 - target reached

#variable to enable or disable bug0 algorithm
active_ = False


def setBug0(req):
	"""
	Function created with the aim to be able to
	enable or disable the bug0 algorithm
	"""
	global active_
	
	active_= req.data
	res = SetBoolResponse()
	res.success = True
	res.message = 'Done!'
	return res


#callbacks

def clbk_odom(msg):
	"""
	CallBack function that achieves the actual position
	of the robot through Odometry
	"""
	
	global position_, yaw_

    #position
	position_ = msg.pose.pose.position


	#yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]


def clbk_laser(msg):
	"""
	Callback function that reads the laser value
	of the regions using LaserScan
	"""
	
	global regions_
	regions_ = {
		'right':  min(min(msg.ranges[0:143]), 10),
		'fright': min(min(msg.ranges[144:287]), 10),
		'front':  min(min(msg.ranges[288:431]), 10),
		'fleft':  min(min(msg.ranges[432:575]), 10),
		'left':   min(min(msg.ranges[576:719]), 10),
	}


def change_state(state):
	"""
	This function makes possible to change the 
	state of bug0
	"""
	
	global state_, state_desc_
	global srv_client_wall_follower_, srv_client_go_to_point_
	global active_, prev_x, prev_y
    
	state_ = state
    
	if state_ == 0: #go to point
		resp = srv_client_go_to_point_(True)
		resp = srv_client_wall_follower_(False)
        
	if state_ == 1: # wall following
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(True)
        
	if state_ == 2: #target reached
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(False)
		twist_msg = Twist()
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		pub.publish(twist_msg)
        
		#if the robot is activated 
		if active_:
			rospy.set_param("/bug0_reached", True)
			
			#change the previous values
			prev_x = rospy.get_param('des_pos_x')
			prev_y = rospy.get_param('des_pos_y')
			
        
def normalize_angle(angle):
	"""
	This function allows to normalize the angle
	"""
	
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle


def main():
	"""
	The main function has to control the robot while this is moving
	using bug0 algorithm previously implemented. In bug0 in fact there 
	can be three types of states:
	0 - go to point
	1 - wall following
	2 - target reached
	A distance is calculated between the actual position of the robot
	and the target and if it is less than a predetermined value the 
	state of the robot is considered as 'target reached'. 
	If the algorithm is not able to find a path such that the robot
	can arrive to the target position there is a limited amount of time
	counted by a timer that stop the robot in this kind of situation 
	of unsuccess.
	"""
	time.sleep(1)
	global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
	global srv_client_go_to_point_, srv_client_wall_follower_, pub
	global active_, prev_x, prev_y
    
	timer = None

	rospy.init_node('bug0')

	#ROS subsciber for LaserScan and Odometry
	sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
	#ROS pubblisher for Twist
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	#ROS services for the states
	srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_service', SetBool)
	srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_service', SetBool)
	srv = rospy.Service('bug0_service', SetBool, setBug0)

	# initialize state of 'target reached'
	change_state(2)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
	#if the algorithm is not activated
		if not active_: 
			rate.sleep()
			srv_client_go_to_point_(False)
			prev_x = 0
			prev_y = 0 
			continue
            
		else:
			
			if regions_ == None:
					continue

			if state_ == 0: #go to point
					err_pos = math.sqrt(pow(desired_position_.y - position_.y,
																	2) + pow(desired_position_.x - position_.x, 2))
                                    
					#if the error computed above is less than 0.3 or the time pre-determined
            		#by the timer has finished                   
					if(err_pos < 0.3) or timer <= rospy.Time.now():
						if timer <= rospy.Time.now():
							print "Time's up"
						change_state(2) #target reached
			
					#if there is not an obstacle in front of the robot
					elif regions_['front'] < 0.5:
						change_state(1) #go to point

			#if there is an obstacle to overcome
			elif state_ == 1:
				desired_yaw = math.atan2(
						desired_position_.y - position_.y, desired_position_.x - position_.x)
				err_yaw = normalize_angle(desired_yaw - yaw_)
				err_pos = math.sqrt(pow(desired_position_.y - position_.y,
																2) + pow(desired_position_.x - position_.x, 2))

				
				#if the error computed above is less than 0.3 or the time pre-determined
				#by the timer has finished                   
				if(err_pos < 0.3) or timer <= rospy.Time.now():
					if timer <= rospy.Time.now():
						print "Time's up"
					change_state(2) #target reached
				
				#if there is not an obstacle in front of the robot
				if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
					change_state(0) #go to point

			#if the target is reached
			elif state_ == 2:
				#first check if the target has actually been reached
				if prev_x != rospy.get_param('des_pos_x') or prev_y != rospy.get_param('des_pos_y'):
					desired_position_.x = rospy.get_param('des_pos_x')
					desired_position_.y = rospy.get_param('des_pos_y')
					err_pos = math.sqrt(pow(desired_position_.y - position_.y,
															2) + pow(desired_position_.x - position_.x, 2))
										
					#if the actual distance is bigger than the pre-dermined one
					#and so the robot hasn't reached the targed
					if(err_pos > 0.35):
						now = rospy.Time.now()
						#the maximum time is set to 30 seconds 
						timer = now + rospy.Duration(30) 
						change_state(0) #go to point

	rate.sleep()


if __name__ == "__main__":
	main()
