#!/usr/bin/env python

import rospy
import random
from std_srvs.srv import *

#X coordinates of each target position
x_pos = [-4.0, -4.0, -4.0, 5.0, 5.0, 5.0]

#Y coordinates of each target position
y_pos = [-3.0, 2.0, 7.0, -7.0, -3.0, 1.0]

def random_position(empty):
	"""	
	This function is created with the aim of generating a new 
	random position.
	It is also checked if the postion choosen does not correspond
	to the actual one.
	"""
	#reading the previous position to avoid repetitions
	prev_pos_x = rospy.get_param("des_pos_x")
	prev_pos_y = rospy.get_param("des_pos_y")
	new_pos_x = prev_pos_x
	new_pos_y = prev_pos_y
	
	while prev_pos_x == new_pos_x and prev_pos_y == new_pos_y:
		
		#choosing randomly one of the 6 possible positions
		random_pos = int(random.uniform(0,5))
		new_pos_x = x_pos[random_pos]
		new_pos_y = y_pos[random_pos]

	#setting the correct position's coordinates	
	rospy.set_param("des_pos_x", x_pos[random_pos])
	rospy.set_param("des_pos_y", y_pos[random_pos])
	
	return []

def main():
	"""
	The main function defines the service first_behavior
	"""
	rospy.init_node('first_behavior') 

	#creating a service related to a function called random_position
	rospy.Service('/first_behavior_service', Empty, random_position)
	
	rospy.spin()
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
