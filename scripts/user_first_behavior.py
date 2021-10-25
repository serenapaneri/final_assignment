#!/usr/bin/env python

import rospy
from std_srvs.srv import *

#X coordinates of each target position
x_pos = [-4.0, -4.0, -4.0, 5.0, 5.0, 5.0]

#Y coordinates of each target position
y_pos = [-3.0, 2.0, 7.0, -7.0, -3.0, 1.0]

def user_position(req):
	"""	
	This function allows the user to select a new target position
	for the robot between the 6 proposed. It obviously check
	if the input recived by the user correspond to the previous
	one selected.
	"""
	
	#checking if the previous position
	prev_pos_x = rospy.get_param("des_pos_x")
	prev_pos_y = rospy.get_param("des_pos_y")
	new_pos_x = prev_pos_x
	new_pos_y = prev_pos_y

	input_ = 0

	#while the new position is equal to the previous one or
	#it is an unknown one.
	while (prev_pos_x == new_pos_x and prev_pos_y == new_pos_y) or (input_ > 6 or input_ < 1):
		print "Choose one of the 6 positions proposed:"
		print "1 - [-4.0,-3.0]\n2 - [-4.0, 2.0]\n3 - [-4.0, 7.0]\n4 - [5.0, -7.0]\n5 - [5.0, -3.0]\n6 - [5.0, 1.0]"
		
		#print the previous target position
		if prev_pos_x != 0 and prev_pos_y != 0:
			print "The previous position was [" + str(prev_pos_x) + ", " + str(prev_pos_y) + "]"
			
		input_ = int(raw_input("\nChoose a number from 1 to 6: "))
		
		#check if it the selected position is one of the 6 possible
		if input_ <= 6 and input_ >= 1:
			new_pos_x = x_pos[input_-1]
			new_pos_y = y_pos[input_-1]

	#setting the correct position's coordinates
	rospy.set_param("des_pos_x", x_pos[input_-1])
	rospy.set_param("des_pos_y", y_pos[input_-1])

	print ("Selected position [" + str(x_pos[input_-1]) + ", " + str(y_pos[input_-1]) + "]\n")
	
	return []

def main():
	"""	
	The main function defines the service called user_first_behavior_service
	"""
	
	rospy.init_node('user_first_behavior')
	
	#the service allows the user to choose a position using 
	#the function user_position 
	rospy.Service('/user_first_behavior_service', Empty, user_position)
	
	rospy.spin()
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
