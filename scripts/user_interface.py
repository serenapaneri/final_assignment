#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *

#to check if the service is active
active_ = False

def unable_service(req):
	"""
	This function contains the variable declared above that is
	used to enable the service.
	"""
	global active_
    
	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message = 'Done!'

	return res
	
def getInput():
	"""
	This function get the input, given by the user, on which of the 5
	behaviors proposed, the robot must follow.
	If one of the input chosen by the user is already active, the 
	function doesn't ask to give again the input.
	"""	
	global active_

	#to disable the service 
	active_ = False 
	
	# reading the previous input
	prev_input_ = rospy.get_param('/input')
	input_ = prev_input_
	
	#in order to make the user to choose one of the 5 possible inputs
	while (prev_input_ == input_) or (input_ > 5 or input_ < 1):
		if input_ > 5 or input_ < 1: 
			#in the case in which the user make another selection
			print "Unknown input, please try again" 
		
		#propose to the user which are the real possibilities
		print("Please select one of the following senteces\n")
		print("1 - Move the robot randomly in the environment, by choosing one of six possible target positions\n")
		print("2 - The user can chose the next target position\n")
		print("3 - Start following the external walls\n")
		print("4 - Stop the robot in the last position\n")
		print("5 - Change the planning algorithm from move_base to bug0 and vice versa\n")

		#read the input typed by the user	
		input_ = (int(raw_input("Please select a number between 1 and 5: ")))

	#set the choice made by the user
	if input_ >= 1 and input_ <= 5:
		rospy.set_param('/input', input_)

def main():
	"""	
	The main function allows the user to choose the robot's behavior.
	If the service is active it call the function getInput that allows
	the user to make a new choice. If it is not, it check if the selected
	behavior is the second one and in that case change it with the fourth one.
	"""
	global active_
	
	#init user_interface
	rospy.init_node('user_interface')

	#service that allows the user to choose a new input
	srv_user_interface = rospy.Service('/user_interface_service', SetBool, unable_service)
	
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		#if the service is not active
		if not active_: 
			rate.sleep()
			
			#if the selected behavior is the second one
			if rospy.get_param("/input") == 2:
				#change it in the fourth behavior
				rospy.set_param("/input",4) 
			
			continue
		
		#if the service is active	
		else: 
			getInput() # allow the user to choose a new behaviour
		
		rate.sleep()
		
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
