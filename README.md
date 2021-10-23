# Final_assignment - Research Track I - Serena Paneri 4506977

This final assignment requires to develop a software architecture for the control of the robot in the environment. The software will rely on the move_base and gmapping packages for localizing the robot and plan the motion.

In order to run the content of final_assignment the user have to execute the launch file called "final_assignment.launch" by typing the following command: 

```
roslaunch final_assignment final_assignment.launch
```

### Expected behavior:
The architecture should be able to get the user request, and let the robot execute one of the following behaviors
(depending on the user’s input):
-  1) Move randomly in the environment, by choosing 1 out of 6 possible target positions:
[(-4,-3); (-4,2); (-4,7); (5,-7); (5,-3); (5,1)]
- 2) Directly ask the user for the next target position (checking that the position is one of the possible six) and reach it
- 3) Start following the external walls
- 4) Stop in the last position
- 5) Change the planning algorithm to dijkstra (move_base) to the bug0

If the robot is in state 1, or 2, the system should wait until the robot reaches the position in order to switch to the state 3 and 4 or (if implemented) to change the planning algorithm.

### Scripts:
In the scripts folder in final_assignment there are 7 different programs.
- **bug_m.py**: it provides a service for carrying out the bug0 algorithm behavior while performing the target achivement.
- **go_to_point_service_m.py**: it provides a service used for making the bug0 algorithm capable of implementing the go-to-point behavior, which consists of going along a straight line motion between the robot and the target.
- **wall_follow_service_m.py**: it provides a service for simulating the wall-following behavior and it is also used in bug0 algorithm in order to achieve the target position, obviosly only if the robot interfaces with it during its path. 
- **robot_behaviors.py**: it menages all the 5 possible behaviors described above 
- **user_interface.py**: it provides a service that, when it is active, ask the user to choose between the 5 possible behaviors previously described. If the behavior selected by the user is different from the previous chosen one this server saved it inside the parameted input in a way that **robot_behaviors** can easily access to it. 
- **first_behavior.py**: it provides a service that generates a random target position among the six proposed above. Then the new coordinates are saved inside the parameters des_pos_x (the x coordinate of the target position) and des_pos_y (the y coordinate of the target position) in a way that **robot_behaviors** can easily access to them.
- **user_first_behavior.py**: it provides a service that ask the user to choose a new target position between the six possible proposed above. If the user coorectly select one of them the service save the new coordinates of the target position insde the parameters des_pos_x (the x coordinate of the target position) and des_pos_y (the y coordinate of the target position) in a way that **robot_behaviors** can easily access to them.
