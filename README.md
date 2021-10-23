# Final_assignment - Research Track I - Serena Paneri 4506977

This final assignment requires to develop a software architecture for the control of the robot in the environment. The software will rely on the move_base and gmapping packages for localizing the robot and plan the motion.

In order to run the content of final_assignment the user have to execute the launch file called "final_assignment.launch" by typing the following command: 

```
rosrun final_assignment final_assignment.launch
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
