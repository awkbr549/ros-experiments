This test creates an exploration algorithm with ROS Stage.

Running the Code
	There is a script titled `run.sh' provided. It runs catkin_make, then launches
the default world, then launches the exploration code.
	To run just the exploration code, run
$ roslaunch explore_map explore_map_node.launch

Exploration Algorithm
1:	Randomly decide the first direction in which to turn (50/50 left/right)
2:	Take a measurement. 
3:	If there is no obstacle too close to the robot, then
4:		Move forward with a given velocity (linear and angular) for either 
		1.6s or 2.4s (randomly different each time). Ensure that each turn
		direction has different angular speed, such that right turns are
		sharper than left turns (or vice versa). One turn should be
		approximately 1.57x as fast as the other, where rotating with the
		slower speed turn for 2s is approx 45 degrees.
5:	If there is an obstacle too close to the robot, then,
6:		Change the direction in which the robot rotates.
7:		Reverse linear velocity to back up slightly
8:		Rotate in place for either 1.6s or 2.4s (randomly), where rotating
		for 2s is approximately 45 degrees.
9:	Repeat from Step 2

Termination of Algorithm
	After 1 hour, stop exploration. Because the movement is pseudo-random (the
1.6s or 2.4s movements/rotations are chosen at a 50/50 ratio), and because the robot
curves in one direction more than the other (also for random times), as time
approaches infinity, the robot will explore the entire map, but in a realistic
scenario, time is limited for people, so 1 hour seemed good enough.

Video of Near-End Result
https://youtu.be/WOJ0bGraMFA
