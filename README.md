# Braitenberg-Simulation-ROS
#Bachelor thesis 2021 - Albert Lauen

#Use 

roslaunch braitenberg_simulation Braitlaunch.launch

#This launches the Braitenberg simulation with all the different robots
####The robot1 vehicle code is a test mule, so it might not work properly

roslaunch braitenberg_simulation run_robots.launch

#This starts the simulation, and lets the robots converge towards the linear gradient.
#When they move along the x-axis, the gradient will increase. 

roslaunch braitenberg_simulation spotbrait.launch

#This launches the quadruped robots, from the "robots" package from "Champ simulation".
#Just use the run_robots launch file, but change the topic for braitenberg2.py to /odom, instead of /robot2/odom.

roslaunch braitenberg_simulation one_robot_broadcaster.launch

#This launches the broadcaster in the gazebo world. 

#This starts the braitenberg algorithm for robot1.
rosrun braitenberg_simulation bb_new.py

#This starts publishing the x-y coordinates, which the braitenberg algorithm needs to navigate.
rosrun braitenberg_simulation move_broadcaster.py


