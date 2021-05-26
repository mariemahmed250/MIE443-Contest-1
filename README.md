# MIE443-Contest-1

Contest 1 - Mapping an unknown environment autonomously with Turtlebot2

How the code runs:
1. Open four terminator windows on Ubuntu inside the catkin_ws folder from the submission package.
2. Run the following command in each of the four windows:
	source devel/setup.zsh
3. Run the following command in a window:
	catkin_make
4. In the first window, run the following command to launch gazebo: 
	roslaunch mie443_contest1 turtlebot_world.launch world:=practice
5. In the second window, run the following command to run gmapping: 
	roslaunch mie443_contest1 gmapping.launch
6. In the third window, run the following command to see the progress of the mapping: 
	roslaunch turtlebot_rviz_launchers view_navigation.launch
7. In the fourth window, run the following command to run the contest1.cpp file: 
	rosrun mie443_contest1 contest1
8. Once the simulation of map exploration is finished, run the following command to store the scanned map (in the directory of your catkin_ws):
	rosrun map_server map_saver -f <your_filename>

***
Note: If the gazebo, rviz or terminators are altered in any way (repeating source, catkin_make, dragging robot around the map, etc.),
there is a possibility of the code not working properly with minLaserDist, maxLaserDist, and maxDistAngle all being printed as 0 in the code running terminator.
If this occurs, the algorithm is receiving 0 values for laser readings throughout the entire run, and the robot stays in one spot and/or spins in place.
In order to avoid this, check the setup/overwriting issues and make sure aftering running 'catkin_make' command, it displays the following output lines:
	[ 66%] Building CXX object mie443_contest1/CMakeFiles/contest1.dir/src/contest1.cpp.o
	[100%] Linking CXX executable /catkin_ws/devel/lib/mie443_contest1/contest1
You could also try rebooting the VM or Docker, or running 'rosclean purge' in the terminal if this issue continues happening.
