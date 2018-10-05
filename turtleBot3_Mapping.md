# TurtleBot 3 Burger  (Mapping)
We also tested another robot: a [TurtleBot3 Burger](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
This robot has a Lidar (Laser) - Sensor, so it is easy to build a map using sophisticated libraries like gmapping. 

## How-to use
1. set up all parameters correctly, i.e `$ turtleBot` in all terminals
	- first you have to add following lines to your `.bashrc`-File
		- `alias turtleBot='export ROS_IP=<your_ip>; export ROS_MASTER_URI=http://<your_ip>:11311'`
		- `export TURTLEBOT3_MODEL=burger`
2. run `$ roscore`
3. connect the battery and the board
4. switch the board on
5. connect to the robot 
	- `$ ssh robot@robot_ip` 
	- run the bridge there `$ roslaunch turtlebot3_bringup turtlebot3_robot.launch`
	- depending on what you need, open additional terminals on the robot and run other software, e.g. to start the camera `$ roslaunch raspicam_node camerav2_1280x960.launch`
	- you also want `$ rosrun rqt_reconfigure rqt_reconfigure` and there tick `vflip`, to rotate the image 180deg.
6. run all applications you want on the remote PC
	- e.g. the remote receiver node: `$ roslaunch turtlebot3_bringup turtlebot3_remote.launch`
	- SLAM and navigation:
		- `$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping|cartographer|hector|karto|frontier_exploration` (the correct libraries have to be installed)
		- move the robot, e.g. teleop keyboard `$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
		- save the map `$ rosrun map_server map_saver -f <name and location of where to save the map>`
		- stop the SLAM node and run `$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<location of map file.yaml>`
		- navigate (using rviz)
7. shut down the robot with `$ sudo poweroff`
8. wait and then switch the board off
9. disconnect the battery 
10. if you need a raw image topic run: `$ rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=/raspicam_node/image`