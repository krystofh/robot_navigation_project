1) install ROS melodic
2) clone Github repository
3) copy maps from maps folder to your Gazebo models folder
   Attention: This folder is probably only root-accesible, so use sudo
   sudo cp -r ~/catkin_ws/src/robot_navigation_project/maps/map1 /usr/share/gazebo-9/models
   sudo cp -r ~/catkin_ws/src/robot_navigation_project/maps/map1-closed /usr/share/gazebo-9/models
4) run roscore
5) setup robot model variable
   Turtlebot3 will be user with available models: burger, waffle, waffle_pi
   http://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes
	Command:
   export TURTLEBOT3_MODEL=burger	

SIMULATION COMMANDS:
rosservice call /gazebo/reset_world     - resets the robot to initial position
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch     - keyboard control

EXAMPLE SCRIPT USAGE
rosrun robot_navigation_project rotate.py 10.0 90.0 True	 - rotate 90 deg with 10 deg/s clockwise
rosrun robot_navigation_project move_forward.py 0.1		 - move forward at 0.1 m/s
rosrun robot_navigation_project move_distance.py 1.0 0.1 True    - distance, speed, forward?
rosrun robot_navigation_project pose_observer.py 5               - print robot pose with rate 5 Hz



UNIX COMMANDS:
chmod +x pythonscript.py      - use in the folder of the scripts to allow their execution



   
