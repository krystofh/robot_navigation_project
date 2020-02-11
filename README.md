# robot_navigation_project

## Project description
The aim of the project is to investigate the function of graph-based algorithms, 
especially the A* search algorithm. It should be implemented in Matlab and its functionality 
should be demonstrated on a few examples. In the end, the algorithm should be implemented in 
ROS - ideally with a simulation of autonomous robot navigation though an environment in Gazebo.
As it was my first project in ROS, it should serve as an introduction into this system
and could hopefully be useful for other programmers eager to explore its functions.

## How to run the code

1) Install *ROS melodic* or make sure that you have one installed

2) Clone Github repository

3) Copy map files: ```'model.config'```, ```'model.sdf'```
   from maps folder to your Gazebo models folder
   <br>Attention: This folder is probably only root-accesible, so use ```sudo```
   ```
   sudo cp -r ~/catkin_ws/src/robot_navigation_project/maps/map1 /usr/share/gazebo-9/models
   
   sudo cp -r ~/catkin_ws/src/robot_navigation_project/maps/map1-closed /usr/share/gazebo-9/models
   ```

4)  ```catkin_make```
 run this command in the 'catkin_ws' workspace to build your code

4) ```rocore```
    to start the ros core server

5) Setup robot model variable
   ```Turtlebot3``` will be user with available models: ```burger, waffle, waffle_pi```
   
   http://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes
   
	Export command:
      ``` 
      export TURTLEBOT3_MODEL=burger 
      ```
	
6) Launch the environment
      ```
      roslaunch robot_navigation_project robot_navigation.launch
      ```

7) Run the python scripts you like. In the *src* folder, there are:

   **/movement functions** - basic robot movement, see details below

   **/a_star_python_algorithm** - separate codes and maps for small algorithm tests, also with visual input/output

   ***path_planer_client.py*** - client sending two types of requests to the server: 
				```SetOccupancyGrid.srv``` for saving the map on the server
				```CalculatePath.srv``` for request of path calculation between A and B
   
   ***path_planner_server.py*** - server which receives the requests and either saves the map or calculates the path
				``StatusMsg.msg`` is returned when map is successfully saved
				The map is saved in ``Matrix2D.msg`` message type...
        
   ****a_star.py**** - the algorithm in python

### SIMULATION COMMANDS:
- Reset the robot to initial position
```
rosservice call /gazebo/reset_world
``` 
- Keyboard control
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### EXAMPLE SCRIPT USAGE
- rotate 90 deg with 10 deg/s clockwise
```
rosrun robot_navigation_project rotate.py 10.0 90.0 True
```	 
- move forward at 0.1 m/s
```
rosrun robot_navigation_project move_forward.py 0.1		 
```
 - distance, speed, forward-direction
```
rosrun robot_navigation_project move_distance.py 1.0 0.1 True   
```
- print robot pose with rate 5 Hz
```
rosrun robot_navigation_project pose_observer.py 5
```



### UNIX COMMANDS:
- Allow the execution of the scripts (pythonscript obviously has to be changed to the specific name)
```
chmod +x pythonscript.py
``` 

### OTHER USEFUL ROS COMMANDS:
Command | Description
------------ | -------------
```rosnode list```   			| - display list of running ROS nodes <br>
```rostopic list```  			|- display list of topics in the package<br>
```rosnode info <<nodeName>> ```     | - display info about the given node<br>
```rosmsg show <<messageName>>``` <br>e.g. --> ```rosmsg show geometry_msgs/Twist```   | - display info about a message type (custom or standard)<br>
```rostopic echo <<topicName>>```     |- keep printing the data of the topic<br>
```roservice list```               |- list of available ROS services<br>
```rosservice info <<serviceName>>``` |- details about a service<br>
```rossrv info <<messageType>>```     |- info about server request message type<br>
```rosservice call <<serviceName>>``` |- call a service<br>
```rossrv show <<serviceName>>```	|- check that a service is ready for being called<br>
```rosrun rqt_graph rqt_graph```  |- graph of the package nodes and topics<br>
```rostopic pub -1 <<topicName>> <<messageType>> -- '<<parameters>>'``` <br><br>*e.g.*  ```rostopic pub -1 /turtle1/cmd_vel geomtery_msgs/Twist -- '[x y z]''[x y z]'``` <br>where x y z are to be replaced by the desired velocities (first linear, then rotation)<br>| - publish once a topic and finish<br>
        

   

   
