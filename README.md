# Lake_cleaning_machine_autonomous_navigation

Project Description:

The project focused on developing a simulation of an autonomous cleaning robot using ROS, Gazebo, and RViz. The simulation environment consisted of a virtual cleaning area. The control system employed algorithms for autonomous navigation and path planning. There is also a Coordinate Transformation script that converts the marked coordinates (taken from the KML file generated by Google Earth) relative to a reference area. The robot_contorl.py script takes these converted coordinates as input coordinates and proceeds with the simulation. For now, I have used manually given coordinates.

To run the simulation:
1. Open your terminal and run

```
$ roscore
```
2. Launch Gazebo model:

```
$ roslaunch lcm_description gazebo.launch
```
3. Launch the RViz for the path tracing:

```
$ roslaunch lcm_description rviz.launch
```
4. Run the path tracing script:

```
$ rosrun lcm_description path_tracer.py
```
5. Add the path topic in rviz window through the add button:
![30-06-2023](https://github.com/divyansh1109/Lake-cleaning-machine-autonomous-navigation/assets/106006613/e1babefc-9fda-4aa1-b995-1e941ade1ded)

6. Run the robot control node:

```
$ rosrun lcm_description robot_control.py
```
