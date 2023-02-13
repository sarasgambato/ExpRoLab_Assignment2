# ExpRoLab Assignment 2
The code documentation for this assignment can be found [here](https://sarasgambato.github.io/ExpRoLab_Assignment2/index.html).

## Introduction
In the second assignment of the Experimental Robotics Laboratory course, we were requested to integrate the architecture developed in the [1st assignment](https://github.com/sarasgambato/ExpRoLab_Assignment1) (by modifyig it, eventually) with a robotic simulation.

We were provided with [this package](https://github.com/CarmineD8/assignment2) and our requirements were:
- to add a robot in the environment and to spawn it in the initial the position x = -6.0, y = 11.0;
- to the "semantic" map of the environment by detecting, without moving the base of the robot, all seven [AruCo markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) that were present around it, by calling the provided service node;
- to start the patrolling algorithm by relying on autonomous navigation strategies and on the information collected and stored in the ontology during the previous step;
- to perform a complete scan of the room (by rotating the base or the camera) when a room was reached.

## Software architecture
The user can find a detailed description of the software architecture that was used [here](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/README.md#software-architecture).

However, some changes were done.

<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment2/blob/main/images/UML2.png" width=70%, height=70%>
</p>

### The `detect_marker` node
This node is responsible of moving the robot's arm, on the top of which there is a camera, in order to detect the markers around itself: the node subscribes to the topic in which the camera publishes the image data, and an ArUco detector process the image to check if there were some markers every time a new message is received . The ID of the detected markers are stored in a list and published in the topic `id_list` as a message of type `std_msgs/Int32MultiArray`.

### The `marker_server` node
This node implements a server:
- **request**: ID of a marker
- **response**: information about a room (its coordinates, the other rooms it is connected to, the doors that connect these rooms)

Based on the request that the server receives, different information is sent back to client.

### The `load_ontology` node
The script [load_ontology.py](https://github.com/sarasgambato/ExpRoLab_Assignment2/blob/main/scripts/load_ontology.py) has been modified in order to implement a server:
- **request**: ID of a marker
- **responsee**: room name and its _(x,y)_ coordinates

Every time the node receives a new request, it gathers information about the ontology through the `marker_server`, manipulates the ontology with the new information, and then it sends back to the client the new information received.

### The `robot_state` node
This node was modify to make sure that if the battery was low, the robot would start recharging only after having reched the desired room. To do so, the node subscribes to the `/odom` topic and checks that the position of the robot is inside a circumference with radius = 2 and centered in the coordinates of the recharging room.

The radius of the circumference was determined empirically by:
1. making the robot go to the recharging room multiple times;
2. getting each time the pose of the robot when `move_bose` recognized that the goal was reached;
3. among the collected poses, getting the one which was farthest from the actual coordinates of the recharging room, and adding a small offset.

### Other changes
1. The `behavior` node, the one implementing the Finite State Machine (FSM), builds the ontology by waiting for the `detect_marker` node to publish the list of IDs, then it sends one ID at a time to the `load_ontology` node, and in the end it stores all the information received by the last mentioned node for further use in the FSM.
2. Every time the FSM has reasoned about which location to reach, the coordinate of the location are sent as a goal to the [move_base](http://wiki.ros.org/move_base) Action Client, which computes a path to the given point based on the knowledge of the map, which is updated every time the robot moves in the environment through [gmapping](http://wiki.ros.org/gmapping).
3. The `planner` and `controller` nodes, which simply simulated the planning of a path and the control of the robot, have been removed from the overall architecture, given that now all of this is done through `move_base`.
4. The `helper` has been divided into 2 different files for sake of simplicity, one concerning the behavior of the FSM and one concerning the interaction with Action Clients.

## Installation & running
### Installation
To correctly use this software, the user must follow these steps to install the required packages/repositories.
1. Given that the author decided to use the file [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl) from the repository [topological_map](https://github.com/buoncubi/topological_map) (by author [Luca Buoncompagni](https://github.com/buoncubi)), the user must clone the mentioned repository and the current one in the ROS workspace.
2. Install the [aRMOR package](https://github.com/EmaroLab/armor) by following [this tutorial](https://github.com/EmaroLab/armor/issues/7).
3. Clone the [armor_py_api](https://github.com/EmaroLab/armor_py_api) repository in your ROS workspace.
4. Perform what is described in these [notes](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/README.md#note).
4. Install the packages [SMACH](http://wiki.ros.org/smach), [ros_control](http://wiki.ros.org/ros_control) and [gazebo_ros_control](http://wiki.ros.org/gazebo_ros_control), plus the [Navigation stack](http://wiki.ros.org/navigation).
6. Run `chmod +x <file_name>` for each file inside the folders `scripts` and `src` of the package `ExpRoLab_Assignment2`.
7. Run `catkin_make` from the root of your ROS workspace.

### Running
To correctly use launch file, the user must install `xterm` with the following command lines:
```sh
sudo apt-get update
sudo apt-get -y install xterm
```
Now, the user can launch the program frome the source of the ROS workspace with the following command:
```sh
roslaunch ExpRoLab_Assignment2 assignment.launch
```

### ROS Parameters
The software requires the following ROS parameters:
- `config/recharge_room`: name and coordinates of the room where the robot has to recharge.
- `config/robot`: name of the robot.
- `state/initial_pose`: intial position of the robot.
- `config/floor_markers`: AruCo markers placed on the floor.
- `config/ceil_markers`: AruCo markers placed on the top of the walls.


## System's features
### Environment & Robot
The enviornment used for the assignment is the one shown in the following figure. Some of the markers can be seen (there are 7 markers in total), plus the robot in its initial position.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment2/blob/main/images/environment.png" width=70%, height=70%>
</p>

The user can notice that the robot is very simple, with:
- a chassis with two wheels and a caster;
- a laser attached to the chassis to perform collision avoidance and to build the map through gmapping;
- an arm with three links and a camera on top of it to detect the AruCo markers.

The robot moves in the environment based on some rules:
- if there are no urgent locations, visit the corridors
- if there are urgent locations, visit them based on their timestamp (the ones which have not been visited for the longest have precedence)
- once a location has been reached, the robot moves its camera of 360 degrees

### Running code behavior

In the following figure the user can see that the arm of the robot is rotated and that all markers have been detected.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment2/blob/main/images/detecting.png" width=70%, height=70%>
</p>

In the following figure the software creates the ontology based on the information received from the AruCo markers; after that, it calculates the path (highlated in green in Rviz) to reach the desired position. At the left bottom of the panel there is also the camera vision.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment2/blob/main/images/init.png" width=70%, height=70%>
</p>

In the following figure after the robot finishes checking a room, it decides in which location to go next and recalculates the path.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment2/blob/main/images/visit.png" width=70%, height=70%>
</p>

In the following figure there is the whole scanned environment in Rviz.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment2/blob/main/images/final.png" width=49%, height=49%>
</p>


### Limitations and future technical improvement
The detection of the markers is not extensively general, so if the user were to use `detect_marker.cpp` for an environment in which markers were placed differently, it would work badly. Therefore, a possible improvement could be to implement a solution that is as general as possible, in order to use it with whatever environment we want to navgate through.

Another limitation is the robot itself, given that the author opted for a very easy solution. A possible improvement could be to build a robot with four wheels.

## Authors & Contacts
[Sara Sgambato](https://github.com/sarasgambato)

sarasgambato@yahoo.it (personal email)

s4648592@studenti.unige.it (institutional email)
