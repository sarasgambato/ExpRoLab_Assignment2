# ExpRoLab Assignment 2
The code documentation for this assignment can be found here.

## Introduction
In the second assignment of the Experimental Robotics Laboratory course, we were requested to integrate the architecture developed in the [1st assignment](https://github.com/sarasgambato/ExpRoLab_Assignment1) (by modifyig it, eventually) with a robotic simulation.

We were provided with [this package](https://github.com/CarmineD8/assignment2) and our requirements were:
- to add a robot in the environment and to spawn it in the initial the position x = -6.0, y = 11.0;
- to the "semantic" map of the environment by detecting, without moving the base of the robot, all seven AruCo markers that were present around it, by calling the provided service node;
- to start the patrolling algorithm by relying on autonomous navigation strategies and on the information collected and stored in the ontology during the previous step;
- to perform a complete scan of the room (by rotating the base or the camera) when a room was reached.

## Software architecture
The user can find a detailed description of the software architecture that was used [here](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/README.md#software-architecture).

However, some changes were done:
- The author did some changes in the `planner` and `controller` nodes by author [Luca Buoncompagni](https://github.com/buoncubi)
  - in the `planner` the via points that are calculated are taken from the linspace between the start position of the robot (the first via point) and the target position (the last via point);
  - in the `controller` node the robot is ctually controlled through the `move_base` simple action client in order to reach the last via point, which is the target.

the goal that is sent to the `planner` is the actual room the the robot is required to reach 

## Installation & running
### Installation
To correctly use this software, the user must follow these steps to install the required packages/repositories.
1. Given that the author decided to use the file [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl) from the repository [topological_map](https://github.com/buoncubi/topological_map) (by author [Luca Buoncompagni](https://github.com/buoncubi)), the user must clone the mentioned repository and the current one in the ROS workspace.
2. Install the [aRMOR package](https://github.com/EmaroLab/armor) by following [this tutorial](https://github.com/EmaroLab/armor/issues/7).
3. Clone the [armor_py_api](https://github.com/EmaroLab/armor_py_api) repository in your ROS workspace.
4. Perform what is described in these [notes](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/README.md#note).
4. Install the [SMACH package](http://wiki.ros.org/smach) by running `sudo apt-get install ros-noetic-executive-smach*` or equivalent for your ROS distribution.
5. Run `chmod +x <file_name>` for each file inside the folder `scripts` of the package `ExpRoLab_Assignment2`.
6. Run `catkin_make` from the root of your ROS workspace.

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

## System's features
### Environment
The enviornment used for the assignment is the one shown in the following figure. Some of the markers can be seen (there are 7 markers in total), plus the robot in its initial position.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment2/blob/main/images/environment.png" width=70%, height=70%>
</p>

### Limitations and future technical improvement
The detection of the markers is thought only for this particular environment, so if the user were to use `marker_detect.cpp` for another environment, it would not work. Therefore, a possible improvement could be to implement a solution that is as general as possible, in order to use it with whatever environment we have at hand.

Another limitation is the robot itself, given that the author opted for a very easy solution. A possible improvement could be to buil a more sophisticated robot or tu use the URDF of a commercial robot.

## Authors & Contacts
[Sara Sgambato](https://github.com/sarasgambato)

sarasgambato@yahoo.it (personal email)

s4648592@studenti.unige.it (institutional email)
