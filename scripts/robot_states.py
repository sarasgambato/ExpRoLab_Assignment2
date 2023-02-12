#!/usr/bin/env python3

"""
.. module:: robot_states
    :platform: ROS
    :synopsis: Python module for the publisher of the state of the battery.

.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This node has a class in which two services are defined: one to get the current robot pose and one to set it.
Also, the class defines a publisher to notify that the battery is low.

Params:
    :attr:`config/recharge_room`: name and (x,y) coordinates of the recharging room

Publishes to:
    :attr:`state/battery_low` the state of the battery (high/low)

Subscribes to:
    :attr:`/odom` the position of the robot

Servers: 
    :attr:`state/set_pose`: server to set the current robot pose

    :attr:`state/get_pose`: server to get the current robot pose
"""

import threading
import rospy
import math
from helper import InterfaceHelper
from ExpRoLab_Assignment2 import architecture_name_mapper as anm
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ExpRoLab_Assignment2.msg import Point
from ExpRoLab_Assignment2.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse

# A tag for identifying logs producer
LOG_TAG = anm.NODE_ROBOT_STATE

# Constant value representing the battery usage time
BATTERY_TIME = anm.BATTERY_TIME

class RobotState:
    """
    Class implementing the services of the robot position and the publisher for the battery level.
    """

    def __init__(self, helper):
        # Initialise robot position
        self._pose = None
        # Initialise battery level
        self._battery_low = False
        # Initialise the helper
        self._helper = helper
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)
        # Subscriber to odometry
        rospy.Subscriber("/odom", Odometry, self.odometry_cb)
        # Start publisher on a separate thread
        th = threading.Thread(target=self.is_battery_low_)
        th.start()
        # Log information
        log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}` and '
                   f'`{anm.SERVER_SET_POSE}`, and topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # variable to get
        self._recharge_room = rospy.get_param("config/recharge_room")

    def odometry_cb(self, msg):
        """
        Function implementing the callback of the subscriber to the /odom topic.

        Args:
            msg(Odometry): pose of the robot

        Returns:
            None
        """

        self._odom = msg.pose.pose.position #store the (x,y,z) position of the robot

    def set_pose(self, request):
        """
        Function implementing the 'state/set_pose' service.

        Args:
            request(Point): current robot position to be set

        Returns:
            SetPoseResponse(): empty response
        """

        if request.pose is not None:
            # Store the new current robot position
            self._pose = request.pose
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        return SetPoseResponse()

    def get_pose(self, request):
        """
        Function implementing the 'state/get_pose' service.
        
        Args:
            request: given by the client as empty, it is not used
            
        Returns:
            response(Point): current robot position
        """

        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        # Create the response with the robot pose and return it
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def is_battery_low_(self):
        """
        Fucntion that publishes the changes of the battery level.
        
        Args:
            None
            
        Returns:
            None
        """

        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        self.battery_notifier_(publisher)

    def battery_notifier_(self, publisher):
        """
        Function that publishes when the battery changes state (high/low) based on a constant delay.
        
        Args:
            publisher(Publisher): publisher for the message
            
        Returns:
            None
        """

        while not rospy.is_shutdown():
            # Publish battery level: 'True' if the battery is low, 'False' otherwise
            publisher.publish(Bool(self._battery_low))
            # Simulate battery usage, count if the battery is high or if the robot has reached the recharging position
            if self._battery_low == False:
                rospy.sleep(BATTERY_TIME)
                # Change battery state
                self._battery_low = not self._battery_low
                rospy.loginfo(anm.tag_log('Robot got low battery.', LOG_TAG))
            else:
                # If the battery is low, start recharging as soon as the robot enters a 
                # circumference of radius 2 centered in the recharging point
                if(self._odom.x-self._recharge_room["x"])**2+(self._odom.y-self._recharge_room["y"])**2 <= 2**2:
                    rospy.sleep(BATTERY_TIME)
                    # Change battery state
                    self._battery_low = not self._battery_low
                    rospy.loginfo(anm.tag_log('Robot got a fully charged battery.', LOG_TAG))

if __name__ == "__main__":
    # Initialise this node
    rospy.init_node(anm.NODE_ROBOT_STATE, log_level = rospy.INFO)
    # Define the helper
    helper = InterfaceHelper()

    # Instantiate the node manager class
    RobotState(helper)

    # Initialize robot position
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    helper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))

    rospy.spin()