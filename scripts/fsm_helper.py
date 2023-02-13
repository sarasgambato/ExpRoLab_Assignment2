#!/usr/bin/env python3

"""
.. module:: fsm_helper
    :platform: ROS
    :synopsis: Python module used to simplify the fsm code.
    
.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This module implements one class representing an helper to help the fsm taking decisions.

Params:
    :attr:`config/recharge_room`: name and (x,y) coordinates of the recharging room

    :attr:`config/robot`: name of the robot

Publishes to:
    :attr:`/robot_assignment/joint1_position_controller/command`: to move the 1st joint of the robot's arm

Clients:
    :attr:`armor_client`: client to communicate with the aRMOR server
"""

import rospy
import random
import time
from armor_api.armor_client import ArmorClient
from ExpRoLab_Assignment2 import architecture_name_mapper as anm
from std_msgs.msg import Float64

client = ArmorClient("armor_client", "my_ontology")

# Parameter for the busy time in the recharge function of class BehaviorHelper
BUSY_TIME = anm.BUSY_TIME
SLEEP_TIME = anm.SLEEP_TIME

class BehaviorHelper:
    """
    Class that implements some function useful for the fsm to take decisions.
    """

    def __init__(self):
        # If the main individual has to be changed, change the value in the architecture.launch
        self.robot = rospy.get_param("config/robot")
        # If the recharging room has to be changed, change the value in the architecture.launch
        self.recharging_room = rospy.get_param("config/recharge_room")
        self.recharging_room = self.recharging_room["room"]
        # Publisher to the joint that moves the camera
        self.joint01_pub = rospy.Publisher("/robot_assignment/joint1_position_controller/command", Float64, queue_size=1)

    def clean_list(self, type, list_):
        """
        Function to clean a list.

        Args:
            type(Int): value specifying how to clean the string, given that timestamps 
                              and strings indicating locations have a different length
            list_(Str): list that has to be cleaned

        Returns:
            list_(Str): cleaned list
        """

        if type == 1:
            if len(list_) == 1:
                list_ = list_[0][32:-1]
            else:
                for i in range (len(list_)):
                    list_[i] = list_[i][32:-1]

        if type == 2:
            list_ = list_[0][1:-11]

        return list_  

    def get_queried(self, obj):
        """
        Function to get the queried object.
        
        Args:
            obj(Str): string representing the object we want to query
            
        Returns:
            Str: list of queried objects
        """

        if obj == 'position':
            list_ = client.query.objectprop_b2_ind('isIn', self.robot)
            return self.clean_list(1, list_)

        if obj == 'reachable':
            list_ = client.query.objectprop_b2_ind('canReach', self.robot)
            list_ = self.clean_list(1, list_)
            if isinstance(list_, str):
                list_ = [list_]
            return list_

        if obj == 'urgencies':
            list_ = client.query.ind_b2_class('URGENT')
            return self.clean_list(1, list_) 

        if obj == 'corridors':
            list_ = client.query.ind_b2_class('CORRIDOR')
            return self.clean_list(1, list_) 
            

    def get_timestamp(self, dataprop, ind):
        """
        Function to get the timestamp for the 'dataprop' property of the 'ind' individual.
        
        Args:
            dataprop(Str): string representing the property for which we want the timestamp
            ind(Str): string representing the individual of interest
            
        Returns:
            Str: cleaned timestamp of the property of the individual
        """

        timestamp = client.query.dataprop_b2_ind(dataprop, ind)
        return self.clean_list(2, timestamp)
    
    def reason_reach(self, reachable, position):
        """
        Function to decide in which location to go next. It is called when there are no urgent locations.
        
        Args:
            reachable(Str): list of all the reachable locations
            position(Str): string representing the current pose of the robot
            
        Returns:
            Str: the location in which the robot will go next
        """

        corridors = self.get_queried('corridors')
        # if there is no urgency, and the robot is already in a corridor, continuosly check the corridors
        if position in corridors:
            corridors.remove(position)
            # I assume that there is more then one corridor
            return random.choice(corridors)
        # if there is no urgency, and the robot is in a room, there will be only one reachable location
        else:
            return reachable[0]

    def reason_urg(self, reachable_urg):
        """
        Function to decide how to manage the urgent locations. It is called only when there are urgent locations.
        The robot goes in the most urgent location, which is the one that has not been visited for the longest time.
        
        Args:
            reachable_urg(Str): list of all the reachable locations that are urgent
            
        Returns:
            Str: the location in which the robot will go next
        """

        # if there is only one urgent location, go there
        if len(reachable_urg) == 1:
            return reachable_urg[0]
        # else choose based on the timestamp
        else:
            visits = []
            for loc in reachable_urg:
                last_visit = self.get_timestamp('visitedAt', loc)
                visits.append(last_visit)
            return reachable_urg[visits.index(min(visits))]

    def decide_location(self):
        """
        Function to decide in which location the robot will go to next.
        
        Args:
            None
            
        Returns:
            position(Str): current position of the robot
            target(Str): string representing the location in which the robot will go to
        """

        position = self.get_queried('position')
        reachable = self.get_queried('reachable')
        urgencies = self.get_queried('urgencies')
        reachable_urg = list(set(urgencies) & set(reachable))
        print('Current position: ' + position)
        print('Reachable locations: ['+', '.join(reachable)+']')
        print('Urgent: ['+', '.join(reachable_urg)+']')

        # if there are no urgent locations to check, reason where to go next in a certain way
        if (reachable_urg == []):
            target = self.reason_reach(reachable, position)
        # if there are some urgent locations to check, reason in a different way
        else:
            target = self.reason_urg(reachable_urg)
            
        print('Going to visit location ' + target)
        return position, target

    def check_location(self, position, target):
        """
        Function to reach and check the location that the robot has to go to.
        
        Args:
            position(Str): current position of the robot, which has to be updated
            target(Str): location with which to update the robot position
            
        Returns:
            None
        """

        client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, target, position)
        last_change = self.get_timestamp('now', self.robot)
        now = str(int(time.time()))
        client.manipulation.replace_dataprop_b2_ind('now', self.robot, 'Long', now, last_change)
        last_visit = self.get_timestamp('visitedAt', target)
        client.manipulation.replace_dataprop_b2_ind('visitedAt', target, 'Long', now, last_visit)
        # Check the room
        self.look_around()        
        print('Reached target...mmh...everything clear')
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner() 

    def look_around(self):
        """
        Function to check the location that the robot is into. It is done by rotating the camera of approximately 360 degrees.
        
        Args:
            None
            
        Returns:
            None
        """

        cam_pose = 0
        msg = Float64()
        # move the camera of 0.5 rad until its position will be equal to 6 rad
        while (cam_pose != 6):
            cam_pose = cam_pose + 0.5
            msg.data = cam_pose
            self.joint01_pub.publish(msg)
            rospy.sleep(0.2)
        # return to the initial configuration
        msg.data = 0
        self.joint01_pub.publish(msg)

    def recharge(self, position):
        """
        Function to recharge the battery. The robot checks the reachable locations: if room E can be reached, then the robot goes there;
        if room E is not in the reachable locations, then choose randomly in which location to go next and repeat until room E is reachable.
        
        Args: 
            position(Str): current position of the robot, which whill be updated with location 'E'
            
        Returns:
            None
        """

        print('I need to recharge, going to room ' + self.recharging_room)
        if position == self.recharging_room:
            reached = True
        else:
            reached = False
        while(reached == False):
            reachable = self.get_queried('reachable')
            # If room E is reachable, go there
            if self.recharging_room in reachable:
                reached = True
            # Else choose randomly and repeat
            else:
                if len(reachable) == 1:
                    choice = reachable[0]
                else:
                    choice = random.choice(reachable)
                client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, choice, position)
                client.utils.apply_buffered_changes()
                client.utils.sync_buffered_reasoner()
                # Update the current position
                position = choice       
                print('Almost there...I am in location ' + position + ' now')
            rospy.sleep(BUSY_TIME)
        client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, self.recharging_room, position)       
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()