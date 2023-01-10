#!/usr/bin/env python3

"""
.. module:: helper
    :platform: ROS
    :synopsis: Python module used to simplify the fsm code.
    
.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This module implements three classes representing three different helpers: one to simplify the implementation of a client for ROS action servers, 
one that manages the synchronization with subscribers and action servers and one to help the fsm machine taking decisions.

Clients:
    :attr:`armor_client`: client to communicate with the aRMOR server
    :attr:`motion/planner`: client to communicate with the planner server, which plans a random path with via points
    :attr:`motion/controller`: client to communicate with the controller server, which has to follow the path provided by the planner server

Subscribes to:
    :attr:`state/battery_low` where the state of the battery (high/low) is published

Servers:
    :attr:`state/set_pose`: server to set the current robot pose, stored in the 'robot_state' node
"""

import rospy
import random
import time
from actionlib import SimpleActionClient
from threading import Lock
from armor_api.armor_client import ArmorClient
from Assignment_1 import architecture_name_mapper as anm
from std_msgs.msg import Bool
from Assignment_1.msg import PlanAction, ControlAction
from Assignment_1.srv import SetPose

client = ArmorClient("armor_client", "my_ontology")

# Parameter for the busy time in the recharge function of class BehaviorHelper
BUSY_TIME = anm.BUSY_TIME

class ActionClientHelper:
    """
    Class that simplifies the implementation of a client for ROS action servers.
    """

    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`
        self.reset_client_states()
        # Set the name of the server to be invoked
        self._service_name = service_name
        # Get or create a new mutex
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive
        self._client.wait_for_server()

    def send_goal(self, goal):
        """
        Function to send the action server a new goal only if it is not running; the server can be used only by one client at a time.

        Args:
            goal(PlanGoal): goal to be sent made up of two Points, start and target in (x, y) coordinates

        Returns:
            None
        """

        if not self._is_running:
            # Start the action server
            self._client.send_goal(goal,
                                   done_cb = self.done_callback_,
                                   feedback_cb = self.feedback_callback_)
            # Set the client's states
            self._is_running = True
            self._is_done = False
            self._results = None
            print("SENDING GOALS")
        else:
            print("Warning: to send a new goal cancel the current request first!")

    def cancel_goals(self):
        """
        Fucntion to stop the computation of the action server, only if it is actually computing.
        
        Args:
            None
            
        Returns:
            None
        """
        
        if self._is_running:
            # Stop the computation
            self._client.cancel_all_goals()
            # Reset the client's state
            self.reset_client_states()
        else:
            print("Warning: cannot cancel a not running service!")

    def reset_client_states(self):
        """
        Function to reset the client state variables stored in this class.
        
        Args:
            None
            
        Returns:
            None
        """

        self._is_running = False
        self._is_done = False
        self._results = None

    def feedback_callback_(self, feedback):
        """
        Function called when the action server has to send a feedback to the client.
        
        Args:
            feedback: feedback message to be sent to the client
            
        Returns:
            None
        """

        self._mutex.acquire()
        try:
            # Call the method provided by the node that uses this action client to manage a feedback
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
        finally:
            # Realise the mutex to unblock ROS-based thread waiting on the same mutex
            self._mutex.release()

    def done_callback_(self, status, results):
        """
        Function called when the action server has finished its computation.
        
        Args:
            status: status of the action server
            results: results from the action server
            
        Returns:
            None
        """
        
        self._mutex.acquire()
        try:
            # Set the client's states
            self._is_running = False
            self._is_done = True
            self._results = results
            # Call the method provided by the node that uses this action client to manage a result
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
        finally:
            self._mutex.release()

    def is_done(self):
        """
        Function returning whether the action server finished its computation or not. This method should be mutex safe.
        
        Args:
            None
            
        Returns:
            Bool: 'True' if the action server finished its computation, 'False' otherwise
        """
        
        return self._is_done

    def is_running(self):
        """
        Function returning whether the action server is running or not. This method should be mutex safe.
        
        Args:
            None
            
        Returns:
            Bool: `True` if the action server is running, `False` otherwise
        """

        return self._is_running

    def get_results(self):
        """
        Function that gets the result of the action server.
        
        Args:
            None
            
        Returns:
            Result of the action server, if any, 'None' otherwise
        """

        if self._is_done:
            return self._results
        else:
            print("Error: cannot get result")
            return None

class InterfaceHelper:
    """
    Class that manages the synchronization with subscribers and action servers.
    """

    def __init__(self):
        # Create a shared mutex to synchronize action clients and subscribers
        self.mutex = Lock()
        # Set the initial state
        self.reset_states()
        # Define the callback associated with the battery low ROS subscribers
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.battery_callback_)
        # Define the clients for the the plan and control action servers
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    def reset_states(self):
        """
        Function to reset the stimulus for the battery stored as state variable. 
        This function assumes that no states of the Finite State Machine run concurrently.
        
        Args: 
            None
        
        Returns:
            None
        """

        self._battery_low = False

    def battery_callback_(self, msg):
        """
        Function for the subscriber to get messages published from the `robot_state` node into the `/state/battery_low/` topic.
        
        Args:
            msg(Bool): status of the battery
            
        Returns:
            None
        """

        # Acquire the mutex to assure the synchronization with the other subscribers and action clients
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class
            self._battery_low = msg.data
        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting
            self.mutex.release()

    def is_battery_low(self):
        """
        Fucntion to get the state variable encoded in this class about the battery level.
        
        Args:
            None
        
        Returns:
            Bool: `True` if the battery is low, `False` otherwise
        """

        return self._battery_low

    @staticmethod
    def init_robot_pose(point):
        """
        Function to update the current position of the robot stored in the 'robot_state' node.
        
        Args:
            point(Point): point representing the robot pose in (x, y) coordinates
            
        Returns:
            None
        """

        # Wait for the server to be initialised
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            # Call the service and set the current robot position
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)
            print("Setting initial robot position")
        except rospy.ServiceException as e:
            print("Cannot set current robot position")

class BehaviorHelper:
    """
    Class that implements some function useful for the fsm to take decisions.
    """

    def __init__(self):
        # If the main individual has to be changed, change this value
        self.robot = 'Robot1'
        # If the recharging room has to be changed, change this value
        self.recharging_room = 'E'

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
        corridors = self.get_queried('corridors')
        if target not in corridors: # the 'visitedAt' property of the corridors has not been considered, given that they will never be urgent
            last_visit = self.get_timestamp('visitedAt', target)
            client.manipulation.replace_dataprop_b2_ind('visitedAt', target, 'Long', now, last_visit)
        print('Reached target...mmh...everything clear')
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()      

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
        print('Reached room ' +  self.recharging_room + ', recharging...')