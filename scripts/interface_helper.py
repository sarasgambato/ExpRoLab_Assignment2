#!/usr/bin/env python3

"""
.. module:: interface_helper
    :platform: ROS
    :synopsis: Python module used to simplify the interaction with Action Clients.
    
.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This module implements two classes representing two different helpers: one to simplify the implementation of a client for ROS action servers and 
one that manages the synchronization with subscribers and action servers.

Subscribes to:
    :attr:`state/battery_low`: where the state of the battery (high/low) is published

    :attr:`id_list`: where the list with the markers' ID is published

Servers:
    :attr:`state/set_pose`: server to set the current robot pose, stored in the 'robot_state' node

Clients:
    :attr:`move_base`: client to communicate with move_base in order for the robot to move in the simulation
"""

import rospy
from actionlib import SimpleActionClient
from threading import Lock
from ExpRoLab_Assignment2 import architecture_name_mapper as anm
from std_msgs.msg import Bool, Int32MultiArray
from ExpRoLab_Assignment2.srv import SetPose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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
            goal: goal to be sent made up of two Points, start and target in (x, y) coordinates

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
        # Define the callback associated with the marker ID ROS subscribers
        rospy.Subscriber(anm.TOPIC_MARKER_LIST, Int32MultiArray, self.list_callback_)
        # Define the client for move_base
        self.move_base_client = ActionClientHelper(anm.CLIENT_MOVE_BASE, MoveBaseAction, mutex=self.mutex)
        # Variable to store the markers' ID
        self._marker_list = []

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

    def list_callback_(self, msg):
        """
        Callback of the subscriber to get messages published from the `detect_marker` node into the `/id_list` topic.
        
        Args:
            msg(List): list of found markers
            
        Returns:
            None
        """
        
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class
            self._marker_list = msg.data
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

    @staticmethod
    def create_move_base_goal(room):
        """
        Function used to create the goal to send to the /move_base Action Client

        Args:
            room(Point): (x,y) coordinates of the room the robot has to reach

        Returns:
            goal(MoveBaseGoal): goal that will be sent to move_base
        """

        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = room.x
        goal.target_pose.pose.position.y = room.y
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        return goal
