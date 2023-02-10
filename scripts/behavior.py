#!/usr/bin/env python3 

"""
.. module:: behavior
    :platform: ROS
    :synopsis: Python module for the initialization and definition of the finite state machine.
    
.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This module defines the behavior of the Finite State Machine. It has two states and one inner FSM:

* Initialization state: the ontology is created;
* Recharging state: the robot goes to recharge;
* Normal: inner FSM with two states, Decision state and Checking state, to decide what location to visit next and to check it.

Servers:
    :attr:`fsm_server`: introspection server of SMACH, necessary for the developer to visualize the state machine
"""

import rospy
import random
import smach_ros
from smach import StateMachine, State
from load_ontology import LoadMap
from ExpRoLab_Assignment2 import architecture_name_mapper as anm
from helper import InterfaceHelper, BehaviorHelper
from arch_skeleton.msg import Point, ControlGoal, PlanGoal
from ExpRoLab_Assignment2.srv import LoadMap
from os.path import dirname, realpath

# list of states in the machine
STATE_INIT = 'INITIALIZE_MAP'
STATE_DECISION = 'DECIDE_LOCATION'
STATE_PLAN_TARGET = 'PLAN_TO_TARGET'
STATE_CHECK_TARGET = 'CHECK_TARGET'
STATE_NORMAL = 'NORMAL'
STATE_RECHARGING = 'RECHARGING'

# list of transition states
TRANS_INITIALIZED = 'everything_loaded'
TRANS_DECIDED = 'target_acquired'
TRANS_BATTERY_LOW = 'battery_low'
TRANS_RECHARGING = 'recharging'
TRANS_RECHARGED = 'recharged'
TRANS_WENT_TARGET = 'went_target'

# Sleeping time of the waiting thread
SLEEP_TIME = anm.SLEEP_TIME

class LoadOntology(State):
    """
    Class related to the state that loads the ontology.
    """

    def __init__(self, helper):
        State.__init__(self, outcomes=[TRANS_INITIALIZED],
                       output_keys=['rooms'])
        self._helper = helper
        # Usefull variables
        self.rooms = {}

    def execute(self, userdata):
        """
        Function that is executed when entering the INITIALIZE_MAP state of the fsm. 
        It calls the detecter of the markers and waits for the map to be built, then it transitions to the state NORMAL.

        Args: 
            userdata: not used

        Returns: 
            Str: 'everything_loaded' once the map has been intialized
        """

        while not self._helper._marker_list:
            pass
        
        # Create a client for the load_map service
        load_map_client = rospy.ServiceProxy("/load_map", LoadMap)

        for i in range(0, len(self._helper._marker_list)):
            # Wait for the service to become available
            rospy.wait_for_service("/load_map")
            try:
                # Call the load_map service
                res = load_map_client(self._helper._marker_list[i])
                self.rooms[res.coordinate.room] = {"x": res.coordinate.x, "y": res.coordinate.y}
                print("Added location " + res.coordinate.room + 
                      " with coordinates (" + str(res.coordinate.x) + "," + str(res.coordinate.y) + ")")
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        print("Map successfully initialized.")
        userdata.rooms = self.rooms

        return TRANS_INITIALIZED

class Recharging(State):
    """
    Class related to the state that recharges the robot's battery.
    """

    def __init__(self, helper):
        State.__init__(self, outcomes=[TRANS_RECHARGED])
        self._helper = helper

    def execute(self, userdata):
        """
        Function that is executed every time the fsm enters the RECHARGING state.
        It recharges the battery of the robot and transitions to the state NORMAL.
        
        Args:
            userdata: not used
            
        Returns:
            Str: 'recharged' once the battery is recharged
        """

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if not self._helper.is_battery_low():
                    self._helper.reset_states()
                    return TRANS_RECHARGED
            finally:
                self._helper.mutex.release()
            rospy.sleep(SLEEP_TIME)

class DecideTarget(State):
    """
    Class related to the state that reasons about the next location that the robot will check.
    """

    def __init__(self, helper, sm_helper):
        self._helper = helper
        self._sm_helper = sm_helper
        State.__init__(self, outcomes=[TRANS_DECIDED, TRANS_RECHARGING], 
                       input_keys=['rooms'],
                       output_keys=['current_pose', 'choice', 'path_plan'])

    def execute(self, userdata):
        """
        Function that is executed every time the fsm enters the DECIDE_LOCATION state.
        It decides in which location to go next based on some criteria.

        Args:
            userdata: used to pass data between states of the FSM

        Returns:
            Str: 'recharging' if the battery is low, 'target_acquired' otherwise
        """
        goal = PlanGoal()
        current_pose, choice = self._sm_helper.decide_location()
        rooms = userdata.rooms
        # Get the coordinates of the decided location
        goal.target.x = rooms[choice].get("x")
        goal.target.y = rooms[choice].get("y")
        self._helper.planner_client.send_goal(goal)

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if self._helper.is_battery_low(): #higher priority
                    self._helper.planner_client.cancel_goals()
                    self._sm_helper.recharge(current_pose)
                    return TRANS_RECHARGING
                
                if self._helper.planner_client.is_done():
                    userdata.choice = choice
                    userdata.current_pose = current_pose
                    userdata.path_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_DECIDED

            finally:
                self._helper.mutex.release()
            rospy.sleep(SLEEP_TIME)

class CheckTarget(State):
    """
    Class related to the state that checks the location the robot is in.
    """

    def __init__(self, helper, sm_helper):
        self._helper = helper
        self._sm_helper = sm_helper
        State.__init__(self, outcomes=[TRANS_WENT_TARGET, TRANS_RECHARGING], 
                       input_keys=['current_pose', 'choice', 'path_plan'])

    def execute(self, userdata):
        """
        Function that is executed every time the fsm enters the CHECK_TARGET state.
        It makes the robot go to the wanted location and check it.
        
        Args:
            userdata: used to pass data between states of the FSM
            
        Returns:
            Str: 'recharging' if the battery is low, 'went_target' otherwise
        """

        # send the goal to the controller and check that the via points are reached
        plan = userdata.path_plan
        current_pose = userdata.current_pose
        goal = ControlGoal(via_points=plan)
        self._helper.controller_client.send_goal(goal)

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if self._helper.is_battery_low():
                    self._helper.controller_client.cancel_goals()
                    self._sm_helper.recharge(current_pose)
                    return TRANS_RECHARGING
                    
                if self._helper.controller_client.is_done():
                    choice = userdata.choice
                    self._sm_helper.check_location(current_pose, choice)
                    return TRANS_WENT_TARGET
            finally:
                self._helper.mutex.release()
            rospy.sleep(SLEEP_TIME)

def main():
    """
    Main function which initalizes the Finite State Machine node 'behavior'.
    Two helpers are defined to help the state machine take decisions and to manage the battery.
    The fsm is hierarchical as to create a more modular and robust architecture. 
    """
    rospy.init_node('behavior', log_level = rospy.INFO)

    # Define the helper that manages the battery stimulus
    helper = InterfaceHelper()

    # Define the helper for the states of the fsm
    sm_helper = BehaviorHelper()

    sm_main = StateMachine([])

    with sm_main:
        # Initialization state
        StateMachine.add(STATE_INIT, LoadOntology(helper),
                        transitions={TRANS_INITIALIZED: STATE_NORMAL})

        # Inner fsm
        sm_normal = StateMachine(outcomes=[TRANS_BATTERY_LOW], input_keys = ['rooms'])
        with sm_normal:
            # Decision state
            StateMachine.add(STATE_DECISION, DecideTarget(helper, sm_helper),
                            transitions={TRANS_DECIDED: STATE_CHECK_TARGET,
                                        TRANS_RECHARGING: TRANS_BATTERY_LOW})
            # Checking state
            StateMachine.add(STATE_CHECK_TARGET, CheckTarget(helper, sm_helper),
                            transitions={TRANS_WENT_TARGET: STATE_DECISION,
                                        TRANS_RECHARGING: TRANS_BATTERY_LOW})
        StateMachine.add(STATE_NORMAL, sm_normal,
                         transitions={TRANS_BATTERY_LOW: STATE_RECHARGING})   
        # Recharging state
        StateMachine.add(STATE_RECHARGING, Recharging(helper),
                         transitions={TRANS_RECHARGED: STATE_NORMAL}) 

    # Create and start the introspection server for visualization 
    sis = smach_ros.IntrospectionServer('fsm_server', sm_main, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
   
if __name__ == '__main__':
    main()