#!/usr/bin/env python3

"""
.. module:: architecture_name_mapper
    :platform: ROS
    :synopsis: Python module containing the name of each node, topic, server, actions and parameters used in this architecture.
    
.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This module was created to define some useful parameters and the names of servers, nodes and topics, so that if the user wants to change them, they do not
have to do so in every file, it is sufficient to change them here.
"""
import rospy

# The name of parameter to set the initial robot position.
PARAM_INITIAL_POSE = 'state/initial_pose'
# ---------------------------------------------------------


# The name of the node that sets/gets the pose of the robot and manages its battery.
NODE_ROBOT_STATE = 'robot_states'

# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the current robot pose. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the client to control the robot
CLIENT_MOVE_BASE = 'move_base'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# The name of the topic where the list of markers' ID is published
TOPIC_MARKER_LIST = '/id_list'
# ---------------------------------------------------------


# Parameter indicating the busy waiting time [s]
BUSY_TIME = 5

# Parameter indicating the sleep time [s]
SLEEP_TIME = 0.3

# Parameter indicating the battery time [s]
BATTERY_TIME = 1800
# ---------------------------------------------------------


def tag_log(msg, producer_tag):
    """
	Function used to label each log with a producer tag.
	
    Args:
        msg(Str): message that will be visualized
        producer_tag(Str): tag identifying the log producer
            
    Returns:
        log_msg(Str): message for the log
    """

    return '@%s>> %s' % (producer_tag, msg)
