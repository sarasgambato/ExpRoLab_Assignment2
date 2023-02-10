#! /usr/bin/env python3

import random
import rospy
import numpy as np
# Import constant name defined to structure the architecture.
from ExpRoLab_Assignment2 import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from ExpRoLab_Assignment2.msg import Point, PlanFeedback, PlanResult, PlanAction
from ExpRoLab_Assignment2.srv import GetPose
import ExpRoLab_Assignment2  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER

# An action server to simulate motion planning.
# Given a target position, it retrieve the current robot position from the 
# `robot-state` node, and return a plan as a set of via points.
class PlaningAction(object):

    def __init__(self):
        # Get random-based parameters used by this server
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'`{anm.ACTION_PLANNER}` Action Server initialised. It will calculate some via points to the target.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
      
    def execute_callback(self, goal):
        # Get the input parameters to compute the plan, i.e., the start (or current) and target positions.
        start_point = _get_pose_client()
        target_point = goal.target

        # Check if the start and target positions are correct. If not, this service will be aborted.
        if start_point is None or target_point is None:
            log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        
        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)

        x_lin = np.linspace(start_point.x, target_point.x, 8)
        y_lin = np.linspace(start_point.y, target_point.y, 8)
        log_msg = f'Server is planning {8 + 1} points...'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


        # Generate the points of the plan.
        for i in range(1, 8):
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Server has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()  
                return
            # Generate a new point of the plan.
            new_point = Point()
            new_point.x = x_lin[i]
            new_point.y = y_lin[i]
            feedback.via_points.append(new_point)
        
        # Publish the feedback
        self._as.publish_feedback(feedback)

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points
        self._as.set_succeeded(result)
        log_msg = 'Motion plan succeeded with plan: '
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


# Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.
def _get_pose_client():
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        # Call the service and get a response with the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        # Log service response.
        log_msg = f'Retrieving current robot position from the `{anm.NODE_ROBOT_STATE}` node as: ({pose.x}, {pose.y}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return pose
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.    
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()