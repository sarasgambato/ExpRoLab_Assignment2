#! /usr/bin/env python

import random
import rospy
import actionlib
from ExpRoLab_Assignment2 import architecture_name_mapper as anm
from actionlib import SimpleActionServer
from ExpRoLab_Assignment2.msg import ControlFeedback, ControlResult
from ExpRoLab_Assignment2.srv import SetPose
import ExpRoLab_Assignment2  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
from move_base_msgs.msg import MoveBaseAction

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER

class ControllingAction(object):

    def __init__(self):
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      ExpRoLab_Assignment2.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self._as.start()

        # Log information.


    def execute_callback(self, goal):
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return
        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            _set_pose_client(point)
            # Log current robot position.
            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.


# Update the current robot `pose` stored in the `robot-state` node.
# This method is performed for each point provided in the action's server feedback.
def _set_pose_client(pose):
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:
        # Log service call.
        log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
        service(pose)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
